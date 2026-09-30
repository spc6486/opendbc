"""
Toyota EPS steer-rate fault avoidance (opendbc/car/toyota/carcontroller.py): on the Lexus IS with the TSS2 EPS retrofit
(ToyotaFlagsSP.TSS2_EPS), every request cut is followed by one frame with no STEERING_LKA, so the cut is not replaced
within a few milliseconds by the next request frame when pandad sends it one cycle late. Every other car keeps the
upstream stream. Drives the real CarController and checks the STEERING_LKA stream against the panda's Toyota rules
through libsafety.
"""
import itertools
import math
import unittest

from opendbc.car import DT_CTRL, gen_empty_fingerprint, structs
from opendbc.car.lateral import apply_meas_steer_torque_limits, common_fault_avoidance
from opendbc.car.toyota.carcontroller import MAX_STEER_RATE, MAX_STEER_RATE_FRAMES
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.structs import CarParams
from opendbc.car.toyota.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

STEERING_LKA = 0x2E4
EPS_STATUS = 0x262
FRAME_US = int(DT_CTRL * 1e6)


def make_interface(candidate=CAR.LEXUS_IS, eps_length=8):
  fingerprint = gen_empty_fingerprint()
  if eps_length is not None:
    fingerprint[0][EPS_STATUS] = eps_length
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


def lka_fields(msg):
  _, dat, _ = msg
  torque = (dat[1] << 8) | dat[2]
  torque = torque - 65536 if torque & 0x8000 else torque
  return dat[0] & 1, torque


def run(ci, rates, lat_active=None, torque=1.0, eps=None):
  """One entry per control frame: None when no STEERING_LKA was sent, else its (address, dat, bus).
  torque: one value for every frame or one per frame; eps: the EPS-reported torque per frame (default 0)."""
  lat_active = lat_active if lat_active is not None else [True] * len(rates)
  torques = torque if isinstance(torque, list) else [torque] * len(rates)
  eps = eps if eps is not None else [0.] * len(rates)
  out = []
  now_nanos = 0
  for rate, lat, tq, eps_torque in zip(rates, lat_active, torques, eps, strict=True):
    CC = structs.CarControl()
    CC.enabled = lat
    CC.latActive = lat
    CC.actuators.torque = tq
    ci.update([])
    ci.CS.out.steeringRateDeg = rate
    ci.CS.out.steeringTorqueEps = eps_torque
    _, sends = ci.apply(CC.as_reader(), structs.CarControlSP(), now_nanos)
    lka = [m for m in sends if m[0] == STEERING_LKA]  # (address, dat, bus)
    assert len(lka) <= 1
    out.append(lka[0] if lka else None)
    now_nanos += int(DT_CTRL * 1e9)
  return out


def upstream_stream(params, rates, lat_active, torques, eps):
  """(request, torque) per frame of the upstream controller, from the library functions it calls."""
  last, count, out = 0, 0, []
  for rate, lat, tq, eps_torque in zip(rates, lat_active, torques, eps, strict=True):
    torque = apply_meas_steer_torque_limits(int(round(tq * params.STEER_MAX)), last, eps_torque, params)
    count, req = common_fault_avoidance(abs(rate) >= MAX_STEER_RATE, lat, count, MAX_STEER_RATE_FRAMES)
    torque = torque if lat else 0
    last = torque
    out.append((int(req), torque))
  return out


def varied_inputs(n=1500):
  """Fast and slow wheel, lateral control dropping out, a varying torque request and an EPS report that lags it."""
  rates = [MAX_STEER_RATE + 50 if (i // 70) % 3 != 2 else MAX_STEER_RATE - 50 for i in range(n)]
  lat = [not (300 <= i % 400 < 330) for i in range(n)]
  torques = [math.sin(i / 15) for i in range(n)]
  eps = [1200. * math.sin((i - 8) / 15) for i in range(n)]
  return rates, lat, torques, eps


def panda_rejects(ci, frames, lat_active, eps=None):
  """Frames the panda's Toyota safety rejects, as indices; skipped frames only advance the clock.
  eps: the EPS-reported torque per frame in command units (default 0), given to the panda as its measured torque."""
  safety = libsafety_py.libsafety
  safety.set_current_safety_param_sp(ci.CP_SP.safetyParam)
  assert ci.CP.safetyConfigs[-1].safetyModel == CarParams.SafetyModel.toyota
  safety.set_safety_hooks(CarParams.SafetyModel.toyota, ci.CP.safetyConfigs[-1].safetyParam)
  safety.init_tests()
  eps = eps if eps is not None else [0.] * len(frames)
  rejected = []
  for i, (msg, lat, eps_torque) in enumerate(zip(frames, lat_active, eps, strict=True)):
    safety.set_timer(i * FRAME_US)
    safety.set_torque_meas(math.floor(eps_torque), math.ceil(eps_torque))
    if msg is None:
      continue
    safety.set_controls_allowed(lat)
    addr, dat, bus = msg
    if not safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)):
      rejected.append(i)
  return rejected


class TestSteerRateGap(unittest.TestCase):
  def setUp(self):
    self.ci = make_interface()

  def test_gap_follows_each_cut(self):
    frames = run(self.ci, [MAX_STEER_RATE + 50] * 400)
    cuts = [i for i, m in enumerate(frames) if m is not None and lka_fields(m)[0] == 0]
    gaps = [i for i, m in enumerate(frames) if m is None]
    self.assertGreater(len(cuts), 15)
    self.assertEqual(gaps, [c + 1 for c in cuts if c + 1 < len(frames)])
    # MAX_STEER_RATE_FRAMES request frames, the cut, the gap: one cut every MAX_STEER_RATE_FRAMES + 2 frames
    self.assertEqual({b - a for a, b in itertools.pairwise(cuts)}, {MAX_STEER_RATE_FRAMES + 2})
    for a, b in itertools.pairwise(cuts):
      sent = [m for m in frames[a + 1:b] if m is not None]
      self.assertEqual(len(sent), MAX_STEER_RATE_FRAMES)
      self.assertTrue(all(lka_fields(m)[0] == 1 for m in sent))

  def test_cut_keeps_torque_and_gap_holds_it(self):
    frames = run(self.ci, [MAX_STEER_RATE + 50] * 60)
    cut = next(i for i, m in enumerate(frames) if m is not None and lka_fields(m)[0] == 0)
    before, at, after = lka_fields(frames[cut - 1])[1], lka_fields(frames[cut])[1], lka_fields(frames[cut + 2])[1]
    self.assertNotEqual(at, 0)
    # the cut frame keeps ramping, the gap frame computes nothing, so the next frame ramps one step from the cut
    self.assertEqual(at - before, after - at)

  def test_no_gap_below_rate(self):
    frames = run(self.ci, [MAX_STEER_RATE - 1] * 200)
    self.assertTrue(all(m is not None and lka_fields(m)[0] == 1 for m in frames))

  def test_no_gap_when_lateral_ends_at_cut(self):
    rates = [MAX_STEER_RATE + 50] * 60
    frames = run(self.ci, rates)
    cut = next(i for i, m in enumerate(frames) if m is not None and lka_fields(m)[0] == 0)
    ci = make_interface()
    lat = [True] * (cut + 1) + [False] * (len(rates) - cut - 1)
    frames = run(ci, rates, lat)
    self.assertIsNotNone(frames[cut + 1])
    self.assertEqual(lka_fields(frames[cut + 1]), (0, 0))

  def test_panda_accepts_stream(self):
    for pattern in ([MAX_STEER_RATE + 50] * 600,
                    ([MAX_STEER_RATE + 50] * 45 + [0] * 7) * 12,
                    ([MAX_STEER_RATE + 50] * 19 + [0] * 1) * 30):
      with self.subTest(frames=len(pattern)):
        ci = make_interface()
        lat = [True] * len(pattern)
        frames = run(ci, pattern, lat)
        self.assertEqual(panda_rejects(ci, frames, lat), [])

  def test_upstream_stream_without_the_is_retrofit(self):
    # the Lexus IS with its stock EPS (5-byte or absent EPS_STATUS), and another platform with the retrofit detected:
    # every frame is sent, and its request bit and torque are the upstream controller's
    other = next(c for c in CAR if c != CAR.LEXUS_IS and make_interface(c).CP_SP.flags & ToyotaFlagsSP.TSS2_EPS)
    rates, lat, torques, eps = varied_inputs()
    for candidate, eps_length in ((CAR.LEXUS_IS, 5), (CAR.LEXUS_IS, None), (other, 8)):
      with self.subTest(candidate=candidate, eps_length=eps_length):
        ci = make_interface(candidate, eps_length)
        self.assertEqual(bool(ci.CP_SP.flags & ToyotaFlagsSP.TSS2_EPS), eps_length == 8)
        frames = run(ci, rates, lat, torques, eps)
        self.assertTrue(all(m is not None for m in frames))
        self.assertEqual([lka_fields(m) for m in frames], upstream_stream(ci.CC.params, rates, lat, torques, eps))
        self.assertGreater(sum(lka_fields(m)[0] == 0 and on for m, on in zip(frames, lat, strict=True)), 15)
        self.assertEqual(panda_rejects(ci, frames, lat, eps), [])

  def test_gap_frames_compute_nothing(self):
    # with the retrofit: a gap after every cut while lateral control stays on, and the frames sent are exactly the
    # upstream controller's on the same inputs with the gap frames left out
    rates, lat, torques, eps = varied_inputs()
    ci = make_interface()
    frames = run(ci, rates, lat, torques, eps)
    sent = [i for i, m in enumerate(frames) if m is not None]
    gaps = [i for i, m in enumerate(frames) if m is None]
    cuts = [i for i in sent if lat[i] and lka_fields(frames[i])[0] == 0]
    self.assertGreater(len(cuts), 15)
    self.assertEqual(gaps, [c + 1 for c in cuts if c + 1 < len(frames) and lat[c + 1]])
    pick = [rates, lat, torques, eps]
    expected = upstream_stream(ci.CC.params, *([x[i] for i in sent] for x in pick))
    self.assertEqual([lka_fields(frames[i]) for i in sent], expected)
    self.assertEqual(panda_rejects(ci, frames, lat, eps), [])

  def test_panda_harness_rejects_double_cut(self):
    # sensitivity check: the same stream with the gap filled by a second cut must be rejected
    ci = make_interface()
    pattern = [MAX_STEER_RATE + 50] * 200
    lat = [True] * len(pattern)
    frames = run(ci, pattern, lat)
    doubled = list(frames)
    first_gap = frames.index(None)
    doubled[first_gap] = frames[first_gap - 1]
    self.assertIn(first_gap, panda_rejects(make_interface(), doubled, lat))


if __name__ == "__main__":
  unittest.main()
