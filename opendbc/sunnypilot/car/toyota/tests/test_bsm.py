import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, CanData, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import CarParams
from opendbc.car.toyota.values import CAR, DBC, FW_QUERY_CONFIG, Ecu
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.toyota.bsm import BSM_CLOSE_SPEED, BSM_DIAG_MSG, BSM_LEFT, BSM_OPEN_SPEED, BSM_POLL_PERIOD, \
                                             BSM_RIGHT, BSM_START_FRAME, BSM_TIMEOUT_FRAMES, BsmCarController, \
                                             BsmCarState, bsm_detected
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, ToyotaSafetyFlagsSP, TSS2_EPS_DBC

PT_DBC = DBC[CAR.LEXUS_IS][Bus.pt]
REPLY_ADDR = 0x758

# the only requests sent: read local identifier 0x69, default session (the panda allows these and four more)
POLL = {BSM_LEFT: bytes.fromhex("4102216900000000"), BSM_RIGHT: bytes.fromhex("4202216900000000")}
SESSION = {BSM_LEFT: bytes.fromhex("4102106000000000"), BSM_RIGHT: bytes.fromhex("4202106000000000")}
CLOSE = {BSM_LEFT: bytes.fromhex("4102100100000000"), BSM_RIGHT: bytes.fromhex("4202100100000000")}
DRIVING = 30 * CV.MPH_TO_MS
# d4 values logged on route 365 (left master: 0x04 off, 0x84 lamp on; right slave: 0x00 off, 0x20 lamp on, 0x10 flashing)
L_OFF, L_ON, L_FLASH = 0x04, 0x84, 0x44
R_OFF, R_ON, R_FLASH = 0x00, 0x20, 0x10


def reply(sensor: int, data_4: int, data_5: int = 0, data_6: int = 0, pci: int = 6, sid: int = 0x61,
          local_id: int = 0x69) -> CanData:
  return CanData(REPLY_ADDR, bytes([sensor, pci, sid, local_id, data_4, data_5, data_6, 0]), 0)


def bsm_cp(enhanced=True):
  CP = structs.CarParams()
  CP_SP = structs.CarParamsSP()
  if enhanced:
    CP_SP.flags = int(ToyotaFlagsSP.ENHANCED_BSM)
  return CP, CP_SP


def cornerRadar_fw(sub_addr: int) -> CarParams.CarFw:
  fw = CarParams.CarFw()
  fw.ecu = Ecu.cornerRadar
  fw.address = 0x750
  fw.subAddress = sub_addr
  fw.fwVersion = b""
  fw.logging = True
  return fw


class BsmStateHarness:
  """BsmCarState with a real parser on the Lexus IS DBC"""
  def __init__(self, dbc=PT_DBC):
    self.cs = BsmCarState(*bsm_cp())
    self.cp = CANParser(dbc, [(BSM_DIAG_MSG, float('nan'))], 0)
    self.nanos = 0

  def step(self, frames=(), v_ego=DRIVING) -> structs.CarState:
    self.nanos += round(DT_CTRL * 1e9)
    self.cp.update([(self.nanos, list(frames))])
    ret = structs.CarState(vEgo=v_ego)
    self.cs.update_bsm(ret, {Bus.pt: self.cp})
    return ret


class TestBsmState(unittest.TestCase):
  def test_rule(self):
    # the lamp command bits only; the master's main-switch bit (0x04) and the lamp voltage/current bytes do not count
    self.assertFalse(bsm_detected(BSM_LEFT, L_OFF))
    self.assertTrue(bsm_detected(BSM_LEFT, L_ON))
    self.assertTrue(bsm_detected(BSM_LEFT, L_FLASH))
    self.assertFalse(bsm_detected(BSM_LEFT, R_ON | R_FLASH))   # the slave's bit positions mean nothing on the master
    self.assertFalse(bsm_detected(BSM_RIGHT, R_OFF))
    self.assertTrue(bsm_detected(BSM_RIGHT, R_ON))
    self.assertTrue(bsm_detected(BSM_RIGHT, R_FLASH))
    self.assertFalse(bsm_detected(BSM_RIGHT, 0xC0 | 0x08 | 0x04))

  def test_both_sides(self):
    h = BsmStateHarness()
    ret = h.step([reply(BSM_LEFT, L_ON, 32, 13), reply(BSM_RIGHT, R_OFF)])
    self.assertEqual((ret.leftBlindspot, ret.rightBlindspot), (True, False))
    ret = h.step([reply(BSM_RIGHT, R_FLASH)])
    self.assertEqual((ret.leftBlindspot, ret.rightBlindspot), (True, True))
    ret = h.step([reply(BSM_LEFT, L_OFF, 32, 13)])   # lamp circuit still decaying: not occupied
    self.assertEqual((ret.leftBlindspot, ret.rightBlindspot), (False, True))

  def test_ignored_frames(self):
    for frame in (reply(BSM_LEFT, L_ON, sid=0x7F),         # negative response
                  reply(BSM_LEFT, L_ON, sid=0x50),         # a session reply
                  reply(BSM_LEFT, L_ON, local_id=0x68),    # another local identifier
                  reply(0x0F, L_ON),                        # another ECU (the radar's sub-address)
                  reply(BSM_LEFT, L_ON, pci=0x10),         # first frame of a multi-frame reply
                  reply(BSM_LEFT, L_ON, pci=2)):            # too short to carry d4
      h = BsmStateHarness()
      ret = h.step([frame])
      self.assertFalse(ret.leftBlindspot or ret.rightBlindspot, frame.dat.hex())
      self.assertIsNone(h.cs.bsm_reply_frame[BSM_LEFT])

  def test_short_master_reply(self):
    # the master's reply as Techstream read it parked in the default session: 61 69 04 01 00 00
    h = BsmStateHarness()
    h.step([CanData(REPLY_ADDR, bytes.fromhex("410661690401000000")[:8], 0)])
    self.assertIsNotNone(h.cs.bsm_reply_frame[BSM_LEFT])
    self.assertFalse(h.cs.bsm_detected[BSM_LEFT])

  def test_silent_sensor_reads_occupied(self):
    h = BsmStateHarness()
    for _ in range(BSM_TIMEOUT_FRAMES + BSM_POLL_PERIOD):
      ret = h.step([reply(BSM_RIGHT, R_OFF)])
      self.assertFalse(ret.leftBlindspot)                # within the first second of polling: not yet
    ret = h.step([reply(BSM_RIGHT, R_OFF)])
    self.assertEqual((ret.leftBlindspot, ret.rightBlindspot), (True, False))
    self.assertFalse(h.step([reply(BSM_LEFT, L_OFF)]).leftBlindspot)   # answers again: clear

  def test_timeout_while_detected(self):
    h = BsmStateHarness()
    for _ in range(3 * BSM_TIMEOUT_FRAMES):
      h.step([reply(BSM_LEFT, L_OFF)])
    self.assertTrue(h.step([reply(BSM_LEFT, L_ON)]).leftBlindspot)
    for _ in range(BSM_TIMEOUT_FRAMES - 1):
      self.assertTrue(h.step().leftBlindspot)
    for _ in range(5 * BSM_TIMEOUT_FRAMES):
      self.assertTrue(h.step().leftBlindspot)          # still silent: occupied, not clear
    self.assertFalse(h.step([reply(BSM_LEFT, L_OFF)]).leftBlindspot)

  def test_speed(self):
    h = BsmStateHarness()
    self.assertFalse(h.step([reply(BSM_LEFT, L_ON)], v_ego=BSM_OPEN_SPEED - 0.01).leftBlindspot)   # gate not open
    self.assertTrue(h.step(v_ego=BSM_OPEN_SPEED + 0.01).leftBlindspot)
    self.assertTrue(h.step(v_ego=BSM_CLOSE_SPEED + 0.01).leftBlindspot)                              # hysteresis
    self.assertFalse(h.step(v_ego=BSM_CLOSE_SPEED - 0.01).leftBlindspot)
    # below the gate nothing is reported, silent or not
    for _ in range(5 * BSM_TIMEOUT_FRAMES):
      ret = h.step(v_ego=0.)
      self.assertFalse(ret.leftBlindspot or ret.rightBlindspot)

  def test_not_enhanced(self):
    cs = BsmCarState(*bsm_cp(enhanced=False))
    ret = structs.CarState()
    cs.update_bsm(ret, {})   # the parser has no BSM_DIAG_RESPONSE without the flag; it is never read
    self.assertFalse(ret.leftBlindspot or ret.rightBlindspot)

  def test_both_eps_dbcs(self):
    for dbc in (PT_DBC, TSS2_EPS_DBC[PT_DBC]):
      h = BsmStateHarness(dbc)
      self.assertTrue(h.step([reply(BSM_RIGHT, R_ON)]).rightBlindspot, dbc)


class TestBsmController(unittest.TestCase):
  def run_frames(self, frames, speed=lambda frame: DRIVING):
    cc = BsmCarController(*bsm_cp())
    sends = {}
    for frame in range(frames):
      CS = SimpleNamespace(out=SimpleNamespace(vEgo=speed(frame)))
      msgs = cc.create_bsm_msgs(CS, frame)
      if msgs:
        sends[frame] = [(m.address, m.dat, m.src) for m in msgs]
    return sends

  def test_schedule(self):
    sends = self.run_frames(BSM_START_FRAME + 5 * BSM_POLL_PERIOD)
    self.assertNotIn(BSM_START_FRAME - 1, sends)
    for k in range(5):
      self.assertEqual(sends[BSM_START_FRAME + k * BSM_POLL_PERIOD], [(0x750, POLL[BSM_LEFT], 0)])
      self.assertEqual(sends[BSM_START_FRAME + k * BSM_POLL_PERIOD + 10], [(0x750, POLL[BSM_RIGHT], 0)])
    self.assertEqual(len(sends), 10)   # one request per sensor per poll period, no session requests

  def test_speed_gate(self):
    # parked, then 10 mph from 4 s, slowing through the hysteresis band from 8 s, below 8 mph from 12 s, 10 mph from 16 s
    def speed(frame):
      t = frame * DT_CTRL
      if t < 4:
        return 0.
      if t < 8:
        return BSM_OPEN_SPEED
      if t < 12:
        return (BSM_OPEN_SPEED + BSM_CLOSE_SPEED) / 2
      if t < 16:
        return BSM_CLOSE_SPEED - 0.01
      return BSM_OPEN_SPEED

    sends = self.run_frames(round(20 / DT_CTRL), speed=speed)
    self.assertTrue(all(dat in POLL.values() for msgs in sends.values() for _, dat, _ in msgs))
    polls = sorted(sends)
    self.assertTrue(all(400 <= f < 1200 or f >= 1600 for f in polls))
    self.assertEqual(len([f for f in polls if 400 <= f < 1200]), 2 * (800 // BSM_POLL_PERIOD))

  def test_never_fast_enough(self):
    self.assertEqual(self.run_frames(round(30 / DT_CTRL), speed=lambda frame: BSM_OPEN_SPEED - 0.01), {})

  def test_not_enhanced(self):
    cc = BsmCarController(*bsm_cp(enhanced=False))
    self.assertFalse(any(cc.create_bsm_msgs(None, f) for f in range(3 * BSM_START_FRAME)))

  def test_panda_allows_these(self):
    # every request passes the panda with the flags interface.py sets, none without them; the session requests the
    # panda still allows are never sent
    def speed(frame):
      return DRIVING if (frame // 1000) % 2 == 0 else 0.

    requests = {dat for msgs in self.run_frames(round(40 / DT_CTRL), speed=speed).values() for _, dat, _ in msgs}
    self.assertEqual(requests, set(POLL.values()))
    safety = libsafety_py.libsafety
    for sp, allowed in ((ToyotaSafetyFlagsSP.UNSUPPORTED_DSU | ToyotaSafetyFlagsSP.ENHANCED_BSM, True),
                        (ToyotaSafetyFlagsSP.UNSUPPORTED_DSU, False)):
      for stock_long in (False, True):
        safety.set_current_safety_param_sp(sp)
        param = 77 | (2 << 8 if stock_long else 0)   # Lexus IS EPS factor, TOYOTA_PARAM_STOCK_LONGITUDINAL
        safety.set_safety_hooks(CarParams.SafetyModel.toyota, param)
        safety.init_tests()
        for dat in requests | set(SESSION.values()) | set(CLOSE.values()):
          self.assertEqual(allowed, safety.safety_tx_hook(libsafety_py.make_CANPacket(0x750, 0, dat)), (sp, stock_long, dat.hex()))


class TestBsmInterface(unittest.TestCase):
  """Detection at startup and the path through the real Lexus IS CarInterface"""
  def interface(self, car_fw, candidate=CAR.LEXUS_IS, smart_dsu=True, eps_len=None):
    fingerprint = {0: {0x2FF: 8, 0x3F6: 8} if smart_dsu else {0x3F6: 8}, 1: {}, 2: {}}
    if eps_len is not None:
      fingerprint[0][0x262] = eps_len
    CarInterface = interfaces[candidate]
    CP = CarInterface.get_params(candidate, fingerprint, car_fw, alpha_long=smart_dsu, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, car_fw, alpha_long=smart_dsu, is_release_sp=False, docs=False)
    return CarInterface(CP, CP_SP)

  def test_fw_query(self):
    requests = [r for r in FW_QUERY_CONFIG.requests if Ecu.cornerRadar in r.whitelist_ecus]
    self.assertEqual(len(requests), 1)
    self.assertTrue(requests[0].logging)
    self.assertEqual(requests[0].request, [b"\x3e"])
    self.assertEqual(requests[0].bus, 0)
    self.assertEqual({(e[1], e[2]) for e in FW_QUERY_CONFIG.extra_ecus if e[0] == Ecu.cornerRadar}, {(0x750, 0x41)})

  def test_detection(self):
    for candidate, car_fw, expected in ((CAR.LEXUS_IS, [], False),
                                        (CAR.LEXUS_IS, [cornerRadar_fw(0x41)], True),
                                        (CAR.LEXUS_IS, [cornerRadar_fw(0x42)], False),     # only the left sensor is queried
                                        (CAR.LEXUS_IS, [cornerRadar_fw(0x43)], False),
                                        (CAR.LEXUS_RC, [cornerRadar_fw(0x41)], False),      # sub-addresses not known there
                                        (CAR.LEXUS_IS_TSS2, [cornerRadar_fw(0x41)], False)):
      for smart_dsu in (True, False):
        with self.subTest(candidate=candidate, fw=[hex(f.subAddress) for f in car_fw], smart_dsu=smart_dsu):
          ci = self.interface(car_fw, candidate, smart_dsu)
          self.assertEqual(bool(ci.CP_SP.flags & ToyotaFlagsSP.ENHANCED_BSM), expected)
          self.assertEqual(bool(ci.CP_SP.safetyParam & ToyotaSafetyFlagsSP.ENHANCED_BSM), expected)
          if candidate == CAR.LEXUS_IS:
            self.assertEqual(ci.CP.enableBsm, expected)
          self.assertEqual(REPLY_ADDR in ci.can_parsers[Bus.pt].addresses, expected)

  def run_frames(self, ci, frames, left=L_OFF, right=R_OFF, speed_kph=50.):
    CC = structs.CarControl().as_reader()
    CC_SP = structs.CarControlSP()
    packer = CANPacker(ci.can_parsers[Bus.pt].dbc_name)
    wheels = {f"WHEEL_SPEED_{w}": speed_kph for w in ("FL", "FR", "RL", "RR")}
    sends, states = [], []
    for i in range(frames):
      nanos = round((i + 1) * DT_CTRL * 1e9)
      cans = [CanData(*packer.make_can_msg("WHEEL_SPEEDS", 0, wheels))]
      if i >= BSM_START_FRAME and i % BSM_POLL_PERIOD == 1:
        cans.append(reply(BSM_LEFT, left))
      if right is not None and i >= BSM_START_FRAME and i % BSM_POLL_PERIOD == BSM_POLL_PERIOD // 2 + 1:
        cans.append(reply(BSM_RIGHT, right))
      states.append(ci.update([(nanos, cans)])[0])
      sends.append([CanData(*m) for m in ci.apply(CC, CC_SP, nanos)[1]])
    return sends, states

  def test_through_interface(self):
    for smart_dsu in (True, False):          # openpilot longitudinal with the C-SDSU, and stock longitudinal
      for eps_len in (None, 5, 8):
        with self.subTest(smart_dsu=smart_dsu, eps_len=eps_len):
          ci = self.interface([cornerRadar_fw(0x41)], smart_dsu=smart_dsu, eps_len=eps_len)
          self.assertEqual(ci.CP.openpilotLongitudinalControl, smart_dsu)
          self.assertTrue(ci.can_parsers[Bus.pt].message_states[REPLY_ADDR].ignore_alive)   # no CAN error without replies
          sends, states = self.run_frames(ci, BSM_START_FRAME + 60, left=L_ON)
          self.assertGreater(states[-1].vEgo, BSM_OPEN_SPEED)
          bsm = [(i, m.dat) for i, s in enumerate(sends) for m in s if m.address == 0x750]
          self.assertEqual(bsm[:3], [(BSM_START_FRAME, POLL[BSM_LEFT]), (BSM_START_FRAME + 10, POLL[BSM_RIGHT]),
                                     (BSM_START_FRAME + 20, POLL[BSM_LEFT])])
          self.assertTrue(states[-1].leftBlindspot)
          self.assertFalse(states[-1].rightBlindspot)
          # a right sensor that never answers reads occupied once polling has run for more than a second
          sends, states = self.run_frames(ci, BSM_START_FRAME + 2 * BSM_TIMEOUT_FRAMES, left=L_OFF, right=None)
          self.assertFalse(states[-1].leftBlindspot)
          self.assertTrue(states[-1].rightBlindspot)
          self.assertTrue(all(m.src == 0 for s in sends for m in s if m.address == 0x750))

  def test_parked(self):
    ci = self.interface([cornerRadar_fw(0x41)])
    sends, states = self.run_frames(ci, BSM_START_FRAME + 200, left=L_ON, speed_kph=0.)
    self.assertFalse(any(m.address == 0x750 for s in sends for m in s))
    self.assertFalse(any(s.leftBlindspot for s in states))   # below the closing speed a reply (another tester's) is not reported

  def test_without_sensor(self):
    ci = self.interface([])
    sends, states = self.run_frames(ci, BSM_START_FRAME + 60, left=L_ON)
    self.assertFalse(any(m.address == 0x750 for s in sends for m in s))
    self.assertFalse(any(s.leftBlindspot for s in states))


if __name__ == "__main__":
  unittest.main()
