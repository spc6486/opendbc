"""
Lexus IS engage_cancel_grace (opendbc/sunnypilot/car/toyota/engage_cancel.py): the real CarController, with the PCM's cruise
and CarControl driven frame by frame as in route 940 (cruise rising edge, one frame of cancel with openpilot not yet
enabled, then enabled). Checks the cancel bit in ACC_CONTROL (0x343) and the cluster chime (0x412 TWO_BEEPS).
"""
import unittest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.toyota.values import CAR
from opendbc.sunnypilot.car.toyota.engage_cancel import GRACE_FRAMES, EngageCancelGrace
from opendbc.sunnypilot.car.toyota.tests.test_lexus_is_settings import build
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

ACC_CONTROL = 0x343
LKAS_HUD = 0x412
PCM_CRUISE = 0x1D2


def drive(ci, frames):
  """frames: (pcm cruise on, CC.enabled, CC.cruiseControl.cancel) per control frame.
  Returns per frame (CANCEL_REQ sent on 0x343 or None if no 0x343 that frame, TWO_BEEPS sent on 0x412 or None)."""
  cp = CANParser("toyota_tnga_k_pt_generated", [("ACC_CONTROL", 0), ("LKAS_HUD", 0)], 0)
  out = []
  t = 0
  for cruise, enabled, cancel in frames:
    CC = structs.CarControl()
    CC.enabled = enabled
    CC.longActive = enabled
    CC.cruiseControl.cancel = cancel
    ci.update([])
    ci.CS.out.cruiseState.enabled = cruise
    _, sends = ci.apply(CC.as_reader(), structs.CarControlSP(), t)
    acc = lka = None
    for addr, dat, bus in sends:
      if bus != 0 or addr not in (ACC_CONTROL, LKAS_HUD):
        continue
      cp.update([(t, [(addr, dat, 0)])])
      if addr == ACC_CONTROL:
        acc = bool(cp.vl["ACC_CONTROL"]["CANCEL_REQ"])
      else:
        lka = bool(cp.vl["LKAS_HUD"]["TWO_BEEPS"])
    out.append((acc, lka))
    t += 10_000_000
  return out


def engage_with_stale_frame(offset):
  """offset frames with cruise off, then the rising edge: one frame of cancel=1/enabled=0 (the stale selfdriveState),
  then enabled. The offset moves the stale frame across the 0x343 send phase (every 3rd frame)."""
  return [(False, False, False)] * offset + [(True, False, True)] + [(True, True, False)] * 20


class TestEngageCancelGraceUnit(unittest.TestCase):
  def test_off_is_identity(self):
    g = EngageCancelGrace(False)
    for cancel, cruise, en in [(True, True, False), (True, False, False), (False, True, True)]:
      self.assertEqual(g.update(cancel, cruise, en), cancel)

  def test_stale_frame_suppressed(self):
    g = EngageCancelGrace(True)
    self.assertFalse(g.update(False, False, False))
    self.assertFalse(g.update(True, True, False))   # rising edge, stale frame
    self.assertFalse(g.update(False, True, True))

  def test_refused_engagement_cancels_after_grace(self):
    g = EngageCancelGrace(True)
    sent = [g.update(True, True, False) for _ in range(GRACE_FRAMES + 3)]
    self.assertEqual(sent, [False] * GRACE_FRAMES + [True] * 3)

  def test_late_cancel_untouched(self):
    g = EngageCancelGrace(True)
    for _ in range(GRACE_FRAMES + 1):
      g.update(False, True, True)
    self.assertTrue(g.update(True, True, False))  # openpilot disengaged with cruise on: immediate

  def test_withheld_cancel_logged_once_per_edge(self):
    g = EngageCancelGrace(True)
    with self.assertLogs("carlog", level="WARNING") as logs:
      g.update(True, True, False)
      g.update(True, True, False)
      g.update(False, False, False)
      g.update(True, True, False)
    self.assertEqual(len(logs.output), 2)
    self.assertIn("lexus_is_engage_cancel_withheld", logs.output[0])

  def test_cruise_off_resets(self):
    g = EngageCancelGrace(True)
    for _ in range(50):
      g.update(False, True, True)
    g.update(False, False, False)
    self.assertFalse(g.update(True, True, False))  # a new rising edge gets the grace again


class TestEngageCancelGraceCarController(unittest.TestCase):
  def _ci(self, settings=None):
    _, CP_SP, ci = build(settings=settings)
    return CP_SP, ci

  def test_flag_default_on_and_setting_clears_it(self):
    CP_SP, _ = self._ci()
    self.assertTrue(CP_SP.flags & ToyotaFlagsSP.ENGAGE_CANCEL_GRACE)
    CP_SP, _ = self._ci({"LexusIsEngageCancelGrace": 0})
    self.assertFalse(CP_SP.flags & ToyotaFlagsSP.ENGAGE_CANCEL_GRACE)

  def test_off_upstream_cancel_reaches_bus_on_send_phase(self):
    # setting off = upstream: across the three send phases, the stale frame reaches 0x343 at least once and chimes always
    hits = 0
    for offset in range(3):
      _, ci = self._ci({"LexusIsEngageCancelGrace": 0})
      out = drive(ci, engage_with_stale_frame(10 + offset))
      stale = out[10 + offset]
      hits += stale[0] is True
      self.assertTrue(stale[1], "upstream sends the 0x412 chime with every cancel frame")
    self.assertEqual(hits, 1)

  def test_on_no_cancel_no_chime_any_phase(self):
    for offset in range(3):
      _, ci = self._ci()
      out = drive(ci, engage_with_stale_frame(10 + offset))
      self.assertFalse(any(acc for acc, _ in out if acc is not None), offset)
      self.assertFalse(any(lka for _, lka in out if lka is not None), offset)

  def test_on_refused_engagement_still_cancels(self):
    _, ci = self._ci()
    frames = [(False, False, False)] * 10 + [(True, False, True)] * 15
    out = drive(ci, frames)
    sent = [i for i, (acc, _) in enumerate(out) if acc]
    self.assertTrue(sent)
    self.assertGreaterEqual(sent[0], 10 + GRACE_FRAMES)
    self.assertLessEqual(sent[0], 10 + GRACE_FRAMES + 2)  # next 0x343 send frame after the grace

  def test_on_disengage_with_cruise_on_cancels_at_once(self):
    _, ci = self._ci()
    frames = [(True, True, False)] * 30 + [(True, False, True)] * 6
    out = drive(ci, frames)
    first = next(i for i, (acc, _) in enumerate(out) if acc)
    self.assertLessEqual(first, 30 + 2)  # first 0x343 send frame, as upstream

  def _stock_long_cancels(self, settings, frames):
    """Stock longitudinal (or an IS without a C-SDSU): the cancel goes out on PCM_CRUISE (0x1D2) only, every frame."""
    _, _, ci = build(settings=settings)
    self.assertFalse(ci.CP.openpilotLongitudinalControl)
    out = []
    for i, (cruise, enabled, cancel) in enumerate(frames):
      CC = structs.CarControl()
      CC.enabled = enabled
      CC.cruiseControl.cancel = cancel
      ci.update([])
      ci.CS.out.cruiseState.enabled = cruise
      _, sends = ci.apply(CC.as_reader(), structs.CarControlSP(), i * 10_000_000)
      out.append(any(addr == PCM_CRUISE and bus == 0 for addr, _, bus in sends))
    return out

  def test_stock_long_cancel_path(self):
    stock = {"ToyotaEnforceStockLongitudinal": 1}
    stale = engage_with_stale_frame(10)
    self.assertTrue(self._stock_long_cancels({**stock, "LexusIsEngageCancelGrace": 0}, stale)[10])  # upstream
    self.assertFalse(any(self._stock_long_cancels(stock, stale)))
    refused = [(False, False, False)] * 10 + [(True, False, True)] * 10
    sent = self._stock_long_cancels(stock, refused)
    self.assertEqual(sent.index(True), 10 + GRACE_FRAMES)  # first frame after the grace

  def test_other_toyota_unaffected(self):
    _, CP_SP, ci = build(CAR.TOYOTA_RAV4, bsm_sensor=False)
    self.assertFalse(CP_SP.flags & ToyotaFlagsSP.ENGAGE_CANCEL_GRACE)
    self.assertFalse(ci.CC.engage_cancel_grace.enabled)


if __name__ == "__main__":
  unittest.main()
