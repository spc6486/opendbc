import unittest

from opendbc.can import CANPacker
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.values import CAR
from opendbc.sunnypilot.car.toyota.tests.test_lexus_is_settings import build
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

GEAR_PACKET = 0x3BC
NS = 1_000_000_000


def gear_stream(burst_frames=16, burst_dt=0.08, gaps=10, gap_dt=1.024):
  """GEAR_PACKET as the Lexus IS transmission sends it: bursts (frames 41-130 ms apart) while a value changes, otherwise one frame about
  every 1.02-1.04 s (routes 937/940). A drive that starts in a burst lets the parser learn > 10 Hz (route 937)."""
  t = [i * burst_dt for i in range(burst_frames)]
  for _ in range(gaps):
    t.append(t[-1] + gap_dt)
  return t


class TestGearPacketRate(unittest.TestCase):
  def test_flag_default_on_and_setting_clears_it(self):
    _, CP_SP, _ = build()
    self.assertTrue(CP_SP.flags & ToyotaFlagsSP.GEAR_PACKET_1HZ)
    _, CP_SP, _ = build(settings={"LexusIsGearPacketRate": 0})
    self.assertFalse(CP_SP.flags & ToyotaFlagsSP.GEAR_PACKET_1HZ)

  def test_declared_at_1hz(self):
    CP, CP_SP, _ = build()
    cp = list(CarState.get_can_parsers(CP, CP_SP).values())[0]
    st = cp.message_states[GEAR_PACKET]
    self.assertEqual(st.frequency, 1)
    self.assertEqual(st.timeout_threshold, 10 * NS)

  def test_undeclared_when_off(self):
    CP, CP_SP, _ = build(settings={"LexusIsGearPacketRate": 0})
    cp = list(CarState.get_can_parsers(CP, CP_SP).values())[0]
    self.assertNotIn(GEAR_PACKET, cp.message_states)  # upstream: added on first read, rate learned

  def test_burst_start_off_times_out(self):
    # setting off = upstream: the rate is learned from the burst and the 1.024 s gaps time out (route 937)
    CP, CP_SP, _ = build(settings={"LexusIsGearPacketRate": 0})
    cp, invalid = self._run(CP, CP_SP)
    self.assertGreater(cp.message_states[GEAR_PACKET].frequency, 9.8)
    self.assertGreater(invalid, 0)

  def test_burst_start_on_stays_valid(self):
    CP, CP_SP, _ = build()
    cp, invalid = self._run(CP, CP_SP)
    self.assertEqual(cp.message_states[GEAR_PACKET].frequency, 1)
    self.assertEqual(invalid, 0)

  def test_on_still_times_out_when_the_message_stops(self):
    CP, CP_SP, _ = build()
    cp = list(CarState.get_can_parsers(CP, CP_SP).values())[0]
    dat = CANPacker("toyota_tnga_k_pt_generated").make_can_msg("GEAR_PACKET", 0, {"GEAR": 0})[1]
    cp.update([(0, [(GEAR_PACKET, dat, 0)])])
    st = cp.message_states[GEAR_PACKET]
    self.assertTrue(st.valid(int(9.9 * NS), False))
    self.assertFalse(st.valid(int(10.1 * NS), False))

  def test_all_unsupported_dsu_platforms_declare_it(self):
    # the flag is set on every UNSUPPORTED_DSU platform (Lexus IS, RC, GS F); each DBC has GEAR_PACKET
    for car in (CAR.LEXUS_IS, CAR.LEXUS_RC, CAR.LEXUS_GS_F):
      with self.subTest(car):
        CP, CP_SP, _ = build(car)
        cp = list(CarState.get_can_parsers(CP, CP_SP).values())[0]
        self.assertEqual(cp.message_states[GEAR_PACKET].frequency, 1)

  def test_other_toyota_unaffected(self):
    CP, CP_SP, _ = build(CAR.TOYOTA_RAV4, bsm_sensor=False)
    self.assertFalse(CP_SP.flags & ToyotaFlagsSP.GEAR_PACKET_1HZ)
    cp = list(CarState.get_can_parsers(CP, CP_SP).values())[0]
    self.assertNotIn(GEAR_PACKET, cp.message_states)

  def _run(self, CP, CP_SP):
    cp = list(CarState.get_can_parsers(CP, CP_SP).values())[0]
    dat = CANPacker("toyota_tnga_k_pt_generated").make_can_msg("GEAR_PACKET", 0, {"GEAR": 0})[1]
    times = gear_stream()
    invalid = 0
    i = 0
    now = 0.0
    while now <= times[-1] + 0.5:
      batch = []
      while i < len(times) and times[i] <= now + 1e-9:
        batch.append((GEAR_PACKET, dat, 0))
        i += 1
      cp.update([(int(now * NS), batch)])
      _ = cp.vl["GEAR_PACKET"]["GEAR"]  # carstate reads it every frame (adds it on first use when undeclared)
      st = cp.message_states[GEAR_PACKET]
      if st.frequency > 0 and now < times[-1] and not st.valid(int(now * NS), False):
        invalid += 1
      now = round(now + 0.01, 6)  # card runs at 100 Hz
    return cp, invalid


if __name__ == "__main__":
  unittest.main()
