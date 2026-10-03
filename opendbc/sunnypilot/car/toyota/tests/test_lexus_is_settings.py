import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import CarParams
from opendbc.car.toyota.values import CAR, Ecu
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, ToyotaSafetyFlagsSP

FEATURES = {"LexusIsRsa": ToyotaFlagsSP.RSA, "LexusIsLdaMads": ToyotaFlagsSP.LDA_MADS,
            "LexusIsEnhancedBsm": ToyotaFlagsSP.ENHANCED_BSM}


def bsm_sensor_fw() -> CarParams.CarFw:
  fw = CarParams.CarFw()
  fw.ecu = Ecu.cornerRadar
  fw.address = 0x750
  fw.subAddress = 0x41
  fw.fwVersion = b""
  fw.logging = True
  return fw


def build(candidate=CAR.LEXUS_IS, settings=None, bsm_sensor=True):
  """get_car's order: get_params, get_params_sp, setup_interfaces (the settings), then the car interface"""
  CarInterface = interfaces[candidate]
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x2FF] = 8  # C-SDSU: openpilot longitudinal
  car_fw = [bsm_sensor_fw()] if bsm_sensor else []
  CP = CarInterface.get_params(candidate, fingerprint, car_fw, alpha_long=True, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, car_fw, alpha_long=True, is_release_sp=False, docs=False)
  params_list = [{k: v} for k, v in (settings or {}).items()]
  setup_interfaces(CarInterface, CP, CP_SP, params_list)
  return CP, CP_SP, CarInterface(CP, CP_SP)


class TestLexusIsSettings(unittest.TestCase):
  def test_defaults_on(self):
    # no settings (or a sunnypilot without them): every feature as before
    CP, CP_SP, ci = build()
    for name, flag in FEATURES.items():
      self.assertTrue(CP_SP.flags & flag, name)
    self.assertTrue(CP_SP.safetyParam & ToyotaSafetyFlagsSP.ENHANCED_BSM)
    self.assertTrue(CP.enableBsm)
    self.assertTrue(ci.CC.rsa_tx and ci.CS.rsa_nav)
    self.assertTrue(ci.CS.lkas_status_mads)

  def test_explicit_on_equals_default(self):
    on = build(settings={k: True for k in FEATURES})
    default = build()
    self.assertEqual(on[1].flags, default[1].flags)
    self.assertEqual(on[1].safetyParam, default[1].safetyParam)

  def test_each_off_alone(self):
    for name, flag in FEATURES.items():
      with self.subTest(name):
        CP, CP_SP, _ = build(settings={name: 0})
        self.assertFalse(CP_SP.flags & flag)
        for other, other_flag in FEATURES.items():
          if other != name:
            self.assertTrue(CP_SP.flags & other_flag, other)

  def test_rsa_off(self):
    _, _, ci = build(settings={"LexusIsRsa": 0})
    self.assertFalse(ci.CC.rsa_tx)
    self.assertFalse(ci.CS.rsa_nav)

  def test_lda_mads_off(self):
    _, _, ci = build(settings={"LexusIsLdaMads": False})
    self.assertFalse(ci.CS.lkas_status_mads)  # no LDA button events from the camera's LKAS_STATUS

  def test_enhanced_bsm_off_clears_safety_bit(self):
    CP, CP_SP, ci = build(settings={"LexusIsEnhancedBsm": 0})
    self.assertFalse(CP_SP.safetyParam & ToyotaSafetyFlagsSP.ENHANCED_BSM)  # the panda allows no 0x750 request
    self.assertTrue(CP_SP.safetyParam & ToyotaSafetyFlagsSP.UNSUPPORTED_DSU)
    self.assertFalse(CP.enableBsm)
    self.assertFalse(ci.CC.enhanced_bsm)
    self.assertEqual(ci.CC.create_bsm_msgs(ci.CS, 1000), [])

  def test_bsm_off_without_sensor_is_a_no_op(self):
    CP, CP_SP, _ = build(settings={"LexusIsEnhancedBsm": 0}, bsm_sensor=False)
    ref_CP, ref_CP_SP, _ = build(bsm_sensor=False)
    self.assertEqual(CP_SP.flags, ref_CP_SP.flags)
    self.assertEqual(CP_SP.safetyParam, ref_CP_SP.safetyParam)
    self.assertEqual(CP.enableBsm, ref_CP.enableBsm)

  def test_other_toyota_unaffected(self):
    for settings in ({}, {k: 0 for k in FEATURES}):
      CP, CP_SP, _ = build(CAR.TOYOTA_RAV4, settings=settings, bsm_sensor=False)
      for name, flag in FEATURES.items():
        self.assertFalse(CP_SP.flags & flag, name)


if __name__ == "__main__":
  unittest.main()
