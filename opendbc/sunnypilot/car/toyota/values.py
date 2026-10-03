"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class ToyotaFlagsSP(IntFlag):
  SMART_DSU = 1
  RADAR_CAN_FILTER = 2
  ZSS = 4
  STOCK_LONGITUDINAL = 8
  STOP_AND_GO_HACK = 16
  TSS2_EPS = 32
  ENHANCED_BSM = 64
  RSA = 128       # road-sign display from the navigation speed limit (rsa.py); set on UNSUPPORTED_DSU cars
  LDA_MADS = 256  # LDA button pauses/resumes MADS, cluster LKA indicator follows MADS (mads.py); set on UNSUPPORTED_DSU cars


# DBCs that define the stock 5-byte EPS_STATUS, mapped to the variant with the 8-byte EPS_STATUS a TSS2 power-steering
# ECU sends (checksum in the last byte). carstate parses with the variant when ToyotaFlagsSP.TSS2_EPS is detected.
TSS2_EPS_DBC = {
  "toyota_tnga_k_pt_generated": "toyota_tnga_k_tss2_eps_pt_generated",
}

# EPS_SCALE (STEER_TORQUE_EPS to command units, in %) for that TSS2 power-steering ECU, on the platforms where it was
# measured; others keep EPS_SCALE. interface.py writes it into the panda safety param and carstate.py scales
# steeringTorqueEps with it, so the controller and the panda use one value. On a 2017 Lexus IS the retrofit EPS reports
# about 0.80-0.81 x the command in steady hands-off steering, as the stock IS/RC EPS does; 124 converts that report to
# command units (1/0.81). Driven on routes 353-355 at 124 with no EPS fault. Keyed by platform name (CAR is a str enum).
TSS2_EPS_SCALE = {"LEXUS_IS": 124}


class ToyotaSafetyFlagsSP:
  DEFAULT = 0
  UNSUPPORTED_DSU = 1
  GAS_INTERCEPTOR = 2
  ENHANCED_BSM = 4
