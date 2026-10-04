"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, CanData, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

# Enhanced BSM: blind-spot status for the 2017-20 Lexus IS, whose blind spot monitor sensors do not broadcast it.
# Their 0x3F6 (BSM) carries only the two "enabled" flags: 80 80 00 00 00 00 00 00 at 1 Hz on a whole logged route with
# numerous detections on both sides. openpilot reads 0x3F6 only on TSS2 cars.
#
# The sensors answer diagnostic requests on 0x750 (sub-addressed; left 0x41, right 0x42), replies on 0x758. Each sensor
# is read with KWP2000 read data by local identifier 0x69 (41 02 21 69 ...) every 20 control frames, left and right 10
# frames apart, in the default diagnostic session, as Toyota's Techstream reads it (data list "Blind Spot Monitor
# Master"/"Slave"). No session is requested: the sensors keep their factory behaviour. (Session 0x60, used by
# dragonpilot's Enhanced BSM and by this branch before, made the mirror lamps light for almost any roadside object.)
#
# The positive reply is 4x 06 61 69 d4 d5 d6 d7. d4 holds the sensor's own mirror-lamp command (Techstream's decode,
# confirmed in the car): left (master) bit 7 lamp on, bit 6 lamp flashing; right (slave) bit 5 lamp on, bit 4 lamp
# flashing. A side is reported occupied exactly when its lamp is commanded on or flashing. d5/d6 are the lamp circuit's
# voltage and current; they are not used.
#
# Requests are sent only while driving: from 10 mph, above which the factory system reports vehicles (opendbc's notes
# on 0x3F6, from TSS2 cars), until the car slows below 8 mph; nothing in the first 2 s after the controls start. While
# driving, a side with no positive reply for 1 s (after the first second of polling) is reported occupied, so a sensor
# that stops answering is visible and blocks a lane change instead of silently reading clear. The panda allows the
# requests with ToyotaSafetyFlagsSP.ENHANCED_BSM (safety/modes/toyota.h).
#
# Enabled in interface.py for the Lexus IS when the left sensor, the one on the CAN bus, answered the tester-present
# query at startup (Ecu.cornerRadar at (0x750, 0x41), values.py FW_QUERY_CONFIG).

BSM_DIAG_MSG = "BSM_DIAG_RESPONSE"
BSM_REQUEST_ADDR = 0x750
BSM_LEFT = 0x41
BSM_RIGHT = 0x42
BSM_SENSORS = (BSM_LEFT, BSM_RIGHT)

BSM_POLL_REQUEST = b"\x02\x21\x69\x00\x00\x00\x00"      # read data by local identifier 0x69
BSM_POSITIVE_SID = 0x61
BSM_LOCAL_ID = 0x69
BSM_LAMP_MASK = {BSM_LEFT: 0xC0, BSM_RIGHT: 0x30}         # d4: lamp on | lamp flashing

BSM_START_FRAME = round(2.0 / DT_CTRL)
BSM_POLL_PERIOD = 20
BSM_POLL_PHASE = {BSM_LEFT: 0, BSM_RIGHT: BSM_POLL_PERIOD // 2}
BSM_TIMEOUT_FRAMES = round(1.0 / DT_CTRL)
BSM_OPEN_SPEED = 10 * CV.MPH_TO_MS
BSM_CLOSE_SPEED = 8 * CV.MPH_TO_MS


def bsm_request(sub_addr: int, request: bytes) -> CanData:
  return CanData(BSM_REQUEST_ADDR, bytes([sub_addr]) + request, 0)


def bsm_detected(sensor: int, data_4: int) -> bool:
  return bool(data_4 & BSM_LAMP_MASK[sensor])


def bsm_driving(driving: bool, v_ego: float) -> bool:
  """the speed gate, with hysteresis: on from BSM_OPEN_SPEED, off below BSM_CLOSE_SPEED"""
  return v_ego >= BSM_CLOSE_SPEED if driving else v_ego >= BSM_OPEN_SPEED


class BsmCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.enhanced_bsm = bool(CP_SP.flags & ToyotaFlagsSP.ENHANCED_BSM)
    self.bsm_frame = 0
    self.bsm_driving = False
    self.bsm_driving_frames = 0                                                      # frames since the gate opened
    self.bsm_detected = {s: False for s in BSM_SENSORS}
    self.bsm_reply_frame: dict[int, int | None] = {s: None for s in BSM_SENSORS}   # last positive reply

  def bsm_fresh(self, sensor: int) -> bool:
    reply = self.bsm_reply_frame[sensor]
    return reply is not None and self.bsm_frame - reply < BSM_TIMEOUT_FRAMES

  def update_bsm(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    if not self.enhanced_bsm:
      return

    self.bsm_frame += 1
    replies = can_parsers[Bus.pt].vl_all[BSM_DIAG_MSG]
    for sub_addr, pci, sid, local_id, data_4 in zip(replies["SUB_ADDRESS"], replies["PCI"], replies["SID"],
                                                    replies["LOCAL_ID"], replies["DATA_4"], strict=True):
      sensor = int(sub_addr)
      # positive single-frame reply to 0x21 0x69, long enough to carry d4
      if sensor in BSM_SENSORS and int(sid) == BSM_POSITIVE_SID and int(local_id) == BSM_LOCAL_ID and 3 <= int(pci) <= 7:
        self.bsm_detected[sensor] = bsm_detected(sensor, int(data_4))
        self.bsm_reply_frame[sensor] = self.bsm_frame

    self.bsm_driving = bsm_driving(self.bsm_driving, ret.vEgo)
    self.bsm_driving_frames = self.bsm_driving_frames + 1 if self.bsm_driving else 0
    # a reply below the gate can only come from another tester: nothing is reported there
    polled_long_enough = self.bsm_driving_frames > BSM_TIMEOUT_FRAMES + BSM_POLL_PERIOD
    for sensor in BSM_SENSORS:
      if not self.bsm_driving:
        occupied = False
      elif self.bsm_fresh(sensor):
        occupied = self.bsm_detected[sensor]
      else:
        occupied = polled_long_enough   # no answer: occupied, so a silent sensor is visible
      if sensor == BSM_LEFT:
        ret.leftBlindspot = occupied
      else:
        ret.rightBlindspot = occupied


class BsmCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.enhanced_bsm = bool(CP_SP.flags & ToyotaFlagsSP.ENHANCED_BSM)
    self.bsm_driving = False

  def create_bsm_msgs(self, CS, frame: int) -> list[CanData]:
    if not self.enhanced_bsm or frame < BSM_START_FRAME:
      return []

    self.bsm_driving = bsm_driving(self.bsm_driving, CS.out.vEgo)
    if not self.bsm_driving:
      return []
    return [bsm_request(sensor, BSM_POLL_REQUEST) for sensor in BSM_SENSORS
            if frame % BSM_POLL_PERIOD == BSM_POLL_PHASE[sensor]]
