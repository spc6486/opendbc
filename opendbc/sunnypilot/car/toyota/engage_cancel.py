"""
Lexus IS branch (setting engage_cancel_grace): no PCM cancel during the first frames after the PCM's cruise comes on.

controlsd computes cruiseControl.cancel = cruise enabled and not CC.enabled, with CC.enabled taken from the newest
selfdriveState it holds. When selfdrived lags by a frame, the first carState with the PCM's cruise on meets the
previous selfdriveState (still disabled), and controlsd asks for a cancel for exactly that frame, although selfdrived
enables openpilot on the same rising edge. On the Lexus IS the C-SDSU passes a single CANCEL_REQ to the PCM, which drops
cruise about 0.1 s later: route 940 had a one-frame cancel at all 9 engagements; the 5 that fell on a 0x343 send frame
lost the engagement, and every one sent the cluster's double beep (0x412 TWO_BEEPS).

The grace only delays a cancel that starts within GRACE_FRAMES of the cruise rising edge while openpilot is not yet
enabled. If openpilot is still not enabled after that (an engagement selfdrived refuses), the cancel goes out as upstream,
GRACE_FRAMES later. A cancel requested later than that, while cruise has been on for longer (disengagement, override,
cruiseMismatch), is not touched. Frames are carcontroller calls (card's apply loop, 100 Hz while carControl is alive), so
the grace is at most 50 ms; after an apply gap or a start with cruise already on, the count restarts at 0. A withheld
cancel is logged once per cruise rising edge (CarControl in the log still shows the cancel controlsd asked for).
"""

from opendbc.car.carlog import carlog

GRACE_FRAMES = 5  # carcontroller frames (100 Hz): 50 ms. Route 940: stale window 1 frame; selfdrived lag max 30 ms


class EngageCancelGrace:
  def __init__(self, enabled: bool):
    self.enabled = enabled
    self.cruise_on_frames = 0  # consecutive frames with the PCM's cruise on, counting the current one
    self.logged = False        # a withheld cancel has been logged since the last cruise rising edge

  def update(self, cancel: bool, cruise_enabled: bool, op_enabled: bool) -> bool:
    """The cancel to send this frame."""
    if not self.enabled:
      return cancel
    self.cruise_on_frames = self.cruise_on_frames + 1 if cruise_enabled else 0
    if not cruise_enabled:
      self.logged = False
    if cancel and cruise_enabled and not op_enabled and self.cruise_on_frames <= GRACE_FRAMES:
      if not self.logged:
        carlog.warning(f"lexus_is_engage_cancel_withheld: cruise on for {self.cruise_on_frames} frame(s), openpilot not enabled")
        self.logged = True
      return False
    return cancel
