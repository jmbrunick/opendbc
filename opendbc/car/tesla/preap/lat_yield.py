"""Pre-AP "lateral yielded" tracking (Python mirror of tesla_preap_latyield.h).

A steering yank / hands-on cancels OP only while OP is in FULL control of
lateral (A). While lateral is released to the driver (blinker turn,
lane-change handoff, driver handoff, roundabout takeover, low-speed
inhibit ...) a wheel input is a driver maneuver and must not cancel (B);
longitudinal stays engaged.

OP's own state is not the source of truth. What counts is what the car
actually received: ``DAS_steeringControlType == 1`` frames. Panda derives
the same fact from the 0x488 frames it lets through, so both sides agree
without a new CAN bit or a cereal field.

  full_control()  -> True: OP steered within RECENT_S and no re-arm grace
                     is running. A hands-on edge is a yank: cancel.
                  -> False: lateral yielded. A hands-on edge is B.

Re-arm grace: when the active stream resumes after a lapse (a yield; never
the first active frame of a session) the next GRACE_S also read as yielded
so a hand-grab during the 1 s take-back blend is B.

INFERENCE CONTRACT (read before touching any lateral handoff/transition):
  OP never tells panda or the card "lateral is yielded". Both INFER it from
  the 0x488 type-1 send history, using these assumptions about how OP hands
  lateral back and forth:
    * OP streams type-1 every 20 ms while it steers, and ONLY then (every
      release / yield / pause / inhibit ends in type 0, i.e. CC.latActive
      false, including the carcontroller's own hands-level-3 / B-edge block);
    * a yield lasts at least HANDS_OFF_CONFIRM_S (0.15 s hands-off) before the
      take-back blend starts, so a lapse > GAP_S is always a yield;
    * the take-back blend lasts BLEND_TIME_S (1.0 s) and sends type-1 the
      whole time at authority < 1 (partial lateral), so GRACE_S must cover it.
  ANY change to lateral transitions or handoff timing (driver_lateral_handoff
  thresholds/blend/confirm, blinker or lane-change/roundabout lateral pause,
  a new lateral release path, authority shaping, a new reason to send type 1
  while the driver has the wheel) MUST update this inference: RECENT_S /
  GAP_S / GRACE_S / ASSUMED_* here, tesla_preap_latyield.h, the panda and
  card tests, and the pins in selfdrive/controls/lib/tests/
  test_lat_yield_inference_guard.py. That guard fails when the handoff
  constants and these windows diverge; do not "fix" it by editing only the
  pin.

Ordering contract with panda (see tesla_preap_latyield.h): this side is
the MORE EAGER to cancel. Panda alone must never drop controls_allowed
while OP still believes it is engaged (controlsMismatch).

  RECENT_S 0.15 >= panda 0.10     (+50 ms: stamp-before-wire latency)
  GAP_S    0.14 >= panda 0.10     (a Python gap implies a panda gap)
  grace starts GRACE_DELAY_S after resume and ends at GRACE_S, panda
  grace is [resume, resume + 1.2 s] so ours is always inside it.
"""

import time

# OP counts as steering for this long after the last active 0x488 frame.
RECENT_S = 0.15
# A lapse in the active stream this long is a yield; resuming starts grace.
# Must stay below the shortest yield: HANDS_OFF_CONFIRM_S (0.15) + one 20 ms
# frame, with margin. Panda uses 0.10.
GAP_S = 0.14
# Blend is 1.0 s (DriverLateralHandoff.BLEND_TIME_S).
GRACE_S = 1.0
# Python stamps before the frame reaches panda; start grace late so ours
# never begins before panda's (a few ms of cancel-eager, never the reverse).
GRACE_DELAY_S = 0.05
# Active steering stays blocked after a B edge until hands < disengage level
# for this long. Panda clears at 0.15 s; we clear later (superset).
BLOCK_CLEAR_S = 0.20

# Handoff behavior this inference assumes. selfdrive/controls/lib/
# driver_lateral_handoff.py must still match (guard test in openpilot).
ASSUMED_BLEND_TIME_S = 1.0
ASSUMED_HANDS_OFF_CONFIRM_S = 0.15
ASSUMED_FRAME_PERIOD_S = 0.02  # DAS_steeringControl, 50 Hz

# Panda constants (tesla_preap_latyield.h) — checked by tests.
PANDA_RECENT_S = 0.100
PANDA_GAP_S = 0.100
PANDA_GRACE_S = 1.200
PANDA_BLOCK_CLEAR_S = 0.150
PANDA_ASSUMED_BLEND_S = 1.000

# steeringTorqueEps stash: hands level (0..3) + this fraction when lateral
# was yielded at that carState frame. Older logs / non-Pre-AP carry an
# integer, which decodes as full control (today's behavior).
STASH_YIELDED_FRAC = 0.25


def encode_hands_stash(hands: int, full_control: bool) -> float:
  hands = int(max(0, min(3, int(hands))))
  return float(hands) if full_control else hands + STASH_YIELDED_FRAC


def stash_full_control(value) -> bool:
  """False when the steeringTorqueEps stash says lateral was yielded."""
  try:
    v = float(value or 0.0)
  except (TypeError, ValueError):
    return True
  if not (0.0 <= v <= 3.0 + STASH_YIELDED_FRAC):
    return True  # a real EPS torque from another brand, not the stash
  frac = v - int(v)
  return not (abs(frac - STASH_YIELDED_FRAC) < 1e-3)


class LatYieldTracker:
  """Last-active-steering stamp, re-arm grace, and the B-edge steering block."""

  def __init__(self, clock=time.monotonic):
    self._clock = clock
    self.reset()

  def reset(self):
    self._last: float | None = None
    self._grace_from: float | None = None
    self._block = False
    self._clear_since: float | None = None

  # -- TX side (carcontroller, every DAS_steeringControl frame) ---------
  def note_steer_tx(self, active: bool, engaged: bool = True) -> None:
    """Call for each 0x488 frame built. ``active`` == DAS_steeringControlType 1."""
    if not engaged:
      self.reset()
      return
    if not active:
      return
    now = self._clock()
    if self._last is not None and (now - self._last) > GAP_S:
      self._grace_from = now  # resumed after a yield
    self._last = now

  # -- decision (carstate edge + stash) ---------------------------------
  def full_control(self) -> bool:
    if self._last is None:
      return False
    now = self._clock()
    if (now - self._last) > RECENT_S:
      return False
    if self._grace_from is not None:
      since = now - self._grace_from
      if GRACE_DELAY_S <= since < GRACE_S:
        return False
    return True

  # -- B-edge steering block (mirror of preap_lat_block) -----------------
  @property
  def blocked(self) -> bool:
    return self._block

  def block(self) -> None:
    """B edge: refuse active steering until the hands release."""
    self._block = True
    self._clear_since = None

  def update_block(self, steering_disengage: bool) -> None:
    if not self._block:
      return
    if steering_disengage:
      self._clear_since = None
      return
    now = self._clock()
    if self._clear_since is None:
      self._clear_since = now
    elif (now - self._clear_since) >= BLOCK_CLEAR_S:
      self._block = False
      self._clear_since = None
