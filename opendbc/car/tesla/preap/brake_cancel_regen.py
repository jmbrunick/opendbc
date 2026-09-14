"""Tip vs hold brake-cancel: coast first, then stock regen.

When a short/light brake knocks software long off, dropping the
interceptor immediately (GAS_COMMAND enable=0) lets stock Tesla regen
bite. A tip keeps interceptor ENABLE and requests a=0 (coast) for
BRAKE_CANCEL_COAST_S, then RELEASE. A held or deeper brake RELEASEs
immediately so stock friction + regen can decelerate.

Does not rewrite interceptor DI. Uses the existing ENABLE path with
accel_request=0. FCW / AEB / hard lead braking stay on the planner
while long is still on — this only runs after a driver-brake silent
long pause (cruiseEnabled stays up). Full session cancel does not
keep coast. Gas-lift handoff is unchanged.
"""
from enum import IntEnum

import numpy as np

# Like stalk tip/hold (0.40 s). A brake tap springs back sooner; a
# held pedal is still Applied well past this.
BRAKE_TIP_HOLD_S = 0.30

# Ease to coast after a confirmed tip, then allow stock regen.
# Tunable in the 0.3–0.8 s band.
BRAKE_CANCEL_COAST_S = 0.50

# Deeper / panic: measured decel at or past the pre-AP planner clip.
# Comfort Accel-5 is −0.80; planner clip −1.5. At/under clip is
# "driver is braking for real" — do not hold coast.
BRAKE_CANCEL_FIRM_A_EGO = -1.5

# Don't hold coast at a stop / KF chatter.
BRAKE_CANCEL_MIN_SPEED = 1.0


class BrakeCancelMode(IntEnum):
  IDLE = 0
  PENDING = 1  # brake down after long drop; classifying tip vs hold
  COAST = 2    # tip confirmed; keep interceptor at a=0
  FIRM = 3     # held / deeper; stock regen immediately


def _is_firm_decel(a_ego: float) -> bool:
  return bool(np.isfinite(a_ego) and a_ego <= BRAKE_CANCEL_FIRM_A_EGO)


class BrakeCancelRegen:
  """Classify a brake long-pause as tip (coast) vs hold (firm)."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.mode = BrakeCancelMode.IDLE
    self.held_s = 0.0
    self.coast_s = 0.0
    self.prev_requested_long = False

  @property
  def keep_coast(self) -> bool:
    return self.mode in (BrakeCancelMode.PENDING, BrakeCancelMode.COAST)

  def update(self, *, requested_long: bool, cruise_enabled: bool,
             brake_pressed: bool, gas_pressed: bool,
             a_ego: float, v_ego: float, dt: float) -> bool:
    """Return True when interceptor should stay ENABLE at coast."""
    if (not cruise_enabled) or gas_pressed or float(v_ego) < BRAKE_CANCEL_MIN_SPEED:
      self.reset()
      self.prev_requested_long = bool(requested_long)
      return False

    if requested_long:
      self.mode = BrakeCancelMode.IDLE
      self.held_s = 0.0
      self.coast_s = 0.0
      self.prev_requested_long = True
      return False

    long_falling = self.prev_requested_long and not requested_long
    self.prev_requested_long = bool(requested_long)

    if long_falling and brake_pressed:
      self.mode = BrakeCancelMode.PENDING
      self.held_s = 0.0
      self.coast_s = 0.0

    if self.mode == BrakeCancelMode.IDLE:
      return False

    if _is_firm_decel(a_ego):
      self.mode = BrakeCancelMode.FIRM
      return False

    if self.mode == BrakeCancelMode.FIRM:
      if not brake_pressed:
        self.mode = BrakeCancelMode.IDLE
      return False

    if self.mode == BrakeCancelMode.PENDING:
      if brake_pressed:
        self.held_s += float(dt)
        if self.held_s + 1e-9 >= BRAKE_TIP_HOLD_S:
          self.mode = BrakeCancelMode.FIRM
          return False
        return True
      self.mode = BrakeCancelMode.COAST
      self.coast_s = 0.0

    if self.mode == BrakeCancelMode.COAST:
      if brake_pressed:
        self.mode = BrakeCancelMode.PENDING
        self.held_s = 0.0
        return True
      self.coast_s += float(dt)
      if self.coast_s + 1e-9 >= BRAKE_CANCEL_COAST_S:
        self.mode = BrakeCancelMode.IDLE
        return False
      return True

    return False
