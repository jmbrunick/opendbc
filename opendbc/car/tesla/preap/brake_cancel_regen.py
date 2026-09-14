"""Tip vs hold brake-cancel: ramp interceptor regen, then stock.

How regen strength is actually commanded (no GAS_COMMAND DI rewrite):

- ENABLE=1 + accel_request=0 → VDAS feedforward at zero-torque DI → coast.
- ENABLE=1 + accel_request<0 → more negative DI (toward PEDAL_DI_MIN=-5)
  → stronger interceptor regen. REGEN_MAX (-1.5 m/s²) is stock / planner
  full regen.
- ENABLE=0 + command=0 → RELEASE. Tesla reads the physical pedal (at rest
  after a tap) → stock regen in one step.
- aEgo is measured vehicle accel. It classifies tip vs hold
  (≤ BRAKE_CANCEL_FIRM_A_EGO is "already braking for real") and is not
  the regen command.

#153 held ENABLE at a=0 for 0.50 s then RELEASEd. That is a coast
plateau plus a sudden stock bite. A tip now keeps ENABLE and interpolates
accel_request from 0 → REGEN_MAX over BRAKE_CANCEL_RAMP_S, then RELEASEs
so the handoff is continuous. A held or deeper brake RELEASEs immediately.

FCW / AEB / hard lead stay on the planner while long is still on. Full
session cancel does not keep the interceptor. Gas-lift handoff is
unchanged.
"""
from enum import IntEnum

import numpy as np

from opendbc.car.tesla.preap.nap_conf import REGEN_MAX

# Like stalk tip/hold (0.40 s). A brake tap springs back sooner; a
# held pedal is still Applied well past this.
BRAKE_TIP_HOLD_S = 0.30

# Progressive interceptor regen after a confirmed tip, then stock.
# Tunable in the 0.5–1.0 s band.
BRAKE_CANCEL_RAMP_S = 0.75

# Stock Tesla / planner full regen. Ramp target so RELEASE is not a step.
BRAKE_CANCEL_STOCK_REGEN_A = float(REGEN_MAX)

# Deeper / panic: measured decel at or past the pre-AP planner clip.
# Comfort Accel-5 is −0.80; planner clip −1.5. At/under clip is
# "driver is braking for real" — do not ramp, RELEASE immediately.
BRAKE_CANCEL_FIRM_A_EGO = -1.5

# Don't hold the interceptor at a stop / KF chatter.
BRAKE_CANCEL_MIN_SPEED = 1.0


class BrakeCancelMode(IntEnum):
  IDLE = 0
  PENDING = 1  # brake down after long drop; classifying tip vs hold
  RAMP = 2     # tip confirmed; interpolate 0 → stock regen
  FIRM = 3     # held / deeper; stock regen immediately


def _is_firm_decel(a_ego: float) -> bool:
  return bool(np.isfinite(a_ego) and a_ego <= BRAKE_CANCEL_FIRM_A_EGO)


def interpolate_regen_accel(elapsed_s: float, ramp_s: float = BRAKE_CANCEL_RAMP_S,
                            stock_a: float = BRAKE_CANCEL_STOCK_REGEN_A) -> float:
  """Linear 0 → stock regen over ramp_s. elapsed_s is time on the ramp."""
  if ramp_s <= 0.0:
    return float(stock_a)
  progress = float(np.clip(elapsed_s / ramp_s, 0.0, 1.0))
  return float(stock_a) * progress


class BrakeCancelRegen:
  """Classify a brake long-pause as tip (regen ramp) vs hold (firm)."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.mode = BrakeCancelMode.IDLE
    self.held_s = 0.0
    self.ramp_s = 0.0
    self.prev_requested_long = False

  @property
  def keep_enabled(self) -> bool:
    return self.mode in (BrakeCancelMode.PENDING, BrakeCancelMode.RAMP)

  def commanded_accel(self) -> float:
    """Accel to request while interceptor stays ENABLE after a tip cancel.

    PENDING (brake still down): 0 (coast) so we do not stack interceptor
    regen on top of friction. RAMP: linear 0 → stock.
    """
    if self.mode != BrakeCancelMode.RAMP:
      return 0.0
    return interpolate_regen_accel(self.ramp_s)

  def accel_effort_limits(self) -> tuple[float, float]:
    """Pin VDAS effort to the commanded ramp (planner/FCW a cannot punch)."""
    a = float(np.clip(self.commanded_accel(), BRAKE_CANCEL_STOCK_REGEN_A, 0.0))
    return (a, a)

  def update(self, *, requested_long: bool, cruise_enabled: bool,
             brake_pressed: bool, gas_pressed: bool,
             a_ego: float, v_ego: float, dt: float) -> bool:
    """Return True when interceptor should stay ENABLE for the soft ramp."""
    if (not cruise_enabled) or gas_pressed or float(v_ego) < BRAKE_CANCEL_MIN_SPEED:
      self.reset()
      self.prev_requested_long = bool(requested_long)
      return False

    if requested_long:
      self.mode = BrakeCancelMode.IDLE
      self.held_s = 0.0
      self.ramp_s = 0.0
      self.prev_requested_long = True
      return False

    long_falling = self.prev_requested_long and not requested_long
    self.prev_requested_long = bool(requested_long)

    if long_falling and brake_pressed:
      self.mode = BrakeCancelMode.PENDING
      self.held_s = 0.0
      self.ramp_s = 0.0

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
      self.mode = BrakeCancelMode.RAMP
      self.ramp_s = 0.0

    if self.mode == BrakeCancelMode.RAMP:
      if brake_pressed:
        self.mode = BrakeCancelMode.PENDING
        self.held_s = 0.0
        return True
      self.ramp_s += float(dt)
      if self.ramp_s + 1e-9 >= BRAKE_CANCEL_RAMP_S:
        self.mode = BrakeCancelMode.IDLE
        return False
      return True

    return False
