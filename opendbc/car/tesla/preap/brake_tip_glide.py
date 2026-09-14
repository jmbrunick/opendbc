"""Tip vs hold brake-cancel: comfort-shaped regen ramp, then stock.

Product (digital Applied only — no travel/pressure):

1. **No hard bite at the start.** Do not dump full regen or RELEASE-to-
   stock on frame 1 of a tip cancel.
2. **As soon as the tip cancel starts**, command regen that is
   noticeable but gentle (`BRAKE_GLIDE_A_START`, low initial jerk).
3. **The ramp is the product.** Quadratic ease-in over ~2.5 s (tunable
   2–3): stronger later, fairly hard / full interceptor regen at the
   end. Not a coast plateau. Not the reverted 0.75 s 0→REGEN_MAX fade.
   Not a step to constant decel.
4. The 10–15 mph band **guides** `a_end` when that Δv is reachable
   within interceptor authority. If not, the same gentle→strong shape
   still runs to `REGEN_MAX`, then hands off. Do not invent friction.
5. **Held** Applied past the tip window, or **hard** aEgo: RELEASE
   immediately — Tesla friction + stock regen.

Handoff: when ego reaches ~12 mph, or the 2.5 s window expires,
RELEASE to stock. Already in the 10–15 mph band: no ramp, RELEASE
after the tip is classified (or immediately on hold/firm).

How commands map (no GAS_COMMAND DI rewrite):

- ENABLE=1 + accel_request=0 → zero-torque DI → coast.
- ENABLE=1 + accel_request<0 → more negative DI → interceptor regen
  only, clipped at REGEN_MAX.
- ENABLE=0 + command=0 → RELEASE → Tesla physical pedal / stock regen.
- aEgo classifies tip vs hold only; it is not the ramp command.

PENDING (brake still down, classifying): if a glide is armed, command
the start of the same comfort ramp (gentle). Firm aEgo short-circuits
to RELEASE on the same frame. Do not RELEASE a light tip (that would
be the stock bite).

FCW / AEB / hard lead stay on the planner while long is still on.
Full session cancel does not keep the interceptor. Gas-lift handoff
is unchanged.
"""
from enum import IntEnum

import numpy as np

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.preap.nap_conf import REGEN_MAX


# Like stalk tip/hold (0.40 s). A brake tap springs back sooner; a
# held pedal is still Applied well past this.
BRAKE_TIP_HOLD_S = 0.30

# Mid of the 10–15 mph band. End the ramp when we reach this, or when
# the window expires — then RELEASE to stock (no interceptor friction).
BRAKE_GLIDE_V_TARGET = 12.0 * CV.MPH_TO_MS
BRAKE_GLIDE_V_BAND_LO = 10.0 * CV.MPH_TO_MS
BRAKE_GLIDE_V_BAND_HI = 15.0 * CV.MPH_TO_MS

# Tunable in the 2–3 s band. Not the reverted 0.75 s regen fade.
BRAKE_GLIDE_DURATION_S = 2.5

# Noticeable but not a pitch. Comfort Accel-5 overlay peaks around
# −0.55; stock lift-regen is the −1.5 bite we must not dump on frame 1.
BRAKE_GLIDE_A_START = -0.30

# Interceptor regen floor. Cannot command friction.
BRAKE_GLIDE_A_MIN = float(REGEN_MAX)

# Deeper / panic: measured decel at or past the pre-AP planner clip.
# Comfort Accel-5 is −0.80; planner clip −1.5. At/under clip is
# "driver is braking for real" — do not ramp, RELEASE immediately.
BRAKE_CANCEL_FIRM_A_EGO = -1.5

# Don't hold the interceptor at a stop / KF chatter.
BRAKE_CANCEL_MIN_SPEED = 1.0


class BrakeTipGlideMode(IntEnum):
  IDLE = 0
  PENDING = 1  # brake down after long drop; classifying tip vs hold
  GLIDE = 2    # tip confirmed; comfort ramp toward ~12 mph / REGEN_MAX
  FIRM = 3     # held / deeper; stock immediately


def _is_firm_decel(a_ego: float) -> bool:
  return bool(np.isfinite(a_ego) and a_ego <= BRAKE_CANCEL_FIRM_A_EGO)


def _ease_in(elapsed_s: float, duration_s: float) -> float:
  """Quadratic ease-in: da/dt = 0 at t=0 (low initial jerk), hard at the end."""
  if duration_s <= 0.0:
    return 1.0
  u = float(np.clip(elapsed_s / duration_s, 0.0, 1.0))
  return u * u


def glide_end_accel(v0: float, duration_s: float = BRAKE_GLIDE_DURATION_S,
                    v_target: float = BRAKE_GLIDE_V_TARGET,
                    a_start: float = BRAKE_GLIDE_A_START,
                    a_min: float = BRAKE_GLIDE_A_MIN) -> float:
  """a_end so the ease-in ramp integrates toward v_target, clipped to regen.

  a(t) = a_start + (a_end − a_start) * (t/T)²
  ∫₀ᵀ a dt = T * (2*a_start + a_end) / 3
  """
  if not np.isfinite(v0) or duration_s <= 0.0:
    return float(a_min)
  dv = float(v_target) - float(v0)
  a_end = 3.0 * dv / float(duration_s) - 2.0 * float(a_start)
  # a_start is the gentlest (least negative) end we will command.
  return float(np.clip(a_end, a_min, a_start))


def comfort_ramp_accel(elapsed_s: float, a_end: float,
                       a_start: float = BRAKE_GLIDE_A_START,
                       duration_s: float = BRAKE_GLIDE_DURATION_S,
                       a_min: float = BRAKE_GLIDE_A_MIN) -> float:
  """Gentle at t=0, progressive to a_end. Never a step to stock / REGEN_MAX."""
  a_lo = float(np.clip(a_end, a_min, 0.0))
  a_hi = float(np.clip(a_start, a_lo, 0.0))
  progress = _ease_in(elapsed_s, duration_s)
  return float(a_hi + (a_lo - a_hi) * progress)


def glide_speed_ref(v0: float, elapsed_s: float,
                    duration_s: float = BRAKE_GLIDE_DURATION_S,
                    v_target: float = BRAKE_GLIDE_V_TARGET,
                    a_start: float = BRAKE_GLIDE_A_START,
                    a_min: float = BRAKE_GLIDE_A_MIN) -> float:
  """Speed after integrating the ease-in ramp (for tests / docs)."""
  a_end = glide_end_accel(v0, duration_s, v_target, a_start, a_min)
  t = min(max(0.0, float(elapsed_s)), float(duration_s))
  # ∫₀ᵗ a_start + (a_end−a_start)*(τ/T)² dτ
  #   = a_start*t + (a_end−a_start)*t³/(3*T²)
  t2 = duration_s * duration_s
  dv = a_start * t
  if t2 > 0.0:
    dv += (a_end - a_start) * (t ** 3) / (3.0 * t2)
  return float(max(v_target, float(v0) + dv))


class BrakeTipGlide:
  """Classify a brake long-pause as tip (comfort ramp) vs hold (firm)."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.mode = BrakeTipGlideMode.IDLE
    self.held_s = 0.0
    self.glide_s = 0.0
    self.v0 = 0.0
    self.a_end = 0.0
    self.ramp_active = False
    self.prev_requested_long = False

  @property
  def keep_enabled(self) -> bool:
    return self.mode in (BrakeTipGlideMode.PENDING, BrakeTipGlideMode.GLIDE)

  def commanded_accel(self) -> float:
    """Accel to request while interceptor stays ENABLE after a tip cancel.

    PENDING without a ramp (already in-band): 0. PENDING/GLIDE with a
    ramp: comfort shape — gentle first, stronger later. Not a constant
    speed-derived step, and not a 0→REGEN_MAX 0.75 s fade.
    """
    if self.mode not in (BrakeTipGlideMode.PENDING, BrakeTipGlideMode.GLIDE):
      return 0.0
    if not self.ramp_active:
      return 0.0
    return comfort_ramp_accel(self.glide_s, self.a_end)

  def accel_effort_limits(self) -> tuple[float, float]:
    """Pin VDAS effort to the commanded ramp (planner/FCW a cannot punch)."""
    a = float(np.clip(self.commanded_accel(), BRAKE_GLIDE_A_MIN, 0.0))
    return (a, a)

  def _arm_from_long_drop(self, v_ego: float):
    self.mode = BrakeTipGlideMode.PENDING
    self.held_s = 0.0
    self.glide_s = 0.0
    self.v0 = float(v_ego) if np.isfinite(v_ego) else 0.0
    self.ramp_active = np.isfinite(v_ego) and float(v_ego) > BRAKE_GLIDE_V_BAND_HI
    self.a_end = glide_end_accel(self.v0) if self.ramp_active else 0.0

  def _confirm_tip(self, v_ego: float) -> bool:
    """Lift before the hold window: keep ramping, or RELEASE if in-band."""
    if not self.ramp_active:
      self.mode = BrakeTipGlideMode.IDLE
      return False
    if np.isfinite(v_ego) and float(v_ego) <= BRAKE_GLIDE_V_BAND_HI:
      self.mode = BrakeTipGlideMode.IDLE
      self.ramp_active = False
      return False
    self.mode = BrakeTipGlideMode.GLIDE
    return True

  def update(self, *, requested_long: bool, cruise_enabled: bool,
             brake_pressed: bool, gas_pressed: bool,
             a_ego: float, v_ego: float, dt: float) -> bool:
    """Return True when interceptor should stay ENABLE for the tip ramp."""
    if (not cruise_enabled) or gas_pressed or float(v_ego) < BRAKE_CANCEL_MIN_SPEED:
      self.reset()
      self.prev_requested_long = bool(requested_long)
      return False

    if requested_long:
      self.mode = BrakeTipGlideMode.IDLE
      self.held_s = 0.0
      self.glide_s = 0.0
      self.a_end = 0.0
      self.ramp_active = False
      self.prev_requested_long = True
      return False

    long_falling = self.prev_requested_long and not requested_long
    self.prev_requested_long = bool(requested_long)

    if long_falling and brake_pressed:
      self._arm_from_long_drop(v_ego)

    if self.mode == BrakeTipGlideMode.IDLE:
      return False

    if _is_firm_decel(a_ego):
      self.mode = BrakeTipGlideMode.FIRM
      self.ramp_active = False
      return False

    if self.mode == BrakeTipGlideMode.FIRM:
      if not brake_pressed:
        self.mode = BrakeTipGlideMode.IDLE
      return False

    if self.mode == BrakeTipGlideMode.PENDING:
      if brake_pressed:
        self.held_s += float(dt)
        self.glide_s += float(dt)
        if self.held_s + 1e-9 >= BRAKE_TIP_HOLD_S:
          self.mode = BrakeTipGlideMode.FIRM
          self.ramp_active = False
          return False
        return True
      return self._confirm_tip(v_ego)

    if self.mode == BrakeTipGlideMode.GLIDE:
      # Second press during the ramp is a real brake — stock immediately.
      if brake_pressed:
        self.mode = BrakeTipGlideMode.FIRM
        self.ramp_active = False
        return False
      self.glide_s += float(dt)
      reached_band = np.isfinite(v_ego) and float(v_ego) <= BRAKE_GLIDE_V_TARGET
      window_done = self.glide_s + 1e-9 >= BRAKE_GLIDE_DURATION_S
      if reached_band or window_done:
        self.mode = BrakeTipGlideMode.IDLE
        self.ramp_active = False
        return False
      return True

    return False
