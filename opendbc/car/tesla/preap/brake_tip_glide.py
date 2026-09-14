"""Tip vs hold brake-cancel: speed-target glide, then stock.

Product (digital Applied only — no travel/pressure):

- **Tip** (Applied shorter than BRAKE_TIP_HOLD_S, then lift, not firm
  aEgo): keep interceptor ENABLE and command a decelerating *speed*
  profile from v0 toward ~12 mph (mid of 10–15) over ~2.5 s (tunable
  2–3). Not a coast plateau. Not the reverted 0.75 s 0→REGEN_MAX fade.
- **Held** Applied past the tip window, or **hard** (aEgo at/under
  planner clip): RELEASE immediately — Tesla friction + stock regen.
- After the glide reaches ~12 mph, or the 2.5 s window expires: RELEASE
  to stock. The interceptor cannot friction-brake; this is the clean
  handoff. At high speed, REGEN_MAX (−1.5 m/s²) cannot reach 10–15 mph
  in 2–3 s — command that authority for the window, then hand off.
  Do not invent extra decel.

How commands map (no GAS_COMMAND DI rewrite):

- ENABLE=1 + accel_request=0 → zero-torque DI → coast.
- ENABLE=1 + accel_request<0 → more negative DI → interceptor regen
  only, clipped at REGEN_MAX.
- ENABLE=0 + command=0 → RELEASE → Tesla physical pedal / stock regen.
- aEgo classifies tip vs hold only; it is not the glide command.

PENDING (brake still down, classifying): keep ENABLE at a=0 so we do
not stack interceptor regen on Tesla friction, and do not RELEASE
(that would start stock regen before we know tip vs hold). Firm aEgo
short-circuits to RELEASE on the same frame.

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

# Mid of the 10–15 mph band. End the glide when we reach this, or when
# the window expires — then RELEASE to stock (no interceptor friction).
BRAKE_GLIDE_V_TARGET = 12.0 * CV.MPH_TO_MS
BRAKE_GLIDE_V_BAND_LO = 10.0 * CV.MPH_TO_MS
BRAKE_GLIDE_V_BAND_HI = 15.0 * CV.MPH_TO_MS

# Tunable in the 2–3 s band. Not the reverted 0.75 s regen fade.
BRAKE_GLIDE_DURATION_S = 2.5

# Interceptor regen floor. Cannot command friction.
BRAKE_GLIDE_A_MIN = float(REGEN_MAX)

# Deeper / panic: measured decel at or past the pre-AP planner clip.
# Comfort Accel-5 is −0.80; planner clip −1.5. At/under clip is
# "driver is braking for real" — do not glide, RELEASE immediately.
BRAKE_CANCEL_FIRM_A_EGO = -1.5

# Don't hold the interceptor at a stop / KF chatter.
BRAKE_CANCEL_MIN_SPEED = 1.0


class BrakeTipGlideMode(IntEnum):
  IDLE = 0
  PENDING = 1  # brake down after long drop; classifying tip vs hold
  GLIDE = 2    # tip confirmed; speed-target decel toward ~12 mph
  FIRM = 3     # held / deeper; stock immediately


def _is_firm_decel(a_ego: float) -> bool:
  return bool(np.isfinite(a_ego) and a_ego <= BRAKE_CANCEL_FIRM_A_EGO)


def glide_target_accel(v0: float, duration_s: float = BRAKE_GLIDE_DURATION_S,
                       v_target: float = BRAKE_GLIDE_V_TARGET,
                       a_min: float = BRAKE_GLIDE_A_MIN) -> float:
  """Constant decel from v0 toward v_target over duration_s.

  Clipped to interceptor regen. If Δv/T is steeper than REGEN_MAX,
  command REGEN_MAX — the window will expire above the band and we
  hand off. Do not fake physics the pedal cannot do.
  """
  if not np.isfinite(v0) or duration_s <= 0.0:
    return float(a_min)
  a_req = (float(v_target) - float(v0)) / float(duration_s)
  return float(np.clip(a_req, a_min, 0.0))


def glide_speed_ref(v0: float, elapsed_s: float,
                    duration_s: float = BRAKE_GLIDE_DURATION_S,
                    v_target: float = BRAKE_GLIDE_V_TARGET,
                    a_min: float = BRAKE_GLIDE_A_MIN) -> float:
  """Linear speed profile implied by glide_target_accel (for tests / docs)."""
  a = glide_target_accel(v0, duration_s, v_target, a_min)
  v = float(v0) + a * max(0.0, float(elapsed_s))
  return float(max(v_target, v))


class BrakeTipGlide:
  """Classify a brake long-pause as tip (speed glide) vs hold (firm)."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.mode = BrakeTipGlideMode.IDLE
    self.held_s = 0.0
    self.glide_s = 0.0
    self.v0 = 0.0
    self.a_cmd = 0.0
    self.prev_requested_long = False

  @property
  def keep_enabled(self) -> bool:
    return self.mode in (BrakeTipGlideMode.PENDING, BrakeTipGlideMode.GLIDE)

  def commanded_accel(self) -> float:
    """Accel to request while interceptor stays ENABLE after a tip cancel.

    PENDING (brake still down): 0 (coast) so we do not stack interceptor
    regen on top of Tesla friction. GLIDE: constant speed-derived decel,
    already clipped to REGEN_MAX. Not a 0→REGEN_MAX time fade.
    """
    if self.mode != BrakeTipGlideMode.GLIDE:
      return 0.0
    return float(self.a_cmd)

  def accel_effort_limits(self) -> tuple[float, float]:
    """Pin VDAS effort to the commanded glide (planner/FCW a cannot punch)."""
    a = float(np.clip(self.commanded_accel(), BRAKE_GLIDE_A_MIN, 0.0))
    return (a, a)

  def _arm_glide(self, v_ego: float) -> bool:
    """Start the speed-target glide, or RELEASE if already in the band."""
    if not np.isfinite(v_ego) or float(v_ego) <= BRAKE_GLIDE_V_BAND_HI:
      self.mode = BrakeTipGlideMode.IDLE
      return False
    self.mode = BrakeTipGlideMode.GLIDE
    self.v0 = float(v_ego)
    self.glide_s = 0.0
    self.a_cmd = glide_target_accel(self.v0)
    return True

  def update(self, *, requested_long: bool, cruise_enabled: bool,
             brake_pressed: bool, gas_pressed: bool,
             a_ego: float, v_ego: float, dt: float) -> bool:
    """Return True when interceptor should stay ENABLE for the tip glide."""
    if (not cruise_enabled) or gas_pressed or float(v_ego) < BRAKE_CANCEL_MIN_SPEED:
      self.reset()
      self.prev_requested_long = bool(requested_long)
      return False

    if requested_long:
      self.mode = BrakeTipGlideMode.IDLE
      self.held_s = 0.0
      self.glide_s = 0.0
      self.a_cmd = 0.0
      self.prev_requested_long = True
      return False

    long_falling = self.prev_requested_long and not requested_long
    self.prev_requested_long = bool(requested_long)

    if long_falling and brake_pressed:
      self.mode = BrakeTipGlideMode.PENDING
      self.held_s = 0.0
      self.glide_s = 0.0
      self.a_cmd = 0.0

    if self.mode == BrakeTipGlideMode.IDLE:
      return False

    if _is_firm_decel(a_ego):
      self.mode = BrakeTipGlideMode.FIRM
      return False

    if self.mode == BrakeTipGlideMode.FIRM:
      if not brake_pressed:
        self.mode = BrakeTipGlideMode.IDLE
      return False

    if self.mode == BrakeTipGlideMode.PENDING:
      if brake_pressed:
        self.held_s += float(dt)
        if self.held_s + 1e-9 >= BRAKE_TIP_HOLD_S:
          self.mode = BrakeTipGlideMode.FIRM
          return False
        return True
      return self._arm_glide(v_ego)

    if self.mode == BrakeTipGlideMode.GLIDE:
      # Second press during the glide is a real brake — stock immediately.
      if brake_pressed:
        self.mode = BrakeTipGlideMode.FIRM
        return False
      self.glide_s += float(dt)
      reached_band = np.isfinite(v_ego) and float(v_ego) <= BRAKE_GLIDE_V_TARGET
      window_done = self.glide_s + 1e-9 >= BRAKE_GLIDE_DURATION_S
      if reached_band or window_done:
        self.mode = BrakeTipGlideMode.IDLE
        return False
      return True

    return False
