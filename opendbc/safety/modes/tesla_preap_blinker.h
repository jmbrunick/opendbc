#pragma once

// Blinker-turn latch for tesla_preap — included from tesla_preap.h.
// Matches BlinkerLateralHold / blinker_turn_blocks_steering_disengage.

// Blinker-turn latch — matches BlinkerLateralHold / blinker_turn_blocks_
// steering_disengage on nap-dev. GTW lamp bits flash (~0.3s dark); latch
// through those gaps until both lamps have been dark for ~1s. A held
// LEFT/RIGHT stalk is a driver turn (0.40s from idle; 1.0s same-direction
// after a tip / ALC keep-alive). A tip (LEFT/RIGHT then IDLE within 0.40s)
// is ALC: do not latch leftover keep-alive flashes as a turn. Hazards
// (both lamps) are not a turn. Stalk cancel / doors / gear still drop.
#define PREAP_LAMP_OFF_DEBOUNCE_US     1000000U
#define PREAP_STALK_TIP_HOLD_US         400000U
#define PREAP_STALK_ALC_TURN_HOLD_US   1000000U

static bool preap_left_lamp = false;
static bool preap_right_lamp = false;
static bool preap_steering_disengage = false;
static int preap_hands_on_level = 0;
static bool preap_turn_active = false;
static bool preap_turn_holding = false;
static bool preap_lamp_dark_counting = false;
static uint32_t preap_lamp_dark_since_us = 0;
static bool preap_alc_keep = false;
static bool preap_alc_dark_counting = false;
static uint32_t preap_alc_dark_since_us = 0;
static int preap_alc_direction = 0;  // 0 unknown, 1 left, 2 right
static int preap_stalk_dir = 0;      // 0 idle/SNA, 1 left, 2 right
static uint32_t preap_stalk_held_since_us = 0;
static bool preap_stalk_tip_event = false;
static int preap_stalk_tip_dir = 0;

static void preap_reset_blinker_hold(void) {
  preap_left_lamp = false;
  preap_right_lamp = false;
  preap_steering_disengage = false;
  preap_hands_on_level = 0;
  preap_turn_active = false;
  preap_turn_holding = false;
  preap_lamp_dark_counting = false;
  preap_lamp_dark_since_us = 0;
  preap_alc_keep = false;
  preap_alc_dark_counting = false;
  preap_alc_dark_since_us = 0;
  preap_alc_direction = 0;
  preap_stalk_dir = 0;
  preap_stalk_held_since_us = 0;
  preap_stalk_tip_event = false;
  preap_stalk_tip_dir = 0;
}

static int preap_normalize_stalk(int raw) {
  return ((raw == 1) || (raw == 2)) ? raw : 0;
}

static void preap_update_stalk(int raw) {
  const int stalk = preap_normalize_stalk(raw);
  const uint32_t now = microsecond_timer_get();
  preap_stalk_tip_event = false;
  preap_stalk_tip_dir = 0;
  if ((stalk == 1) || (stalk == 2)) {
    if (preap_stalk_dir != stalk) {
      preap_stalk_dir = stalk;
      preap_stalk_held_since_us = now;
    }
  } else {
    if ((preap_stalk_dir == 1) || (preap_stalk_dir == 2)) {
      const uint32_t held = now - preap_stalk_held_since_us;
      if (held < PREAP_STALK_TIP_HOLD_US) {
        preap_stalk_tip_event = true;
        preap_stalk_tip_dir = preap_stalk_dir;
      }
    }
    preap_stalk_dir = 0;
    preap_stalk_held_since_us = 0;
  }
}

static bool preap_stalk_pending(void) {
  if ((preap_stalk_dir != 1) && (preap_stalk_dir != 2)) {
    return false;
  }
  return (microsecond_timer_get() - preap_stalk_held_since_us) < PREAP_STALK_TIP_HOLD_US;
}

static bool preap_stalk_is_driver_turn(void) {
  if ((preap_stalk_dir != 1) && (preap_stalk_dir != 2)) {
    return false;
  }
  const uint32_t held = microsecond_timer_get() - preap_stalk_held_since_us;
  if (!preap_alc_keep) {
    return held >= PREAP_STALK_TIP_HOLD_US;
  }
  if (((preap_alc_direction == 1) || (preap_alc_direction == 2)) &&
      (preap_stalk_dir != preap_alc_direction)) {
    return held >= PREAP_STALK_TIP_HOLD_US;
  }
  return held >= PREAP_STALK_ALC_TURN_HOLD_US;
}

static void preap_enter_alc_keep(int direction) {
  preap_turn_active = false;
  preap_turn_holding = false;
  preap_lamp_dark_counting = false;
  preap_lamp_dark_since_us = 0;
  preap_alc_keep = true;
  preap_alc_dark_counting = false;
  preap_alc_dark_since_us = 0;
  if (((direction == 1) || (direction == 2)) &&
      (preap_alc_direction != 1) && (preap_alc_direction != 2)) {
    preap_alc_direction = direction;
  }
}

static void preap_update_blinker_hold(void) {
  // Disengaged: drop latch. Same as BlinkerLateralHold.update(engaged=False).
  if (!controls_allowed) {
    preap_reset_blinker_hold();
    return;
  }

  const uint32_t now = microsecond_timer_get();
  const bool one_lamp = preap_left_lamp != preap_right_lamp;
  const bool both_dark = (!preap_left_lamp) && (!preap_right_lamp);
  const int lamp_dir = (preap_left_lamp && !preap_right_lamp) ? 1 :
                       (preap_right_lamp && !preap_left_lamp) ? 2 : 0;

  if (preap_alc_keep && ((lamp_dir == 1) || (lamp_dir == 2)) &&
      (preap_alc_direction != 1) && (preap_alc_direction != 2)) {
    preap_alc_direction = lamp_dir;
  }

  if (preap_stalk_is_driver_turn()) {
    preap_alc_keep = false;
    preap_alc_dark_counting = false;
    preap_alc_dark_since_us = 0;
    preap_alc_direction = 0;
    preap_turn_active = true;
    preap_lamp_dark_counting = false;
    preap_lamp_dark_since_us = 0;
  } else if ((!preap_turn_active) && (!preap_turn_holding) && preap_stalk_tip_event) {
    const int direction = (preap_stalk_tip_dir != 0) ? preap_stalk_tip_dir : lamp_dir;
    preap_enter_alc_keep(direction);
    preap_stalk_tip_event = false;
    return;
  } else if (preap_stalk_pending()) {
    // Not yet tip vs turn. Do not latch a turn from the driver's lamps.
    if (!(preap_turn_active || preap_turn_holding)) {
      return;
    }
  }

  if (preap_alc_keep) {
    if (both_dark) {
      if (!preap_alc_dark_counting) {
        preap_alc_dark_counting = true;
        preap_alc_dark_since_us = now;
      }
      if ((now - preap_alc_dark_since_us) >= PREAP_LAMP_OFF_DEBOUNCE_US) {
        preap_alc_keep = false;
        preap_alc_dark_counting = false;
        preap_alc_dark_since_us = 0;
        preap_alc_direction = 0;
      }
    } else {
      preap_alc_dark_counting = false;
      preap_alc_dark_since_us = 0;
      if (((lamp_dir == 1) || (lamp_dir == 2)) &&
          (preap_alc_direction != 1) && (preap_alc_direction != 2)) {
        preap_alc_direction = lamp_dir;
      }
    }
    if (preap_alc_keep) {
      preap_turn_active = false;
      preap_turn_holding = false;
      return;
    }
  }

  if (one_lamp) {
    preap_turn_active = true;
    preap_lamp_dark_counting = false;
    preap_lamp_dark_since_us = 0;
  } else if (preap_turn_active) {
    if (both_dark) {
      // High EPS torque means the driver is still in the corner.
      if (preap_steering_disengage) {
        preap_lamp_dark_counting = false;
        preap_lamp_dark_since_us = 0;
      } else {
        if (!preap_lamp_dark_counting) {
          preap_lamp_dark_counting = true;
          preap_lamp_dark_since_us = now;
        }
        if ((now - preap_lamp_dark_since_us) >= PREAP_LAMP_OFF_DEBOUNCE_US) {
          preap_turn_active = false;
          preap_lamp_dark_counting = false;
          preap_lamp_dark_since_us = 0;
        }
      }
    } else {
      // Hazards (both lamps) are not a turn.
      preap_turn_active = false;
      preap_lamp_dark_counting = false;
      preap_lamp_dark_since_us = 0;
    }
  }

  // Python holding uses torsion-bar steeringPressed or hands-on >= 2.
  // Panda has EPAS_handsOnLevel; >= 1 is "wheel still held" after the turn.
  const bool wheel_held = preap_steering_disengage || (preap_hands_on_level >= 1);
  if (preap_turn_active) {
    preap_turn_holding = true;
  } else if (preap_turn_holding && wheel_held) {
    // post-turn hand-on window
  } else {
    preap_turn_holding = false;
  }
}

static bool preap_blinker_turn_blocks_disengage(void) {
  // Hard gate matches blinker_turn_blocks_steering_disengage: one lamp
  // XOR, physical LEFT/RIGHT, or the flash-latch / post-turn hold.
  // ALC keep-alive is not a driver turn (no turn_active), so leftover
  // flashes do not by themselves keep controls_allowed once lamps are dark.
  if (preap_left_lamp != preap_right_lamp) {
    return true;
  }
  if ((preap_stalk_dir == 1) || (preap_stalk_dir == 2)) {
    return true;
  }
  return preap_turn_active || preap_turn_holding;
}
