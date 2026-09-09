#pragma once

// ============================================
// SAFETY_TESLA_PREAP — Pre-Autopilot Tesla Model S (2012-2014)
// ============================================
//
// Standalone safety mode for Pre-AP Tesla Model S. These cars have NO
// Autopilot ECU, NO harness relay, and a different EPAS/CAN layout than
// HW1+ Teslas. This is the spiritual successor to Tinkla (Boggyver's
// Pre-AP openpilot fork, tesla_unity_betaC3 branch).
//
// WHY check_relay=false AND disable_static_blocking=true:
//
//   Pre-AP has no harness relay hardware. Standard openpilot uses a relay
//   on the harness to switch between stock AP ECU and openpilot — when
//   openpilot is not active, the relay routes CAN to the stock ECU. On
//   Pre-AP there is no AP ECU and no relay; the panda connects directly
//   to the car's CAN bus. Setting check_relay=true would cause the panda
//   to falsely detect a "relay malfunction" and block ALL TX permanently.
//   disable_static_blocking=true is required for the same reason — without
//   a relay, the panda's static blocking logic (which assumes relay state)
//   would incorrectly block messages.
//
//   Tinkla handled this identically via generic_rx_checks(false) in the
//   older panda API, with the comment "PreAP has no relay" (safety_tesla.h
//   line 1071, tesla_unity_betaC3 branch). The modern API added check_relay
//   and disable_static_blocking with restrictive defaults, so we explicitly
//   set them to get the same behavior Tinkla had implicitly.
//
// WHY ignore_checksum=true AND ignore_counter=true on RX:
//
//   Pre-AP EPAS firmware uses a byte-sum checksum, but the exact algorithm
//   has not been fully verified against all firmware versions. A checksum
//   mismatch caused a silent 21-second steering dropout during testing.
//   Tinkla's RX checks also had no checksum/counter validation (frequency
//   set to 0 for all messages). Once the checksum algorithm is verified
//   across all Pre-AP EPAS firmware versions, these can be re-enabled.
//
// ALL ACTUAL SAFETY CHECKS REMAIN FULLY ACTIVE:
//   - Steering angle + rate limits via steer_angle_cmd_checks_vm()
//   - controls_allowed gating on all TX
//   - Disengage on hands-on override (level >= 2), except during a
//     blinker-latched driver turn (one lamp XOR, physical LEFT/RIGHT,
//     flash-latched ~1s, then hand-on until release). ALC keep-alive
//     flashes and hazards are not a driver turn.
//   - Disengage on EPAS error codes 6-9, except during that same turn
//   - Disengage on door open, gear out of Drive
//   - Disengage on stalk cancel (with 600ms echo filter)
//   - AEB events blocked from openpilot
//   - EPB_epasControl mode validation
//   - Pedal TX gated by PREAP_FLAG_ENABLE_PEDAL + get_longitudinal_allowed()
//
// Completely independent from tesla_legacy.h — has its own hooks struct,
// counter/checksum functions, init, RX/TX/fwd hooks, and GTW emulation.
// Registered as SAFETY_TESLA_PREAP in declarations.h.

#include "opendbc/safety/declarations.h"

// Forward declarations for panda firmware CAN send (defined in can_common.h)
#if defined(STM32H7) || defined(STM32F4)
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);
void can_set_checksum(CANPacket_t *packet);
#endif

// ============================================
// Byte manipulation macros
// ============================================

#define PREAP_GET_BYTES_04(msg) ((msg)->data[0] | ((msg)->data[1] << 8) | ((msg)->data[2] << 16) | ((msg)->data[3] << 24))
#define PREAP_GET_BYTES_48(msg) ((msg)->data[4] | ((msg)->data[5] << 8) | ((msg)->data[6] << 16) | ((msg)->data[7] << 24))
#define PREAP_WORD_TO_BYTES(dst8, src32) 0[dst8] = ((src32) & 0xFFU); 1[dst8] = (((src32) >> 8U) & 0xFFU); 2[dst8] = (((src32) >> 16U) & 0xFFU); 3[dst8] = (((src32) >> 24U) & 0xFFU)

// ============================================
// Safety param flags
// ============================================
// Longitudinal is gated by PREAP_FLAG_ENABLE_PEDAL + get_longitudinal_allowed().
// There is no separate LONG_CONTROL flag — the framework's get_longitudinal_allowed()
// is a derived check (controls_allowed && !gas_pressed_prev), not a settable flag.
// This matches how tesla.h, honda.h, and hyundai.h handle longitudinal gating.

#define PREAP_FLAG_ENABLE_PEDAL         1U
#define PREAP_FLAG_RADAR_EMULATION      2U
// Leftover bit. Position comes from host 0x560, not this flag.
#define PREAP_FLAG_RADAR_BEHIND_NOSECONE 4U
#define PREAP_FLAG_PEDAL_BUS_ZERO       (1U << 5)
#define PREAP_FLAG_PEDAL_CALIBRATION    (1U << 6)
#define PREAP_HANDS_ON_DISENGAGE_LEVEL  2
#define PREAP_CALIBRATION_SOURCE_TIMEOUT_US 1000000U
#define PREAP_CALIBRATION_STANDSTILL_CENTI_KPH 180U  // 1.8 kph == 0.5 m/s; independent of driving 0.5 kph

// ============================================
// State variables
// ============================================

static bool preap_enable_pedal = false;
static bool preap_radar_emulation = false;
static bool preap_pedal_calibration = false;
static uint8_t preap_pedal_bus = 2U;

static int preap_pedal_can = -1;

// Gear and door checks
static int preap_gear = 4;        // init to Drive to avoid false disables on startup
static int preap_gear_prev = 4;
static bool preap_doors_open = false;
static bool preap_di_brake_pressed = false;
static bool preap_brake_message_pressed = false;
static bool preap_gear_seen = false;
static uint32_t preap_gear_ts = 0U;
static bool preap_di_brake_seen = false;
static uint32_t preap_di_brake_ts = 0U;
static bool preap_brake_message_seen = false;
static uint32_t preap_brake_message_ts = 0U;
static bool preap_esp_seen = false;
static uint32_t preap_esp_ts = 0U;
static bool preap_esp_standstill = false;
static bool preap_pedal_tx_counter_seen = false;
static uint8_t preap_pedal_tx_counter = 0U;

// Stalk echo filter
static uint32_t preap_last_stalk_engage_us = 0;
#define PREAP_CANCEL_ECHO_WINDOW_US 600000U  // 600ms

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

// Radar emulation state
static int preap_radar_status = 0;
static uint32_t preap_last_radar_signal = 0;
static int preap_radar_epas_type = 0;
static int preap_radar_position = 0;
static uint8_t preap_radar_vin[17];
static uint8_t preap_radar_vin_complete = 0;
static bool preap_radar_should_send = false;

// Host→panda donor config. 0x560 never goes on the car; tesla_preap_tx_hook
// consumes it. Layout matches Tinkla 0.6.6 create_radar_VIN_msg.
#define PREAP_RADAR_VIN_ADDR 0x560U
#define PREAP_RADAR_UDS_ADDR 0x641U
