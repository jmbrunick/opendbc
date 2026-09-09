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

static bool preap_f190_payload_allowed(const CANPacket_t *msg) {
  static const uint8_t tester[8] = {0x02U, 0x3EU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t default_session[8] = {0x02U, 0x10U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t extended_session[8] = {0x02U, 0x10U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t read_f190[8] = {0x03U, 0x22U, 0xF1U, 0x90U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t flow_control[8] = {0x30U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t cleanup_marker[8] = {0x02U, 0x3EU, 0x80U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  const uint8_t *allowed[] = {tester, default_session, extended_session, read_f190, flow_control, cleanup_marker};
  if (GET_LEN(msg) != 8U) {
    return false;
  }
  for (unsigned int i = 0U; i < (sizeof(allowed) / sizeof(allowed[0])); i++) {
    bool match = true;
    for (int b = 0; b < 8; b++) {
      if (msg->data[b] != allowed[i][b]) {
        match = false;
        break;
      }
    }
    if (match) {
      return true;
    }
  }
  return false;
}

static bool preap_f190_tx_ok(const CANPacket_t *msg) {
  // Read-only F190 on the radar bus. Never writes, routines, or security.
  if (!preap_radar_emulation || controls_allowed) {
    return false;
  }
  if (GET_BUS(msg) != 1U) {
    return false;
  }
  return preap_f190_payload_allowed(msg);
}

static uint32_t preap_radar_vin_char(int pos, int shift) {
  return ((uint32_t)preap_radar_vin[pos]) << (shift * 8);
}

static bool preap_radar_ready(void) {
  // Tinkla 0.6.6 sent no radar-bus GTW frames until the host 0x560
  // stream was complete and useRadar was set. Talking earlier lets the
  // radar hear this-car VIN/2WD/EPAS0, then a different contract a
  // second later.
  return (preap_radar_vin_complete == 7U) && preap_radar_should_send;
}

static bool preap_radar_donor_active(void) {
  if (preap_radar_vin_complete != 7U) {
    return false;
  }
  // 0.6.6 default was 17 spaces. Treat that as "use this car."
  for (int i = 0; i < 17; i++) {
    if ((preap_radar_vin[i] != 0U) && (preap_radar_vin[i] != (uint8_t)' ')) {
      return true;
    }
  }
  return false;
}

static void preap_apply_radar_vin_msg(const CANPacket_t *msg) {
  const int rec = msg->data[0];
  if (rec == 0) {
    preap_radar_should_send = (msg->data[2] & 0x01U) != 0U;
    preap_radar_position = (msg->data[2] >> 1) & 0x03;
    preap_radar_epas_type = (msg->data[2] >> 3) & 0x07;
    preap_radar_vin[0] = msg->data[5];
    preap_radar_vin[1] = msg->data[6];
    preap_radar_vin[2] = msg->data[7];
    preap_radar_vin_complete |= 1U;
  } else if (rec == 1) {
    preap_radar_vin[3] = msg->data[1];
    preap_radar_vin[4] = msg->data[2];
    preap_radar_vin[5] = msg->data[3];
    preap_radar_vin[6] = msg->data[4];
    preap_radar_vin[7] = msg->data[5];
    preap_radar_vin[8] = msg->data[6];
    preap_radar_vin[9] = msg->data[7];
    preap_radar_vin_complete |= 2U;
  } else if (rec == 2) {
    preap_radar_vin[10] = msg->data[1];
    preap_radar_vin[11] = msg->data[2];
    preap_radar_vin[12] = msg->data[3];
    preap_radar_vin[13] = msg->data[4];
    preap_radar_vin[14] = msg->data[5];
    preap_radar_vin[15] = msg->data[6];
    preap_radar_vin[16] = msg->data[7];
    preap_radar_vin_complete |= 4U;
  }
}

// ============================================
// Checksum and counter (for EPAS validation)
// ============================================

static uint8_t tesla_preap_get_counter(const CANPacket_t *msg) {
  if (msg->addr == 0x370U) {
    return msg->data[6] & 0x0FU;  // EPAS_sysStatusCounter
  }
  return 0U;
}

static uint32_t tesla_preap_get_checksum(const CANPacket_t *msg) {
  if (msg->addr == 0x370U) {
    return msg->data[7];  // EPAS_sysStatusChecksum at byte 7
  }
  if (msg->addr == 0x488U) {
    return msg->data[3];  // DAS_steeringControlChecksum at byte 3
  }
  return 0U;
}

static uint32_t tesla_preap_compute_checksum(const CANPacket_t *msg) {
  // Tesla byte-sum checksum: sum of address bytes + all data bytes except checksum byte
  int checksum_byte = -1;
  if (msg->addr == 0x370U) {
    checksum_byte = 7;
  } else if (msg->addr == 0x488U) {
    checksum_byte = 3;
  }
  if (checksum_byte == -1) {
    return 0U;
  }

  uint8_t chksum = (uint8_t)(msg->addr & 0xFFU) + (uint8_t)((msg->addr >> 8) & 0xFFU);
  int len = GET_LEN(msg);
  for (int i = 0; i < len; i++) {
    if (i != checksum_byte) {
      chksum += msg->data[i];
    }
  }
  return chksum;
}

static bool tesla_preap_source_fresh(bool seen, uint32_t ts, uint32_t now) {
  return seen && (safety_get_ts_elapsed(now, ts) <= PREAP_CALIBRATION_SOURCE_TIMEOUT_US);
}

static bool tesla_preap_calibration_window_open(uint32_t now) {
  return preap_pedal_calibration &&
         tesla_preap_source_fresh(preap_gear_seen, preap_gear_ts, now) &&
         tesla_preap_source_fresh(preap_di_brake_seen, preap_di_brake_ts, now) &&
         tesla_preap_source_fresh(preap_brake_message_seen, preap_brake_message_ts, now) &&
         tesla_preap_source_fresh(preap_esp_seen, preap_esp_ts, now) &&
         (preap_gear == 3) &&
         (preap_di_brake_pressed || preap_brake_message_pressed) &&
         preap_esp_standstill;
}

// CRC-8 lookup table (polynomial 0x1D) for steering angle re-addressing
static const int preap_crc_lookup[256] = {
  0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
  0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
  0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
  0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
  0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
  0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
  0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
  0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
  0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
  0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
  0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
  0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
  0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
  0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
  0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
  0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

static int preap_compute_crc8(uint32_t lo, uint32_t hi, int msg_len) {
  int crc = 0xFF;
  for (int x = 0; x < msg_len; x++) {
    int v = (x <= 3) ? ((lo >> (x * 8)) & 0xFF) : ((hi >> ((x - 4) * 8)) & 0xFF);
    crc = preap_crc_lookup[crc ^ v];
  }
  return crc ^ 0xFF;
}
