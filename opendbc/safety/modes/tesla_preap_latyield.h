#pragma once

// Lateral-yield tracking for tesla_preap — included from tesla_preap.h.
//
// A steering yank / hands-on cancels openpilot only while openpilot is in
// FULL control of lateral (A). While lateral is released to the driver
// (blinker turn, lane-change handoff, driver handoff, roundabout takeover,
// low-speed inhibit, ...) a wheel input must not cancel (B): longitudinal
// stays engaged.
//
// Panda cannot see openpilot's internal state. It derives "lateral yielded"
// from what it actually let through: DAS_steeringControl (0x488) with
// DAS_steeringControlType == 1 (active). If no active frame has been
// allowed for PREAP_LAT_RECENT_US, openpilot is not steering and a
// hands-on edge is a driver maneuver (B). openpilot dying also reads as
// yielded, so this fails safe.
//
// Re-arm grace: when the active stream resumes after a lapse (a yield; not
// the first active frame of the session) the next PREAP_LAT_GRACE_US also
// read as yielded so a hand-grab during the 1 s take-back blend is B.
//
// INFERENCE CONTRACT: openpilot does not send panda any "lateral yielded"
// state; the A/B decision is inferred from the 0x488 type-1 history alone,
// assuming (1) openpilot sends type 1 only while it steers (every release,
// yield, pause and inhibit ends in type 0), (2) a yield lasts at least the
// 0.15 s hands-off confirm before the take-back blend, and (3) the blend is
// PREAP_LAT_ASSUMED_BLEND_US long and sends type 1 throughout. ANY change to
// lateral transitions or handoff timing in openpilot (driver_lateral_handoff,
// blinker / lane-change / roundabout lateral pauses, a new release path,
// authority shaping) MUST update this inference: the constants below,
// opendbc/car/tesla/preap/lat_yield.py, and the tests (including
// selfdrive/controls/lib/tests/test_lat_yield_inference_guard.py in openpilot).
// Reflash the panda after changing anything here.
//
// Host-only 0x561 is not a "lateral yielded" announcement and is not part
// of the inference above. It only widens a hands-on edge from "cancel" to
// "block lateral, keep controls_allowed" (roundabout, or under-tracking
// when torque matches the wheel). It cannot enable actuation, raise a
// limit, or keep latActive. Stale after PREAP_LAT_FLAG_TIMEOUT_US.
//
// The Python card mirrors this (opendbc/car/tesla/preap/lat_yield.py).
// Python MUST be the more eager side to cancel (longer recent window,
// shorter grace) so panda alone never drops controls_allowed while
// openpilot still believes it is engaged (controlsMismatch).

// 0x488 is sent every 20 ms. 5 frames of silence == yielded.
#define PREAP_LAT_RECENT_US       100000U
// A lapse this long in the active stream is a yield; the stream restarting
// afterwards starts the re-arm grace.
#define PREAP_LAT_GAP_US          100000U
// Take-back blend length assumed by the inference (DriverLateralHandoff.
// BLEND_TIME_S); guard-tested against the real constant.
#define PREAP_LAT_ASSUMED_BLEND_US 1000000U
// Grace = blend + margin.
#define PREAP_LAT_GRACE_US       1200000U
// Active steering stays blocked this long after hands-on drops below the
// disengage level, so a stale request cannot re-grab a releasing wheel.
#define PREAP_LAT_BLOCK_CLEAR_US  150000U

// Angle-error / torque deadbands. 10 = 1.0 deg (0.1 deg units), 30 = 0.30 Nm
// (0.01 Nm units). Inside the deadband the sign is ambiguous: not "help".
#define PREAP_ANGLE_HELP_DEADBAND 10
#define PREAP_TORQUE_DIR_DEADBAND 30
#define PREAP_LAT_FLAG_ADDR       0x561U
#define PREAP_LAT_FLAG_TIMEOUT_US 250000U
#define PREAP_LAT_FLAG_MAGIC      0xA0U
#define PREAP_LAT_FLAG_ROUNDABOUT 0x01U
#define PREAP_LAT_FLAG_UNDERTRACK 0x02U

static bool preap_lat_seen = false;          // an active frame was allowed this session
static uint32_t preap_lat_last_us = 0U;      // time of the last allowed active frame
static bool preap_lat_gap = false;           // the active stream lapsed since that frame
static bool preap_lat_grace = false;         // re-arm grace running
static uint32_t preap_lat_grace_start_us = 0U;
static bool preap_lat_block = false;         // B edge: no active steering until hands release
static bool preap_lat_block_clearing = false;
static uint32_t preap_lat_block_clear_since_us = 0U;
// Last allowed type-1 command and the latest EPAS angle / torsion, CAN frame
// (0.1 deg, 0.01 Nm, 0 = 0). Signs match the carState frame, so help does
// not depend on which side negates them.
static bool preap_lat_cmd_valid = false;
static int preap_lat_cmd_angle = 0;
static int preap_lat_meas_angle = 0;
static int preap_lat_torque = 0;
static bool preap_lat_flag_seen = false;
static uint32_t preap_lat_flag_us = 0U;
static bool preap_lat_flag_roundabout = false;
static bool preap_lat_flag_undertrack = false;

static void preap_lat_yield_reset(void) {
  preap_lat_seen = false;
  preap_lat_last_us = 0U;
  preap_lat_gap = false;
  preap_lat_grace = false;
  preap_lat_grace_start_us = 0U;
  preap_lat_block = false;
  preap_lat_block_clearing = false;
  preap_lat_block_clear_since_us = 0U;
  preap_lat_cmd_valid = false;
  preap_lat_cmd_angle = 0;
  preap_lat_meas_angle = 0;
  preap_lat_torque = 0;
  preap_lat_flag_seen = false;
  preap_lat_flag_us = 0U;
  preap_lat_flag_roundabout = false;
  preap_lat_flag_undertrack = false;
}

// Called from the RX path (EPAS 25 Hz) so the 32-bit microsecond timer
// can never wrap around a stale stamp.
static void preap_lat_yield_tick(void) {
  const uint32_t now = microsecond_timer_get();
  if (preap_lat_seen && !preap_lat_gap && ((now - preap_lat_last_us) > PREAP_LAT_GAP_US)) {
    preap_lat_gap = true;
  }
  if (preap_lat_grace && ((now - preap_lat_grace_start_us) >= PREAP_LAT_GRACE_US)) {
    preap_lat_grace = false;
  }
}

// An allowed DAS_steeringControl frame with DAS_steeringControlType == 1.
static void preap_lat_note_active_tx(void) {
  const uint32_t now = microsecond_timer_get();
  if (!preap_lat_seen) {
    // First lateral of the session: full control, no grace.
    preap_lat_seen = true;
    preap_lat_gap = false;
    preap_lat_grace = false;
  } else if (preap_lat_gap || ((now - preap_lat_last_us) > PREAP_LAT_GAP_US)) {
    // Resumed after a yield.
    preap_lat_gap = false;
    preap_lat_grace = true;
    preap_lat_grace_start_us = now;
  }
  preap_lat_last_us = now;
}

// True when openpilot is in full control of lateral: a hands-on edge now is
// a yank (A) and cancels.
static bool preap_lat_in_full_control(void) {
  preap_lat_yield_tick();
  if (!preap_lat_seen || preap_lat_gap || preap_lat_grace) {
    return false;
  }
  return (microsecond_timer_get() - preap_lat_last_us) <= PREAP_LAT_RECENT_US;
}

// Release the B-edge steering block once hands-on has stayed below the
// disengage level for PREAP_LAT_BLOCK_CLEAR_US.
static void preap_lat_block_update(bool steering_disengage_now) {
  if (!preap_lat_block) {
    return;
  }
  if (steering_disengage_now) {
    preap_lat_block_clearing = false;
    return;
  }
  const uint32_t now = microsecond_timer_get();
  if (!preap_lat_block_clearing) {
    preap_lat_block_clearing = true;
    preap_lat_block_clear_since_us = now;
  } else if ((now - preap_lat_block_clear_since_us) >= PREAP_LAT_BLOCK_CLEAR_US) {
    preap_lat_block = false;
    preap_lat_block_clearing = false;
  }
}

static void preap_lat_note_cmd_angle(int desired_angle_can) {
  preap_lat_cmd_valid = true;
  preap_lat_cmd_angle = desired_angle_can;
}

static void preap_lat_note_meas(int angle_can, int torque_can) {
  preap_lat_meas_angle = angle_can;
  preap_lat_torque = torque_can;
}

static int preap_lat_sign(int value, int deadband) {
  if (value > deadband) return 1;
  if (value < -deadband) return -1;
  return 0;
}

static bool preap_lat_flag_fresh(void) {
  if (!preap_lat_flag_seen) {
    return false;
  }
  return (microsecond_timer_get() - preap_lat_flag_us) <= PREAP_LAT_FLAG_TIMEOUT_US;
}

// Torque helps the last allowed command, or (flag only) matches the wheel
// while the planner says OP is under-tracking. Deadband is not help.
static bool preap_lat_torque_helps(void) {
  const int tsign = preap_lat_sign(preap_lat_torque, PREAP_TORQUE_DIR_DEADBAND);
  if (tsign == 0) {
    return false;
  }
  if (preap_lat_cmd_valid) {
    const int esign = preap_lat_sign(preap_lat_cmd_angle - preap_lat_meas_angle, PREAP_ANGLE_HELP_DEADBAND);
    if ((esign != 0) && (esign == tsign)) {
      return true;
    }
  }
  if (preap_lat_flag_fresh() && preap_lat_flag_undertrack) {
    const int wsign = preap_lat_sign(preap_lat_meas_angle, PREAP_ANGLE_HELP_DEADBAND);
    if ((wsign != 0) && (wsign == tsign)) {
      return true;
    }
  }
  return false;
}

// Hands-on (not an EPAS reject with hands off) should yield instead of cancel.
static bool preap_lat_yield_instead(int hands_on_level) {
  if (hands_on_level < 2) {
    return false;
  }
  if (preap_lat_flag_fresh() && preap_lat_flag_roundabout) {
    return true;
  }
  return preap_lat_torque_helps();
}

// 0x561 byte. Magic high nibble, reserved bits clear, else ignored.
// Always called only for a message that will not be forwarded.
static void preap_lat_note_flag(uint8_t byte) {
  if (((byte & 0xF0U) != PREAP_LAT_FLAG_MAGIC) || ((byte & 0x0CU) != 0U)) {
    return;
  }
  preap_lat_flag_seen = true;
  preap_lat_flag_us = microsecond_timer_get();
  preap_lat_flag_roundabout = (byte & PREAP_LAT_FLAG_ROUNDABOUT) != 0U;
  preap_lat_flag_undertrack = (byte & PREAP_LAT_FLAG_UNDERTRACK) != 0U;
}
