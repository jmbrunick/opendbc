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
//     blinker-latched driver turn in tesla_preap_blinker.h (one lamp XOR,
//     physical LEFT/RIGHT, flash-latched ~1s, then hand-on until release).
//     ALC keep-alive flashes and hazards are not a driver turn.
//   - Disengage on EPAS error codes 6-9, except during that same turn
//   - Disengage on door open, gear out of Drive via pcm_cruise_check(false)
//     (also on the Drive rising edge) so cruise_engaged_prev clears.
//     Python CANCEL spoof is TX-only and cannot clear that latch; a real
//     stalk cancel on RX can. Drive SET while !controls_allowed pulses
//     false then true so R/P→Drive does not need an extra stalk cycle.
//   - Disengage on stalk cancel (with 600ms echo filter). TX CANCEL while
//     already !controls_allowed also clears the latch (spoof path).
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


#include "opendbc/safety/modes/tesla_preap_blinker.h"
#include "opendbc/safety/modes/tesla_preap_radar.h"
#include "opendbc/safety/modes/tesla_preap_rx.h"

#include "opendbc/safety/modes/tesla_preap_tx.h"

// ============================================
// Init
// ============================================

static safety_config tesla_preap_init(uint16_t param) {
  const bool calib_requested = GET_FLAG(param, PREAP_FLAG_PEDAL_CALIBRATION);
  const bool mixed_calib = calib_requested &&
                           (param != PREAP_FLAG_PEDAL_CALIBRATION) &&
                           (param != (PREAP_FLAG_PEDAL_CALIBRATION | PREAP_FLAG_PEDAL_BUS_ZERO));
  preap_pedal_calibration = calib_requested && !mixed_calib;
  preap_enable_pedal = GET_FLAG(param, PREAP_FLAG_ENABLE_PEDAL) && !preap_pedal_calibration && !mixed_calib;
  preap_radar_emulation = GET_FLAG(param, PREAP_FLAG_RADAR_EMULATION) && !preap_pedal_calibration && !mixed_calib;
  preap_pedal_bus = GET_FLAG(param, PREAP_FLAG_PEDAL_BUS_ZERO) ? 0U : 2U;

  preap_gear = 4;
  preap_gear_prev = 4;
  preap_doors_open = false;
  preap_di_brake_pressed = false;
  preap_brake_message_pressed = false;
  preap_gear_seen = false;
  preap_gear_ts = 0U;
  preap_di_brake_seen = false;
  preap_di_brake_ts = 0U;
  preap_brake_message_seen = false;
  preap_brake_message_ts = 0U;
  preap_esp_seen = false;
  preap_esp_ts = 0U;
  preap_esp_standstill = false;
  preap_pedal_tx_counter_seen = false;
  preap_pedal_tx_counter = 0U;
  preap_pedal_can = -1;
  preap_radar_status = 0;
  preap_last_radar_signal = 0;
  preap_last_stalk_engage_us = 0;
  preap_reset_blinker_hold();
  preap_radar_position = 0;
  preap_radar_epas_type = 0;
  preap_radar_vin_complete = 0;
  preap_radar_should_send = false;
  for (int i = 0; i < 17; i++) {
    preap_radar_vin[i] = (uint8_t)' ';
  }
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
  preap_radar_car_config_captured = false;
  preap_radar_vin_feed_captured = false;
#endif

  // TX whitelist — no harness relay on Pre-AP
  static const CanMsg PREAP_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = false, .disable_static_blocking = true},  // DAS_steeringControl
    {0x2B9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_control
    {0x214, 0, 3, .check_relay = false, .disable_static_blocking = true},  // EPB_epasControl
    {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on bus 0
    {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on bus 2
    {0x45,  0, 8, .check_relay = false, .disable_static_blocking = true},  // STW_ACTN_RQ (stalk spoof)
    {0x3E9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_bodyControls (turn signal)
    {0x560, 0, 8, .check_relay = false, .disable_static_blocking = true},  // donor VIN/config to panda
    {0x641, 1, 8, .check_relay = false, .disable_static_blocking = true},  // radar F190 read
  };

  // RX checks — disable EPAS counter/checksum until we verify the Pre-AP
  // EPAS firmware's checksum matches our compute_checksum exactly.
  // Mismatched validation caused silent 21s steering dropout.
  static RxCheck preap_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x118, 0, 6, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque2
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x318, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // GTW_carState
    {.msg = {{0x45,  0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // STW_ACTN_RQ
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_B
  };

  // Pedal-enabled variant: adds 0x552 (GAS_SENSOR) to rx_checks so the
  // framework routes it to the rx hook. Split into its own array because
  // frequency=0 causes divide-by-zero in safety_tick (safety.h:330), which
  // marks the check as lagging and trips safetyRxChecksInvalid → controls
  // mismatch on cars without a pedal. 50Hz matches the Comma Pedal firmware.
  static RxCheck preap_rx_checks_with_pedal[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x118, 0, 6, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x318, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x45,  0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x552, 0, 6, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true},
             {0x552, 2, 6, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }}},  // GAS_SENSOR
  };

  static const CanMsg PREAP_TX_MSGS_CAL_BUS0[] = {
    {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},
  };
  static const CanMsg PREAP_TX_MSGS_CAL_BUS2[] = {
    {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},
  };
  if (mixed_calib) {
    return (safety_config){NULL, 0, NULL, 0, true}; // NOLINT(readability/braces)
  }
  if (preap_pedal_calibration) {
    return (preap_pedal_bus == 0U) ? BUILD_SAFETY_CFG(preap_rx_checks, PREAP_TX_MSGS_CAL_BUS0)
                                   : BUILD_SAFETY_CFG(preap_rx_checks, PREAP_TX_MSGS_CAL_BUS2);
  }
  return preap_enable_pedal ? BUILD_SAFETY_CFG(preap_rx_checks_with_pedal, PREAP_TX_MSGS)
                            : BUILD_SAFETY_CFG(preap_rx_checks, PREAP_TX_MSGS);
}

// ============================================
// Hooks struct
// ============================================

const safety_hooks tesla_preap_hooks = {
  .init = tesla_preap_init,
  .rx = tesla_preap_rx_hook,
  .rx_all = tesla_preap_gtw_emulation,  // must see ALL CAN traffic for radar GTW forwarding
  .tx = tesla_preap_tx_hook,
  .fwd = tesla_preap_fwd_hook,
  .get_counter = tesla_preap_get_counter,
  .get_checksum = tesla_preap_get_checksum,
  .compute_checksum = tesla_preap_compute_checksum,
  .get_quality_flag_valid = NULL,
};
