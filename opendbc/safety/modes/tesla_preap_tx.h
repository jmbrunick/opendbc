#pragma once

// TX and forwarding hooks for tesla_preap — included from tesla_preap.h.

// ============================================
// TX Hook
// ============================================

static bool tesla_preap_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits PREAP_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // Pre-AP Model S is physically the same car as HW1/HW2/HW3 Model S.
  // These values MUST match VehicleModel(TESLA_MODEL_S_HW3) in carcontroller.py.
  // Verified: mass=2100+STD_CARGO_KG, wheelbase=2.960, steerRatio=15.0
  //           → slip_factor = -0.0005666 (calc_slip_factor)
  // Confirmed by Lukas (xnor-tech, former comma employee, Tesla port author).
  const AngleSteeringParams PREAP_STEERING_PARAMS = {
    .slip_factor = -0.0005666,
    .steer_ratio = 15.,
    .wheelbase = 2.96,
  };

  bool tx = true;
  bool violation = false;

  // Host→panda donor VIN/config. Intercept; do not put 0x560 on the car.
  if (msg->addr == PREAP_RADAR_VIN_ADDR) {
    preap_apply_radar_vin_msg(msg);
    return false;
  }

  // Radar UDS on bus 1. Allow only the F190 read sequence while disengaged.
  if (msg->addr == PREAP_RADAR_UDS_ADDR) {
    return preap_f190_tx_ok(msg);
  }

  // DAS_steeringControl (0x488)
  if (msg->addr == 0x488U) {
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    bool steer_control_enabled = steer_control_type == 1;

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, PREAP_STEERING_LIMITS, PREAP_STEERING_PARAMS)) {
      violation = true;
    }
    if ((steer_control_type != 0) && (steer_control_type != 1)) {
      violation = true;
    }
  }

  // EPB_epasControl (0x214): only allow valid EAC modes (0=disable, 1=enable)
  if (msg->addr == 0x214U) {
    int epas_control_type = msg->data[0] & 0x07U;  // EPB_epasEACAllow: bits 2:0 of byte 0
    if (epas_control_type > 1) {
      violation = true;
    }
  }

  // DAS_control (0x2B9): no AEB events from openpilot
  if (msg->addr == 0x2B9U) {
    int aeb_event = msg->data[2] & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }
  }

  // Pedal interceptor (0x551 GAS_COMMAND): parse ENABLE bit and GAS_COMMAND
  // value to distinguish authoritative accel commands from release commands.
  //   DBC: SG_ ENABLE : 39|1@0+  →  bit 7 of data[4]
  //   DBC: SG_ GAS_COMMAND : 7|16@0+  →  bytes 0-1 big-endian (physical 0 = raw 450)
  //
  //   ENABLE=0: openpilot is releasing control. Comma Pedal ignores GAS_COMMAND
  //   and passes the driver's OEM pedal voltage through. The controller sends
  //   one disabled-zero frame when relinquishing active authority or resetting
  //   faulted firmware, then stays silent.
  //   Defense-in-depth: we still require the GAS_COMMAND raw value to be at or
  //   below the zero point (raw <= 500, which is ~2.5% physical) so a bugged or
  //   malicious ENABLE=0 + high-value message can't sneak through a potential
  //   Comma Pedal firmware bug.
  //
  //   ENABLE=1: authoritative actuation command. Gated by get_longitudinal_allowed()
  //   (controls_allowed && !gas_pressed_prev).
  if (msg->addr == 0x551U) {
    if (preap_pedal_calibration) {
      const bool pedal_enable = (msg->data[4] & 0x80U) != 0U;
      const int raw_gas_cmd = (msg->data[0] << 8) | msg->data[1];
      const int raw_gas_cmd2 = (msg->data[2] << 8) | msg->data[3];
      const uint8_t counter = msg->data[4] & 0x0FU;
      uint8_t chksum = (uint8_t)(msg->addr & 0xFFU) + (uint8_t)((msg->addr >> 8) & 0xFFU);
      for (int i = 0; i < 5; i++) {
        chksum += msg->data[i];
      }
      const bool protocol_valid = (GET_BUS(msg) == preap_pedal_bus) &&
                                  (GET_LEN(msg) == 6U) &&
                                  !msg->fd &&
                                  ((msg->data[4] & 0x70U) == 0U) &&
                                  (chksum == msg->data[5]) &&
                                  (raw_gas_cmd < 65535) && (raw_gas_cmd2 < 65535) &&
                                  (!preap_pedal_tx_counter_seen ||
                                   (counter == (uint8_t)((preap_pedal_tx_counter + 1U) & 0x0FU)));
      if (!protocol_valid) {
        violation = true;
      } else if (pedal_enable) {
        if (!tesla_preap_calibration_window_open(microsecond_timer_get())) {
          violation = true;
        }
      } else if ((raw_gas_cmd > 500) || (raw_gas_cmd2 > 500)) {
        violation = true;
      }
      if (!violation) {
        preap_pedal_tx_counter_seen = true;
        preap_pedal_tx_counter = counter;
      }
    } else if (!preap_enable_pedal) {
      violation = true;
    } else {
      bool pedal_enable = (msg->data[4] & 0x80U) != 0U;
      int raw_gas_cmd = (msg->data[0] << 8) | msg->data[1];
      if (pedal_enable) {
        if (!get_longitudinal_allowed() || preap_di_brake_pressed || preap_brake_message_pressed) {
          violation = true;
        }
      } else {
        // ENABLE=0: only allow near-zero GAS_COMMAND values (defense-in-depth).
        // This admits the production raw-zero release and DBC physical zero.
        if (raw_gas_cmd > 500) {
          violation = true;
        }
      }
    }
  }

  // DAS_bodyControls (0x3E9): turn-signal actuation. Gate on controls_allowed
  // (matches all other Pre-AP TX) so the indicator can only be driven while
  // openpilot is engaged — on disengage controlsd clears the blinker anyway,
  // and this is the defense-in-depth backstop. Also bound the turn-indicator
  // request to valid values (0-3); the field is 2 bits (bit 8 = byte 1 bits
  // 0-1) so it cannot exceed 3 — a guard if the field width ever changes.
  if (msg->addr == 0x3E9U) {
    int turn_req = (msg->data[1] & 0x03U);  // DAS_turnIndicatorRequest at bit 8
    if (turn_req > 3) {
      violation = true;
    }
    if (!controls_allowed) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

// ============================================
// Forwarding Hook
// ============================================

static bool tesla_preap_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  (void)addr;
  // Pre-AP has no AP ECU on bus 2. Block default 0↔2 forwarding to avoid
  // flooding a dead TX queue.
  return true;
}
