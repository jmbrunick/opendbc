#pragma once

// RX hook for tesla_preap — included from tesla_preap.h.

static void tesla_preap_rx_hook(const CANPacket_t *msg) {
  // Pedal interceptor (0x552) — may arrive on bus 0 OR bus 2 depending on wiring.
  // Must be handled BEFORE the bus-0-only bailout below.
  // Whitelisted on both bus 0 and bus 2 in preap_rx_checks; the framework has
  // already verified the message matches one of them, so accept either here.
  //
  // Gas-press threshold: 650 raw, chosen from real Pre-AP drive data:
  //   - At-rest noise (driver not pressing): raw range 424-633, mean 470 (p99.9=602)
  //   - Actual gas press: raw range 441-1246, mean 799 (p10=607, p50=802)
  // The original threshold of 450 was inside the resting noise distribution and
  // caused false gas_pressed readings that blocked pedal TX → pedal wouldn't engage.
  // 650 gives zero false positives on rest noise while still catching the vast
  // majority of real driver presses. Python-layer DI_pedalPos is the primary
  // gas-override detection; the panda threshold here is a safety backstop.
  if (preap_enable_pedal && (msg->addr == 0x552U)) {
    int pedal_val = ((msg->data[0] << 8) | msg->data[1]);
    gas_pressed = (pedal_val > 650);
    if (preap_pedal_can == -1) {
      preap_pedal_can = msg->bus;
    }
    return;
  }

  // All other RX handlers are bus 0 only.
  if (msg->bus != 0U) return;

  // EPAS (0x370): steering angle, hands-on level, disengage detection
  if (msg->addr == 0x370U) {
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;
    const int eac_status = msg->data[6] >> 5;
    const int eac_error_code = msg->data[2] >> 4;

    // Disengage on hands-on override OR EPAS actively rejecting steering commands.
    // Error codes 6/7/8 = EPAS request validators rejected angle/rate, 9 = safety layer.
    // All indicate the EPAS stopped steering — driver must be notified immediately.
    // During a blinker-latched driver turn, lat is already released; dropping
    // controls_allowed here is what produces controlsMismatch after the Python
    // hold ends. Stalk cancel / doors / gear still drop independently.
    bool epas_rejecting = (eac_status == 0) && (eac_error_code >= 6) && (eac_error_code <= 9);
    steering_disengage = (hands_on_level >= PREAP_HANDS_ON_DISENGAGE_LEVEL) || epas_rejecting;
    preap_steering_disengage = steering_disengage;
    preap_hands_on_level = hands_on_level;
    preap_update_blinker_hold();

    // Re-arm fix: force cruise_engaged_prev reset on steering disengage
    // so next stalk pull creates a clean rising edge
    if (steering_disengage && !steering_disengage_prev) {
      if (!preap_blinker_turn_blocks_disengage()) {
        pcm_cruise_check(false);
      }
    }
  }

  // Vehicle speed (ESP_B: 0x155) — derive vehicle_moving from actual speed
  if (msg->addr == 0x155U) {
    float speed = (((msg->data[5] << 8) | msg->data[6]) * 0.01f) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
    vehicle_moving = speed > (0.5f * KPH_TO_MS);
    preap_esp_seen = true;
    preap_esp_ts = microsecond_timer_get();
    preap_esp_standstill = ((((uint32_t)msg->data[5] << 8) | (uint32_t)msg->data[6]) <= PREAP_CALIBRATION_STANDSTILL_CENTI_KPH);
  }

  // Gas pressed from DI_torque1 (0x108) — only when pedal interceptor is not active.
  // (The pedal interceptor path is handled above the bus-0-only bailout since it may
  // arrive on bus 0 or bus 2.)
  if (msg->addr == 0x108U) {
    if (!preap_enable_pedal) {
      gas_pressed = msg->data[6] != 0U;
    }
  }

  // Brake (0x20a) — latch pedal authority separately, but keep the framework
  // brake false so generic_rx_checks doesn't drop lateral controls_allowed.
  if (msg->addr == 0x20aU) {
    preap_brake_message_pressed = ((msg->data[0] >> 2) & 0x03U) == 2U;
    brake_pressed = false;
    preap_brake_message_seen = true;
    preap_brake_message_ts = microsecond_timer_get();
  }

  // Cruise state (DI_state: 0x368) — vehicle_moving only, engagement via stalk
  if (msg->addr == 0x368U) {
    int cruise_state = (msg->data[1] >> 4) & 0x07U;
    // Backup vehicle_moving from cruise state (standstill detection)
    if (cruise_state == 3) {
      vehicle_moving = false;
    }
  }

  // DI brake closes the interval before the slower BrakeMessage arrives.
  // Leaving Drive (and returning to Drive) re-arms via pcm_cruise_check(false).
  if (msg->addr == 0x118U) {
    preap_di_brake_pressed = ((msg->data[1] >> 7) & 0x01U) != 0U;
    preap_gear = (msg->data[1] >> 4) & 0x07;
    preap_di_brake_seen = true;
    preap_di_brake_ts = microsecond_timer_get();
    preap_gear_seen = true;
    preap_gear_ts = preap_di_brake_ts;
    // R/P/N must leave the same clean PCM state as a real stalk disable:
    // controls_allowed=false AND cruise_engaged_prev=false. Setting
    // controls_allowed=false alone leaves the latch set, so the next Drive
    // SET is not a rising edge: selfdrived enables, panda does not,
    // controlsMismatch after ~2s.
    //
    // Python hard_cancel_session spoofs CANCEL on 0x45 TX. Panda does not
    // RX its own TX, so that spoof cannot clear the latch — only a real
    // stalk cancel on RX can, which is why Justin could recover with an
    // extra stalk disable→enable after returning to Drive. Pulse false on
    // every non-Drive 0x118 and again on the Drive rising edge so that
    // extra cycle is not required.
    if (preap_gear != 4) {
      pcm_cruise_check(false);
    } else if (preap_gear_prev != 4) {
      pcm_cruise_check(false);
    }
    preap_gear_prev = preap_gear;
  }

  // Door check + blinker lamps (GTW_carState: 0x318)
  if (msg->addr == 0x318U) {
    int d_fl = (msg->data[1] >> 4) & 0x03;
    int d_fr = (msg->data[1] >> 6) & 0x03;
    int d_rl = (msg->data[2] >> 6) & 0x03;
    int d_rr = (msg->data[3] >> 5) & 0x03;
    int d_ft = (msg->data[6] >> 2) & 0x03;
    int d_tr = (msg->data[5] >> 6) & 0x03;
    preap_doors_open = (d_fl == 1) || (d_fr == 1) || (d_rl == 1) || (d_rr == 1) || (d_ft == 1) || (d_tr == 1);
    if (preap_doors_open) {
      pcm_cruise_check(false);
    }
    // BC_indicatorLStatus 59|2@0, BC_indicatorRStatus 61|2@0. Lamp on == 1.
    preap_left_lamp = ((msg->data[7] >> 2) & 0x03U) == 1U;
    preap_right_lamp = ((msg->data[7] >> 4) & 0x03U) == 1U;
    preap_update_blinker_hold();
  }

  // Stalk engagement (STW_ACTN_RQ: 0x45) with echo-filtered cancel
  if (msg->addr == 0x45U) {
    int lever = msg->data[0] & 0x3FU;
    // TurnIndLvr_Stat 16|2@1: IDLE=0 LEFT=1 RIGHT=2 SNA=3
    preap_update_stalk((int)(msg->data[2] & 0x03U));
    if (lever == 2) {  // RWD = pull toward driver = enable
      if ((preap_gear == 4) && !preap_doors_open) {
        // Drive SET while !controls_allowed must match stalk cancel→SET.
        // Leftover cruise_engaged_prev (TX-only CANCEL spoof, or a drop
        // that only cleared controls_allowed) is not a rising edge.
        if (!controls_allowed) {
          pcm_cruise_check(false);
        }
        pcm_cruise_check(true);
        preap_last_stalk_engage_us = microsecond_timer_get();
      }
    } else if (lever == 1) {  // FWD = push away = cancel
      uint32_t elapsed = microsecond_timer_get() - preap_last_stalk_engage_us;
      if (elapsed > PREAP_CANCEL_ECHO_WINDOW_US) {
        pcm_cruise_check(false);
      }
    }
    preap_update_blinker_hold();
  }

  if (preap_pedal_calibration) {
    controls_allowed = false;
  }

  // generic_rx_checks runs after this hook and drops on a steering_disengage
  // rising edge. Publish the blinker-turn hold so that path matches
  // pcm_cruise_check and Python blinker_turn_blocks_steering_disengage.
  steering_disengage_keep_controls = controls_allowed && preap_blinker_turn_blocks_disengage();
}
