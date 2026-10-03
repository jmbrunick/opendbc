#pragma once

// Radar / GTW emulation for tesla_preap — included from tesla_preap.h.

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

// ============================================
// GTW Emulation helpers
// ============================================

static void preap_radar_readdr(const CANPacket_t *src, uint16_t new_addr) {
#if defined(STM32H7) || defined(STM32F4)
  CANPacket_t pkt;
  pkt.returned = 0U;
  pkt.rejected = 0U;
  pkt.extended = src->extended;
  pkt.bus = 1;
  pkt.addr = new_addr;
  pkt.data_len_code = src->data_len_code;
  for (int i = 0; i < GET_LEN(src); i++) {
    pkt.data[i] = src->data[i];
  }
  can_set_checksum(&pkt);
  can_send(&pkt, 1, true);
#else
  (void)src;
  (void)new_addr;
#endif
}

static void preap_transform_radar_car_config(const CANPacket_t *src, CANPacket_t *dst) {
  *dst = (CANPacket_t){.returned = 0U, .rejected = 0U, .extended = src->extended,
                       .bus = 1, .addr = 0x2A9, .data_len_code = src->data_len_code};
  uint32_t lo = PREAP_GET_BYTES_04(src);
  uint32_t hi = PREAP_GET_BYTES_48(src);
  lo = (lo & 0xFFFFF33F) | 0x100 | 0x440;  // country=US, radar_type=Bosch
  hi = (hi & 0xCFFF0F0F) | 0x10000000 | (preap_radar_position << 4) | (preap_radar_epas_type << 12);
  // Bosch xWD checks 0x2A9 against the VIN on 0x2B9, not chassis 0x398.
  // Tesla char 8 '2'/'4' is dual motor. This car VIN 5YJSA1E25FF106153 is
  // '2'; GTW still declares 2WD. Honest 2WD then latches xwdValidity and
  // freezes the object table (~12 s). Tinkla 0.6.6 ORed 4WD from that char.
  // Routes 8/9 with the OR kept a live table. Empty donor leaves 0x398 xWD
  // (old A, char 8 '1', 2WD, rock-solid).
  if (preap_radar_donor_active()) {
    const uint8_t drive = preap_radar_vin[7];
    if ((drive == (uint8_t)'2') || (drive == (uint8_t)'4')) {
      lo |= 0x08U;
    }
  }
  PREAP_WORD_TO_BYTES(&dst->data[0], lo);
  PREAP_WORD_TO_BYTES(&dst->data[4], hi);
}

static void preap_transform_radar_vin_feed(const CANPacket_t *src, CANPacket_t *dst) {
  *dst = (CANPacket_t){.returned = 0U, .rejected = 0U, .extended = src->extended,
                       .bus = 1, .addr = 0x2B9, .data_len_code = src->data_len_code};
  uint32_t lo = PREAP_GET_BYTES_04(src);
  uint32_t hi = PREAP_GET_BYTES_48(src);
  if (preap_radar_donor_active() && ((lo & 0x10U) == 0x10U)) {
    const int rec = (int)(lo & 0xFFU);
    if (rec == 0x10) {
      lo = (uint32_t)rec;
      hi = preap_radar_vin_char(0, 1) | preap_radar_vin_char(1, 2) | preap_radar_vin_char(2, 3);
    } else if (rec == 0x11) {
      lo = (uint32_t)rec | preap_radar_vin_char(3, 1) | preap_radar_vin_char(4, 2) | preap_radar_vin_char(5, 3);
      hi = preap_radar_vin_char(6, 0) | preap_radar_vin_char(7, 1) | preap_radar_vin_char(8, 2) | preap_radar_vin_char(9, 3);
    } else if (rec == 0x12) {
      lo = (uint32_t)rec | preap_radar_vin_char(10, 1) | preap_radar_vin_char(11, 2) | preap_radar_vin_char(12, 3);
      hi = preap_radar_vin_char(13, 0) | preap_radar_vin_char(14, 1) | preap_radar_vin_char(15, 2) | preap_radar_vin_char(16, 3);
    }
  }
  PREAP_WORD_TO_BYTES(&dst->data[0], lo);
  PREAP_WORD_TO_BYTES(&dst->data[4], hi);
}

#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
static bool preap_radar_car_config_captured = false;
static CANPacket_t preap_radar_car_config_capture;
static bool preap_radar_vin_feed_captured = false;
static CANPacket_t preap_radar_vin_feed_capture;
#endif

// ============================================
// GTW Emulation: CAN0 → CAN1 for Bosch radar
// ============================================

static void tesla_preap_gtw_emulation(const CANPacket_t *to_fwd) {
  int bus_num = GET_BUS(to_fwd);
  int addr = GET_ADDR(to_fwd);

  if (bus_num == 0 && preap_radar_emulation && preap_radar_ready()) {
    // Group A: Simple re-addresses
    switch (addr) {
      case 0x45:   preap_radar_readdr(to_fwd, 0x219); break;  // STW_ACTN_RQ
      case 0x108:  preap_radar_readdr(to_fwd, 0x109); break;  // DI_torque1
      case 0x145:  preap_radar_readdr(to_fwd, 0x149); break;  // ESP_145h
      case 0x20A:  preap_radar_readdr(to_fwd, 0x159); break;  // BrakeMessage -> ESP_C
      case 0x308:  preap_radar_readdr(to_fwd, 0x209); break;  // GTW_odo
      case 0x30A:  preap_radar_readdr(to_fwd, 0x2D9); break;  // BC_status
      default: break;
    }

    if (addr == 0x405) {
      CANPacket_t vin_pkt;
      preap_transform_radar_vin_feed(to_fwd, &vin_pkt);
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
      preap_radar_vin_feed_capture = vin_pkt;
      preap_radar_vin_feed_captured = true;
#endif
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&vin_pkt);
      can_send(&vin_pkt, 1, true);
#endif
    }

    // Group B: GTW_carConfig (0x398) → 0x2A9 with bitfield patching
    if (addr == 0x398) {
      CANPacket_t pkt;
      preap_transform_radar_car_config(to_fwd, &pkt);
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
      preap_radar_car_config_capture = pkt;
      preap_radar_car_config_captured = true;
#endif
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group B: STW_ANGLHP_STAT (0x0E) → 0x199 with SNA replacement
    if (addr == 0x0E) {
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = to_fwd->extended,
                         .bus = 1, .addr = 0x199, .data_len_code = to_fwd->data_len_code};
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      uint32_t hi = PREAP_GET_BYTES_48(to_fwd);
      if (((lo >> 16) & 0xFF3F) == 0xFF3F) {
        lo = (lo & 0x00C0FFFF) | (0x0020 << 16);
        hi = (hi & 0x00FFFFF0) | 0x00000004;  // force DELPHI sensor ID
        int crc = preap_compute_crc8(lo, hi, 7);
        hi = hi | ((uint32_t)crc << 24);
      }
      PREAP_WORD_TO_BYTES(&pkt.data[0], lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], hi);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group C: ESP_115h (0x115) → 0x129 + synthetic DI_espControl (0x1A9)
    if (addr == 0x115) {
      preap_radar_readdr(to_fwd, 0x129);
      uint32_t hi_src = PREAP_GET_BYTES_48(to_fwd);
      int counter = ((hi_src & 0xF0) >> 4) & 0x0F;
      uint32_t syn_lo = 0x000C0000U | ((uint32_t)counter << 28);
      int cksm = (0x38 + 0x0C + (counter << 4)) & 0xFF;
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x1A9, .data_len_code = 5};
      PREAP_WORD_TO_BYTES(&pkt.data[0], syn_lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], (uint32_t)cksm);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group C: DI_torque2 (0x118) → 0x119 + synthetic ESP_wheelSpeeds (0x169)
    if (addr == 0x118) {
      preap_radar_readdr(to_fwd, 0x119);
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      int ws_counter = PREAP_GET_BYTES_48(to_fwd) & 0x0F;
      int raw_speed = (int)((0xFFF0000U & lo) >> 16);
      int speed;
      if (raw_speed == 0xFFF) {
        speed = 0x1FFF;
      } else {
        int mph_x100 = raw_speed * 5 - 2500;
        int kph_x100 = mph_x100 * 1609 / 1000;
        speed = (kph_x100 < 0) ? 0 : ((kph_x100 / 4) & 0x1FFF);
      }
      uint32_t ws_lo = (uint32_t)(speed | (speed << 13) | (speed << 26));
      uint32_t ws_hi = (uint32_t)((speed >> 6) | (speed << 7) | (ws_counter << 20)) & 0x00FFFFFFU;
      int ws_cksm = 0x76;
      ws_cksm = (ws_cksm + (int)(ws_lo & 0xFF) + (int)((ws_lo >> 8) & 0xFF) + (int)((ws_lo >> 16) & 0xFF) + (int)((ws_lo >> 24) & 0xFF)) & 0xFF;
      ws_cksm = (ws_cksm + (int)(ws_hi & 0xFF) + (int)((ws_hi >> 8) & 0xFF) + (int)((ws_hi >> 16) & 0xFF)) & 0xFF;
      ws_hi = ws_hi | ((uint32_t)ws_cksm << 24);
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x169, .data_len_code = 8};
      PREAP_WORD_TO_BYTES(&pkt.data[0], ws_lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], ws_hi);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }
  }

  // Radar status tracking (CAN1 → informational only)
  if (bus_num == 1 && preap_radar_emulation) {
    if (addr == 0x631 && preap_radar_status == 0) {
      preap_radar_status = 1;
      preap_last_radar_signal = microsecond_timer_get();
    }
    if (addr == 0x300 && preap_radar_status == 1) {
      preap_radar_status = 2;
      preap_last_radar_signal = microsecond_timer_get();
    }
  }
}

#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
bool tesla_preap_radar_car_config_captured(void) {
  return preap_radar_car_config_captured;
}

uint32_t tesla_preap_radar_car_config_addr(void) {
  return preap_radar_car_config_capture.addr;
}

uint8_t tesla_preap_radar_car_config_bus(void) {
  return preap_radar_car_config_capture.bus;
}

uint8_t tesla_preap_radar_car_config_dlc(void) {
  return preap_radar_car_config_capture.data_len_code;
}

uint8_t tesla_preap_radar_car_config_data(int index) {
  if ((index < 0) || (index >= 8)) {
    return 0U;
  }
  return preap_radar_car_config_capture.data[index];
}

bool tesla_preap_radar_vin_feed_captured(void) {
  return preap_radar_vin_feed_captured;
}

uint8_t tesla_preap_radar_vin_feed_data(int index) {
  if ((index < 0) || (index >= 8)) {
    return 0U;
  }
  return preap_radar_vin_feed_capture.data[index];
}

bool tesla_preap_radar_donor_active_debug(void) {
  return preap_radar_donor_active();
}

bool tesla_preap_radar_ready_debug(void) {
  return preap_radar_ready();
}
#endif

// ============================================
// RX Hook
// ============================================
