// c3_compat: health packet of the comma three's internal panda (STM32F4 "dos").
//
// The BMW plugin's F4 firmware (panda a0848226) answers USB 0xd2 with health
// packet v16: voltage/current as uint32, every boolean its own byte, a float
// interrupt load. openpilot 0.11.2's health_t (panda a5919516) is 48 bytes with
// uint16 voltage/current and a flags_pkt bitfield, so reading the F4's 58 bytes
// into it shifts every field after uptime: ignition came out of faults_pkt (always
// 0), safety mode out of faults' top byte, and the car never went onroad. boot
// §16c installs this next to panda.cc, which reads health_f4_t on a dos panda and
// translates it here.
#pragma once
#include <cstdint>

struct __attribute__((packed)) health_f4_t {
  uint32_t uptime_pkt;
  uint32_t voltage_pkt;
  uint32_t current_pkt;
  uint32_t safety_tx_blocked_pkt;
  uint32_t safety_rx_invalid_pkt;
  uint32_t tx_buffer_overflow_pkt;
  uint32_t rx_buffer_overflow_pkt;
  uint32_t faults_pkt;
  uint8_t ignition_line_pkt;
  uint8_t ignition_can_pkt;
  uint8_t controls_allowed_pkt;
  uint8_t car_harness_status_pkt;
  uint8_t safety_mode_pkt;
  uint16_t safety_param_pkt;
  uint8_t fault_status_pkt;
  uint8_t power_save_enabled_pkt;
  uint8_t heartbeat_lost_pkt;
  uint16_t alternative_experience_pkt;
  float interrupt_load_pkt;
  uint8_t fan_power;
  uint8_t safety_rx_checks_invalid_pkt;
  uint16_t spi_error_count_pkt;
  uint8_t fan_stall_count;
  uint16_t sbu1_voltage_mV;
  uint16_t sbu2_voltage_mV;
  uint8_t som_reset_triggered;
};
static_assert(sizeof(health_f4_t) == 58, "health v16 is 58 bytes");

// bit positions of openpilot 0.11.2's HEALTH_FLAG_* (panda/board/health.h)
enum : uint16_t {
  HEALTH_F4_FLAG_IGNITION_LINE = 1U << 0,
  HEALTH_F4_FLAG_IGNITION_CAN = 1U << 1,
  HEALTH_F4_FLAG_CONTROLS_ALLOWED = 1U << 2,
  HEALTH_F4_FLAG_POWER_SAVE_ENABLED = 1U << 3,
  HEALTH_F4_FLAG_HEARTBEAT_LOST = 1U << 4,
  HEALTH_F4_FLAG_SAFETY_RX_CHECKS_INVALID = 1U << 5,
  HEALTH_F4_FLAG_SOM_RESET_TRIGGERED = 1U << 6,
};

template <typename H>
inline void health_from_f4(H &h, const health_f4_t &v) {
  auto u16 = [](uint32_t x) -> uint16_t { return x > 0xFFFFU ? 0xFFFFU : (uint16_t)x; };
  h.uptime_pkt = v.uptime_pkt;
  h.voltage_pkt = u16(v.voltage_pkt);
  h.current_pkt = u16(v.current_pkt);
  h.safety_tx_blocked_pkt = v.safety_tx_blocked_pkt;
  h.safety_rx_invalid_pkt = v.safety_rx_invalid_pkt;
  h.tx_buffer_overflow_pkt = v.tx_buffer_overflow_pkt;
  h.rx_buffer_overflow_pkt = v.rx_buffer_overflow_pkt;
  h.faults_pkt = v.faults_pkt;
  h.flags_pkt = (v.ignition_line_pkt ? HEALTH_F4_FLAG_IGNITION_LINE : 0) |
                (v.ignition_can_pkt ? HEALTH_F4_FLAG_IGNITION_CAN : 0) |
                (v.controls_allowed_pkt ? HEALTH_F4_FLAG_CONTROLS_ALLOWED : 0) |
                (v.power_save_enabled_pkt ? HEALTH_F4_FLAG_POWER_SAVE_ENABLED : 0) |
                (v.heartbeat_lost_pkt ? HEALTH_F4_FLAG_HEARTBEAT_LOST : 0) |
                (v.safety_rx_checks_invalid_pkt ? HEALTH_F4_FLAG_SAFETY_RX_CHECKS_INVALID : 0) |
                (v.som_reset_triggered ? HEALTH_F4_FLAG_SOM_RESET_TRIGGERED : 0);
  h.car_harness_status_pkt = v.car_harness_status_pkt;
  h.safety_mode_pkt = v.safety_mode_pkt;
  h.safety_param_pkt = v.safety_param_pkt;
  h.fault_status_pkt = v.fault_status_pkt;
  h.alternative_experience_pkt = v.alternative_experience_pkt;
  float load = v.interrupt_load_pkt * 255.0f;   // v16: fraction; 0.11.2: uint8 that pandad divides by 255
  h.interrupt_load_pkt = load <= 0.0f ? 0 : (load >= 255.0f ? 255 : (uint8_t)(load + 0.5f));
  h.fan_power = v.fan_power;
  h.spi_error_count_pkt = v.spi_error_count_pkt;
  h.sbu1_voltage_mV = v.sbu1_voltage_mV;
  h.sbu2_voltage_mV = v.sbu2_voltage_mV;
  h.sound_output_level_pkt = 0;   // no amplifier telemetry on the F4
  h.temperature_pkt = 0;
}
