#pragma once

// Single source of record for BMW E8x/E9x panda safety. This file is NOT
// compiled in place: safety/build_firmware.sh injects it into a stock
// opendbc/panda firmware workspace (default ~/openpilot) as
// opendbc/safety/modes/bmw.h, builds and tests there, then restores the tree —
// no opendbc fork is maintained. The compat shims below let the same file
// build against both the current opendbc layout and the firmware workspace's
// safety_declarations.h layout.
#if __has_include("opendbc/safety/declarations.h")
#include "opendbc/safety/declarations.h"
#else
#include "../safety_declarations.h"
#define BMW_FIRMWARE_TREE 1
#endif
#ifndef SAFETY_UNUSED
#define SAFETY_UNUSED(x) UNUSED(x)
#endif

// CAN msgs we care about
#define BMW_EngineAndBrake 0xA8U
#define BMW_AccPedal 0xAAU
#define BMW_Speed 0x1A0U
#define BMW_SteeringWheelAngle_slow 0xC8U
#define BMW_CruiseControlStatus 0x200U
#define BMW_DynamicCruiseControlStatus 0x193U
#define BMW_CruiseControlStalk 0x194U
#define BMW_TransmissionDataDisplay 0x1D2U

// BMW Stepper Servo CAN Messages
#define STEPPER_STEERING_COMMAND 0x22eU
#define STEPPER_STEERING_STATUS 0x22fU

// BMW UDS Diagnostic Messages
#define BMW_UDS_REQUEST_DME 0x7E0U      // UDS request to DME (Engine Control)
#define BMW_UDS_RESPONSE_DME 0x7E8U     // UDS response from DME
#define BMW_UDS_FUNCTIONAL_REQUEST 0x7DFU  // UDS functional request
#define BMW_DIAGNOSTIC_RESPONSE 0x612U  // BMW diagnostic response channel

#define BMW_PT_CAN 0U
#define BMW_F_CAN 1U
#define BMW_AUX_CAN 2U

#define CAN_BMW_SPEED_FAC 0.1
#define CAN_BMW_ACC_FAC 0.025
#define CAN_ACTUATOR_TQ_FAC 0.125

static float bmw_speed = 0.0f;
static bool bmw_cancel_prev = false;


static void bmw_rx_hook(const CANPacket_t *msg) {
  int addr = msg->addr;
  int bus = msg->bus;

  // LKA mode (two-stage disengagement): DCC engaging latches controls_allowed;
  // DCC dropping does NOT clear it — lateral stays live while the driver owns
  // gas/brake. Panda-side disengage paths: stage-2 cancel below, torque limits,
  // and openpilot commanding steer mode 0. cruise_engaged_prev is the library
  // global (kept for test introspection); pcm_cruise_check is deliberately not
  // used because it clears controls_allowed on cruise disengage.
  if (addr == BMW_DynamicCruiseControlStatus) { // VO544
    bool cruise_engaged = (((msg->data[5] >> 3) & 0x1U) == 1U);
    if (cruise_engaged && !cruise_engaged_prev) {
      controls_allowed = true;
    }
    cruise_engaged_prev = cruise_engaged;
  } else if (addr == BMW_CruiseControlStatus) { // VO540
    bool cruise_engaged = (((msg->data[1] >> 5) & 0x1U) == 1U);
    if (cruise_engaged && !cruise_engaged_prev) {
      controls_allowed = true;
    }
    cruise_engaged_prev = cruise_engaged;
  }

  // Stage-2 mirror: a human cancel press while DCC is already off fully
  // disengages, independent of openpilot. F-CAN only: human stalk presses are
  // rx'd on bus 1, our own TX'd stalk emulation is never self-received, and
  // the PT-CAN gateway echo of our TX (bus 0) must not count as human.
  // 0x194 must be in bmw_rx_checks — safety_rx_hook only forwards whitelisted
  // addresses to this hook. Note: DCC status is 5 Hz, so a double-tap cancel
  // faster than 200 ms reads as one stage-1 press. NCC (PT-CAN stalk) cars
  // rely on the openpilot-side stage 2.
  if ((addr == BMW_CruiseControlStalk) && (bus == (int)BMW_F_CAN)) {
    bool cancel_pressed = (msg->data[2] & 0x10U) != 0U;
    if (cancel_pressed && !bmw_cancel_prev && !cruise_engaged_prev) {
      controls_allowed = false;
    }
    bmw_cancel_prev = cancel_pressed;
  }

  // BMW TransmissionDataDisplay not needed for safety
  // BMW's own cruise control system handles gear position requirements

  // get vehicle speed
  if (addr == BMW_Speed) {
    uint32_t speed_raw = (msg->data[0] << 8) | msg->data[1];  // Get bytes 0-1
    // raw to km/h to m/s
    bmw_speed = to_signed(speed_raw & 0xFFFU, 12) * CAN_BMW_SPEED_FAC * KPH_TO_MS;

    // check moving forward and reverse
    vehicle_moving = (msg->data[1] & 0x30U) != 0U;
  }

  // STEPPER_SERVO_CAN: get STEERING_STATUS
  if ((addr == STEPPER_STEERING_STATUS) && ((bus == (int)BMW_F_CAN) || (bus == (int)BMW_AUX_CAN))) {
    int8_t torque_meas_new = (int8_t)(msg->data[2]); // torque raw
    update_sample(&torque_meas, torque_meas_new);

    // SOFT_OFF status is monitored by openpilot for UI warnings
    // Stepper servo auto-recovers on next command - no panda intervention needed
  }

  // BMW E90 uses torque-controlled steering only - no angle monitoring needed

  // LKA mode: brake_pressed is deliberately NOT reported. The common safety
  // code hard-disengages on brake rising edge (no alt-experience flag exists
  // for brake), which would kill lateral on every brake tap — brake means
  // "drop to LKA" (openpilot-side), factory-LKA semantics. Trade-off accepted
  // 2026-08-14; driver override is covered by the torque limits + stepper
  // override detection. (BMW_EngineAndBrake data[7] & 0x20 if ever needed.)

  if (addr == BMW_AccPedal) {
    gas_pressed = (msg->data[6] & 0x30U) != 0U;
  }

  // BMW E8x/E9x dummy states for generic_rx_checks() compatibility
  // No steering torque sensor for override detection
  steering_disengage = false;

  // No regen braking paddle in BMW E8x/E9x
  regen_braking = false;

}

static bool bmw_tx_hook(const CANPacket_t *msg) {
  int addr = msg->addr;

  // UDS diagnostic request validation - ALLOW even when controls_allowed=false
  if ((addr == (int)BMW_UDS_REQUEST_DME) || (addr == (int)BMW_UDS_FUNCTIONAL_REQUEST)) {
    // Check for UDS Service 0x14 (Clear Diagnostic Information)
    if ((GET_LEN(msg) >= 2U) && (msg->data[1] == 0x14U)) {
      // BMW DTC clearing safety requirement: ignition ON and vehicle stationary
#ifdef BMW_FIRMWARE_TREE
      bool ignition_on = ignition_can;  // CAN-based ignition detection (firmware-only global)
#else
      bool ignition_on = true;          // no ignition_can in this tree; stationary gate still applies
#endif
      bool vehicle_safe = !vehicle_moving && (bmw_speed < 1.0f);   // Vehicle stationary

      if (!ignition_on || !vehicle_safe) {
        return false;  // Block unsafe DTC clear attempts
      }
    }
    // Allow all UDS diagnostic operations (0x14 Clear, 0x19 Read, 0x22 Read Data, etc.)
    // even when controls_allowed=false (engine OFF scenario)
    return true;
  }

  const TorqueSteeringLimits STEPPER_SERVO_LIMITS = {
    .max_torque = (12.f / CAN_ACTUATOR_TQ_FAC),     // < 12Nm
    .dynamic_max_torque = true,
    .max_torque_lookup = {
      {15., 22., 28.},    // m/s: 54kph, 80kph, 100kph
      {96, 64, 32},       // CAN units: 12Nm, 8Nm, 4Nm (full torque up to 54kph)
    },
    .max_rate_up = 2,                               // <= 0.125Nm/10ms
    // 1.0 Nm/10ms matches the field-proven flashed firmware. The controller's
    // STEER_DELTA_DOWN is 0.2; tightening the panda mirror to 0.2 is a
    // candidate but needs on-car validation of the disengage/SoftOff torque
    // decay path first.
    .max_rate_down = (1.0f / CAN_ACTUATOR_TQ_FAC),  // < 1Nm/10ms
    .max_rt_delta = (25.0f / CAN_ACTUATOR_TQ_FAC),  // 25Nm/250ms
    .max_torque_error = (1.0f / CAN_ACTUATOR_TQ_FAC),  // 1Nm
    .type = TorqueMotorLimited,
  };

  bool tx = true;

  // STEPPER_SERVO_CAN: BMW E90 torque control only
  if (addr == STEPPER_STEERING_COMMAND) {
    // Torque Control Mode:
    uint8_t steer_mode = (msg->data[1] >> 4) & 0b11u;
    if (steer_mode != 0x0U) {
      int8_t steer_torque = (int8_t)(msg->data[4]); // Nm / CAN_ACTUATOR_TQ_FAC

      // BMW stepper servo: treat any non-zero torque as steer request
      int steer_req = (steer_torque != 0) ? 1 : 0;
      if (steer_torque_cmd_checks(steer_torque, steer_req, STEPPER_SERVO_LIMITS)) {
        tx = false;
      }
    }
    // Always allow mode 0 (disabled) commands for safety
  }

  return tx;
}

static safety_config bmw_init(uint16_t param) {
  SAFETY_UNUSED(param);

  static RxCheck bmw_rx_checks[] = {
    // Core safety: brake, gas, speed on Bus 0, steering torque on Bus 1 (same pattern as Toyota 0xaa, 0x260, 0x1D2, 0x226)
    {.msg = {{BMW_EngineAndBrake, BMW_PT_CAN, 8, .frequency = 100U,
              .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_AccPedal, BMW_PT_CAN, 8, .frequency = 100U,
              .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_Speed, BMW_PT_CAN, 8, .frequency = 50U,
              .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_DynamicCruiseControlStatus, BMW_PT_CAN, 8, .frequency = 5U,
              .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {BMW_CruiseControlStatus, BMW_PT_CAN, 8, .frequency = 5U,
              .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             { 0 }}},
    // Cruise stalk: needed for the LKA stage-2 cancel (safety_rx_hook only
    // forwards whitelisted addresses to the mode rx hook). SZL sends 0x194 at
    // 5 Hz idle continuously. F-CAN on DCC cars, PT-CAN on NCC cars —
    // alternatives so either satisfies the check.
    {.msg = {{BMW_CruiseControlStalk, BMW_F_CAN, 4, .frequency = 5U,
              .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {BMW_CruiseControlStalk, BMW_PT_CAN, 4, .frequency = 5U,
              .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             { 0 }}},
    {.msg = {{STEPPER_STEERING_STATUS,  BMW_F_CAN, 8, .ignore_counter = true, .frequency = 100U,
              .ignore_quality_flag = true, .ignore_checksum = true},
             {STEPPER_STEERING_STATUS,  BMW_AUX_CAN, 8, .ignore_counter = true, .frequency = 100U,
              .ignore_quality_flag = true, .ignore_checksum = true},
             { 0 }}},
  };

  // TX_MSGS configuration - allowed outgoing CAN messages
  static const CanMsg BMW_TX_MSGS[] = {
    {BMW_CruiseControlStalk, BMW_PT_CAN, 4, .check_relay = false}, // Normal cruise control send status on PT-CAN
    {BMW_CruiseControlStalk, BMW_F_CAN, 4, .check_relay = false}, // Dynamic cruise control send status on F-CAN
    {STEPPER_STEERING_COMMAND, BMW_F_CAN, 5, .check_relay = false}, // STEPPER_SERVO_CAN is allowed on F-CAN network
    {STEPPER_STEERING_COMMAND, BMW_AUX_CAN, 5, .check_relay = false},  // or an standalone network
    {BMW_UDS_REQUEST_DME, BMW_PT_CAN, 8, .check_relay = false}, // UDS diagnostic requests to DME
    {BMW_UDS_FUNCTIONAL_REQUEST, BMW_PT_CAN, 8, .check_relay = false}, // UDS functional requests
  };

  bmw_speed = 0.0f;
  bmw_cancel_prev = false;
  cruise_engaged_prev = false;

  safety_config ret = BUILD_SAFETY_CFG(bmw_rx_checks, BMW_TX_MSGS);
  ret.disable_forwarding = true;

  return ret;
}

const safety_hooks bmw_hooks = {
  .init = bmw_init,
  .rx = bmw_rx_hook,
  .tx = bmw_tx_hook,
  .fwd = NULL,
  .get_counter = NULL,
  .get_checksum = NULL,
  .compute_checksum = NULL,
  .get_quality_flag_valid = NULL,
};
