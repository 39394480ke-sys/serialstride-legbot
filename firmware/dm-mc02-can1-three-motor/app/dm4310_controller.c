#include "dm4310_controller.h"

#include <stddef.h>

#define ARM_WINDOW_MS 10000u
#define ENABLE_WAIT_MS 100u
#define ZERO_SPEED_TEST_MS 500u
#define MOTION_PULSE_MS 2000u
#define ZERO_HOLD_MS 200u
#define REFRESH_MS 10u
#define FEEDBACK_MAX_AGE_MS 100u
#define START_SPEED_LIMIT_MILLIRAD_S 100
#define RUNTIME_SPEED_LIMIT_MILLIRAD_S 800
#define RUNTIME_TORQUE_LIMIT_MILLINEWTON_M 500
#define TEMPERATURE_LIMIT_C 60u

static Dm4310MotionDecision decision(Dm4310MotionEvent event,
                                     const char *reason)
{
    Dm4310MotionDecision result = {0};

    result.event = event;
    result.reason = reason;
    return result;
}

static bool elapsed(uint32_t now, uint32_t then, uint32_t interval)
{
    return (uint32_t)(now - then) >= interval;
}

static uint8_t normalize(uint8_t command)
{
    return command >= (uint8_t)'a' && command <= (uint8_t)'z'
               ? (uint8_t)(command - ((uint8_t)'a' - (uint8_t)'A'))
               : command;
}

static int32_t absolute(int32_t value)
{
    return value < 0 ? -value : value;
}

static const char *start_rejection(const Dm4310SafetySnapshot *safety)
{
    if (!safety->powered) return "POWER_OFF";
    if (!safety->probe_active) return "SEND_R_FIRST";
    if (!safety->parameters_valid) return "PARAMETERS_MISMATCH";
    if (!safety->feedback_valid ||
        safety->feedback_age_ms > FEEDBACK_MAX_AGE_MS) return "FEEDBACK_STALE";
    if (safety->motor_state != 0u) return "STATE_NOT_DISABLED";
    if (absolute(safety->velocity_millirad_s) >=
        START_SPEED_LIMIT_MILLIRAD_S) return "SPEED_NOT_ZERO";
    if (absolute(safety->torque_millinewton_m) >
        RUNTIME_TORQUE_LIMIT_MILLINEWTON_M) return "TORQUE_LIMIT";
    if (safety->mos_temperature_c >= TEMPERATURE_LIMIT_C ||
        safety->rotor_temperature_c >= TEMPERATURE_LIMIT_C)
        return "TEMPERATURE_LIMIT";
    if (safety->can_warning || safety->can_passive || safety->can_bus_off)
        return "CAN_NOT_ACTIVE";
    return NULL;
}

static const char *runtime_trip(const Dm4310SafetySnapshot *safety,
                                bool require_enabled)
{
    if (!safety->feedback_valid ||
        safety->feedback_age_ms > FEEDBACK_MAX_AGE_MS) return "FEEDBACK_STALE";
    if (absolute(safety->velocity_millirad_s) >
        RUNTIME_SPEED_LIMIT_MILLIRAD_S) return "SPEED_LIMIT";
    if (absolute(safety->torque_millinewton_m) >
        RUNTIME_TORQUE_LIMIT_MILLINEWTON_M) return "TORQUE_LIMIT";
    if (safety->mos_temperature_c >= TEMPERATURE_LIMIT_C ||
        safety->rotor_temperature_c >= TEMPERATURE_LIMIT_C)
        return "TEMPERATURE_LIMIT";
    if (safety->can_warning || safety->can_passive || safety->can_bus_off)
        return "CAN_NOT_ACTIVE";
    if (require_enabled && safety->motor_state != 1u)
        return "STATE_NOT_ENABLED";
    return NULL;
}

static Dm4310MotionDecision fail_closed(Dm4310Controller *controller,
                                        Dm4310MotionEvent event,
                                        const char *reason)
{
    Dm4310MotionDecision result = decision(event, reason);

    controller->state = DM4310_MOTION_DISABLED;
    controller->target_velocity_millirad_s = 0;
    result.send_mit = true;
    result.send_disable = true;
    result.cut_power = true;
    return result;
}

void dm4310_controller_init(Dm4310Controller *controller)
{
    if (controller != NULL) {
        *controller = (Dm4310Controller){.state = DM4310_MOTION_DISABLED};
    }
}

Dm4310MotionDecision dm4310_controller_command(
    Dm4310Controller *controller, uint8_t command,
    const Dm4310SafetySnapshot *safety)
{
    Dm4310MotionDecision result = decision(DM4310_EVENT_NONE, NULL);
    const char *reason;

    if (controller == NULL || safety == NULL) {
        return decision(DM4310_EVENT_REJECTED, "INVALID_ARGUMENT");
    }
    command = normalize(command);
    if (command == (uint8_t)'\r' || command == (uint8_t)'\n') return result;
    if (command == (uint8_t)'X') {
        return fail_closed(controller, DM4310_EVENT_EMERGENCY_STOP,
                           "EMERGENCY_STOP");
    }
    if (command == (uint8_t)'S') return decision(DM4310_EVENT_STATUS, NULL);
    if (command == (uint8_t)'A') {
        if (controller->state != DM4310_MOTION_DISABLED &&
            controller->state != DM4310_MOTION_ARMED)
            return decision(DM4310_EVENT_REJECTED, "MOTION_ACTIVE");
        controller->state = DM4310_MOTION_ARMED;
        controller->armed_at_ms = safety->now_ms;
        return decision(DM4310_EVENT_ARMED, NULL);
    }
    if (command != (uint8_t)'N' && command != (uint8_t)'G' &&
        command != (uint8_t)'B') {
        return decision(DM4310_EVENT_REJECTED, "UNKNOWN_COMMAND");
    }
    if (controller->state == DM4310_MOTION_ARMED &&
        elapsed(safety->now_ms, controller->armed_at_ms, ARM_WINDOW_MS)) {
        controller->state = DM4310_MOTION_DISABLED;
        return decision(DM4310_EVENT_REJECTED, "ARM_TIMEOUT");
    }
    if (controller->state != DM4310_MOTION_ARMED)
        return decision(DM4310_EVENT_REJECTED, "SEND_A_FIRST");
    reason = start_rejection(safety);
    if (reason != NULL) return decision(DM4310_EVENT_REJECTED, reason);

    controller->state = DM4310_MOTION_ENABLE_WAIT;
    controller->target_velocity_millirad_s =
        command == (uint8_t)'G' ? 200 : command == (uint8_t)'B' ? -200 : 0;
    controller->pulse_duration_ms =
        command == (uint8_t)'N' ? ZERO_SPEED_TEST_MS : MOTION_PULSE_MS;
    controller->phase_started_ms = safety->now_ms;
    controller->last_command_ms = safety->now_ms;
    result = decision(DM4310_EVENT_ENABLE_REQUESTED, NULL);
    result.send_enable = true;
    result.target_velocity_millirad_s =
        controller->target_velocity_millirad_s;
    return result;
}

Dm4310MotionDecision dm4310_controller_step(
    Dm4310Controller *controller, const Dm4310SafetySnapshot *safety)
{
    Dm4310MotionDecision result = decision(DM4310_EVENT_NONE, NULL);
    const char *reason;

    if (controller == NULL || safety == NULL)
        return decision(DM4310_EVENT_REJECTED, "INVALID_ARGUMENT");
    if (controller->state == DM4310_MOTION_ARMED) {
        if (elapsed(safety->now_ms, controller->armed_at_ms, ARM_WINDOW_MS)) {
            controller->state = DM4310_MOTION_DISABLED;
            return decision(DM4310_EVENT_ARM_TIMEOUT, "ARM_TIMEOUT");
        }
        return result;
    }
    if (controller->state == DM4310_MOTION_ENABLE_WAIT) {
        if (safety->feedback_valid &&
            safety->feedback_age_ms <= FEEDBACK_MAX_AGE_MS) {
            reason = runtime_trip(safety, false);
            if (reason != NULL)
                return fail_closed(controller, DM4310_EVENT_SAFETY_TRIP,
                                   reason);
            if (safety->motor_state == 1u &&
                (uint32_t)(safety->now_ms - controller->phase_started_ms) <=
                    ENABLE_WAIT_MS) {
                controller->state = DM4310_MOTION_PULSE;
                controller->phase_started_ms = safety->now_ms;
                controller->last_command_ms = safety->now_ms;
                result = decision(DM4310_EVENT_RUNNING, NULL);
                result.send_mit = true;
                result.target_velocity_millirad_s =
                    controller->target_velocity_millirad_s;
                return result;
            }
            if (safety->motor_state != 0u && safety->motor_state != 1u)
                return fail_closed(controller, DM4310_EVENT_SAFETY_TRIP,
                                   "STATE_NOT_ENABLED");
        }
        if ((uint32_t)(safety->now_ms - controller->phase_started_ms) >
            ENABLE_WAIT_MS)
            return fail_closed(controller, DM4310_EVENT_SAFETY_TRIP,
                               "ENABLE_TIMEOUT");
        return result;
    }
    if (controller->state == DM4310_MOTION_PULSE) {
        reason = runtime_trip(safety, true);
        if (reason != NULL)
            return fail_closed(controller, DM4310_EVENT_SAFETY_TRIP, reason);
        if (elapsed(safety->now_ms, controller->phase_started_ms,
                    controller->pulse_duration_ms)) {
            controller->state = DM4310_MOTION_ZERO_HOLD;
            controller->target_velocity_millirad_s = 0;
            controller->phase_started_ms = safety->now_ms;
            controller->last_command_ms = safety->now_ms;
            result = decision(DM4310_EVENT_ZERO_HOLD, NULL);
            result.send_mit = true;
            return result;
        }
        if (elapsed(safety->now_ms, controller->last_command_ms, REFRESH_MS)) {
            controller->last_command_ms = safety->now_ms;
            result.send_mit = true;
            result.target_velocity_millirad_s =
                controller->target_velocity_millirad_s;
        }
        return result;
    }
    if (controller->state == DM4310_MOTION_ZERO_HOLD) {
        reason = runtime_trip(safety, true);
        if (reason != NULL)
            return fail_closed(controller, DM4310_EVENT_SAFETY_TRIP, reason);
        if (elapsed(safety->now_ms, controller->phase_started_ms,
                    ZERO_HOLD_MS)) {
            controller->state = DM4310_MOTION_DISABLED;
            result = decision(DM4310_EVENT_COMPLETE, NULL);
            result.send_disable = true;
            return result;
        }
        if (elapsed(safety->now_ms, controller->last_command_ms, REFRESH_MS)) {
            controller->last_command_ms = safety->now_ms;
            result.send_mit = true;
        }
    }
    return result;
}

Dm4310MotionDecision dm4310_controller_tx_failed(
    Dm4310Controller *controller)
{
    if (controller == NULL)
        return decision(DM4310_EVENT_TX_FAILURE, "INVALID_ARGUMENT");
    return fail_closed(controller, DM4310_EVENT_TX_FAILURE, "CAN_TX_FAILED");
}

const char *dm4310_motion_state_name(Dm4310MotionState state)
{
    switch (state) {
    case DM4310_MOTION_DISABLED: return "DISABLED";
    case DM4310_MOTION_ARMED: return "ARMED";
    case DM4310_MOTION_ENABLE_WAIT: return "ENABLE_WAIT";
    case DM4310_MOTION_PULSE: return "PULSE";
    case DM4310_MOTION_ZERO_HOLD: return "ZERO_HOLD";
    default: return "UNKNOWN";
    }
}
