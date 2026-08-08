#include "motion_controller.h"

#include <stddef.h>

#define MOTION_PARAMETER_MASK_COMPLETE 0x1fu
#define MOTION_CONTROL_MODE_VELOCITY 3u
#define MOTION_STATE_DISABLED 0u
#define MOTION_STATE_ENABLED 1u
#define MOTION_ARM_WINDOW_MS 10000u
#define MOTION_ENABLE_WAIT_MS 100u
#define MOTION_RUNNING_MS 1000u
#define MOTION_ZERO_HOLD_MS 200u
#define MOTION_REFRESH_MS 10u
#define MOTION_START_FEEDBACK_MAX_AGE_MS 500u
#define MOTION_RUNTIME_FEEDBACK_MAX_AGE_MS 100u
#define MOTION_START_SPEED_LIMIT_MILLIRAD_S 100
#define MOTION_RUNTIME_SPEED_LIMIT_MILLIRAD_S 800
#define MOTION_TEMPERATURE_LIMIT_C 60u

static MotionDecision decision(MotionAction action, MotionEvent event,
                               const char *reason)
{
    return (MotionDecision){
        .action = action,
        .event = event,
        .reason = reason,
    };
}

static bool elapsed_at_least(uint32_t now_ms, uint32_t then_ms,
                             uint32_t interval_ms)
{
    return now_ms - then_ms >= interval_ms;
}

static uint8_t normalize_command(uint8_t command)
{
    if (command >= (uint8_t)'a' && command <= (uint8_t)'z') {
        return command - ((uint8_t)'a' - (uint8_t)'A');
    }
    return command;
}

static const char *start_rejection_reason(const MotionSafetySnapshot *safety)
{
    if (safety->parameter_mask != MOTION_PARAMETER_MASK_COMPLETE) {
        return "PARAMETERS_INCOMPLETE";
    }
    if (safety->control_mode != MOTION_CONTROL_MODE_VELOCITY) {
        return "MODE_NOT_VELOCITY";
    }
    if (!safety->feedback_valid ||
        safety->feedback_age_ms > MOTION_START_FEEDBACK_MAX_AGE_MS) {
        return "FEEDBACK_STALE";
    }
    if (safety->motor_state != MOTION_STATE_DISABLED) {
        return "STATE_NOT_DISABLED";
    }
    if (safety->velocity_millirad_s <= -MOTION_START_SPEED_LIMIT_MILLIRAD_S ||
        safety->velocity_millirad_s >= MOTION_START_SPEED_LIMIT_MILLIRAD_S) {
        return "SPEED_NOT_ZERO";
    }
    if (safety->mos_temperature_c >= MOTION_TEMPERATURE_LIMIT_C ||
        safety->rotor_temperature_c >= MOTION_TEMPERATURE_LIMIT_C) {
        return "TEMPERATURE_LIMIT";
    }
    if (safety->can_warning || safety->can_passive || safety->can_bus_off) {
        return "CAN_NOT_ACTIVE";
    }
    return NULL;
}

static const char *enable_wait_trip_reason(const MotionSafetySnapshot *safety)
{
    if (!safety->feedback_valid ||
        safety->feedback_age_ms > MOTION_RUNTIME_FEEDBACK_MAX_AGE_MS) {
        return "FEEDBACK_STALE";
    }
    if (safety->velocity_millirad_s < -MOTION_RUNTIME_SPEED_LIMIT_MILLIRAD_S ||
        safety->velocity_millirad_s > MOTION_RUNTIME_SPEED_LIMIT_MILLIRAD_S) {
        return "SPEED_LIMIT";
    }
    if (safety->mos_temperature_c >= MOTION_TEMPERATURE_LIMIT_C ||
        safety->rotor_temperature_c >= MOTION_TEMPERATURE_LIMIT_C) {
        return "TEMPERATURE_LIMIT";
    }
    if (safety->can_passive || safety->can_bus_off) {
        return "CAN_NOT_ACTIVE";
    }
    return NULL;
}

static const char *runtime_trip_reason(const MotionSafetySnapshot *safety)
{
    const char *reason = enable_wait_trip_reason(safety);

    if (reason != NULL) {
        return reason;
    }
    if (safety->motor_state != MOTION_STATE_ENABLED) {
        return "STATE_NOT_ENABLED";
    }
    return NULL;
}

static MotionDecision start_fault_shutdown(MotionController *controller,
                                           uint32_t now_ms,
                                           MotionEvent event,
                                           const char *reason)
{
    controller->armed = false;
    controller->state = MOTION_FAULT_ZERO;
    controller->phase_started_ms = now_ms;
    controller->last_action_ms = now_ms;

    /* Zero is returned in the triggering call; the following millisecond disables. */
    controller->state = MOTION_FAULT_DISABLE;
    return decision(MOTION_ACTION_ZERO_VELOCITY, event, reason);
}

void motion_controller_init(MotionController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->state = MOTION_IDLE_DISABLED;
    controller->armed = false;
    controller->armed_at_ms = 0u;
    controller->phase_started_ms = 0u;
    controller->last_action_ms = 0u;
}

MotionDecision motion_controller_command(MotionController *controller,
                                          uint8_t command,
                                          const MotionSafetySnapshot *safety)
{
    const char *reason;

    if (controller == NULL || safety == NULL) {
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "INVALID_ARGUMENT");
    }

    command = normalize_command(command);
    if (command == (uint8_t)'\r' || command == (uint8_t)'\n') {
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "IGNORED");
    }

    switch (command) {
    case 'S':
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_STATUS, NULL);
    case 'A':
        if (controller->state != MOTION_IDLE_DISABLED &&
            controller->state != MOTION_ARMED) {
            return decision(MOTION_ACTION_NONE, MOTION_EVENT_START_REJECTED,
                            "MOTION_ACTIVE");
        }
        controller->state = MOTION_ARMED;
        controller->armed = true;
        controller->armed_at_ms = safety->now_ms;
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_ARMED, NULL);
    case 'G':
        if (controller->armed &&
            elapsed_at_least(safety->now_ms, controller->armed_at_ms,
                             MOTION_ARM_WINDOW_MS)) {
            controller->armed = false;
            controller->state = MOTION_IDLE_DISABLED;
            return decision(MOTION_ACTION_NONE, MOTION_EVENT_ARM_TIMEOUT,
                            "ARM_TIMEOUT");
        }
        if (!controller->armed || controller->state != MOTION_ARMED) {
            return decision(MOTION_ACTION_NONE, MOTION_EVENT_START_REJECTED,
                            "SEND_A_FIRST");
        }
        reason = start_rejection_reason(safety);
        if (reason != NULL) {
            return decision(MOTION_ACTION_NONE, MOTION_EVENT_START_REJECTED,
                            reason);
        }
        controller->armed = false;
        controller->state = MOTION_ENABLE_WAIT;
        controller->phase_started_ms = safety->now_ms;
        controller->last_action_ms = safety->now_ms;
        return decision(MOTION_ACTION_ENABLE, MOTION_EVENT_START_REQUESTED,
                        NULL);
    case 'X':
        return start_fault_shutdown(controller, safety->now_ms,
                                    MOTION_EVENT_EMERGENCY_STOP,
                                    "EMERGENCY_STOP");
    default:
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE,
                        "UNKNOWN_COMMAND");
    }
}

MotionDecision motion_controller_step(MotionController *controller,
                                       const MotionSafetySnapshot *safety)
{
    const char *reason;

    if (controller == NULL || safety == NULL) {
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "INVALID_ARGUMENT");
    }

    switch (controller->state) {
    case MOTION_IDLE_DISABLED:
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL);
    case MOTION_ARMED:
        if (elapsed_at_least(safety->now_ms, controller->armed_at_ms,
                             MOTION_ARM_WINDOW_MS)) {
            controller->armed = false;
            controller->state = MOTION_IDLE_DISABLED;
            return decision(MOTION_ACTION_NONE, MOTION_EVENT_ARM_TIMEOUT,
                            "ARM_TIMEOUT");
        }
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL);
    case MOTION_ENABLE_WAIT:
        if ((!safety->feedback_valid ||
             safety->feedback_age_ms > MOTION_RUNTIME_FEEDBACK_MAX_AGE_MS) &&
            (safety->can_passive || safety->can_bus_off)) {
            return start_fault_shutdown(controller, safety->now_ms,
                                        MOTION_EVENT_SAFETY_TRIP,
                                        "CAN_NOT_ACTIVE");
        }
        if (safety->feedback_valid &&
            safety->feedback_age_ms <= MOTION_RUNTIME_FEEDBACK_MAX_AGE_MS) {
            reason = enable_wait_trip_reason(safety);
            if (reason != NULL) {
                return start_fault_shutdown(controller, safety->now_ms,
                                            MOTION_EVENT_SAFETY_TRIP, reason);
            }
            if (safety->motor_state == MOTION_STATE_ENABLED &&
                safety->now_ms - controller->phase_started_ms <=
                    MOTION_ENABLE_WAIT_MS) {
                controller->state = MOTION_RUNNING;
                controller->phase_started_ms = safety->now_ms;
                controller->last_action_ms = safety->now_ms;
                return decision(MOTION_ACTION_POSITIVE_VELOCITY,
                                MOTION_EVENT_RUNNING, NULL);
            }
            if (safety->motor_state != MOTION_STATE_DISABLED) {
                return start_fault_shutdown(controller, safety->now_ms,
                                            MOTION_EVENT_SAFETY_TRIP,
                                            "STATE_NOT_ENABLED");
            }
        }
        if (elapsed_at_least(safety->now_ms, controller->phase_started_ms,
                             MOTION_ENABLE_WAIT_MS)) {
            reason = !safety->feedback_valid ||
                             safety->feedback_age_ms >
                                 MOTION_RUNTIME_FEEDBACK_MAX_AGE_MS
                         ? "FEEDBACK_STALE"
                         : "STATE_NOT_ENABLED";
            return start_fault_shutdown(controller, safety->now_ms,
                                        MOTION_EVENT_SAFETY_TRIP, reason);
        }
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL);
    case MOTION_RUNNING:
        reason = runtime_trip_reason(safety);
        if (reason != NULL) {
            return start_fault_shutdown(controller, safety->now_ms,
                                        MOTION_EVENT_SAFETY_TRIP, reason);
        }
        if (elapsed_at_least(safety->now_ms, controller->phase_started_ms,
                             MOTION_RUNNING_MS)) {
            controller->state = MOTION_ZERO_HOLD;
            controller->phase_started_ms = safety->now_ms;
            controller->last_action_ms = safety->now_ms;
            return decision(MOTION_ACTION_ZERO_VELOCITY,
                            MOTION_EVENT_ZERO_HOLD, NULL);
        }
        if (elapsed_at_least(safety->now_ms, controller->last_action_ms,
                             MOTION_REFRESH_MS)) {
            controller->last_action_ms = safety->now_ms;
            return decision(MOTION_ACTION_POSITIVE_VELOCITY,
                            MOTION_EVENT_NONE, NULL);
        }
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL);
    case MOTION_ZERO_HOLD:
        reason = runtime_trip_reason(safety);
        if (reason != NULL) {
            return start_fault_shutdown(controller, safety->now_ms,
                                        MOTION_EVENT_SAFETY_TRIP, reason);
        }
        if (elapsed_at_least(safety->now_ms, controller->phase_started_ms,
                             MOTION_ZERO_HOLD_MS)) {
            controller->state = MOTION_IDLE_DISABLED;
            controller->last_action_ms = safety->now_ms;
            return decision(MOTION_ACTION_DISABLE, MOTION_EVENT_COMPLETE,
                            NULL);
        }
        if (elapsed_at_least(safety->now_ms, controller->last_action_ms,
                             MOTION_REFRESH_MS)) {
            controller->last_action_ms = safety->now_ms;
            return decision(MOTION_ACTION_ZERO_VELOCITY, MOTION_EVENT_NONE,
                            NULL);
        }
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL);
    case MOTION_FAULT_ZERO:
        controller->state = MOTION_FAULT_DISABLE;
        controller->last_action_ms = safety->now_ms;
        return decision(MOTION_ACTION_ZERO_VELOCITY, MOTION_EVENT_SAFETY_TRIP,
                        "FAULT_STOP");
    case MOTION_FAULT_DISABLE:
        if (elapsed_at_least(safety->now_ms, controller->last_action_ms, 1u)) {
            controller->state = MOTION_IDLE_DISABLED;
            return decision(MOTION_ACTION_DISABLE, MOTION_EVENT_NONE, NULL);
        }
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL);
    default:
        motion_controller_init(controller);
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "INVALID_STATE");
    }
}

MotionDecision motion_controller_tx_failed(MotionController *controller)
{
    if (controller == NULL) {
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "INVALID_ARGUMENT");
    }

    if (controller->state == MOTION_FAULT_DISABLE) {
        controller->state = MOTION_IDLE_DISABLED;
        controller->armed = false;
        return decision(MOTION_ACTION_DISABLE, MOTION_EVENT_TX_FAILURE,
                        "TX_FAILURE");
    }

    return start_fault_shutdown(controller, controller->last_action_ms,
                                MOTION_EVENT_TX_FAILURE, "TX_FAILURE");
}
