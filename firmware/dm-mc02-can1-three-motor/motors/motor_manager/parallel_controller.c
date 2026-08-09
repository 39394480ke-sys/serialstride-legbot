#include "parallel_controller.h"

#include <stddef.h>

#define ARM_WINDOW_MS 10000u
#define ENABLE_WAIT_MS 100u
#define PULSE_MS 1000u
#define ZERO_HOLD_MS 200u
#define DISABLE_WAIT_MS 200u
#define REFRESH_MS 10u

static ParallelDecision decision(ParallelEvent event, const char *reason)
{
    ParallelDecision result = {0};

    result.event = event;
    result.reason = reason;
    return result;
}

static bool elapsed(uint32_t now, uint32_t then, uint32_t interval)
{
    return (uint32_t)(now - then) >= interval;
}

static const char *start_rejection(const ParallelSafetySnapshot *safety)
{
    if (!safety->powered) return "POWER_OFF";
    if (!safety->probe_active) return "SEND_R_FIRST";
    if (!safety->parameters_valid) return "PARAMETERS_MISMATCH";
    if (!safety->feedback_fresh) return "FEEDBACK_STALE";
    if (!safety->all_disabled) return "MOTOR_NOT_DISABLED";
    if (!safety->start_speed_zero) return "SPEED_NOT_ZERO";
    if (!safety->runtime_torque_safe) return "TORQUE_LIMIT";
    if (!safety->temperature_safe) return "TEMPERATURE_LIMIT";
    if (!safety->can_active) return "CAN_NOT_ACTIVE";
    if (!safety->state_normal) return "STATE_ABNORMAL";
    return NULL;
}

static const char *runtime_fault(const ParallelSafetySnapshot *safety)
{
    if (!safety->feedback_fresh) return "FEEDBACK_STALE";
    if (!safety->runtime_speed_safe) return "SPEED_LIMIT";
    if (!safety->runtime_torque_safe) return "TORQUE_LIMIT";
    if (!safety->temperature_safe) return "TEMPERATURE_LIMIT";
    if (!safety->can_active) return "CAN_NOT_ACTIVE";
    if (!safety->state_normal) return "STATE_ABNORMAL";
    return NULL;
}

static ParallelDecision fail_closed(ParallelController *controller,
                                    const char *reason)
{
    ParallelDecision result = decision(PARALLEL_EVENT_SAFETY_TRIP, reason);

    controller->state = PARALLEL_DISABLED;
    controller->direction = 0;
    result.send_velocity_all = true;
    result.send_disable_all = true;
    result.cut_power = true;
    return result;
}

void parallel_controller_init(ParallelController *controller)
{
    if (controller != NULL)
        *controller = (ParallelController){.state = PARALLEL_DISABLED};
}

bool parallel_controller_active(const ParallelController *controller)
{
    return controller != NULL &&
           controller->state != PARALLEL_DISABLED &&
           controller->state != PARALLEL_ARMED;
}

ParallelDecision parallel_controller_command(
    ParallelController *controller, uint8_t command,
    const ParallelSafetySnapshot *safety)
{
    const char *reason;

    if (controller == NULL || safety == NULL)
        return decision(PARALLEL_EVENT_REJECTED, "INVALID_ARGUMENT");
    if (command >= (uint8_t)'a' && command <= (uint8_t)'z')
        command = (uint8_t)(command - ((uint8_t)'a' - (uint8_t)'A'));
    if (command == (uint8_t)'A') {
        if (parallel_controller_active(controller))
            return decision(PARALLEL_EVENT_REJECTED, "MOTION_ACTIVE");
        controller->state = PARALLEL_ARMED;
        controller->armed_at_ms = safety->now_ms;
        return decision(PARALLEL_EVENT_ARMED, NULL);
    }
    if (command != (uint8_t)'G' && command != (uint8_t)'B')
        return decision(PARALLEL_EVENT_REJECTED, "UNKNOWN_COMMAND");
    if (controller->state == PARALLEL_ARMED &&
        elapsed(safety->now_ms, controller->armed_at_ms, ARM_WINDOW_MS)) {
        controller->state = PARALLEL_DISABLED;
        return decision(PARALLEL_EVENT_ARM_TIMEOUT, "ARM_TIMEOUT");
    }
    if (controller->state != PARALLEL_ARMED)
        return decision(PARALLEL_EVENT_REJECTED, "SEND_A_FIRST");
    reason = start_rejection(safety);
    if (reason != NULL) return decision(PARALLEL_EVENT_REJECTED, reason);

    controller->state = PARALLEL_ENABLE_WAIT;
    controller->direction = command == (uint8_t)'G' ? 1 : -1;
    controller->phase_started_ms = safety->now_ms;
    controller->last_command_ms = safety->now_ms;
    {
        ParallelDecision result = decision(PARALLEL_EVENT_ENABLE_WAIT, NULL);

        result.send_enable_all = true;
        result.direction = controller->direction;
        return result;
    }
}

ParallelDecision parallel_controller_step(
    ParallelController *controller, const ParallelSafetySnapshot *safety)
{
    ParallelDecision result = decision(PARALLEL_EVENT_NONE, NULL);
    const char *reason;

    if (controller == NULL || safety == NULL)
        return decision(PARALLEL_EVENT_REJECTED, "INVALID_ARGUMENT");
    if (controller->state == PARALLEL_ARMED) {
        if (elapsed(safety->now_ms, controller->armed_at_ms, ARM_WINDOW_MS)) {
            controller->state = PARALLEL_DISABLED;
            return decision(PARALLEL_EVENT_ARM_TIMEOUT, "ARM_TIMEOUT");
        }
        return result;
    }
    if (controller->state == PARALLEL_ENABLE_WAIT) {
        reason = runtime_fault(safety);
        if (reason != NULL) return fail_closed(controller, reason);
        if (safety->all_enabled &&
            !elapsed(safety->now_ms, controller->phase_started_ms,
                     ENABLE_WAIT_MS + 1u)) {
            controller->state = PARALLEL_RUNNING;
            controller->phase_started_ms = safety->now_ms;
            controller->last_command_ms = safety->now_ms;
            result = decision(PARALLEL_EVENT_RUNNING, NULL);
            result.send_velocity_all = true;
            result.direction = controller->direction;
            return result;
        }
        if (elapsed(safety->now_ms, controller->phase_started_ms,
                    ENABLE_WAIT_MS))
            return fail_closed(controller, "ENABLE_TIMEOUT");
        return result;
    }
    if (controller->state == PARALLEL_RUNNING) {
        reason = runtime_fault(safety);
        if (reason != NULL) return fail_closed(controller, reason);
        if (!safety->all_enabled)
            return fail_closed(controller, "STATE_NOT_ENABLED");
        if (elapsed(safety->now_ms, controller->phase_started_ms, PULSE_MS)) {
            controller->state = PARALLEL_ZERO_HOLD;
            controller->phase_started_ms = safety->now_ms;
            controller->last_command_ms = safety->now_ms;
            result = decision(PARALLEL_EVENT_ZERO_HOLD, NULL);
            result.send_velocity_all = true;
            return result;
        }
        if (elapsed(safety->now_ms, controller->last_command_ms, REFRESH_MS)) {
            controller->last_command_ms = safety->now_ms;
            result.send_velocity_all = true;
            result.direction = controller->direction;
        }
        return result;
    }
    if (controller->state == PARALLEL_ZERO_HOLD) {
        reason = runtime_fault(safety);
        if (reason != NULL) return fail_closed(controller, reason);
        if (!safety->all_enabled)
            return fail_closed(controller, "STATE_NOT_ENABLED");
        if (elapsed(safety->now_ms, controller->phase_started_ms,
                    ZERO_HOLD_MS)) {
            controller->state = PARALLEL_DISABLE_WAIT;
            controller->phase_started_ms = safety->now_ms;
            result = decision(PARALLEL_EVENT_DISABLE_WAIT, NULL);
            result.send_disable_all = true;
            return result;
        }
        if (elapsed(safety->now_ms, controller->last_command_ms, REFRESH_MS)) {
            controller->last_command_ms = safety->now_ms;
            result.send_velocity_all = true;
        }
        return result;
    }
    if (controller->state == PARALLEL_DISABLE_WAIT) {
        reason = runtime_fault(safety);
        if (reason != NULL) return fail_closed(controller, reason);
        if (safety->all_disabled) {
            controller->state = PARALLEL_DISABLED;
            controller->direction = 0;
            return decision(PARALLEL_EVENT_COMPLETE, NULL);
        }
        if (elapsed(safety->now_ms, controller->phase_started_ms,
                    DISABLE_WAIT_MS))
            return fail_closed(controller, "DISABLE_TIMEOUT");
    }
    return result;
}

const char *parallel_state_name(ParallelState state)
{
    switch (state) {
    case PARALLEL_DISABLED: return "DISABLED";
    case PARALLEL_ARMED: return "ARMED";
    case PARALLEL_ENABLE_WAIT: return "ENABLE_WAIT";
    case PARALLEL_RUNNING: return "RUNNING";
    case PARALLEL_ZERO_HOLD: return "ZERO_HOLD";
    case PARALLEL_DISABLE_WAIT: return "DISABLE_WAIT";
    default: return "UNKNOWN";
    }
}
