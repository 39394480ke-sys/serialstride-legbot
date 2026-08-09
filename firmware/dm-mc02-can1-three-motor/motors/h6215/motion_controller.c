#include "motion_controller.h"

#include <stddef.h>

#define PARAMETER_MASK_COMPLETE 0x1fu
#define CONTROL_MODE_VELOCITY 3u
#define MOTOR_DISABLED 0u
#define MOTOR_ENABLED 1u
#define ARM_WINDOW_MS 10000u
#define ENABLE_WAIT_MS 100u
#define PULSE_MS 1000u
#define ZERO_HOLD_MS 200u
#define DISABLE_WAIT_MS 200u
#define REFRESH_MS 10u
#define RAMP_MS 100u
#define WATCHDOG_MS 5000u
#define START_FEEDBACK_MAX_AGE_MS 500u
#define RUNTIME_FEEDBACK_MAX_AGE_MS 100u
#define START_SPEED_LIMIT_MILLIRAD_S 100
#define RUNTIME_SPEED_LIMIT_MILLIRAD_S 800
#define TEMPERATURE_LIMIT_C 60u

static MotionDecision decision(MotionAction action, MotionEvent event,
                               const char *reason, int8_t step)
{
    return (MotionDecision){action, event, reason, step};
}

static bool elapsed(uint32_t now, uint32_t then, uint32_t interval)
{
    return now - then >= interval;
}

static uint8_t normalize(uint8_t command)
{
    return command >= 'a' && command <= 'z' ? command - ('a' - 'A') : command;
}

static const char *start_rejection(const MotionSafetySnapshot *s)
{
    if (s->parameter_mask != PARAMETER_MASK_COMPLETE) return "PARAMETERS_INCOMPLETE";
    if (s->control_mode != CONTROL_MODE_VELOCITY) return "MODE_NOT_VELOCITY";
    if (!s->feedback_valid || s->feedback_age_ms > START_FEEDBACK_MAX_AGE_MS) return "FEEDBACK_STALE";
    if (s->motor_state != MOTOR_DISABLED) return "STATE_NOT_DISABLED";
    if (s->velocity_millirad_s <= -START_SPEED_LIMIT_MILLIRAD_S ||
        s->velocity_millirad_s >= START_SPEED_LIMIT_MILLIRAD_S) return "SPEED_NOT_ZERO";
    if (s->mos_temperature_c >= TEMPERATURE_LIMIT_C ||
        s->rotor_temperature_c >= TEMPERATURE_LIMIT_C) return "TEMPERATURE_LIMIT";
    if (s->can_warning || s->can_passive || s->can_bus_off) return "CAN_NOT_ACTIVE";
    return NULL;
}

static const char *runtime_trip(const MotionSafetySnapshot *s, bool require_enabled)
{
    if (!s->feedback_valid || s->feedback_age_ms > RUNTIME_FEEDBACK_MAX_AGE_MS) return "FEEDBACK_STALE";
    if (s->velocity_millirad_s < -RUNTIME_SPEED_LIMIT_MILLIRAD_S ||
        s->velocity_millirad_s > RUNTIME_SPEED_LIMIT_MILLIRAD_S) return "SPEED_LIMIT";
    if (s->mos_temperature_c >= TEMPERATURE_LIMIT_C ||
        s->rotor_temperature_c >= TEMPERATURE_LIMIT_C) return "TEMPERATURE_LIMIT";
    if (s->can_passive || s->can_bus_off) return "CAN_NOT_ACTIVE";
    if (require_enabled && s->motor_state != MOTOR_ENABLED) return "STATE_NOT_ENABLED";
    return NULL;
}

static MotionDecision fault_stop(MotionController *c, uint32_t now,
                                 MotionEvent event, const char *reason)
{
    c->armed = false;
    c->state = MOTION_FAULT_DISABLE;
    c->current_step = 0;
    c->target_step = 0;
    c->last_action_ms = now;
    return decision(MOTION_ACTION_VELOCITY, event, reason, 0);
}

void motion_controller_init(MotionController *c)
{
    if (c == NULL) return;
    *c = (MotionController){.state = MOTION_IDLE_DISABLED};
}

static MotionDecision request_start(MotionController *c, uint8_t command,
                                    const MotionSafetySnapshot *s)
{
    const char *reason;
    if (c->armed && elapsed(s->now_ms, c->armed_at_ms, ARM_WINDOW_MS)) {
        c->armed = false;
        c->state = MOTION_IDLE_DISABLED;
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_ARM_TIMEOUT, "ARM_TIMEOUT", 0);
    }
    if (!c->armed || c->state != MOTION_ARMED)
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_START_REJECTED, "SEND_A_FIRST", 0);
    reason = start_rejection(s);
    if (reason != NULL)
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_START_REJECTED, reason, 0);
    c->armed = false;
    c->state = MOTION_ENABLE_WAIT;
    c->phase_started_ms = s->now_ms;
    c->last_action_ms = s->now_ms;
    c->continuous_requested = command == 'C';
    c->pulse_step = command == 'B' ? -2 : 2;
    c->current_step = 0;
    c->target_step = 0;
    c->watchdog_stop = false;
    return decision(MOTION_ACTION_ENABLE, MOTION_EVENT_START_REQUESTED, NULL,
                    c->continuous_requested ? 0 : c->pulse_step);
}

MotionDecision motion_controller_command(MotionController *c, uint8_t command,
                                          const MotionSafetySnapshot *s)
{
    if (c == NULL || s == NULL)
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "INVALID_ARGUMENT", 0);
    command = normalize(command);
    if (command == '\r' || command == '\n')
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "IGNORED", 0);
    if (command == 'X')
        return fault_stop(c, s->now_ms, MOTION_EVENT_EMERGENCY_STOP, "EMERGENCY_STOP");
    if (command == 'S')
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_STATUS, NULL, 0);
    if (command == 'A') {
        if (c->state != MOTION_IDLE_DISABLED && c->state != MOTION_ARMED)
            return decision(MOTION_ACTION_NONE, MOTION_EVENT_START_REJECTED, "MOTION_ACTIVE", 0);
        c->state = MOTION_ARMED;
        c->armed = true;
        c->armed_at_ms = s->now_ms;
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_ARMED, NULL, 0);
    }
    if (command == 'G' || command == 'B' || command == 'C')
        return request_start(c, command, s);
    if (c->state == MOTION_CONTINUOUS) {
        if (command == '+' || command == '-' || command == '0' || command == 'K') {
            c->last_control_ms = s->now_ms;
            if (command == '+' && c->target_step < 5) c->target_step++;
            if (command == '-' && c->target_step > -5) c->target_step--;
            if (command == '0') c->target_step = 0;
            return decision(MOTION_ACTION_NONE,
                            command == 'K' ? MOTION_EVENT_KEEPALIVE : MOTION_EVENT_TARGET_UPDATED,
                            NULL, c->target_step);
        }
    }
    return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "UNKNOWN_COMMAND", 0);
}

static MotionDecision refresh(MotionController *c, uint32_t now, int8_t step)
{
    c->last_action_ms = now;
    return decision(MOTION_ACTION_VELOCITY, MOTION_EVENT_NONE, NULL, step);
}

MotionDecision motion_controller_step(MotionController *c,
                                       const MotionSafetySnapshot *s)
{
    const char *reason;
    if (c == NULL || s == NULL)
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "INVALID_ARGUMENT", 0);
    switch (c->state) {
    case MOTION_IDLE_DISABLED:
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL, 0);
    case MOTION_ARMED:
        if (elapsed(s->now_ms, c->armed_at_ms, ARM_WINDOW_MS)) {
            c->armed = false; c->state = MOTION_IDLE_DISABLED;
            return decision(MOTION_ACTION_NONE, MOTION_EVENT_ARM_TIMEOUT, "ARM_TIMEOUT", 0);
        }
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL, 0);
    case MOTION_ENABLE_WAIT:
        if ((!s->feedback_valid ||
             s->feedback_age_ms > RUNTIME_FEEDBACK_MAX_AGE_MS) &&
            (s->can_passive || s->can_bus_off))
            return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP,
                              "CAN_NOT_ACTIVE");
        if (s->feedback_valid &&
            s->feedback_age_ms <= RUNTIME_FEEDBACK_MAX_AGE_MS) {
            reason = runtime_trip(s, false);
            if (reason != NULL)
                return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP,
                                  reason);
            if (s->motor_state != MOTOR_DISABLED &&
                s->motor_state != MOTOR_ENABLED)
                return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP,
                                  "STATE_NOT_ENABLED");
        }
        if (s->feedback_valid && s->feedback_age_ms <= RUNTIME_FEEDBACK_MAX_AGE_MS &&
            s->motor_state == MOTOR_ENABLED &&
            s->now_ms - c->phase_started_ms <= ENABLE_WAIT_MS) {
            c->phase_started_ms = c->last_action_ms = c->last_control_ms = c->last_ramp_ms = s->now_ms;
            if (c->continuous_requested) {
                c->state = MOTION_CONTINUOUS;
                return decision(MOTION_ACTION_VELOCITY, MOTION_EVENT_CONTINUOUS_READY, NULL, 0);
            }
            c->state = MOTION_RUNNING;
            c->current_step = c->target_step = c->pulse_step;
            return decision(MOTION_ACTION_VELOCITY, MOTION_EVENT_RUNNING, NULL, c->pulse_step);
        }
        if (elapsed(s->now_ms, c->phase_started_ms, ENABLE_WAIT_MS))
            return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP,
                              !s->feedback_valid || s->feedback_age_ms > RUNTIME_FEEDBACK_MAX_AGE_MS ?
                                  "FEEDBACK_STALE" : "STATE_NOT_ENABLED");
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL, 0);
    case MOTION_RUNNING:
        reason = runtime_trip(s, true);
        if (reason != NULL) return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP, reason);
        if (elapsed(s->now_ms, c->phase_started_ms, PULSE_MS)) {
            c->state = MOTION_ZERO_HOLD; c->watchdog_stop = false;
            c->phase_started_ms = c->last_action_ms = s->now_ms; c->current_step = c->target_step = 0;
            return decision(MOTION_ACTION_VELOCITY, MOTION_EVENT_ZERO_HOLD, NULL, 0);
        }
        if (elapsed(s->now_ms, c->last_action_ms, REFRESH_MS)) return refresh(c, s->now_ms, c->current_step);
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL, 0);
    case MOTION_CONTINUOUS:
        reason = runtime_trip(s, true);
        if (reason != NULL) return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP, reason);
        if (elapsed(s->now_ms, c->last_control_ms, WATCHDOG_MS)) {
            c->state = MOTION_WATCHDOG_RAMP; c->watchdog_stop = true; c->target_step = 0;
            c->last_ramp_ms = s->now_ms;
            return decision(MOTION_ACTION_NONE, MOTION_EVENT_HOST_WATCHDOG, "HOST_WATCHDOG", 0);
        }
        if (elapsed(s->now_ms, c->last_ramp_ms, RAMP_MS) && c->current_step != c->target_step) {
            c->current_step += c->current_step < c->target_step ? 1 : -1;
            c->last_ramp_ms = c->last_action_ms = s->now_ms;
            return decision(MOTION_ACTION_VELOCITY, MOTION_EVENT_NONE, NULL, c->current_step);
        }
        if (elapsed(s->now_ms, c->last_action_ms, REFRESH_MS)) return refresh(c, s->now_ms, c->current_step);
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL, 0);
    case MOTION_WATCHDOG_RAMP:
        reason = runtime_trip(s, true);
        if (reason != NULL) return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP, reason);
        if (c->current_step == 0) {
            c->state = MOTION_ZERO_HOLD; c->phase_started_ms = c->last_action_ms = s->now_ms;
            return decision(MOTION_ACTION_VELOCITY, MOTION_EVENT_ZERO_HOLD, "HOST_WATCHDOG", 0);
        }
        if (elapsed(s->now_ms, c->last_ramp_ms, RAMP_MS)) {
            c->current_step += c->current_step > 0 ? -1 : 1;
            c->last_ramp_ms = c->last_action_ms = s->now_ms;
            return decision(MOTION_ACTION_VELOCITY, MOTION_EVENT_NONE, NULL, c->current_step);
        }
        if (elapsed(s->now_ms, c->last_action_ms, REFRESH_MS)) return refresh(c, s->now_ms, c->current_step);
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL, 0);
    case MOTION_ZERO_HOLD:
        reason = runtime_trip(s, true);
        if (reason != NULL) return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP, reason);
        if (elapsed(s->now_ms, c->phase_started_ms, ZERO_HOLD_MS)) {
            c->state = MOTION_DISABLE_WAIT;
            c->phase_started_ms = s->now_ms;
            return decision(MOTION_ACTION_DISABLE,
                            MOTION_EVENT_DISABLE_REQUESTED,
                            c->watchdog_stop ? "HOST_WATCHDOG" : NULL, 0);
        }
        if (elapsed(s->now_ms, c->last_action_ms, REFRESH_MS)) return refresh(c, s->now_ms, 0);
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL, 0);
    case MOTION_DISABLE_WAIT:
        reason = runtime_trip(s, false);
        if (reason != NULL)
            return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP,
                              reason);
        if (s->motor_state == MOTOR_DISABLED) {
            c->state = MOTION_IDLE_DISABLED;
            return decision(MOTION_ACTION_NONE, MOTION_EVENT_COMPLETE,
                            c->watchdog_stop ? "HOST_WATCHDOG" : NULL, 0);
        }
        if (s->motor_state != MOTOR_ENABLED)
            return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP,
                              "STATE_ABNORMAL");
        if (elapsed(s->now_ms, c->phase_started_ms, DISABLE_WAIT_MS))
            return fault_stop(c, s->now_ms, MOTION_EVENT_SAFETY_TRIP,
                              "DISABLE_TIMEOUT");
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL, 0);
    case MOTION_FAULT_ZERO:
        c->state = MOTION_FAULT_DISABLE; c->last_action_ms = s->now_ms;
        return decision(MOTION_ACTION_VELOCITY, MOTION_EVENT_SAFETY_TRIP, "FAULT_STOP", 0);
    case MOTION_FAULT_DISABLE:
        if (elapsed(s->now_ms, c->last_action_ms, 1u)) {
            c->state = MOTION_IDLE_DISABLED;
            return decision(MOTION_ACTION_DISABLE, MOTION_EVENT_NONE, NULL, 0);
        }
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, NULL, 0);
    default:
        motion_controller_init(c);
        return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "INVALID_STATE", 0);
    }
}

MotionDecision motion_controller_tx_failed(MotionController *c)
{
    if (c == NULL) return decision(MOTION_ACTION_NONE, MOTION_EVENT_NONE, "INVALID_ARGUMENT", 0);
    if (c->state == MOTION_FAULT_DISABLE) {
        c->state = MOTION_IDLE_DISABLED; c->armed = false;
        return decision(MOTION_ACTION_DISABLE, MOTION_EVENT_TX_FAILURE, "TX_FAILURE", 0);
    }
    return fault_stop(c, c->last_action_ms, MOTION_EVENT_TX_FAILURE, "TX_FAILURE");
}

const char *motion_controller_state_name(MotionState state)
{
    switch (state) {
    case MOTION_IDLE_DISABLED: return "DISABLED";
    case MOTION_ARMED: return "ARMED";
    case MOTION_ENABLE_WAIT: return "ENABLE_WAIT";
    case MOTION_RUNNING: return "PULSE";
    case MOTION_CONTINUOUS: return "CONTINUOUS";
    case MOTION_WATCHDOG_RAMP: return "WATCHDOG_RAMP";
    case MOTION_ZERO_HOLD: return "ZERO_HOLD";
    case MOTION_DISABLE_WAIT: return "DISABLE_WAIT";
    case MOTION_FAULT_ZERO: return "FAULT_ZERO";
    case MOTION_FAULT_DISABLE: return "FAULT_DISABLE";
    default: return "UNKNOWN";
    }
}

uint32_t motion_controller_watchdog_age(const MotionController *c, uint32_t now_ms)
{
    return c != NULL ? now_ms - c->last_control_ms : 0u;
}
