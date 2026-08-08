#include "motion_io.h"

#include <string.h>

void motion_log_queue_init(MotionLogQueue *queue)
{
    if (queue == NULL) {
        return;
    }
    memset(queue, 0, sizeof(*queue));
}

bool motion_log_queue_push(MotionLogQueue *queue, const void *record,
                           size_t length)
{
    if (queue == NULL || record == NULL || length == 0u ||
        length > MOTION_LOG_RECORD_CAPACITY ||
        queue->count >= MOTION_LOG_QUEUE_CAPACITY) {
        return false;
    }

    memcpy(queue->records[queue->tail], record, length);
    queue->lengths[queue->tail] = (uint16_t)length;
    queue->tail = (uint8_t)((queue->tail + 1u) % MOTION_LOG_QUEUE_CAPACITY);
    queue->count++;
    return true;
}

bool motion_log_queue_peek(const MotionLogQueue *queue,
                           const uint8_t **record, uint16_t *length)
{
    if (queue == NULL || record == NULL || length == NULL ||
        queue->count == 0u) {
        return false;
    }

    *record = queue->records[queue->head];
    *length = queue->lengths[queue->head];
    return true;
}

void motion_log_queue_pop(MotionLogQueue *queue)
{
    if (queue == NULL || queue->count == 0u) {
        return;
    }
    queue->head = (uint8_t)((queue->head + 1u) % MOTION_LOG_QUEUE_CAPACITY);
    queue->count--;
}

void motion_log_queue_finish_attempt(MotionLogQueue *queue, bool accepted,
                                     uint32_t *dropped_logs)
{
    if (queue == NULL || dropped_logs == NULL || queue->count == 0u) {
        return;
    }
    motion_log_queue_pop(queue);
    if (!accepted) {
        (*dropped_logs)++;
    }
}

uint32_t motion_telemetry_period_ms(MotionState state)
{
    return state == MOTION_IDLE_DISABLED || state == MOTION_ARMED
               ? MOTION_TELEMETRY_IDLE_PERIOD_MS
               : MOTION_TELEMETRY_ACTIVE_PERIOD_MS;
}

bool motion_telemetry_reschedule(MotionState state, uint32_t now_ms,
                                 uint32_t *period_ms,
                                 uint32_t *next_health_ms,
                                 uint32_t *next_wheel_ms)
{
    uint32_t next_period_ms;

    if (period_ms == NULL || next_health_ms == NULL || next_wheel_ms == NULL) {
        return false;
    }
    next_period_ms = motion_telemetry_period_ms(state);
    if (*period_ms == next_period_ms) {
        return false;
    }
    *period_ms = next_period_ms;
    *next_health_ms = now_ms + next_period_ms;
    *next_wheel_ms = now_ms + next_period_ms;
    return true;
}

void motion_feedback_probe_schedule_init(
    MotionFeedbackProbeSchedule *schedule)
{
    if (schedule != NULL) {
        memset(schedule, 0, sizeof(*schedule));
    }
}

bool motion_feedback_probe_should_send(
    MotionFeedbackProbeSchedule *schedule, MotionState state,
    bool pending_action, uint32_t now_ms)
{
    bool allowed;

    if (schedule == NULL) {
        return false;
    }

    allowed = !pending_action &&
              (state == MOTION_IDLE_DISABLED || state == MOTION_ARMED);
    if (!allowed) {
        schedule->window_open = false;
        schedule->next_probe_ms = now_ms + MOTION_FEEDBACK_PROBE_PERIOD_MS;
        return false;
    }
    if (!schedule->window_open) {
        schedule->window_open = true;
        schedule->next_probe_ms = now_ms + MOTION_FEEDBACK_PROBE_PERIOD_MS;
        return false;
    }
    if ((int32_t)(now_ms - schedule->next_probe_ms) < 0) {
        return false;
    }

    schedule->next_probe_ms = now_ms + MOTION_FEEDBACK_PROBE_PERIOD_MS;
    return true;
}

void pending_motion_action_init(PendingMotionAction *pending)
{
    if (pending != NULL) {
        pending->action = MOTION_ACTION_NONE;
    }
}

bool pending_motion_action_has_value(const PendingMotionAction *pending)
{
    return pending != NULL && pending->action != MOTION_ACTION_NONE;
}

MotionAction pending_motion_action_begin_attempt(
    const PendingMotionAction *pending)
{
    return pending != NULL ? pending->action : MOTION_ACTION_NONE;
}

void pending_motion_action_succeeded(PendingMotionAction *pending)
{
    if (pending != NULL) {
        pending->action = MOTION_ACTION_NONE;
    }
}

void pending_motion_action_failed(PendingMotionAction *pending,
                                  MotionAction attempted,
                                  MotionAction recovery)
{
    if (pending == NULL) {
        return;
    }
    pending->action = recovery != MOTION_ACTION_NONE ? recovery : attempted;
}
