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
