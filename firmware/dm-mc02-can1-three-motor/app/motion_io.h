#ifndef MOTION_IO_H
#define MOTION_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motion_controller.h"

#define MOTION_LOG_QUEUE_CAPACITY 8u
#define MOTION_LOG_RECORD_CAPACITY 640u

typedef struct {
    uint8_t records[MOTION_LOG_QUEUE_CAPACITY][MOTION_LOG_RECORD_CAPACITY];
    uint16_t lengths[MOTION_LOG_QUEUE_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} MotionLogQueue;

typedef struct {
    MotionAction action;
} PendingMotionAction;

void motion_log_queue_init(MotionLogQueue *queue);
bool motion_log_queue_push(MotionLogQueue *queue, const void *record,
                           size_t length);
bool motion_log_queue_peek(const MotionLogQueue *queue,
                           const uint8_t **record, uint16_t *length);
void motion_log_queue_pop(MotionLogQueue *queue);
void motion_log_queue_finish_attempt(MotionLogQueue *queue, bool accepted,
                                     uint32_t *dropped_logs);

void pending_motion_action_init(PendingMotionAction *pending);
bool pending_motion_action_has_value(const PendingMotionAction *pending);
MotionAction pending_motion_action_begin_attempt(
    const PendingMotionAction *pending);
void pending_motion_action_succeeded(PendingMotionAction *pending);
void pending_motion_action_failed(PendingMotionAction *pending,
                                  MotionAction attempted,
                                  MotionAction recovery);

#endif
