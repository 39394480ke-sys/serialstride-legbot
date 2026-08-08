#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "motion_io.h"

static void test_log_queue_preserves_composed_records_in_fifo_order(void)
{
    static const char status_record[] =
        "STATUS_REQUESTED\r\n[WHEEL] first\r\nHEALTH first\r\n";
    static const char transition_record[] =
        "MOTOR_ENABLE_TX_OK\r\n"
        "MOTION_RUNNING TARGET=+0.200rad/s DURATION_MS=1000\r\n";
    MotionLogQueue queue;
    const uint8_t *record;
    uint16_t length;

    motion_log_queue_init(&queue);
    assert(motion_log_queue_push(&queue, status_record,
                                 sizeof(status_record) - 1u));
    assert(motion_log_queue_push(&queue, transition_record,
                                 sizeof(transition_record) - 1u));

    assert(motion_log_queue_peek(&queue, &record, &length));
    assert(length == sizeof(status_record) - 1u);
    assert(memcmp(record, status_record, length) == 0);
    assert(motion_log_queue_peek(&queue, &record, &length));
    assert(memcmp(record, status_record, length) == 0);

    motion_log_queue_pop(&queue);
    assert(motion_log_queue_peek(&queue, &record, &length));
    assert(length == sizeof(transition_record) - 1u);
    assert(memcmp(record, transition_record, length) == 0);
    motion_log_queue_pop(&queue);
    assert(!motion_log_queue_peek(&queue, &record, &length));
}

static void test_log_queue_rejects_overflow_without_corrupting_oldest(void)
{
    static const char record[] = "record";
    MotionLogQueue queue;
    const uint8_t *queued_record;
    uint16_t length;

    motion_log_queue_init(&queue);
    for (uint8_t index = 0u; index < MOTION_LOG_QUEUE_CAPACITY; ++index) {
        assert(motion_log_queue_push(&queue, record, sizeof(record) - 1u));
    }
    assert(!motion_log_queue_push(&queue, "overflow", 8u));
    assert(motion_log_queue_peek(&queue, &queued_record, &length));
    assert(length == sizeof(record) - 1u);
    assert(memcmp(queued_record, record, length) == 0);
}

static void test_log_queue_rejects_invalid_records(void)
{
    char oversized[MOTION_LOG_RECORD_CAPACITY + 1u] = {0};
    MotionLogQueue queue;

    motion_log_queue_init(&queue);
    assert(!motion_log_queue_push(NULL, "x", 1u));
    assert(!motion_log_queue_push(&queue, NULL, 1u));
    assert(!motion_log_queue_push(&queue, "", 0u));
    assert(!motion_log_queue_push(&queue, oversized, sizeof(oversized)));
}

static void test_pending_action_remains_owned_until_success(void)
{
    PendingMotionAction pending;

    pending_motion_action_init(&pending);
    assert(!pending_motion_action_has_value(&pending));
    pending_motion_action_failed(&pending, MOTION_ACTION_NONE,
                                 MOTION_ACTION_DISABLE);
    assert(pending_motion_action_has_value(&pending));

    assert(pending_motion_action_begin_attempt(&pending) ==
           MOTION_ACTION_DISABLE);
    assert(pending_motion_action_begin_attempt(&pending) ==
           MOTION_ACTION_DISABLE);

    pending_motion_action_failed(&pending, MOTION_ACTION_DISABLE,
                                 MOTION_ACTION_DISABLE);
    assert(pending_motion_action_begin_attempt(&pending) ==
           MOTION_ACTION_DISABLE);
    pending_motion_action_succeeded(&pending);
    assert(!pending_motion_action_has_value(&pending));
}

static void test_failed_action_without_replacement_is_retried(void)
{
    PendingMotionAction pending;

    pending_motion_action_init(&pending);
    pending_motion_action_failed(&pending, MOTION_ACTION_ZERO_VELOCITY,
                                 MOTION_ACTION_NONE);
    assert(pending_motion_action_begin_attempt(&pending) ==
           MOTION_ACTION_ZERO_VELOCITY);
}

static void test_disable_returned_after_recovery_failure_stays_pending(void)
{
    MotionController controller;
    PendingMotionAction pending;
    MotionDecision zero_recovery;
    MotionDecision disable_recovery;

    motion_controller_init(&controller);
    controller.state = MOTION_RUNNING;
    zero_recovery = motion_controller_tx_failed(&controller);
    assert(zero_recovery.action == MOTION_ACTION_ZERO_VELOCITY);
    assert(controller.state == MOTION_FAULT_DISABLE);

    disable_recovery = motion_controller_tx_failed(&controller);
    assert(disable_recovery.action == MOTION_ACTION_DISABLE);
    assert(controller.state == MOTION_IDLE_DISABLED);

    pending_motion_action_init(&pending);
    pending_motion_action_failed(&pending, zero_recovery.action,
                                 disable_recovery.action);
    assert(pending_motion_action_begin_attempt(&pending) ==
           MOTION_ACTION_DISABLE);
}

int main(void)
{
    test_log_queue_preserves_composed_records_in_fifo_order();
    test_log_queue_rejects_overflow_without_corrupting_oldest();
    test_log_queue_rejects_invalid_records();
    test_pending_action_remains_owned_until_success();
    test_failed_action_without_replacement_is_retried();
    test_disable_returned_after_recovery_failure_stays_pending();
    return 0;
}
