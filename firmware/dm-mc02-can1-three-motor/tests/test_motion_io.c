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

static void test_submission_attempt_always_removes_head_and_counts_rejection(void)
{
    static const char first[] = "first";
    static const char second[] = "second";
    MotionLogQueue queue;
    const uint8_t *record;
    uint16_t length;
    uint32_t dropped_logs = 4u;

    motion_log_queue_init(&queue);
    assert(motion_log_queue_push(&queue, first, sizeof(first) - 1u));
    assert(motion_log_queue_push(&queue, second, sizeof(second) - 1u));

    motion_log_queue_finish_attempt(&queue, false, &dropped_logs);
    assert(dropped_logs == 5u);
    assert(motion_log_queue_peek(&queue, &record, &length));
    assert(length == sizeof(second) - 1u);
    assert(memcmp(record, second, length) == 0);

    motion_log_queue_finish_attempt(&queue, true, &dropped_logs);
    assert(dropped_logs == 5u);
    assert(!motion_log_queue_peek(&queue, &record, &length));

    motion_log_queue_finish_attempt(&queue, false, &dropped_logs);
    assert(dropped_logs == 5u);
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

static void test_telemetry_cadence_matches_motion_activity(void)
{
    static const MotionState active_states[] = {
        MOTION_ENABLE_WAIT,
        MOTION_RUNNING,
        MOTION_ZERO_HOLD,
        MOTION_FAULT_ZERO,
        MOTION_FAULT_DISABLE,
    };

    assert(motion_telemetry_period_ms(MOTION_IDLE_DISABLED) == 1000u);
    assert(motion_telemetry_period_ms(MOTION_ARMED) == 1000u);
    for (size_t index = 0u;
         index < sizeof(active_states) / sizeof(active_states[0]); ++index) {
        assert(motion_telemetry_period_ms(active_states[index]) == 100u);
    }
}

static void test_telemetry_cadence_transition_reschedules_both_timers(void)
{
    uint32_t period_ms = 1000u;
    uint32_t next_health_ms = 700u;
    uint32_t next_wheel_ms = 800u;

    assert(!motion_telemetry_reschedule(MOTION_ARMED, 100u, &period_ms,
                                        &next_health_ms, &next_wheel_ms));
    assert(period_ms == 1000u);
    assert(next_health_ms == 700u && next_wheel_ms == 800u);

    assert(motion_telemetry_reschedule(MOTION_ENABLE_WAIT, 200u, &period_ms,
                                       &next_health_ms, &next_wheel_ms));
    assert(period_ms == 100u);
    assert(next_health_ms == 300u && next_wheel_ms == 300u);

    assert(!motion_telemetry_reschedule(MOTION_FAULT_DISABLE, 250u,
                                        &period_ms, &next_health_ms,
                                        &next_wheel_ms));
    assert(next_health_ms == 300u && next_wheel_ms == 300u);

    assert(motion_telemetry_reschedule(MOTION_ARMED, 300u, &period_ms,
                                       &next_health_ms, &next_wheel_ms));
    assert(period_ms == 1000u);
    assert(next_health_ms == 1300u && next_wheel_ms == 1300u);
}

static void test_feedback_probe_runs_on_200_ms_idle_armed_cadence(void)
{
    MotionFeedbackProbeSchedule schedule;

    motion_feedback_probe_schedule_init(&schedule);
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 100u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_ARMED, false, 299u));
    assert(motion_feedback_probe_should_send(
        &schedule, MOTION_ARMED, false, 300u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 300u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 499u));
    assert(motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 500u));
}

static void test_feedback_probe_never_runs_in_active_or_fault_states(void)
{
    static const MotionState prohibited_states[] = {
        MOTION_ENABLE_WAIT,
        MOTION_RUNNING,
        MOTION_ZERO_HOLD,
        MOTION_FAULT_ZERO,
        MOTION_FAULT_DISABLE,
    };

    for (size_t index = 0u;
         index < sizeof(prohibited_states) / sizeof(prohibited_states[0]);
         ++index) {
        MotionFeedbackProbeSchedule schedule;

        motion_feedback_probe_schedule_init(&schedule);
        assert(!motion_feedback_probe_should_send(
            &schedule, prohibited_states[index], false, 0u));
        assert(!motion_feedback_probe_should_send(
            &schedule, prohibited_states[index], false, 200u));
    }
}

static void test_pending_action_suppresses_and_reschedules_feedback_probe(void)
{
    MotionFeedbackProbeSchedule schedule;

    motion_feedback_probe_schedule_init(&schedule);
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 0u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, true, 200u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 200u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 399u));
    assert(motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 400u));
}

static void test_post_command_active_state_suppresses_same_tick_probe(void)
{
    MotionFeedbackProbeSchedule schedule;

    motion_feedback_probe_schedule_init(&schedule);
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_ARMED, false, 0u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_ENABLE_WAIT, false, 200u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_RUNNING, false, 400u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_ZERO_HOLD, false, 1200u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 1400u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 1599u));
    assert(motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 1600u));
}

static void test_feedback_probe_deadline_handles_tick_wraparound(void)
{
    MotionFeedbackProbeSchedule schedule;
    const uint32_t start_ms = UINT32_MAX - 99u;

    motion_feedback_probe_schedule_init(&schedule);
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, start_ms));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 99u));
    assert(motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 100u));
    assert(!motion_feedback_probe_should_send(
        &schedule, MOTION_IDLE_DISABLED, false, 100u));
}

int main(void)
{
    test_log_queue_preserves_composed_records_in_fifo_order();
    test_log_queue_rejects_overflow_without_corrupting_oldest();
    test_log_queue_rejects_invalid_records();
    test_submission_attempt_always_removes_head_and_counts_rejection();
    test_pending_action_remains_owned_until_success();
    test_failed_action_without_replacement_is_retried();
    test_disable_returned_after_recovery_failure_stays_pending();
    test_telemetry_cadence_matches_motion_activity();
    test_telemetry_cadence_transition_reschedules_both_timers();
    test_feedback_probe_runs_on_200_ms_idle_armed_cadence();
    test_feedback_probe_never_runs_in_active_or_fault_states();
    test_pending_action_suppresses_and_reschedules_feedback_probe();
    test_post_command_active_state_suppresses_same_tick_probe();
    test_feedback_probe_deadline_handles_tick_wraparound();
    return 0;
}
