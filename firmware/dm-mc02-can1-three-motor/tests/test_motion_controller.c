#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "motion_controller.h"

static MotionSafetySnapshot safe_disabled_snapshot(uint32_t now_ms)
{
    return (MotionSafetySnapshot){
        .now_ms = now_ms,
        .feedback_age_ms = 0u,
        .parameter_mask = 0x1fu,
        .control_mode = 3u,
        .motor_state = 0u,
        .velocity_millirad_s = 0,
        .mos_temperature_c = 25u,
        .rotor_temperature_c = 25u,
        .feedback_valid = true,
        .can_warning = false,
        .can_passive = false,
        .can_bus_off = false,
    };
}

static MotionSafetySnapshot safe_enabled_snapshot(uint32_t now_ms)
{
    MotionSafetySnapshot safety = safe_disabled_snapshot(now_ms);

    safety.motor_state = 1u;
    return safety;
}

static MotionDecision arm_and_start(MotionController *controller,
                                    MotionSafetySnapshot *safety)
{
    MotionDecision decision;

    motion_controller_init(controller);
    decision = motion_controller_command(controller, 'A', safety);
    assert(decision.event == MOTION_EVENT_ARMED);
    return motion_controller_command(controller, 'G', safety);
}

static void assert_shutdown_after_next_millisecond(
    MotionController *controller, MotionSafetySnapshot safety)
{
    MotionDecision decision;

    safety.now_ms += 1u;
    decision = motion_controller_step(controller, &safety);
    assert(decision.action == MOTION_ACTION_DISABLE);
    assert(controller->state == MOTION_IDLE_DISABLED);
}

static void test_normalizes_commands_and_ignores_line_endings(void)
{
    MotionSafetySnapshot safety = safe_disabled_snapshot(1000u);
    MotionController controller;
    MotionDecision decision;

    motion_controller_init(&controller);
    decision = motion_controller_command(&controller, 's', &safety);
    assert(decision.event == MOTION_EVENT_STATUS);
    assert(controller.state == MOTION_IDLE_DISABLED);

    decision = motion_controller_command(&controller, '\r', &safety);
    assert(decision.action == MOTION_ACTION_NONE);
    assert(decision.event == MOTION_EVENT_NONE);
    decision = motion_controller_command(&controller, '\n', &safety);
    assert(decision.action == MOTION_ACTION_NONE);
    assert(decision.event == MOTION_EVENT_NONE);

    decision = motion_controller_command(&controller, 'a', &safety);
    assert(decision.event == MOTION_EVENT_ARMED);
    assert(controller.state == MOTION_ARMED);

    decision = motion_controller_command(&controller, 'g', &safety);
    assert(decision.action == MOTION_ACTION_ENABLE);
    assert(decision.event == MOTION_EVENT_START_REQUESTED);
    assert(controller.state == MOTION_ENABLE_WAIT);
    assert(!controller.armed);
}

static void test_rejects_start_without_a_fresh_arm(void)
{
    MotionSafetySnapshot safety = safe_disabled_snapshot(1000u);
    MotionController controller;
    MotionDecision decision;

    motion_controller_init(&controller);
    decision = motion_controller_command(&controller, 'G', &safety);
    assert(decision.action == MOTION_ACTION_NONE);
    assert(decision.event == MOTION_EVENT_START_REJECTED);
    assert(strcmp(decision.reason, "SEND_A_FIRST") == 0);
}

static void test_arm_boundary_and_one_shot_consumption(void)
{
    MotionSafetySnapshot safety = safe_disabled_snapshot(0u);
    MotionController controller;
    MotionDecision decision;

    motion_controller_init(&controller);
    assert(motion_controller_command(&controller, 'A', &safety).event ==
           MOTION_EVENT_ARMED);

    safety.now_ms = 9999u;
    decision = motion_controller_command(&controller, 'G', &safety);
    assert(decision.action == MOTION_ACTION_ENABLE);
    assert(!controller.armed);

    decision = motion_controller_command(&controller, 'G', &safety);
    assert(decision.event == MOTION_EVENT_START_REJECTED);
    assert(strcmp(decision.reason, "SEND_A_FIRST") == 0);

    motion_controller_init(&controller);
    safety.now_ms = 0u;
    assert(motion_controller_command(&controller, 'A', &safety).event ==
           MOTION_EVENT_ARMED);
    safety.now_ms = 10000u;
    decision = motion_controller_command(&controller, 'G', &safety);
    assert(decision.event == MOTION_EVENT_ARM_TIMEOUT);
    assert(strcmp(decision.reason, "ARM_TIMEOUT") == 0);
    assert(controller.state == MOTION_IDLE_DISABLED);
}

static void test_arm_timeout_handles_tick_wraparound(void)
{
    MotionSafetySnapshot safety = safe_disabled_snapshot(UINT32_MAX - 5000u);
    MotionController controller;
    MotionDecision decision;

    motion_controller_init(&controller);
    assert(motion_controller_command(&controller, 'A', &safety).event ==
           MOTION_EVENT_ARMED);

    safety.now_ms = 4998u;
    decision = motion_controller_command(&controller, 'G', &safety);
    assert(decision.action == MOTION_ACTION_ENABLE);

    motion_controller_init(&controller);
    safety.now_ms = UINT32_MAX - 5000u;
    assert(motion_controller_command(&controller, 'A', &safety).event ==
           MOTION_EVENT_ARMED);
    safety.now_ms = 4999u;
    decision = motion_controller_command(&controller, 'G', &safety);
    assert(decision.event == MOTION_EVENT_ARM_TIMEOUT);
}

static void assert_start_rejected_for_unsafe_snapshot(
    MotionSafetySnapshot safety, const char *expected_reason)
{
    MotionController controller;
    MotionDecision decision;

    motion_controller_init(&controller);
    assert(motion_controller_command(&controller, 'A', &safety).event ==
           MOTION_EVENT_ARMED);
    decision = motion_controller_command(&controller, 'G', &safety);
    assert(decision.action == MOTION_ACTION_NONE);
    assert(decision.event == MOTION_EVENT_START_REJECTED);
    assert(strcmp(decision.reason, expected_reason) == 0);
    assert(controller.state == MOTION_ARMED);
}

static void test_rejects_each_unsafe_start_condition(void)
{
    MotionSafetySnapshot safety = safe_disabled_snapshot(100u);

    safety.parameter_mask = 0x0fu;
    assert_start_rejected_for_unsafe_snapshot(safety, "PARAMETERS_INCOMPLETE");
    safety = safe_disabled_snapshot(100u);
    safety.control_mode = 2u;
    assert_start_rejected_for_unsafe_snapshot(safety, "MODE_NOT_VELOCITY");
    safety = safe_disabled_snapshot(100u);
    safety.feedback_valid = false;
    assert_start_rejected_for_unsafe_snapshot(safety, "FEEDBACK_STALE");
    safety = safe_disabled_snapshot(100u);
    safety.feedback_age_ms = 501u;
    assert_start_rejected_for_unsafe_snapshot(safety, "FEEDBACK_STALE");
    safety = safe_disabled_snapshot(100u);
    safety.motor_state = 1u;
    assert_start_rejected_for_unsafe_snapshot(safety, "STATE_NOT_DISABLED");
    safety = safe_disabled_snapshot(100u);
    safety.velocity_millirad_s = 100;
    assert_start_rejected_for_unsafe_snapshot(safety, "SPEED_NOT_ZERO");
    safety = safe_disabled_snapshot(100u);
    safety.velocity_millirad_s = -100;
    assert_start_rejected_for_unsafe_snapshot(safety, "SPEED_NOT_ZERO");
    safety = safe_disabled_snapshot(100u);
    safety.mos_temperature_c = 60u;
    assert_start_rejected_for_unsafe_snapshot(safety, "TEMPERATURE_LIMIT");
    safety = safe_disabled_snapshot(100u);
    safety.rotor_temperature_c = 60u;
    assert_start_rejected_for_unsafe_snapshot(safety, "TEMPERATURE_LIMIT");
    safety = safe_disabled_snapshot(100u);
    safety.can_warning = true;
    assert_start_rejected_for_unsafe_snapshot(safety, "CAN_NOT_ACTIVE");
    safety = safe_disabled_snapshot(100u);
    safety.can_passive = true;
    assert_start_rejected_for_unsafe_snapshot(safety, "CAN_NOT_ACTIVE");
    safety = safe_disabled_snapshot(100u);
    safety.can_bus_off = true;
    assert_start_rejected_for_unsafe_snapshot(safety, "CAN_NOT_ACTIVE");
}

static void test_emergency_stop_requests_zero_from_every_state(void)
{
    static const MotionState states[] = {
        MOTION_IDLE_DISABLED,
        MOTION_ARMED,
        MOTION_ENABLE_WAIT,
        MOTION_RUNNING,
        MOTION_ZERO_HOLD,
        MOTION_FAULT_ZERO,
        MOTION_FAULT_DISABLE,
    };
    MotionSafetySnapshot safety = safe_disabled_snapshot(100u);

    for (uint8_t index = 0u; index < sizeof(states) / sizeof(states[0]); ++index) {
        MotionController controller;
        MotionDecision decision;

        motion_controller_init(&controller);
        controller.state = states[index];
        controller.armed = true;
        controller.last_action_ms = 100u;
        decision = motion_controller_command(&controller, 'x', &safety);
        assert(decision.action == MOTION_ACTION_ZERO_VELOCITY);
        assert(decision.event == MOTION_EVENT_EMERGENCY_STOP);
        assert(!controller.armed);
        assert(controller.state == MOTION_FAULT_DISABLE);
        assert_shutdown_after_next_millisecond(&controller, safety);
    }
}

static void test_runs_the_fixed_positive_timeline(void)
{
    MotionSafetySnapshot safety = safe_disabled_snapshot(0u);
    MotionController controller;
    MotionDecision decision;
    uint32_t now_ms;

    decision = arm_and_start(&controller, &safety);
    assert(decision.action == MOTION_ACTION_ENABLE);
    assert(controller.state == MOTION_ENABLE_WAIT);

    safety = safe_enabled_snapshot(100u);
    decision = motion_controller_step(&controller, &safety);
    assert(decision.action == MOTION_ACTION_POSITIVE_VELOCITY);
    assert(decision.event == MOTION_EVENT_RUNNING);
    assert(controller.state == MOTION_RUNNING);

    for (now_ms = 110u; now_ms <= 1090u; now_ms += 10u) {
        safety.now_ms = now_ms;
        decision = motion_controller_step(&controller, &safety);
        assert(decision.action == MOTION_ACTION_POSITIVE_VELOCITY);
        assert(controller.state == MOTION_RUNNING);
    }

    safety.now_ms = 1100u;
    decision = motion_controller_step(&controller, &safety);
    assert(decision.action == MOTION_ACTION_ZERO_VELOCITY);
    assert(decision.event == MOTION_EVENT_ZERO_HOLD);
    assert(controller.state == MOTION_ZERO_HOLD);

    for (now_ms = 1110u; now_ms <= 1290u; now_ms += 10u) {
        safety.now_ms = now_ms;
        decision = motion_controller_step(&controller, &safety);
        assert(decision.action == MOTION_ACTION_ZERO_VELOCITY);
        assert(controller.state == MOTION_ZERO_HOLD);
    }

    safety.now_ms = 1300u;
    decision = motion_controller_step(&controller, &safety);
    assert(decision.action == MOTION_ACTION_DISABLE);
    assert(decision.event == MOTION_EVENT_COMPLETE);
    assert(controller.state == MOTION_IDLE_DISABLED);
}

static void assert_runtime_trip(MotionSafetySnapshot safety,
                                const char *expected_reason)
{
    MotionController controller;
    MotionDecision decision;
    MotionSafetySnapshot startup = safe_disabled_snapshot(0u);

    assert(arm_and_start(&controller, &startup).action ==
           MOTION_ACTION_ENABLE);
    startup = safe_enabled_snapshot(100u);
    assert(motion_controller_step(&controller, &startup).action ==
           MOTION_ACTION_POSITIVE_VELOCITY);
    safety.now_ms = 110u;
    decision = motion_controller_step(&controller, &safety);
    assert(decision.action == MOTION_ACTION_ZERO_VELOCITY);
    assert(decision.event == MOTION_EVENT_SAFETY_TRIP);
    assert(strcmp(decision.reason, expected_reason) == 0);
    assert(controller.state == MOTION_FAULT_DISABLE);
    assert_shutdown_after_next_millisecond(&controller, safety);
}

static void test_runtime_safety_trips_zero_then_disable(void)
{
    MotionSafetySnapshot safety = safe_enabled_snapshot(110u);

    safety.feedback_age_ms = 101u;
    assert_runtime_trip(safety, "FEEDBACK_STALE");
    safety = safe_enabled_snapshot(110u);
    safety.velocity_millirad_s = 801;
    assert_runtime_trip(safety, "SPEED_LIMIT");
    safety = safe_enabled_snapshot(110u);
    safety.mos_temperature_c = 60u;
    assert_runtime_trip(safety, "TEMPERATURE_LIMIT");
    safety = safe_enabled_snapshot(110u);
    safety.rotor_temperature_c = 60u;
    assert_runtime_trip(safety, "TEMPERATURE_LIMIT");
    safety = safe_enabled_snapshot(110u);
    safety.motor_state = 0u;
    assert_runtime_trip(safety, "STATE_NOT_ENABLED");
    safety = safe_enabled_snapshot(110u);
    safety.can_passive = true;
    assert_runtime_trip(safety, "CAN_NOT_ACTIVE");
    safety = safe_enabled_snapshot(110u);
    safety.can_bus_off = true;
    assert_runtime_trip(safety, "CAN_NOT_ACTIVE");
}

static void test_transmit_failure_requests_zero_then_disable(void)
{
    MotionController controller;
    MotionSafetySnapshot safety = safe_disabled_snapshot(0u);
    MotionDecision decision;

    assert(arm_and_start(&controller, &safety).action == MOTION_ACTION_ENABLE);
    safety = safe_enabled_snapshot(100u);
    assert(motion_controller_step(&controller, &safety).action ==
           MOTION_ACTION_POSITIVE_VELOCITY);
    decision = motion_controller_tx_failed(&controller);
    assert(decision.action == MOTION_ACTION_ZERO_VELOCITY);
    assert(decision.event == MOTION_EVENT_TX_FAILURE);
    assert(controller.state == MOTION_FAULT_DISABLE);
    assert_shutdown_after_next_millisecond(&controller, safety);
}

static void test_failed_zero_command_immediately_falls_back_to_disable(void)
{
    MotionController controller;
    MotionSafetySnapshot safety = safe_disabled_snapshot(100u);
    MotionDecision decision;

    motion_controller_init(&controller);
    assert(motion_controller_command(&controller, 'X', &safety).action ==
           MOTION_ACTION_ZERO_VELOCITY);
    decision = motion_controller_tx_failed(&controller);
    assert(decision.action == MOTION_ACTION_DISABLE);
    assert(decision.event == MOTION_EVENT_TX_FAILURE);
    assert(controller.state == MOTION_IDLE_DISABLED);
}

int main(void)
{
    test_normalizes_commands_and_ignores_line_endings();
    test_rejects_start_without_a_fresh_arm();
    test_arm_boundary_and_one_shot_consumption();
    test_arm_timeout_handles_tick_wraparound();
    test_rejects_each_unsafe_start_condition();
    test_emergency_stop_requests_zero_from_every_state();
    test_runs_the_fixed_positive_timeline();
    test_runtime_safety_trips_zero_then_disable();
    test_transmit_failure_requests_zero_then_disable();
    test_failed_zero_command_immediately_falls_back_to_disable();
    return 0;
}
