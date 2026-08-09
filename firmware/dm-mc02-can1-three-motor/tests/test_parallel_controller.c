#include <assert.h>
#include <string.h>

#include "parallel_controller.h"

static ParallelSafetySnapshot safe_snapshot(uint32_t now_ms)
{
    return (ParallelSafetySnapshot){
        .now_ms = now_ms,
        .powered = true,
        .probe_active = true,
        .parameters_valid = true,
        .feedback_fresh = true,
        .all_disabled = true,
        .start_speed_zero = true,
        .runtime_speed_safe = true,
        .runtime_torque_safe = true,
        .temperature_safe = true,
        .can_active = true,
        .state_normal = true,
    };
}

static void test_parallel_sequence_waits_for_disable_confirmation(void)
{
    ParallelController controller;
    ParallelSafetySnapshot safety = safe_snapshot(0u);
    ParallelDecision result;

    parallel_controller_init(&controller);
    assert(parallel_controller_command(&controller, 'A', &safety).event ==
           PARALLEL_EVENT_ARMED);
    safety.now_ms = 1u;
    result = parallel_controller_command(&controller, 'G', &safety);
    assert(result.send_enable_all && result.direction == 1);

    safety.now_ms = 2u;
    safety.all_disabled = false;
    safety.all_enabled = true;
    result = parallel_controller_step(&controller, &safety);
    assert(result.event == PARALLEL_EVENT_RUNNING);
    assert(result.send_velocity_all && result.direction == 1);

    safety.now_ms = 1002u;
    result = parallel_controller_step(&controller, &safety);
    assert(result.event == PARALLEL_EVENT_ZERO_HOLD);
    assert(result.send_velocity_all && result.direction == 0);

    safety.now_ms = 1202u;
    result = parallel_controller_step(&controller, &safety);
    assert(result.event == PARALLEL_EVENT_DISABLE_WAIT);
    assert(result.send_disable_all);

    safety.now_ms = 1300u;
    result = parallel_controller_step(&controller, &safety);
    assert(result.event == PARALLEL_EVENT_NONE);
    safety.now_ms = 1301u;
    safety.all_enabled = false;
    safety.all_disabled = true;
    result = parallel_controller_step(&controller, &safety);
    assert(result.event == PARALLEL_EVENT_COMPLETE);
    assert(controller.state == PARALLEL_DISABLED);
}

static void test_partial_transition_is_graced_then_times_out(void)
{
    ParallelController controller;
    ParallelSafetySnapshot safety = safe_snapshot(0u);
    ParallelDecision result;

    parallel_controller_init(&controller);
    (void)parallel_controller_command(&controller, 'A', &safety);
    (void)parallel_controller_command(&controller, 'G', &safety);
    safety.now_ms = 2u;
    safety.all_disabled = false;
    result = parallel_controller_step(&controller, &safety);
    assert(result.event == PARALLEL_EVENT_NONE);
    safety.now_ms = 100u;
    result = parallel_controller_step(&controller, &safety);
    assert(result.cut_power && result.send_disable_all);
    assert(strcmp(result.reason, "ENABLE_TIMEOUT") == 0);

    controller.state = PARALLEL_DISABLE_WAIT;
    controller.phase_started_ms = 100u;
    safety = safe_snapshot(300u);
    safety.all_disabled = false;
    safety.all_enabled = true;
    result = parallel_controller_step(&controller, &safety);
    assert(result.cut_power && result.send_disable_all);
    assert(strcmp(result.reason, "DISABLE_TIMEOUT") == 0);
}

int main(void)
{
    test_parallel_sequence_waits_for_disable_confirmation();
    test_partial_transition_is_graced_then_times_out();
    return 0;
}
