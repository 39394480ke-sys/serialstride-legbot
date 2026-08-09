#include <assert.h>
#include <string.h>

#include "dm4310_controller.h"

static Dm4310SafetySnapshot safe_snapshot(uint32_t now_ms)
{
    Dm4310SafetySnapshot safety = {
        .now_ms = now_ms,
        .feedback_age_ms = 0u,
        .motor_state = 0u,
        .powered = true,
        .probe_active = true,
        .parameters_valid = true,
        .feedback_valid = true,
    };

    return safety;
}

static void test_arm_and_short_pulse_sequence(void)
{
    Dm4310Controller controller;
    Dm4310SafetySnapshot safety = safe_snapshot(0u);
    Dm4310MotionDecision result;

    dm4310_controller_init(&controller);
    result = dm4310_controller_command(&controller, 'A', &safety);
    assert(result.event == DM4310_EVENT_ARMED);

    safety.now_ms = 1u;
    result = dm4310_controller_command(&controller, 'G', &safety);
    assert(result.send_enable && result.target_velocity_millirad_s == 200);
    assert(controller.state == DM4310_MOTION_ENABLE_WAIT);

    safety.now_ms = 101u;
    safety.motor_state = 1u;
    result = dm4310_controller_step(&controller, &safety);
    assert(result.event == DM4310_EVENT_RUNNING && result.send_mit);

    safety.now_ms = 601u;
    result = dm4310_controller_step(&controller, &safety);
    assert(result.event == DM4310_EVENT_ZERO_HOLD && result.send_mit);
    assert(result.target_velocity_millirad_s == 0);
    assert(controller.target_velocity_millirad_s == 0);

    safety.now_ms = 801u;
    result = dm4310_controller_step(&controller, &safety);
    assert(result.event == DM4310_EVENT_COMPLETE && result.send_disable);
    assert(controller.state == DM4310_MOTION_DISABLED);
}

static void test_enable_deadline_fails_closed(void)
{
    Dm4310Controller controller;
    Dm4310SafetySnapshot safety = safe_snapshot(10u);
    Dm4310MotionDecision result;

    dm4310_controller_init(&controller);
    (void)dm4310_controller_command(&controller, 'A', &safety);
    safety.now_ms = 11u;
    (void)dm4310_controller_command(&controller, 'N', &safety);

    safety.now_ms = 112u;
    safety.motor_state = 1u;
    result = dm4310_controller_step(&controller, &safety);
    assert(result.event == DM4310_EVENT_SAFETY_TRIP);
    assert(result.send_mit && result.send_disable && result.cut_power);
    assert(strcmp(result.reason, "ENABLE_TIMEOUT") == 0);
}

static void test_start_guards(void)
{
    Dm4310Controller controller;
    Dm4310SafetySnapshot safety = safe_snapshot(0u);
    Dm4310MotionDecision result;

    dm4310_controller_init(&controller);
    (void)dm4310_controller_command(&controller, 'A', &safety);
    safety.parameters_valid = false;
    result = dm4310_controller_command(&controller, 'B', &safety);
    assert(result.event == DM4310_EVENT_REJECTED);
    assert(strcmp(result.reason, "PARAMETERS_MISMATCH") == 0);

    safety.parameters_valid = true;
    safety.velocity_millirad_s = 100;
    result = dm4310_controller_command(&controller, 'B', &safety);
    assert(strcmp(result.reason, "SPEED_NOT_ZERO") == 0);
}

static void test_runtime_guards_and_emergency(void)
{
    Dm4310Controller controller;
    Dm4310SafetySnapshot safety = safe_snapshot(0u);
    Dm4310MotionDecision result;

    dm4310_controller_init(&controller);
    (void)dm4310_controller_command(&controller, 'A', &safety);
    safety.now_ms = 1u;
    (void)dm4310_controller_command(&controller, 'G', &safety);
    safety.now_ms = 2u;
    safety.motor_state = 1u;
    (void)dm4310_controller_step(&controller, &safety);

    safety.now_ms = 12u;
    safety.torque_millinewton_m = 501;
    result = dm4310_controller_step(&controller, &safety);
    assert(result.cut_power && result.send_mit && result.send_disable);
    assert(strcmp(result.reason, "TORQUE_LIMIT") == 0);

    dm4310_controller_init(&controller);
    result = dm4310_controller_command(&controller, 'X', &safety);
    assert(result.event == DM4310_EVENT_EMERGENCY_STOP);
    assert(result.cut_power && result.send_mit && result.send_disable);

    result = dm4310_controller_tx_failed(&controller);
    assert(result.event == DM4310_EVENT_TX_FAILURE && result.cut_power);
}

int main(void)
{
    test_arm_and_short_pulse_sequence();
    test_enable_deadline_fails_closed();
    test_start_guards();
    test_runtime_guards_and_emergency();
    return 0;
}
