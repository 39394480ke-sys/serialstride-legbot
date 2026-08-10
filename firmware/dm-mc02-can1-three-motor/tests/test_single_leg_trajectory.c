#include <assert.h>
#include <string.h>

#include "single_leg_calibration.h"
#include "single_leg_trajectory.h"

static SingleLegTrajectorySafety safe(uint32_t now_ms)
{
    return (SingleLegTrajectorySafety){
        .now_ms = now_ms,
        .powered = true,
        .probe_active = true,
        .parameters_valid = true,
        .feedback_fresh = true,
        .joints_disabled = true,
        .wheel_disabled = true,
        .start_speed_zero = true,
        .runtime_speed_safe = true,
        .runtime_torque_safe = true,
        .temperature_safe = true,
        .can_active = true,
        .states_normal = true,
        .joint_a_position_millirad = 1008,
        .joint_b_position_millirad = -1307,
    };
}

static void test_midpoints_and_common_cubic_progress(void)
{
    SingleLegTrajectory trajectory;
    SingleLegTrajectorySafety safety = safe(0u);
    SingleLegTrajectoryDecision result;

    single_leg_trajectory_init(&trajectory);
    assert(single_leg_trajectory_arm(&trajectory, &safety).event ==
           SINGLE_LEG_TRAJECTORY_EVENT_ARMED);
    safety.now_ms = 1u;
    result = single_leg_trajectory_start(
        &trajectory, SINGLE_LEG_TARGET_MID_CROUCH, &safety);
    assert(result.send_enable_joints);
    assert(trajectory.target_a_millirad == 835);
    assert(trajectory.target_b_millirad == -1129);

    safety.now_ms = 50u;
    safety.joints_disabled = false;
    safety.joints_enabled = true;
    result = single_leg_trajectory_step(&trajectory, &safety);
    assert(result.send_position_joints);
    assert(result.joint_a_target_millirad == 1008);
    assert(result.joint_b_target_millirad == -1307);

    safety.now_ms = 2050u;
    result = single_leg_trajectory_step(&trajectory, &safety);
    assert(result.send_position_joints);
    assert(result.joint_a_target_millirad == 922);
    assert(result.joint_b_target_millirad == -1218);
    assert(result.joint_a_velocity_millirad_s < 0);
    assert(result.joint_b_velocity_millirad_s > 0);
}

static void test_calibration_constants_are_distinct_and_exact(void)
{
    assert(SINGLE_LEG_JOINT_A_MECHANICAL_MIN_MILLIRAD == 662);
    assert(SINGLE_LEG_JOINT_A_SOFT_MIN_MILLIRAD == 731);
    assert(SINGLE_LEG_JOINT_A_MID_CROUCH_MILLIRAD == 835);
    assert(SINGLE_LEG_JOINT_A_ZERO_MILLIRAD == 1008);
    assert(SINGLE_LEG_JOINT_A_MID_EXTEND_MILLIRAD == 1181);
    assert(SINGLE_LEG_JOINT_A_SOFT_MAX_MILLIRAD == 1285);
    assert(SINGLE_LEG_JOINT_A_MECHANICAL_MAX_MILLIRAD == 1354);

    assert(SINGLE_LEG_JOINT_B_MECHANICAL_MIN_MILLIRAD == -1662);
    assert(SINGLE_LEG_JOINT_B_SOFT_MIN_MILLIRAD == -1591);
    assert(SINGLE_LEG_JOINT_B_MID_EXTEND_MILLIRAD == -1484);
    assert(SINGLE_LEG_JOINT_B_ZERO_MILLIRAD == -1307);
    assert(SINGLE_LEG_JOINT_B_MID_CROUCH_MILLIRAD == -1129);
    assert(SINGLE_LEG_JOINT_B_SOFT_MAX_MILLIRAD == -1022);
    assert(SINGLE_LEG_JOINT_B_MECHANICAL_MAX_MILLIRAD == -951);
}

static void test_wheel_and_soft_limits_fail_closed(void)
{
    SingleLegTrajectory trajectory;
    SingleLegTrajectorySafety safety = safe(0u);
    SingleLegTrajectoryDecision result;

    single_leg_trajectory_init(&trajectory);
    safety.wheel_disabled = false;
    result = single_leg_trajectory_arm(&trajectory, &safety);
    assert(result.event == SINGLE_LEG_TRAJECTORY_EVENT_REJECTED);
    assert(strcmp(result.reason, "MOTOR_NOT_DISABLED") == 0);

    safety = safe(0u);
    (void)single_leg_trajectory_arm(&trajectory, &safety);
    safety.now_ms = 1u;
    (void)single_leg_trajectory_start(
        &trajectory, SINGLE_LEG_TARGET_MID_EXTEND, &safety);
    safety.now_ms = 10u;
    safety.joints_disabled = false;
    safety.joints_enabled = true;
    (void)single_leg_trajectory_step(&trajectory, &safety);
    safety.now_ms = 20u;
    safety.joint_a_position_millirad = 1300;
    result = single_leg_trajectory_step(&trajectory, &safety);
    assert(result.event == SINGLE_LEG_TRAJECTORY_EVENT_SAFETY_TRIP);
    assert(result.target_valid);
    assert(result.target == SINGLE_LEG_TARGET_MID_EXTEND);
    assert(result.send_disable_all && result.cut_power);
    assert(strcmp(result.reason, "SOFT_LIMIT") == 0);
}

static void test_complete_event_preserves_original_target(void)
{
    SingleLegTrajectory trajectory;
    SingleLegTrajectorySafety safety = safe(0u);
    SingleLegTrajectoryDecision result;

    single_leg_trajectory_init(&trajectory);
    (void)single_leg_trajectory_arm(&trajectory, &safety);
    safety.now_ms = 1u;
    result = single_leg_trajectory_start(
        &trajectory, SINGLE_LEG_TARGET_MID_EXTEND, &safety);
    assert(result.target_valid);
    assert(result.target == SINGLE_LEG_TARGET_MID_EXTEND);

    safety.now_ms = 2u;
    safety.joints_disabled = false;
    safety.joints_enabled = true;
    (void)single_leg_trajectory_step(&trajectory, &safety);
    safety.now_ms = 4002u;
    (void)single_leg_trajectory_step(&trajectory, &safety);
    safety.joint_a_position_millirad =
        SINGLE_LEG_JOINT_A_MID_EXTEND_MILLIRAD;
    safety.joint_b_position_millirad =
        SINGLE_LEG_JOINT_B_MID_EXTEND_MILLIRAD;
    safety.now_ms = 4302u;
    result = single_leg_trajectory_step(&trajectory, &safety);
    assert(result.event == SINGLE_LEG_TRAJECTORY_EVENT_DISABLE_WAIT);

    safety.now_ms = 4303u;
    safety.joints_disabled = true;
    safety.joints_enabled = false;
    result = single_leg_trajectory_step(&trajectory, &safety);
    assert(result.event == SINGLE_LEG_TRAJECTORY_EVENT_COMPLETE);
    assert(result.target_valid);
    assert(result.target == SINGLE_LEG_TARGET_MID_EXTEND);
    assert(strcmp(single_leg_trajectory_target_name(result.target),
                  "MID_EXTEND") == 0);
}

static void test_midpoint_move_requires_stand_start(void)
{
    SingleLegTrajectory trajectory;
    SingleLegTrajectorySafety safety = safe(0u);
    SingleLegTrajectoryDecision result;

    safety.joint_a_position_millirad = 1133;
    safety.joint_b_position_millirad = -1574;
    single_leg_trajectory_init(&trajectory);
    assert(single_leg_trajectory_arm(&trajectory, &safety).event ==
           SINGLE_LEG_TRAJECTORY_EVENT_ARMED);
    safety.now_ms = 1u;
    result = single_leg_trajectory_start(
        &trajectory, SINGLE_LEG_TARGET_MID_CROUCH, &safety);
    assert(result.event == SINGLE_LEG_TRAJECTORY_EVENT_REJECTED);
    assert(strcmp(result.reason, "START_NOT_STAND") == 0);
    assert(!single_leg_trajectory_active(&trajectory));
}

int main(void)
{
    test_midpoints_and_common_cubic_progress();
    test_calibration_constants_are_distinct_and_exact();
    test_wheel_and_soft_limits_fail_closed();
    test_complete_event_preserves_original_target();
    test_midpoint_move_requires_stand_start();
    return 0;
}
