#include <assert.h>
#include <string.h>

#include "single_leg_bringup.h"

static SingleLegMotorSnapshot safe_snapshot(int32_t position_millirad)
{
    return (SingleLegMotorSnapshot){
        .online = true,
        .feedback_valid = true,
        .parameters_valid = true,
        .powered = true,
        .probe_active = true,
        .can_active = true,
        .motor_state = 0u,
        .position_millirad = position_millirad,
        .mos_temperature_c = 25u,
        .rotor_temperature_c = 25u,
        .feedback_age_ms = 1u,
    };
}

static SingleLegCommand parse(const char *text)
{
    SingleLegCommandParser parser;
    SingleLegCommand command = SINGLE_LEG_COMMAND_NONE;
    SingleLegParseResult result = SINGLE_LEG_PARSE_PENDING;
    size_t index;

    single_leg_command_parser_init(&parser);
    for (index = 0u; text[index] != '\0'; ++index)
        result = single_leg_command_parser_feed(
            &parser, (uint8_t)text[index], &command);
    assert(result == SINGLE_LEG_PARSE_READY);
    return command;
}

static void test_parses_required_line_commands(void)
{
    assert(parse("status\n") == SINGLE_LEG_COMMAND_STATUS);
    assert(parse("disable-all\r") == SINGLE_LEG_COMMAND_DISABLE_ALL);
    assert(parse("read-raw\n") == SINGLE_LEG_COMMAND_READ_RAW);
    assert(parse("capture-stand\n") == SINGLE_LEG_COMMAND_CAPTURE_STAND);
    assert(parse("capture-crouch\n") == SINGLE_LEG_COMMAND_CAPTURE_CROUCH);
    assert(parse("capture-extend\n") == SINGLE_LEG_COMMAND_CAPTURE_EXTEND);
    assert(parse("select-a\n") == SINGLE_LEG_COMMAND_SELECT_A);
    assert(parse("select-b\n") == SINGLE_LEG_COMMAND_SELECT_B);
    assert(parse("select-wheel\n") == SINGLE_LEG_COMMAND_SELECT_WHEEL);
    assert(parse("arm\n") == SINGLE_LEG_COMMAND_ARM);
    assert(parse("jog-positive\n") == SINGLE_LEG_COMMAND_JOG_POSITIVE);
    assert(parse("jog-negative\n") == SINGLE_LEG_COMMAND_JOG_NEGATIVE);
    assert(parse("stop-all\n") == SINGLE_LEG_COMMAND_STOP_ALL);
    assert(parse("power-arm\n") == SINGLE_LEG_COMMAND_POWER_ARM);
    assert(parse("power-on\n") == SINGLE_LEG_COMMAND_POWER_ON);
    assert(parse("probe\n") == SINGLE_LEG_COMMAND_PROBE);
    assert(parse("phase10-arm\n") == SINGLE_LEG_COMMAND_PHASE10_ARM);
    assert(parse("move-stand\n") == SINGLE_LEG_COMMAND_MOVE_STAND);
    assert(parse("move-mid-crouch\n") ==
           SINGLE_LEG_COMMAND_MOVE_MID_CROUCH);
    assert(parse("move-mid-extend\n") ==
           SINGLE_LEG_COMMAND_MOVE_MID_EXTEND);
}

static void test_rejects_unknown_and_overlong_commands(void)
{
    SingleLegCommandParser parser;
    SingleLegCommand command;
    SingleLegParseResult result = SINGLE_LEG_PARSE_PENDING;
    const char *unknown = "move-fast\n";
    size_t index;

    single_leg_command_parser_init(&parser);
    for (index = 0u; unknown[index] != '\0'; ++index)
        result = single_leg_command_parser_feed(
            &parser, (uint8_t)unknown[index], &command);
    assert(result == SINGLE_LEG_PARSE_ERROR);

    single_leg_command_parser_init(&parser);
    for (index = 0u; index < SINGLE_LEG_COMMAND_CAPACITY + 5u; ++index)
        (void)single_leg_command_parser_feed(&parser, 'a', &command);
    assert(single_leg_command_parser_feed(&parser, '\n', &command) ==
           SINGLE_LEG_PARSE_ERROR);
}

static void test_arm_is_one_shot_short_and_anchors_limits(void)
{
    SingleLegBringup bringup;
    SingleLegMotorSnapshot snapshot = safe_snapshot(1000);
    const char *reason = NULL;

    single_leg_bringup_init(&bringup);
    assert(single_leg_bringup_select(&bringup, MOTOR_ROLE_JOINT_A, false,
                                     &reason));
    assert(single_leg_bringup_arm(&bringup, &snapshot, 100u, &reason));
    assert(bringup.commissioning_min_millirad == 900);
    assert(bringup.commissioning_max_millirad == 1100);
    assert(single_leg_bringup_request_jog(&bringup, &snapshot, 1, 3100u,
                                          &reason));
    assert(!bringup.armed && bringup.jog_direction == 1);
    assert(!single_leg_bringup_request_jog(&bringup, &snapshot, 1, 3100u,
                                           &reason));
    assert(strcmp(reason, "ARM_REQUIRED") == 0);

    single_leg_bringup_disarm(&bringup);
    snapshot.position_millirad = 1050;
    assert(single_leg_bringup_arm(&bringup, &snapshot, 3200u, &reason));
    assert(bringup.anchor_millirad == 1000);
    assert(bringup.commissioning_max_millirad == 1100);

    assert(single_leg_bringup_arm(&bringup, &snapshot, 4000u, &reason));
    assert(!single_leg_bringup_request_jog(&bringup, &snapshot, 1, 7001u,
                                           &reason));
    assert(strcmp(reason, "ARM_TIMEOUT") == 0);
}

static void test_limits_and_runtime_safety_fail_closed(void)
{
    SingleLegBringup bringup;
    SingleLegMotorSnapshot snapshot = safe_snapshot(-1307);
    const char *reason = NULL;

    single_leg_bringup_init(&bringup);
    assert(single_leg_bringup_select(&bringup, MOTOR_ROLE_JOINT_B, false,
                                     &reason));
    assert(single_leg_bringup_arm(&bringup, &snapshot, 0u, &reason));
    snapshot.position_millirad = -1197;
    assert(!single_leg_bringup_request_jog(&bringup, &snapshot, 1, 1u,
                                           &reason));
    assert(strcmp(reason, "COMMISSIONING_LIMIT") == 0);

    snapshot = safe_snapshot(-1307);
    assert(single_leg_bringup_arm(&bringup, &snapshot, 2u, &reason));
    assert(single_leg_bringup_request_jog(&bringup, &snapshot, -1, 3u,
                                          &reason));
    snapshot.motor_state = 1u;
    snapshot.torque_millinewton_m = 501;
    assert(strcmp(single_leg_bringup_motion_fault(&bringup, &snapshot),
                  "TORQUE_LIMIT") == 0);
    snapshot.torque_millinewton_m = 0;
    snapshot.position_millirad = -1408;
    assert(strcmp(single_leg_bringup_motion_fault(&bringup, &snapshot),
                  "COMMISSIONING_LIMIT") == 0);
}

static void test_wheel_uses_time_and_speed_guards_not_joint_position_window(void)
{
    SingleLegBringup bringup;
    SingleLegMotorSnapshot snapshot = safe_snapshot(0);
    const char *reason = NULL;

    single_leg_bringup_init(&bringup);
    assert(single_leg_bringup_select(&bringup, MOTOR_ROLE_WHEEL, false,
                                     &reason));
    assert(single_leg_bringup_arm(&bringup, &snapshot, 0u, &reason));
    assert(single_leg_bringup_request_jog(&bringup, &snapshot, 1, 1u,
                                          &reason));

    snapshot.motor_state = 1u;
    snapshot.position_millirad = 500;
    snapshot.velocity_millirad_s = 200;
    assert(single_leg_bringup_motion_fault(&bringup, &snapshot) == NULL);

    snapshot.velocity_millirad_s = 800;
    assert(strcmp(single_leg_bringup_motion_fault(&bringup, &snapshot),
                  "SPEED_LIMIT") == 0);
}

static void test_global_joint_soft_limits(void)
{
    SingleLegBringup bringup;
    SingleLegMotorSnapshot snapshot = safe_snapshot(750);
    const char *reason = NULL;
    int32_t minimum = 0;
    int32_t maximum = 0;

    assert(single_leg_joint_soft_limits(MOTOR_ROLE_JOINT_A, &minimum,
                                        &maximum));
    assert(minimum == 731 && maximum == 1285);
    assert(single_leg_joint_soft_limits(MOTOR_ROLE_JOINT_B, &minimum,
                                        &maximum));
    assert(minimum == -1591 && maximum == -1022);
    assert(!single_leg_joint_soft_limits(MOTOR_ROLE_WHEEL, &minimum,
                                         &maximum));

    single_leg_bringup_init(&bringup);
    assert(single_leg_bringup_select(&bringup, MOTOR_ROLE_JOINT_A, false,
                                     &reason));
    assert(single_leg_bringup_arm(&bringup, &snapshot, 0u, &reason));
    assert(bringup.commissioning_min_millirad == 731);
    assert(bringup.commissioning_max_millirad == 850);
    snapshot.position_millirad = 735;
    assert(!single_leg_bringup_request_jog(&bringup, &snapshot, -1, 1u,
                                           &reason));
    assert(strcmp(reason, "SOFT_LIMIT") == 0);

    single_leg_bringup_invalidate_anchor(&bringup);
    snapshot = safe_snapshot(700);
    assert(single_leg_bringup_arm(&bringup, &snapshot, 2u, &reason));
    assert(!single_leg_bringup_request_jog(&bringup, &snapshot, -1, 3u,
                                           &reason));
    assert(strcmp(reason, "SOFT_LIMIT") == 0);
    assert(single_leg_bringup_arm(&bringup, &snapshot, 4u, &reason));
    assert(single_leg_bringup_request_jog(&bringup, &snapshot, 1, 5u,
                                          &reason));
    snapshot.motor_state = 1u;
    assert(single_leg_bringup_motion_fault(&bringup, &snapshot) == NULL);

    single_leg_bringup_init(&bringup);
    assert(single_leg_bringup_select(&bringup, MOTOR_ROLE_JOINT_A, false,
                                     &reason));
    snapshot = safe_snapshot(800);
    assert(single_leg_bringup_arm(&bringup, &snapshot, 6u, &reason));
    assert(single_leg_bringup_request_jog(&bringup, &snapshot, -1, 7u,
                                          &reason));
    snapshot.motor_state = 1u;
    snapshot.position_millirad = 731;
    assert(strcmp(single_leg_bringup_motion_fault(&bringup, &snapshot),
                  "SOFT_LIMIT") == 0);
}

static void test_jog_profile_checks_target_and_slows_near_limit(void)
{
    SingleLegBringup bringup;
    SingleLegMotorSnapshot snapshot = safe_snapshot(-1307);
    const char *reason = NULL;
    int32_t velocity = 0;
    uint32_t duration = 0u;

    single_leg_bringup_init(&bringup);
    assert(single_leg_bringup_select(&bringup, MOTOR_ROLE_JOINT_B, false,
                                     &reason));
    assert(single_leg_bringup_arm(&bringup, &snapshot, 0u, &reason));
    assert(single_leg_bringup_request_jog(&bringup, &snapshot, -1, 1u,
                                          &reason));
    assert(single_leg_bringup_jog_profile(
        &bringup, &snapshot, 150, 1500u, &velocity, &duration));
    assert(velocity == 150);
    assert(duration == 600u);

    snapshot = safe_snapshot(1260);
    single_leg_bringup_init(&bringup);
    assert(single_leg_bringup_select(&bringup, MOTOR_ROLE_JOINT_A, false,
                                     &reason));
    assert(single_leg_bringup_arm(&bringup, &snapshot, 2u, &reason));
    assert(single_leg_bringup_request_jog(&bringup, &snapshot, 1, 3u,
                                          &reason));
    assert(single_leg_bringup_jog_profile(
        &bringup, &snapshot, 300, 300u, &velocity, &duration));
    assert(velocity == 150);
    assert(duration == 100u);
}

static void test_capture_requires_two_stationary_disabled_joints(void)
{
    SingleLegBringup bringup;
    SingleLegMotorSnapshot joint_a = safe_snapshot(1234);
    SingleLegMotorSnapshot joint_b = safe_snapshot(-567);
    const char *reason = NULL;

    single_leg_bringup_init(&bringup);
    assert(single_leg_bringup_capture(
        &bringup, SINGLE_LEG_POSE_STAND, &joint_a, &joint_b, 42u, &reason));
    assert(bringup.poses[SINGLE_LEG_POSE_STAND].valid);
    assert(bringup.poses[SINGLE_LEG_POSE_STAND].joint_a_raw_millirad == 1234);
    assert(bringup.poses[SINGLE_LEG_POSE_STAND].joint_b_raw_millirad == -567);

    joint_b.motor_state = 1u;
    assert(!single_leg_bringup_capture(
        &bringup, SINGLE_LEG_POSE_CROUCH, &joint_a, &joint_b, 43u, &reason));
    assert(strcmp(reason, "STATE_NOT_DISABLED") == 0);
}

static void test_mechanical_extension_directions(void)
{
    int32_t relative = 0;

    assert(single_leg_joint_direction(MOTOR_ROLE_JOINT_A) == 1);
    assert(single_leg_joint_direction(MOTOR_ROLE_JOINT_B) == -1);
    assert(single_leg_joint_direction(MOTOR_ROLE_WHEEL) == 0);

    assert(single_leg_relative_position_millirad(
        MOTOR_ROLE_JOINT_A, 1200, 1000, &relative));
    assert(relative == 200);
    assert(single_leg_relative_position_millirad(
        MOTOR_ROLE_JOINT_B, -700, -500, &relative));
    assert(relative == 200);
    assert(!single_leg_relative_position_millirad(
        MOTOR_ROLE_WHEEL, 100, 0, &relative));

    assert(single_leg_calibrated_position_millirad(
        MOTOR_ROLE_JOINT_A, 1008, &relative));
    assert(relative == 0);
    assert(single_leg_calibrated_position_millirad(
        MOTOR_ROLE_JOINT_B, -1307, &relative));
    assert(relative == 0);
    assert(single_leg_calibrated_position_millirad(
        MOTOR_ROLE_JOINT_A, 1354, &relative));
    assert(relative == 346);
    assert(single_leg_calibrated_position_millirad(
        MOTOR_ROLE_JOINT_B, -1662, &relative));
    assert(relative == 355);
    assert(!single_leg_calibrated_position_millirad(
        MOTOR_ROLE_WHEEL, 0, &relative));
}

int main(void)
{
    test_parses_required_line_commands();
    test_rejects_unknown_and_overlong_commands();
    test_arm_is_one_shot_short_and_anchors_limits();
    test_limits_and_runtime_safety_fail_closed();
    test_wheel_uses_time_and_speed_guards_not_joint_position_window();
    test_global_joint_soft_limits();
    test_jog_profile_checks_target_and_slows_near_limit();
    test_capture_requires_two_stationary_disabled_joints();
    test_mechanical_extension_directions();
    return 0;
}
