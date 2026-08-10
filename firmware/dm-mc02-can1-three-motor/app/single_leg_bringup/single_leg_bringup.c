#include "single_leg_bringup.h"

#include <stddef.h>
#include <string.h>

#define START_SPEED_LIMIT_MILLIRAD_S 100
#define RUNTIME_SPEED_LIMIT_MILLIRAD_S 200
#define WHEEL_RUNTIME_SPEED_LIMIT_MILLIRAD_S 800
#define TEMPERATURE_LIMIT_C 60u

static int32_t absolute(int32_t value)
{
    return value < 0 ? -value : value;
}

static SingleLegCommand parse_command(const char *command)
{
    static const struct {
        const char *name;
        SingleLegCommand command;
    } commands[] = {
        {"status", SINGLE_LEG_COMMAND_STATUS},
        {"disable-all", SINGLE_LEG_COMMAND_DISABLE_ALL},
        {"read-raw", SINGLE_LEG_COMMAND_READ_RAW},
        {"capture-stand", SINGLE_LEG_COMMAND_CAPTURE_STAND},
        {"capture-crouch", SINGLE_LEG_COMMAND_CAPTURE_CROUCH},
        {"capture-extend", SINGLE_LEG_COMMAND_CAPTURE_EXTEND},
        {"select-a", SINGLE_LEG_COMMAND_SELECT_A},
        {"select-b", SINGLE_LEG_COMMAND_SELECT_B},
        {"select-wheel", SINGLE_LEG_COMMAND_SELECT_WHEEL},
        {"arm", SINGLE_LEG_COMMAND_ARM},
        {"jog-positive", SINGLE_LEG_COMMAND_JOG_POSITIVE},
        {"jog-negative", SINGLE_LEG_COMMAND_JOG_NEGATIVE},
        {"stop-all", SINGLE_LEG_COMMAND_STOP_ALL},
        {"power-arm", SINGLE_LEG_COMMAND_POWER_ARM},
        {"power-on", SINGLE_LEG_COMMAND_POWER_ON},
        {"probe", SINGLE_LEG_COMMAND_PROBE},
        {"phase10-arm", SINGLE_LEG_COMMAND_PHASE10_ARM},
        {"move-stand", SINGLE_LEG_COMMAND_MOVE_STAND},
        {"move-mid-crouch", SINGLE_LEG_COMMAND_MOVE_MID_CROUCH},
        {"move-mid-extend", SINGLE_LEG_COMMAND_MOVE_MID_EXTEND},
    };
    size_t index;

    for (index = 0u; index < sizeof(commands) / sizeof(commands[0]); ++index) {
        if (strcmp(command, commands[index].name) == 0)
            return commands[index].command;
    }
    return SINGLE_LEG_COMMAND_INVALID;
}

void single_leg_command_parser_init(SingleLegCommandParser *parser)
{
    if (parser != NULL) *parser = (SingleLegCommandParser){0};
}

SingleLegParseResult single_leg_command_parser_feed(
    SingleLegCommandParser *parser, uint8_t byte, SingleLegCommand *command)
{
    if (parser == NULL || command == NULL) return SINGLE_LEG_PARSE_ERROR;
    *command = SINGLE_LEG_COMMAND_NONE;
    if (byte == (uint8_t)'\r' || byte == (uint8_t)'\n') {
        SingleLegParseResult result;

        if (parser->length == 0u && !parser->overflow)
            return SINGLE_LEG_PARSE_PENDING;
        if (parser->overflow) {
            result = SINGLE_LEG_PARSE_ERROR;
        } else {
            parser->buffer[parser->length] = '\0';
            *command = parse_command(parser->buffer);
            result = *command == SINGLE_LEG_COMMAND_INVALID
                         ? SINGLE_LEG_PARSE_ERROR
                         : SINGLE_LEG_PARSE_READY;
        }
        single_leg_command_parser_init(parser);
        return result;
    }
    if (byte >= (uint8_t)'A' && byte <= (uint8_t)'Z')
        byte = (uint8_t)(byte + ((uint8_t)'a' - (uint8_t)'A'));
    if ((byte < (uint8_t)'a' || byte > (uint8_t)'z') &&
        (byte < (uint8_t)'0' || byte > (uint8_t)'9') &&
        byte != (uint8_t)'-') {
        single_leg_command_parser_init(parser);
        return SINGLE_LEG_PARSE_ERROR;
    }
    if (parser->length + 1u >= SINGLE_LEG_COMMAND_CAPACITY) {
        parser->overflow = true;
        return SINGLE_LEG_PARSE_PENDING;
    }
    if (!parser->overflow) parser->buffer[parser->length++] = (char)byte;
    return SINGLE_LEG_PARSE_PENDING;
}

const char *single_leg_command_name(SingleLegCommand command)
{
    switch (command) {
    case SINGLE_LEG_COMMAND_STATUS: return "status";
    case SINGLE_LEG_COMMAND_DISABLE_ALL: return "disable-all";
    case SINGLE_LEG_COMMAND_READ_RAW: return "read-raw";
    case SINGLE_LEG_COMMAND_CAPTURE_STAND: return "capture-stand";
    case SINGLE_LEG_COMMAND_CAPTURE_CROUCH: return "capture-crouch";
    case SINGLE_LEG_COMMAND_CAPTURE_EXTEND: return "capture-extend";
    case SINGLE_LEG_COMMAND_SELECT_A: return "select-a";
    case SINGLE_LEG_COMMAND_SELECT_B: return "select-b";
    case SINGLE_LEG_COMMAND_SELECT_WHEEL: return "select-wheel";
    case SINGLE_LEG_COMMAND_ARM: return "arm";
    case SINGLE_LEG_COMMAND_JOG_POSITIVE: return "jog-positive";
    case SINGLE_LEG_COMMAND_JOG_NEGATIVE: return "jog-negative";
    case SINGLE_LEG_COMMAND_STOP_ALL: return "stop-all";
    case SINGLE_LEG_COMMAND_POWER_ARM: return "power-arm";
    case SINGLE_LEG_COMMAND_POWER_ON: return "power-on";
    case SINGLE_LEG_COMMAND_PROBE: return "probe";
    case SINGLE_LEG_COMMAND_PHASE10_ARM: return "phase10-arm";
    case SINGLE_LEG_COMMAND_MOVE_STAND: return "move-stand";
    case SINGLE_LEG_COMMAND_MOVE_MID_CROUCH: return "move-mid-crouch";
    case SINGLE_LEG_COMMAND_MOVE_MID_EXTEND: return "move-mid-extend";
    default: return "invalid";
    }
}

void single_leg_bringup_init(SingleLegBringup *bringup)
{
    if (bringup != NULL)
        *bringup = (SingleLegBringup){.selected = MOTOR_ROLE_COUNT};
}

void single_leg_bringup_disarm(SingleLegBringup *bringup)
{
    if (bringup == NULL) return;
    bringup->armed = false;
    bringup->jog_direction = 0;
}

void single_leg_bringup_invalidate_anchor(SingleLegBringup *bringup)
{
    if (bringup == NULL) return;
    single_leg_bringup_disarm(bringup);
    bringup->anchor_valid = false;
}

bool single_leg_bringup_select(SingleLegBringup *bringup, MotorRole role,
                               bool motion_active, const char **reason)
{
    if (reason != NULL) *reason = NULL;
    if (bringup == NULL || role >= MOTOR_ROLE_COUNT) {
        if (reason != NULL) *reason = "INVALID_SELECTION";
        return false;
    }
    if (motion_active) {
        if (reason != NULL) *reason = "MOTION_ACTIVE";
        return false;
    }
    bringup->selected = role;
    single_leg_bringup_invalidate_anchor(bringup);
    return true;
}

static const char *snapshot_fault(const SingleLegMotorSnapshot *snapshot,
                                  bool require_disabled, MotorRole role)
{
    int32_t speed_limit_millirad_s =
        role == MOTOR_ROLE_WHEEL ? WHEEL_RUNTIME_SPEED_LIMIT_MILLIRAD_S
                                 : RUNTIME_SPEED_LIMIT_MILLIRAD_S;

    if (snapshot == NULL) return "INVALID_SNAPSHOT";
    if (!snapshot->powered) return "POWER_OFF";
    if (!snapshot->probe_active) return "PROBE_REQUIRED";
    if (!snapshot->online || !snapshot->feedback_valid ||
        snapshot->feedback_age_ms > SINGLE_LEG_FEEDBACK_MAX_AGE_MS)
        return "FEEDBACK_STALE";
    if (!snapshot->parameters_valid) return "PARAMETERS_INVALID";
    if (!snapshot->can_active) return "CAN_NOT_ACTIVE";
    if (require_disabled && snapshot->motor_state != 0u)
        return "STATE_NOT_DISABLED";
    if (absolute(snapshot->velocity_millirad_s) >=
        (require_disabled ? START_SPEED_LIMIT_MILLIRAD_S
                          : speed_limit_millirad_s))
        return "SPEED_LIMIT";
    if (absolute(snapshot->torque_millinewton_m) >
        SINGLE_LEG_TORQUE_LIMIT_MILLINEWTON_M)
        return "TORQUE_LIMIT";
    if (snapshot->mos_temperature_c >= TEMPERATURE_LIMIT_C ||
        snapshot->rotor_temperature_c >= TEMPERATURE_LIMIT_C)
        return "TEMPERATURE_LIMIT";
    if (!require_disabled && snapshot->motor_state != 1u)
        return "STATE_NOT_ENABLED";
    return NULL;
}

bool single_leg_bringup_arm(SingleLegBringup *bringup,
                            const SingleLegMotorSnapshot *snapshot,
                            uint32_t now_ms, const char **reason)
{
    const char *fault;
    int32_t soft_minimum_millirad = 0;
    int32_t soft_maximum_millirad = 0;
    bool has_soft_limits;

    if (reason != NULL) *reason = NULL;
    if (bringup == NULL || bringup->selected >= MOTOR_ROLE_COUNT) {
        if (reason != NULL) *reason = "SELECT_MOTOR_FIRST";
        return false;
    }
    fault = snapshot_fault(snapshot, true, bringup->selected);
    if (fault != NULL) {
        if (reason != NULL) *reason = fault;
        return false;
    }
    has_soft_limits = single_leg_joint_soft_limits(
        bringup->selected, &soft_minimum_millirad, &soft_maximum_millirad);
    if (!bringup->anchor_valid) {
        bringup->anchor_millirad = snapshot->position_millirad;
        bringup->commissioning_min_millirad =
            snapshot->position_millirad -
            SINGLE_LEG_COMMISSIONING_WINDOW_MILLIRAD;
        bringup->commissioning_max_millirad =
            snapshot->position_millirad +
            SINGLE_LEG_COMMISSIONING_WINDOW_MILLIRAD;
        if (has_soft_limits &&
            snapshot->position_millirad >= soft_minimum_millirad &&
            snapshot->position_millirad <= soft_maximum_millirad) {
            if (bringup->commissioning_min_millirad < soft_minimum_millirad)
                bringup->commissioning_min_millirad = soft_minimum_millirad;
            if (bringup->commissioning_max_millirad > soft_maximum_millirad)
                bringup->commissioning_max_millirad = soft_maximum_millirad;
        }
        bringup->anchor_valid = true;
    }
    bringup->armed = true;
    bringup->armed_at_ms = now_ms;
    bringup->jog_direction = 0;
    return true;
}

bool single_leg_bringup_request_jog(SingleLegBringup *bringup,
                                    const SingleLegMotorSnapshot *snapshot,
                                    int8_t direction, uint32_t now_ms,
                                    const char **reason)
{
    const char *fault;
    int32_t soft_minimum_millirad = 0;
    int32_t soft_maximum_millirad = 0;

    if (reason != NULL) *reason = NULL;
    if (bringup == NULL || snapshot == NULL ||
        (direction != -1 && direction != 1)) {
        if (reason != NULL) *reason = "INVALID_ARGUMENT";
        return false;
    }
    if (!bringup->armed) {
        if (reason != NULL) *reason = "ARM_REQUIRED";
        return false;
    }
    if ((uint32_t)(now_ms - bringup->armed_at_ms) >
        SINGLE_LEG_ARM_WINDOW_MS) {
        single_leg_bringup_disarm(bringup);
        if (reason != NULL) *reason = "ARM_TIMEOUT";
        return false;
    }
    fault = snapshot_fault(snapshot, true, bringup->selected);
    if (fault != NULL) {
        single_leg_bringup_disarm(bringup);
        if (reason != NULL) *reason = fault;
        return false;
    }
    if (single_leg_joint_soft_limits(
            bringup->selected, &soft_minimum_millirad,
            &soft_maximum_millirad)) {
        bool below_limit =
            snapshot->position_millirad < soft_minimum_millirad;
        bool above_limit =
            snapshot->position_millirad > soft_maximum_millirad;

        if ((below_limit && direction < 0) ||
            (above_limit && direction > 0) ||
            (!below_limit && !above_limit &&
             ((direction > 0 &&
               snapshot->position_millirad >=
                   soft_maximum_millirad -
                       SINGLE_LEG_LIMIT_MARGIN_MILLIRAD) ||
              (direction < 0 &&
               snapshot->position_millirad <=
                   soft_minimum_millirad +
                       SINGLE_LEG_LIMIT_MARGIN_MILLIRAD)))) {
            single_leg_bringup_disarm(bringup);
            if (reason != NULL) *reason = "SOFT_LIMIT";
            return false;
        }
    }
    if (bringup->selected != MOTOR_ROLE_WHEEL &&
        ((direction > 0 &&
          snapshot->position_millirad >=
              bringup->commissioning_max_millirad -
                  SINGLE_LEG_LIMIT_MARGIN_MILLIRAD) ||
         (direction < 0 &&
          snapshot->position_millirad <=
              bringup->commissioning_min_millirad +
                  SINGLE_LEG_LIMIT_MARGIN_MILLIRAD))) {
        single_leg_bringup_disarm(bringup);
        if (reason != NULL) *reason = "COMMISSIONING_LIMIT";
        return false;
    }
    bringup->armed = false;
    bringup->jog_direction = direction;
    return true;
}

const char *single_leg_bringup_motion_fault(
    const SingleLegBringup *bringup,
    const SingleLegMotorSnapshot *snapshot)
{
    const char *fault;
    int32_t soft_minimum_millirad = 0;
    int32_t soft_maximum_millirad = 0;

    if (bringup == NULL || bringup->jog_direction == 0) return NULL;
    fault = snapshot_fault(snapshot, false, bringup->selected);
    if (fault != NULL) return fault;
    if (single_leg_joint_soft_limits(
            bringup->selected, &soft_minimum_millirad,
            &soft_maximum_millirad)) {
        bool below_limit =
            snapshot->position_millirad < soft_minimum_millirad;
        bool above_limit =
            snapshot->position_millirad > soft_maximum_millirad;

        if ((below_limit && bringup->jog_direction < 0) ||
            (above_limit && bringup->jog_direction > 0) ||
            (!below_limit && !above_limit &&
             ((bringup->jog_direction > 0 &&
               snapshot->position_millirad >= soft_maximum_millirad) ||
              (bringup->jog_direction < 0 &&
               snapshot->position_millirad <= soft_minimum_millirad))))
            return "SOFT_LIMIT";
    }
    if (bringup->selected != MOTOR_ROLE_WHEEL) {
        if (snapshot->position_millirad <
                bringup->commissioning_min_millirad ||
            snapshot->position_millirad >
                bringup->commissioning_max_millirad)
            return "COMMISSIONING_LIMIT";
        if ((bringup->jog_direction > 0 &&
             snapshot->position_millirad >=
                 bringup->commissioning_max_millirad) ||
            (bringup->jog_direction < 0 &&
             snapshot->position_millirad <=
                 bringup->commissioning_min_millirad))
            return "COMMISSIONING_LIMIT";
    }
    return NULL;
}

static const char *capture_fault(const SingleLegMotorSnapshot *snapshot)
{
    if (snapshot == NULL || !snapshot->online || !snapshot->feedback_valid ||
        snapshot->feedback_age_ms > SINGLE_LEG_FEEDBACK_MAX_AGE_MS)
        return "FEEDBACK_STALE";
    if (snapshot->motor_state != 0u) return "STATE_NOT_DISABLED";
    if (absolute(snapshot->velocity_millirad_s) >=
        START_SPEED_LIMIT_MILLIRAD_S)
        return "SPEED_NOT_ZERO";
    return NULL;
}

bool single_leg_bringup_capture(SingleLegBringup *bringup, SingleLegPose pose,
                                const SingleLegMotorSnapshot *joint_a,
                                const SingleLegMotorSnapshot *joint_b,
                                uint32_t now_ms, const char **reason)
{
    const char *fault;

    if (reason != NULL) *reason = NULL;
    if (bringup == NULL || pose >= SINGLE_LEG_POSE_COUNT) {
        if (reason != NULL) *reason = "INVALID_POSE";
        return false;
    }
    fault = capture_fault(joint_a);
    if (fault == NULL) fault = capture_fault(joint_b);
    if (fault != NULL) {
        if (reason != NULL) *reason = fault;
        return false;
    }
    bringup->poses[pose] = (SingleLegPoseCapture){
        .valid = true,
        .joint_a_raw_millirad = joint_a->position_millirad,
        .joint_b_raw_millirad = joint_b->position_millirad,
        .captured_at_ms = now_ms,
    };
    return true;
}

const char *single_leg_pose_name(SingleLegPose pose)
{
    switch (pose) {
    case SINGLE_LEG_POSE_CROUCH: return "CROUCH";
    case SINGLE_LEG_POSE_STAND: return "STAND";
    case SINGLE_LEG_POSE_EXTEND: return "EXTEND";
    default: return "UNKNOWN";
    }
}

int8_t single_leg_joint_direction(MotorRole role)
{
    switch (role) {
    case MOTOR_ROLE_JOINT_A: return SINGLE_LEG_JOINT_A_DIRECTION;
    case MOTOR_ROLE_JOINT_B: return SINGLE_LEG_JOINT_B_DIRECTION;
    default: return 0;
    }
}

bool single_leg_relative_position_millirad(MotorRole role,
                                           int32_t raw_position_millirad,
                                           int32_t zero_position_millirad,
                                           int32_t *relative_millirad)
{
    int8_t direction = single_leg_joint_direction(role);

    if (direction == 0 || relative_millirad == NULL) return false;
    *relative_millirad =
        (int32_t)direction * (raw_position_millirad - zero_position_millirad);
    return true;
}

bool single_leg_calibrated_position_millirad(MotorRole role,
                                             int32_t raw_position_millirad,
                                             int32_t *relative_millirad)
{
    int32_t zero_position_millirad;

    switch (role) {
    case MOTOR_ROLE_JOINT_A:
        zero_position_millirad = SINGLE_LEG_JOINT_A_ZERO_MILLIRAD;
        break;
    case MOTOR_ROLE_JOINT_B:
        zero_position_millirad = SINGLE_LEG_JOINT_B_ZERO_MILLIRAD;
        break;
    default:
        return false;
    }
    return single_leg_relative_position_millirad(
        role, raw_position_millirad, zero_position_millirad,
        relative_millirad);
}

bool single_leg_joint_soft_limits(MotorRole role, int32_t *minimum_millirad,
                                  int32_t *maximum_millirad)
{
    if (minimum_millirad == NULL || maximum_millirad == NULL) return false;
    switch (role) {
    case MOTOR_ROLE_JOINT_A:
        *minimum_millirad = SINGLE_LEG_JOINT_A_SOFT_MIN_MILLIRAD;
        *maximum_millirad = SINGLE_LEG_JOINT_A_SOFT_MAX_MILLIRAD;
        return true;
    case MOTOR_ROLE_JOINT_B:
        *minimum_millirad = SINGLE_LEG_JOINT_B_SOFT_MIN_MILLIRAD;
        *maximum_millirad = SINGLE_LEG_JOINT_B_SOFT_MAX_MILLIRAD;
        return true;
    default:
        return false;
    }
}

bool single_leg_bringup_jog_profile(const SingleLegBringup *bringup,
                                    const SingleLegMotorSnapshot *snapshot,
                                    int32_t nominal_velocity_millirad_s,
                                    uint32_t nominal_duration_ms,
                                    int32_t *velocity_millirad_s,
                                    uint32_t *duration_ms)
{
    int32_t velocity;
    int32_t available_distance;
    int32_t soft_minimum_millirad;
    int32_t soft_maximum_millirad;

    if (bringup == NULL || snapshot == NULL || velocity_millirad_s == NULL ||
        duration_ms == NULL || bringup->jog_direction == 0 ||
        nominal_velocity_millirad_s <= 0 || nominal_duration_ms == 0u ||
        !single_leg_joint_soft_limits(bringup->selected,
                                      &soft_minimum_millirad,
                                      &soft_maximum_millirad))
        return false;

    velocity = nominal_velocity_millirad_s;
    if (snapshot->position_millirad >= soft_minimum_millirad &&
        snapshot->position_millirad <= soft_maximum_millirad) {
        int32_t distance_to_soft_limit =
            bringup->jog_direction > 0
                ? soft_maximum_millirad - snapshot->position_millirad
                : snapshot->position_millirad - soft_minimum_millirad;

        if (distance_to_soft_limit < SINGLE_LEG_SOFT_LIMIT_SLOW_ZONE_MILLIRAD)
            velocity =
                nominal_velocity_millirad_s * distance_to_soft_limit /
                SINGLE_LEG_SOFT_LIMIT_SLOW_ZONE_MILLIRAD;
    }
    if (velocity <= 0) return false;

    available_distance =
        bringup->jog_direction > 0
            ? bringup->commissioning_max_millirad -
                  snapshot->position_millirad -
                  SINGLE_LEG_LIMIT_MARGIN_MILLIRAD
            : snapshot->position_millirad -
                  bringup->commissioning_min_millirad -
                  SINGLE_LEG_LIMIT_MARGIN_MILLIRAD;
    if (available_distance <= 0) return false;

    *velocity_millirad_s = velocity;
    *duration_ms = nominal_duration_ms;
    {
        uint32_t target_limited_duration_ms =
            (uint32_t)(((int64_t)available_distance * 1000) / velocity);

        if (target_limited_duration_ms == 0u) return false;
        if (*duration_ms > target_limited_duration_ms)
            *duration_ms = target_limited_duration_ms;
    }
    return true;
}
