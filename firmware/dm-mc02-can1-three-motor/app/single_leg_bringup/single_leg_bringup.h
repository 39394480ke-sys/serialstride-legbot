#ifndef SINGLE_LEG_BRINGUP_H
#define SINGLE_LEG_BRINGUP_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_state.h"
#include "single_leg_calibration.h"

#define SINGLE_LEG_COMMAND_CAPACITY 24u
#define SINGLE_LEG_ARM_WINDOW_MS 3000u
#define SINGLE_LEG_FEEDBACK_MAX_AGE_MS 100u
#define SINGLE_LEG_COMMISSIONING_WINDOW_MILLIRAD 100
#define SINGLE_LEG_LIMIT_MARGIN_MILLIRAD 10
#define SINGLE_LEG_SOFT_LIMIT_SLOW_ZONE_MILLIRAD 50
#define SINGLE_LEG_JOINT_A_JOG_VELOCITY_MILLIRAD_S 150
#define SINGLE_LEG_JOINT_B_JOG_VELOCITY_MILLIRAD_S 150
#define SINGLE_LEG_JOINT_A_JOG_KD_MILLI 2000
#define SINGLE_LEG_JOINT_B_JOG_KD_MILLI 3500
#define SINGLE_LEG_WHEEL_JOG_STEP 2
#define SINGLE_LEG_WHEEL_JOG_DURATION_MS 1000u
#define SINGLE_LEG_JOINT_A_JOG_DURATION_MS 600u
#define SINGLE_LEG_JOINT_B_JOG_DURATION_MS 900u
#define SINGLE_LEG_TORQUE_LIMIT_MILLINEWTON_M 500

typedef enum {
    SINGLE_LEG_COMMAND_NONE = 0,
    SINGLE_LEG_COMMAND_INVALID,
    SINGLE_LEG_COMMAND_STATUS,
    SINGLE_LEG_COMMAND_DISABLE_ALL,
    SINGLE_LEG_COMMAND_READ_RAW,
    SINGLE_LEG_COMMAND_CAPTURE_STAND,
    SINGLE_LEG_COMMAND_CAPTURE_CROUCH,
    SINGLE_LEG_COMMAND_CAPTURE_EXTEND,
    SINGLE_LEG_COMMAND_SELECT_A,
    SINGLE_LEG_COMMAND_SELECT_B,
    SINGLE_LEG_COMMAND_SELECT_WHEEL,
    SINGLE_LEG_COMMAND_ARM,
    SINGLE_LEG_COMMAND_JOG_POSITIVE,
    SINGLE_LEG_COMMAND_JOG_NEGATIVE,
    SINGLE_LEG_COMMAND_STOP_ALL,
    SINGLE_LEG_COMMAND_POWER_ARM,
    SINGLE_LEG_COMMAND_POWER_ON,
    SINGLE_LEG_COMMAND_PROBE,
    SINGLE_LEG_COMMAND_PHASE10_ARM,
    SINGLE_LEG_COMMAND_MOVE_STAND,
    SINGLE_LEG_COMMAND_MOVE_MID_CROUCH,
    SINGLE_LEG_COMMAND_MOVE_MID_EXTEND,
} SingleLegCommand;

typedef enum {
    SINGLE_LEG_PARSE_PENDING = 0,
    SINGLE_LEG_PARSE_READY,
    SINGLE_LEG_PARSE_ERROR,
} SingleLegParseResult;

typedef enum {
    SINGLE_LEG_POSE_CROUCH = 0,
    SINGLE_LEG_POSE_STAND,
    SINGLE_LEG_POSE_EXTEND,
    SINGLE_LEG_POSE_COUNT,
} SingleLegPose;

typedef struct {
    char buffer[SINGLE_LEG_COMMAND_CAPACITY];
    uint8_t length;
    bool overflow;
} SingleLegCommandParser;

typedef struct {
    bool online;
    bool feedback_valid;
    bool parameters_valid;
    bool powered;
    bool probe_active;
    bool can_active;
    uint8_t motor_state;
    int32_t position_millirad;
    int32_t velocity_millirad_s;
    int32_t torque_millinewton_m;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    uint32_t feedback_age_ms;
} SingleLegMotorSnapshot;

typedef struct {
    bool valid;
    int32_t joint_a_raw_millirad;
    int32_t joint_b_raw_millirad;
    uint32_t captured_at_ms;
} SingleLegPoseCapture;

typedef struct {
    MotorRole selected;
    bool armed;
    bool anchor_valid;
    uint32_t armed_at_ms;
    int32_t anchor_millirad;
    int32_t commissioning_min_millirad;
    int32_t commissioning_max_millirad;
    int8_t jog_direction;
    SingleLegPoseCapture poses[SINGLE_LEG_POSE_COUNT];
} SingleLegBringup;

void single_leg_command_parser_init(SingleLegCommandParser *parser);
SingleLegParseResult single_leg_command_parser_feed(
    SingleLegCommandParser *parser, uint8_t byte, SingleLegCommand *command);
const char *single_leg_command_name(SingleLegCommand command);

void single_leg_bringup_init(SingleLegBringup *bringup);
void single_leg_bringup_disarm(SingleLegBringup *bringup);
void single_leg_bringup_invalidate_anchor(SingleLegBringup *bringup);
bool single_leg_bringup_select(SingleLegBringup *bringup, MotorRole role,
                               bool motion_active, const char **reason);
bool single_leg_bringup_arm(SingleLegBringup *bringup,
                            const SingleLegMotorSnapshot *snapshot,
                            uint32_t now_ms, const char **reason);
bool single_leg_bringup_request_jog(SingleLegBringup *bringup,
                                    const SingleLegMotorSnapshot *snapshot,
                                    int8_t direction, uint32_t now_ms,
                                    const char **reason);
const char *single_leg_bringup_motion_fault(
    const SingleLegBringup *bringup,
    const SingleLegMotorSnapshot *snapshot);
bool single_leg_bringup_capture(SingleLegBringup *bringup, SingleLegPose pose,
                                const SingleLegMotorSnapshot *joint_a,
                                const SingleLegMotorSnapshot *joint_b,
                                uint32_t now_ms, const char **reason);
const char *single_leg_pose_name(SingleLegPose pose);
int8_t single_leg_joint_direction(MotorRole role);
bool single_leg_relative_position_millirad(MotorRole role,
                                           int32_t raw_position_millirad,
                                           int32_t zero_position_millirad,
                                           int32_t *relative_millirad);
bool single_leg_calibrated_position_millirad(MotorRole role,
                                             int32_t raw_position_millirad,
                                             int32_t *relative_millirad);
bool single_leg_joint_soft_limits(MotorRole role, int32_t *minimum_millirad,
                                  int32_t *maximum_millirad);
bool single_leg_bringup_jog_profile(const SingleLegBringup *bringup,
                                    const SingleLegMotorSnapshot *snapshot,
                                    int32_t nominal_velocity_millirad_s,
                                    uint32_t nominal_duration_ms,
                                    int32_t *velocity_millirad_s,
                                    uint32_t *duration_ms);

#endif
