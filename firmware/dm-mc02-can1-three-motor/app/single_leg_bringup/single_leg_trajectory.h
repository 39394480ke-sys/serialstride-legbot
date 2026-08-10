#ifndef SINGLE_LEG_TRAJECTORY_H
#define SINGLE_LEG_TRAJECTORY_H

#include <stdbool.h>
#include <stdint.h>

#define SINGLE_LEG_TRAJECTORY_DURATION_MS 4000u
#define SINGLE_LEG_TRAJECTORY_REFRESH_MS 10u
#define SINGLE_LEG_TRAJECTORY_KP_MILLI 5000
#define SINGLE_LEG_TRAJECTORY_KD_MILLI 2000

typedef enum {
    SINGLE_LEG_TARGET_STAND = 0,
    SINGLE_LEG_TARGET_MID_CROUCH,
    SINGLE_LEG_TARGET_MID_EXTEND,
} SingleLegTrajectoryTarget;

typedef enum {
    SINGLE_LEG_TRAJECTORY_DISABLED = 0,
    SINGLE_LEG_TRAJECTORY_ARMED,
    SINGLE_LEG_TRAJECTORY_ENABLE_WAIT,
    SINGLE_LEG_TRAJECTORY_RUNNING,
    SINGLE_LEG_TRAJECTORY_HOLD,
    SINGLE_LEG_TRAJECTORY_DISABLE_WAIT,
} SingleLegTrajectoryState;

typedef enum {
    SINGLE_LEG_TRAJECTORY_EVENT_NONE = 0,
    SINGLE_LEG_TRAJECTORY_EVENT_ARMED,
    SINGLE_LEG_TRAJECTORY_EVENT_ENABLE_WAIT,
    SINGLE_LEG_TRAJECTORY_EVENT_RUNNING,
    SINGLE_LEG_TRAJECTORY_EVENT_HOLD,
    SINGLE_LEG_TRAJECTORY_EVENT_DISABLE_WAIT,
    SINGLE_LEG_TRAJECTORY_EVENT_COMPLETE,
    SINGLE_LEG_TRAJECTORY_EVENT_REJECTED,
    SINGLE_LEG_TRAJECTORY_EVENT_SAFETY_TRIP,
    SINGLE_LEG_TRAJECTORY_EVENT_ARM_TIMEOUT,
} SingleLegTrajectoryEvent;

typedef struct {
    uint32_t now_ms;
    bool powered;
    bool probe_active;
    bool parameters_valid;
    bool feedback_fresh;
    bool joints_disabled;
    bool joints_enabled;
    bool wheel_disabled;
    bool start_speed_zero;
    bool runtime_speed_safe;
    bool runtime_torque_safe;
    bool temperature_safe;
    bool can_active;
    bool states_normal;
    int32_t joint_a_position_millirad;
    int32_t joint_b_position_millirad;
} SingleLegTrajectorySafety;

typedef struct {
    bool send_enable_joints;
    bool send_position_joints;
    bool send_disable_all;
    bool cut_power;
    int32_t joint_a_target_millirad;
    int32_t joint_b_target_millirad;
    int32_t joint_a_velocity_millirad_s;
    int32_t joint_b_velocity_millirad_s;
    SingleLegTrajectoryEvent event;
    bool target_valid;
    SingleLegTrajectoryTarget target;
    const char *reason;
} SingleLegTrajectoryDecision;

typedef struct {
    SingleLegTrajectoryState state;
    SingleLegTrajectoryTarget target;
    uint32_t armed_at_ms;
    uint32_t phase_started_ms;
    uint32_t last_command_ms;
    int32_t start_a_millirad;
    int32_t start_b_millirad;
    int32_t target_a_millirad;
    int32_t target_b_millirad;
} SingleLegTrajectory;

void single_leg_trajectory_init(SingleLegTrajectory *trajectory);
bool single_leg_trajectory_active(const SingleLegTrajectory *trajectory);
SingleLegTrajectoryDecision single_leg_trajectory_arm(
    SingleLegTrajectory *trajectory, const SingleLegTrajectorySafety *safety);
SingleLegTrajectoryDecision single_leg_trajectory_start(
    SingleLegTrajectory *trajectory, SingleLegTrajectoryTarget target,
    const SingleLegTrajectorySafety *safety);
SingleLegTrajectoryDecision single_leg_trajectory_step(
    SingleLegTrajectory *trajectory, const SingleLegTrajectorySafety *safety);
const char *single_leg_trajectory_target_name(SingleLegTrajectoryTarget target);

#endif
