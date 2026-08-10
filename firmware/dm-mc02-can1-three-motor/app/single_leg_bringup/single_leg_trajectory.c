#include "single_leg_trajectory.h"

#include <stddef.h>

#include "single_leg_bringup.h"

#define ARM_WINDOW_MS 3000u
#define ENABLE_WAIT_MS 150u
#define HOLD_MS 300u
#define DISABLE_WAIT_MS 200u
#define TRACKING_ERROR_LIMIT_MILLIRAD 120
#define STAND_START_TOLERANCE_MILLIRAD 100

static SingleLegTrajectoryDecision decision(SingleLegTrajectoryEvent event,
                                            const char *reason)
{
    SingleLegTrajectoryDecision result = {0};

    result.event = event;
    result.reason = reason;
    return result;
}

static bool elapsed(uint32_t now, uint32_t then, uint32_t interval)
{
    return (uint32_t)(now - then) >= interval;
}

static int32_t absolute(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool target_raw(SingleLegTrajectoryTarget target, int32_t *joint_a,
                       int32_t *joint_b)
{
    if (joint_a == NULL || joint_b == NULL) return false;
    switch (target) {
    case SINGLE_LEG_TARGET_STAND:
        *joint_a = SINGLE_LEG_JOINT_A_ZERO_MILLIRAD;
        *joint_b = SINGLE_LEG_JOINT_B_ZERO_MILLIRAD;
        return true;
    case SINGLE_LEG_TARGET_MID_CROUCH:
        *joint_a = SINGLE_LEG_JOINT_A_MID_CROUCH_MILLIRAD;
        *joint_b = SINGLE_LEG_JOINT_B_MID_CROUCH_MILLIRAD;
        return true;
    case SINGLE_LEG_TARGET_MID_EXTEND:
        *joint_a = SINGLE_LEG_JOINT_A_MID_EXTEND_MILLIRAD;
        *joint_b = SINGLE_LEG_JOINT_B_MID_EXTEND_MILLIRAD;
        return true;
    default: return false;
    }
}

static const char *start_fault(const SingleLegTrajectorySafety *safety)
{
    if (safety == NULL) return "INVALID_SAFETY";
    if (!safety->powered) return "POWER_OFF";
    if (!safety->probe_active) return "PROBE_REQUIRED";
    if (!safety->parameters_valid) return "PARAMETERS_INVALID";
    if (!safety->feedback_fresh) return "FEEDBACK_STALE";
    if (!safety->joints_disabled || !safety->wheel_disabled)
        return "MOTOR_NOT_DISABLED";
    if (!safety->start_speed_zero) return "SPEED_NOT_ZERO";
    if (!safety->runtime_torque_safe) return "TORQUE_LIMIT";
    if (!safety->temperature_safe) return "TEMPERATURE_LIMIT";
    if (!safety->can_active) return "CAN_NOT_ACTIVE";
    if (!safety->states_normal) return "STATE_ABNORMAL";
    if (safety->joint_a_position_millirad <
            SINGLE_LEG_JOINT_A_SOFT_MIN_MILLIRAD ||
        safety->joint_a_position_millirad >
            SINGLE_LEG_JOINT_A_SOFT_MAX_MILLIRAD ||
        safety->joint_b_position_millirad <
            SINGLE_LEG_JOINT_B_SOFT_MIN_MILLIRAD ||
        safety->joint_b_position_millirad >
            SINGLE_LEG_JOINT_B_SOFT_MAX_MILLIRAD)
        return "SOFT_LIMIT";
    return NULL;
}

static const char *runtime_fault(const SingleLegTrajectory *trajectory,
                                 const SingleLegTrajectorySafety *safety)
{
    int32_t expected_a;
    int32_t expected_b;

    if (safety == NULL) return "INVALID_SAFETY";
    if (!safety->powered) return "POWER_OFF";
    if (!safety->probe_active) return "PROBE_REQUIRED";
    if (!safety->feedback_fresh) return "FEEDBACK_STALE";
    if (!safety->joints_enabled || !safety->wheel_disabled)
        return "STATE_NOT_ENABLED";
    if (!safety->runtime_speed_safe) return "SPEED_LIMIT";
    if (!safety->runtime_torque_safe) return "TORQUE_LIMIT";
    if (!safety->temperature_safe) return "TEMPERATURE_LIMIT";
    if (!safety->can_active) return "CAN_NOT_ACTIVE";
    if (!safety->states_normal) return "STATE_ABNORMAL";
    if (safety->joint_a_position_millirad <
            SINGLE_LEG_JOINT_A_SOFT_MIN_MILLIRAD ||
        safety->joint_a_position_millirad >
            SINGLE_LEG_JOINT_A_SOFT_MAX_MILLIRAD ||
        safety->joint_b_position_millirad <
            SINGLE_LEG_JOINT_B_SOFT_MIN_MILLIRAD ||
        safety->joint_b_position_millirad >
            SINGLE_LEG_JOINT_B_SOFT_MAX_MILLIRAD)
        return "SOFT_LIMIT";

    expected_a = trajectory->state == SINGLE_LEG_TRAJECTORY_HOLD
                     ? trajectory->target_a_millirad
                     : trajectory->start_a_millirad;
    expected_b = trajectory->state == SINGLE_LEG_TRAJECTORY_HOLD
                     ? trajectory->target_b_millirad
                     : trajectory->start_b_millirad;
    if (trajectory->state == SINGLE_LEG_TRAJECTORY_HOLD &&
        (absolute(safety->joint_a_position_millirad - expected_a) >
             TRACKING_ERROR_LIMIT_MILLIRAD ||
         absolute(safety->joint_b_position_millirad - expected_b) >
             TRACKING_ERROR_LIMIT_MILLIRAD))
        return "TRACKING_ERROR";
    return NULL;
}

static SingleLegTrajectoryDecision fail_closed(SingleLegTrajectory *trajectory,
                                               const char *reason)
{
    SingleLegTrajectoryDecision result =
        decision(SINGLE_LEG_TRAJECTORY_EVENT_SAFETY_TRIP, reason);

    result.target = trajectory->target;
    result.target_valid = true;
    single_leg_trajectory_init(trajectory);
    result.send_disable_all = true;
    result.cut_power = true;
    return result;
}

static void interpolated_command(const SingleLegTrajectory *trajectory,
                                 uint32_t now_ms,
                                 SingleLegTrajectoryDecision *result)
{
    uint32_t time_ms = now_ms - trajectory->phase_started_ms;
    const int64_t duration = SINGLE_LEG_TRAJECTORY_DURATION_MS;
    int64_t smooth_numerator;
    int64_t denominator = duration * duration * duration;
    int64_t velocity_numerator;
    int32_t delta_a = trajectory->target_a_millirad -
                      trajectory->start_a_millirad;
    int32_t delta_b = trajectory->target_b_millirad -
                      trajectory->start_b_millirad;

    if (time_ms > SINGLE_LEG_TRAJECTORY_DURATION_MS)
        time_ms = SINGLE_LEG_TRAJECTORY_DURATION_MS;
    smooth_numerator = 3LL * time_ms * time_ms * duration -
                       2LL * time_ms * time_ms * time_ms;
    velocity_numerator =
        6LL * time_ms * (duration - time_ms) * 1000LL;
    result->joint_a_target_millirad = trajectory->start_a_millirad +
        (int32_t)((int64_t)delta_a * smooth_numerator / denominator);
    result->joint_b_target_millirad = trajectory->start_b_millirad +
        (int32_t)((int64_t)delta_b * smooth_numerator / denominator);
    result->joint_a_velocity_millirad_s =
        (int32_t)((int64_t)delta_a * velocity_numerator / denominator);
    result->joint_b_velocity_millirad_s =
        (int32_t)((int64_t)delta_b * velocity_numerator / denominator);
    result->send_position_joints = true;
}

void single_leg_trajectory_init(SingleLegTrajectory *trajectory)
{
    if (trajectory != NULL)
        *trajectory = (SingleLegTrajectory){
            .state = SINGLE_LEG_TRAJECTORY_DISABLED};
}

bool single_leg_trajectory_active(const SingleLegTrajectory *trajectory)
{
    return trajectory != NULL &&
           trajectory->state != SINGLE_LEG_TRAJECTORY_DISABLED &&
           trajectory->state != SINGLE_LEG_TRAJECTORY_ARMED;
}

SingleLegTrajectoryDecision single_leg_trajectory_arm(
    SingleLegTrajectory *trajectory, const SingleLegTrajectorySafety *safety)
{
    const char *fault;

    if (trajectory == NULL || safety == NULL)
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED,
                        "INVALID_ARGUMENT");
    if (single_leg_trajectory_active(trajectory))
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED,
                        "MOTION_ACTIVE");
    fault = start_fault(safety);
    if (fault != NULL)
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED, fault);
    trajectory->state = SINGLE_LEG_TRAJECTORY_ARMED;
    trajectory->armed_at_ms = safety->now_ms;
    return decision(SINGLE_LEG_TRAJECTORY_EVENT_ARMED, NULL);
}

SingleLegTrajectoryDecision single_leg_trajectory_start(
    SingleLegTrajectory *trajectory, SingleLegTrajectoryTarget target,
    const SingleLegTrajectorySafety *safety)
{
    SingleLegTrajectoryDecision result;
    const char *fault;

    if (trajectory == NULL || safety == NULL)
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED,
                        "INVALID_ARGUMENT");
    if (trajectory->state != SINGLE_LEG_TRAJECTORY_ARMED)
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED,
                        "PHASE10_ARM_REQUIRED");
    if (elapsed(safety->now_ms, trajectory->armed_at_ms, ARM_WINDOW_MS)) {
        single_leg_trajectory_init(trajectory);
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_ARM_TIMEOUT,
                        "ARM_TIMEOUT");
    }
    fault = start_fault(safety);
    if (fault != NULL) {
        single_leg_trajectory_init(trajectory);
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED, fault);
    }
    if (!target_raw(target, &trajectory->target_a_millirad,
                    &trajectory->target_b_millirad))
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED,
                        "INVALID_TARGET");
    if (target != SINGLE_LEG_TARGET_STAND &&
        (absolute(safety->joint_a_position_millirad -
                  SINGLE_LEG_JOINT_A_ZERO_MILLIRAD) >
             STAND_START_TOLERANCE_MILLIRAD ||
         absolute(safety->joint_b_position_millirad -
                  SINGLE_LEG_JOINT_B_ZERO_MILLIRAD) >
             STAND_START_TOLERANCE_MILLIRAD)) {
        single_leg_trajectory_init(trajectory);
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED,
                        "START_NOT_STAND");
    }
    if (trajectory->target_a_millirad <
            SINGLE_LEG_JOINT_A_SOFT_MIN_MILLIRAD ||
        trajectory->target_a_millirad >
            SINGLE_LEG_JOINT_A_SOFT_MAX_MILLIRAD ||
        trajectory->target_b_millirad <
            SINGLE_LEG_JOINT_B_SOFT_MIN_MILLIRAD ||
        trajectory->target_b_millirad >
            SINGLE_LEG_JOINT_B_SOFT_MAX_MILLIRAD)
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED,
                        "TARGET_SOFT_LIMIT");

    trajectory->target = target;
    trajectory->start_a_millirad = safety->joint_a_position_millirad;
    trajectory->start_b_millirad = safety->joint_b_position_millirad;
    trajectory->state = SINGLE_LEG_TRAJECTORY_ENABLE_WAIT;
    trajectory->phase_started_ms = safety->now_ms;
    trajectory->last_command_ms = safety->now_ms;
    result = decision(SINGLE_LEG_TRAJECTORY_EVENT_ENABLE_WAIT, NULL);
    result.target = target;
    result.target_valid = true;
    result.send_enable_joints = true;
    return result;
}

SingleLegTrajectoryDecision single_leg_trajectory_step(
    SingleLegTrajectory *trajectory, const SingleLegTrajectorySafety *safety)
{
    SingleLegTrajectoryDecision result =
        decision(SINGLE_LEG_TRAJECTORY_EVENT_NONE, NULL);
    const char *fault;

    if (trajectory == NULL || safety == NULL)
        return decision(SINGLE_LEG_TRAJECTORY_EVENT_REJECTED,
                        "INVALID_ARGUMENT");
    if (trajectory->state == SINGLE_LEG_TRAJECTORY_ARMED) {
        if (elapsed(safety->now_ms, trajectory->armed_at_ms, ARM_WINDOW_MS)) {
            single_leg_trajectory_init(trajectory);
            return decision(SINGLE_LEG_TRAJECTORY_EVENT_ARM_TIMEOUT,
                            "ARM_TIMEOUT");
        }
        return result;
    }
    if (trajectory->state == SINGLE_LEG_TRAJECTORY_ENABLE_WAIT) {
        if (!safety->feedback_fresh || !safety->can_active)
            return fail_closed(trajectory, "FEEDBACK_OR_CAN");
        if (!safety->wheel_disabled)
            return fail_closed(trajectory, "WHEEL_NOT_DISABLED");
        if (safety->joints_enabled) {
            trajectory->state = SINGLE_LEG_TRAJECTORY_RUNNING;
            trajectory->phase_started_ms = safety->now_ms;
            trajectory->last_command_ms = safety->now_ms;
            result = decision(SINGLE_LEG_TRAJECTORY_EVENT_RUNNING, NULL);
            result.target = trajectory->target;
            result.target_valid = true;
            interpolated_command(trajectory, safety->now_ms, &result);
            return result;
        }
        if (elapsed(safety->now_ms, trajectory->phase_started_ms,
                    ENABLE_WAIT_MS))
            return fail_closed(trajectory, "ENABLE_TIMEOUT");
        return result;
    }
    if (trajectory->state == SINGLE_LEG_TRAJECTORY_RUNNING) {
        fault = runtime_fault(trajectory, safety);
        if (fault != NULL) return fail_closed(trajectory, fault);
        if (elapsed(safety->now_ms, trajectory->phase_started_ms,
                    SINGLE_LEG_TRAJECTORY_DURATION_MS)) {
            trajectory->state = SINGLE_LEG_TRAJECTORY_HOLD;
            trajectory->phase_started_ms = safety->now_ms;
            trajectory->last_command_ms = safety->now_ms;
            result = decision(SINGLE_LEG_TRAJECTORY_EVENT_HOLD, NULL);
            result.target = trajectory->target;
            result.target_valid = true;
            result.send_position_joints = true;
            result.joint_a_target_millirad = trajectory->target_a_millirad;
            result.joint_b_target_millirad = trajectory->target_b_millirad;
            return result;
        }
        if (elapsed(safety->now_ms, trajectory->last_command_ms,
                    SINGLE_LEG_TRAJECTORY_REFRESH_MS)) {
            trajectory->last_command_ms = safety->now_ms;
            interpolated_command(trajectory, safety->now_ms, &result);
        }
        return result;
    }
    if (trajectory->state == SINGLE_LEG_TRAJECTORY_HOLD) {
        fault = runtime_fault(trajectory, safety);
        if (fault != NULL) return fail_closed(trajectory, fault);
        if (elapsed(safety->now_ms, trajectory->phase_started_ms, HOLD_MS)) {
            trajectory->state = SINGLE_LEG_TRAJECTORY_DISABLE_WAIT;
            trajectory->phase_started_ms = safety->now_ms;
            result = decision(SINGLE_LEG_TRAJECTORY_EVENT_DISABLE_WAIT, NULL);
            result.target = trajectory->target;
            result.target_valid = true;
            result.send_disable_all = true;
            return result;
        }
        if (elapsed(safety->now_ms, trajectory->last_command_ms,
                    SINGLE_LEG_TRAJECTORY_REFRESH_MS)) {
            trajectory->last_command_ms = safety->now_ms;
            result.send_position_joints = true;
            result.joint_a_target_millirad = trajectory->target_a_millirad;
            result.joint_b_target_millirad = trajectory->target_b_millirad;
        }
        return result;
    }
    if (trajectory->state == SINGLE_LEG_TRAJECTORY_DISABLE_WAIT) {
        if (safety->joints_disabled && safety->wheel_disabled) {
            SingleLegTrajectoryTarget completed_target = trajectory->target;

            single_leg_trajectory_init(trajectory);
            result = decision(SINGLE_LEG_TRAJECTORY_EVENT_COMPLETE, NULL);
            result.target = completed_target;
            result.target_valid = true;
            return result;
        }
        if (elapsed(safety->now_ms, trajectory->phase_started_ms,
                    DISABLE_WAIT_MS))
            return fail_closed(trajectory, "DISABLE_TIMEOUT");
    }
    return result;
}

const char *single_leg_trajectory_target_name(SingleLegTrajectoryTarget target)
{
    switch (target) {
    case SINGLE_LEG_TARGET_STAND: return "STAND";
    case SINGLE_LEG_TARGET_MID_CROUCH: return "MID_CROUCH";
    case SINGLE_LEG_TARGET_MID_EXTEND: return "MID_EXTEND";
    default: return "UNKNOWN";
    }
}
