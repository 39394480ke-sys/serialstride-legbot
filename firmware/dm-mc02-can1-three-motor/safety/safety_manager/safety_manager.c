#include "safety_manager.h"

#include <stddef.h>

#include "board.h"

#define FEEDBACK_MAX_AGE_MS 100u
#define START_SPEED_LIMIT_MILLIRAD_S 100
#define RUNTIME_SPEED_LIMIT_MILLIRAD_S 800
#define RUNTIME_TORQUE_LIMIT_MILLINEWTON_M 500
#define TEMPERATURE_LIMIT_C 60u

static int32_t absolute(int32_t value)
{
    return value < 0 ? -value : value;
}

const char *safety_manager_global_fault(const MotorManager *manager,
                                        MotorRole selected,
                                        bool motion_active,
                                        bool parallel_active,
                                        uint32_t now_ms)
{
    MotorRole role;

    if (manager == NULL) return "INVALID_MANAGER";
    for (role = MOTOR_ROLE_JOINT_A; role < MOTOR_ROLE_COUNT; ++role) {
        const MotorState *motor = motor_manager_get_const(manager, role);

        if (motor == NULL || !motor->online) continue;
        if (motor->state != 0u &&
            !(motion_active && (parallel_active || role == selected) &&
              motor->state == 1u))
            return "MOTOR_NOT_DISABLED";
        if (motor->mos_temperature_c >= TEMPERATURE_LIMIT_C ||
            motor->rotor_temperature_c >= TEMPERATURE_LIMIT_C)
            return "TEMPERATURE_LIMIT";
        if (motor->feedback_valid &&
            now_ms - motor->last_rx_ms > 500u)
            return "FEEDBACK_STALE";
    }
    return NULL;
}

Dm4310SafetySnapshot safety_manager_joint_snapshot(
    const MotorManager *manager, MotorRole role,
    const CanBusStatus *can_status, bool can_status_valid,
    bool probe_active, uint32_t now_ms)
{
    const MotorState *motor = motor_manager_get_const(manager, role);

    if (motor == NULL) return (Dm4310SafetySnapshot){0};
    return (Dm4310SafetySnapshot){
        .now_ms = now_ms,
        .feedback_age_ms = motor->feedback_valid
                               ? now_ms - motor->last_rx_ms
                               : UINT32_MAX,
        .motor_state = motor->state,
        .velocity_millirad_s = motor->velocity_millirad_s,
        .torque_millinewton_m = motor->torque_millinewton_m,
        .mos_temperature_c = motor->mos_temperature_c,
        .rotor_temperature_c = motor->rotor_temperature_c,
        .powered = board_motor_power_is_enabled(),
        .probe_active = probe_active,
        .parameters_valid = motor_manager_parameters_valid(manager, role),
        .feedback_valid = motor->feedback_valid,
        .can_warning = !can_status_valid || can_status->warning,
        .can_passive = !can_status_valid || can_status->error_passive,
        .can_bus_off = !can_status_valid || can_status->bus_off,
    };
}

MotionSafetySnapshot safety_manager_wheel_snapshot(
    const MotorManager *manager, const CanBusStatus *can_status,
    bool can_status_valid, uint32_t now_ms)
{
    const MotorState *motor = motor_manager_get_const(
        manager, MOTOR_ROLE_WHEEL);

    if (motor == NULL) return (MotionSafetySnapshot){0};
    return (MotionSafetySnapshot){
        .now_ms = now_ms,
        .feedback_age_ms = motor->feedback_valid
                               ? now_ms - motor->last_rx_ms
                               : UINT32_MAX,
        .parameter_mask = motor->parameter_mask,
        .control_mode = motor->control_mode,
        .motor_state = motor->state,
        .velocity_millirad_s = motor->velocity_millirad_s,
        .torque_millinewton_m = motor->torque_millinewton_m,
        .mos_temperature_c = motor->mos_temperature_c,
        .rotor_temperature_c = motor->rotor_temperature_c,
        .feedback_valid = motor->feedback_valid,
        .can_warning = !can_status_valid || can_status->warning,
        .can_passive = !can_status_valid || can_status->error_passive,
        .can_bus_off = !can_status_valid || can_status->bus_off,
    };
}

ParallelSafetySnapshot safety_manager_parallel_snapshot(
    const MotorManager *manager, const CanBusStatus *can_status,
    bool can_status_valid, bool probe_active, uint32_t now_ms)
{
    MotorRole role;
    ParallelSafetySnapshot safety = {
        .now_ms = now_ms,
        .powered = board_motor_power_is_enabled(),
        .probe_active = probe_active,
        .parameters_valid = true,
        .feedback_fresh = true,
        .all_disabled = true,
        .all_enabled = true,
        .start_speed_zero = true,
        .runtime_speed_safe = true,
        .runtime_torque_safe = true,
        .temperature_safe = true,
        .can_active = can_status_valid && !can_status->warning &&
                      !can_status->error_passive && !can_status->bus_off,
        .state_normal = true,
    };

    for (role = MOTOR_ROLE_JOINT_A; role < MOTOR_ROLE_COUNT; ++role) {
        const MotorState *motor = motor_manager_get_const(manager, role);

        if (motor == NULL) {
            safety.feedback_fresh = false;
            safety.parameters_valid = false;
            continue;
        }
        safety.parameters_valid &=
            motor_manager_parameters_valid(manager, role);
        safety.feedback_fresh &= motor->feedback_valid &&
                                 now_ms - motor->last_rx_ms <=
                                     FEEDBACK_MAX_AGE_MS;
        safety.all_disabled &= motor->state == 0u;
        safety.all_enabled &= motor->state == 1u;
        safety.start_speed_zero &=
            absolute(motor->velocity_millirad_s) <
            START_SPEED_LIMIT_MILLIRAD_S;
        safety.runtime_speed_safe &=
            absolute(motor->velocity_millirad_s) <=
            RUNTIME_SPEED_LIMIT_MILLIRAD_S;
        if (motor->type == MOTOR_TYPE_DM4310)
            safety.runtime_torque_safe &=
                absolute(motor->torque_millinewton_m) <=
                RUNTIME_TORQUE_LIMIT_MILLINEWTON_M;
        safety.temperature_safe &=
            motor->mos_temperature_c < TEMPERATURE_LIMIT_C &&
            motor->rotor_temperature_c < TEMPERATURE_LIMIT_C;
        safety.state_normal &= motor->state <= 1u;
    }
    return safety;
}
