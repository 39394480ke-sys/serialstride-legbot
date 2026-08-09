#include <assert.h>
#include <string.h>

#include "can_bus.h"
#include "motor_manager.h"
#include "safety_manager.h"

static uint32_t transmitted_ids[32];
static uint8_t transmit_count;

bool can_bus_transmit(uint32_t standard_id, const uint8_t data[8],
                      uint8_t dlc)
{
    (void)data;
    assert(dlc > 0u && dlc <= 8u);
    transmitted_ids[transmit_count++] = standard_id;
    return true;
}

bool can_bus_receive(uint32_t *standard_id, uint8_t data[8], uint8_t *dlc,
                     uint32_t *age_us)
{
    (void)standard_id;
    (void)data;
    (void)dlc;
    (void)age_us;
    return false;
}

bool board_motor_power_is_enabled(void)
{
    return true;
}

static void make_ready(MotorManager *manager)
{
    MotorRole role;

    motor_manager_init(manager);
    for (role = MOTOR_ROLE_JOINT_A; role < MOTOR_ROLE_COUNT; ++role) {
        MotorState *motor = motor_manager_get(manager, role);

        motor->online = true;
        motor->feedback_valid = true;
        motor->state = 0u;
        motor->last_rx_ms = 100u;
        motor->parameter_mask = MOTOR_PARAMETER_MASK_COMPLETE;
        motor->p_max_milli = 12500;
        motor->t_max_milli = 10000;
        motor->control_mode = role == MOTOR_ROLE_WHEEL ? 3u : 1u;
        motor->v_max_milli = role == MOTOR_ROLE_WHEEL ? 45000 : 30000;
        motor->mos_temperature_c = 30u;
        motor->rotor_temperature_c = 30u;
    }
}

static void test_identity_and_group_transmit(void)
{
    MotorManager manager;

    make_ready(&manager);
    assert(manager.motors[MOTOR_ROLE_JOINT_A].can_id == 6u);
    assert(manager.motors[MOTOR_ROLE_JOINT_A].mst_id == 3u);
    assert(manager.motors[MOTOR_ROLE_JOINT_B].can_id == 8u);
    assert(manager.motors[MOTOR_ROLE_JOINT_B].mst_id == 4u);
    assert(manager.motors[MOTOR_ROLE_WHEEL].can_id == 1u);
    assert(manager.motors[MOTOR_ROLE_WHEEL].mst_id == 0u);
    assert(motor_manager_parameters_valid(&manager, MOTOR_ROLE_JOINT_A));
    assert(motor_manager_parameters_valid(&manager, MOTOR_ROLE_JOINT_B));
    assert(motor_manager_parameters_valid(&manager, MOTOR_ROLE_WHEEL));

    transmit_count = 0u;
    assert(motor_manager_send_enable_all(&manager));
    assert(transmit_count == 3u);
    assert(transmitted_ids[0] == 6u && transmitted_ids[1] == 8u &&
           transmitted_ids[2] == 1u);

    transmit_count = 0u;
    assert(motor_manager_send_disable_all(&manager));
    assert(transmit_count == 6u);
}

static void test_central_safety_uses_unified_states(void)
{
    MotorManager manager;
    CanBusStatus can_status = {0};
    ParallelSafetySnapshot parallel;
    const char *fault;

    make_ready(&manager);
    fault = safety_manager_global_fault(
        &manager, MOTOR_ROLE_JOINT_A, false, false, 100u);
    assert(fault == NULL);

    manager.motors[MOTOR_ROLE_JOINT_A].state = 1u;
    fault = safety_manager_global_fault(
        &manager, MOTOR_ROLE_JOINT_A, true, false, 100u);
    assert(fault == NULL);
    manager.motors[MOTOR_ROLE_JOINT_B].state = 1u;
    fault = safety_manager_global_fault(
        &manager, MOTOR_ROLE_JOINT_A, true, false, 100u);
    assert(fault != NULL && strcmp(fault, "MOTOR_NOT_DISABLED") == 0);

    manager.motors[MOTOR_ROLE_JOINT_A].state = 0u;
    manager.motors[MOTOR_ROLE_JOINT_B].state = 0u;
    parallel = safety_manager_parallel_snapshot(
        &manager, &can_status, true, true, 100u);
    assert(parallel.parameters_valid && parallel.feedback_fresh);
    assert(parallel.all_disabled && parallel.can_active);
}

int main(void)
{
    test_identity_and_group_transmit();
    test_central_safety_uses_unified_states();
    return 0;
}
