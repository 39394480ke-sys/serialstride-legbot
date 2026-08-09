#include "motor_manager.h"

#include <stddef.h>
#include <string.h>

#include "can_bus.h"
#include "dm4310_protocol.h"
#include "feedback_timing.h"
#include "h6215_protocol.h"

#define ONLINE_TIMEOUT_MS 500u
#define MIT_KD_MILLI 1000

static int32_t float_to_milli(float value)
{
    float scaled = value * 1000.0f;

    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static bool decode_version(uint32_t raw_value, char version[5])
{
    uint8_t index;

    for (index = 0u; index < 4u; ++index) {
        uint8_t character = (uint8_t)(raw_value >> (index * 8u));

        if (character < 0x20u || character > 0x7eu) {
            version[0] = '\0';
            return false;
        }
        version[index] = (char)character;
    }
    version[4] = '\0';
    return true;
}

static void update_parameter(MotorState *motor, uint8_t register_id,
                             uint32_t raw_value, float float_value)
{
    switch (register_id) {
    case DM4310_REGISTER_SOFTWARE_VERSION:
        if (decode_version(raw_value, motor->software_version))
            motor->parameter_mask |= 1u << 0;
        break;
    case DM4310_REGISTER_CONTROL_MODE:
        motor->control_mode = (uint8_t)raw_value;
        motor->parameter_mask |= 1u << 1;
        break;
    case DM4310_REGISTER_P_MAX:
        motor->p_max_milli = float_to_milli(float_value);
        motor->parameter_mask |= 1u << 2;
        break;
    case DM4310_REGISTER_V_MAX:
        motor->v_max_milli = float_to_milli(float_value);
        motor->parameter_mask |= 1u << 3;
        break;
    case DM4310_REGISTER_T_MAX:
        motor->t_max_milli = float_to_milli(float_value);
        motor->parameter_mask |= 1u << 4;
        break;
    default:
        break;
    }
}

static void update_feedback(MotorState *motor, uint8_t state,
                            int32_t position, int32_t velocity,
                            int32_t torque, uint8_t mos_temperature,
                            uint8_t rotor_temperature, uint32_t received_ms)
{
    motor->state = state;
    motor->position_millirad = position;
    motor->velocity_millirad_s = velocity;
    motor->torque_millinewton_m = torque;
    motor->mos_temperature_c = mos_temperature;
    motor->rotor_temperature_c = rotor_temperature;
    motor->last_rx_ms = received_ms;
    motor->feedback_valid = true;
    motor->online = true;
    motor->rx_count++;
}

void motor_manager_init(MotorManager *manager)
{
    if (manager == NULL) return;
    memset(manager, 0, sizeof(*manager));
    manager->motors[MOTOR_ROLE_JOINT_A] = (MotorState){
        .type = MOTOR_TYPE_DM4310,
        .role = MOTOR_ROLE_JOINT_A,
        .can_id = JOINT_A_CAN_ID,
        .mst_id = JOINT_A_MST_ID,
    };
    manager->motors[MOTOR_ROLE_JOINT_B] = (MotorState){
        .type = MOTOR_TYPE_DM4310,
        .role = MOTOR_ROLE_JOINT_B,
        .can_id = JOINT_B_CAN_ID,
        .mst_id = JOINT_B_MST_ID,
    };
    manager->motors[MOTOR_ROLE_WHEEL] = (MotorState){
        .type = MOTOR_TYPE_H6215,
        .role = MOTOR_ROLE_WHEEL,
        .can_id = WHEEL_CAN_ID,
        .mst_id = WHEEL_MST_ID,
    };
}

MotorState *motor_manager_get(MotorManager *manager, MotorRole role)
{
    return manager != NULL && role < MOTOR_ROLE_COUNT
               ? &manager->motors[role]
               : NULL;
}

const MotorState *motor_manager_get_const(const MotorManager *manager,
                                          MotorRole role)
{
    return manager != NULL && role < MOTOR_ROLE_COUNT
               ? &manager->motors[role]
               : NULL;
}

void motor_manager_receive(MotorManager *manager, uint32_t now_ms,
                           uint32_t loop_period_ms)
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t age_us;

    if (manager == NULL) return;
    while (can_bus_receive(&id, data, &dlc, &age_us)) {
        Dm4310CanFrame dm_frame = {.id = id, .dlc = dlc};
        H6215CanFrame wheel_frame = {.id = id, .dlc = dlc};
        Dm4310ParameterResponse dm_parameter;
        H6215ParameterResponse wheel_parameter;
        Dm4310Feedback dm_feedback;
        H6215Feedback wheel_feedback;
        uint32_t received_ms = feedback_received_at_ms(
            now_ms, age_us, loop_period_ms);

        memcpy(dm_frame.data, data, sizeof(data));
        memcpy(wheel_frame.data, data, sizeof(data));
        if (dm4310_parse_parameter_response_for(
                JOINT_A_CAN_ID, JOINT_A_MST_ID, &dm_frame, &dm_parameter)) {
            update_parameter(&manager->motors[MOTOR_ROLE_JOINT_A],
                             dm_parameter.register_id,
                             dm_parameter.raw_value,
                             dm_parameter.float_value);
            manager->motors[MOTOR_ROLE_JOINT_A].rx_count++;
        } else if (dm4310_parse_feedback_for(
                       JOINT_A_CAN_ID, JOINT_A_MST_ID, &dm_frame,
                       &dm_feedback)) {
            update_feedback(&manager->motors[MOTOR_ROLE_JOINT_A],
                            dm_feedback.state,
                            dm_feedback.position_millirad,
                            dm_feedback.velocity_millirad_s,
                            dm_feedback.torque_millinewton_m,
                            dm_feedback.mos_temperature_c,
                            dm_feedback.rotor_temperature_c, received_ms);
        } else if (dm4310_parse_parameter_response_for(
                       JOINT_B_CAN_ID, JOINT_B_MST_ID, &dm_frame,
                       &dm_parameter)) {
            update_parameter(&manager->motors[MOTOR_ROLE_JOINT_B],
                             dm_parameter.register_id,
                             dm_parameter.raw_value,
                             dm_parameter.float_value);
            manager->motors[MOTOR_ROLE_JOINT_B].rx_count++;
        } else if (dm4310_parse_feedback_for(
                       JOINT_B_CAN_ID, JOINT_B_MST_ID, &dm_frame,
                       &dm_feedback)) {
            update_feedback(&manager->motors[MOTOR_ROLE_JOINT_B],
                            dm_feedback.state,
                            dm_feedback.position_millirad,
                            dm_feedback.velocity_millirad_s,
                            dm_feedback.torque_millinewton_m,
                            dm_feedback.mos_temperature_c,
                            dm_feedback.rotor_temperature_c, received_ms);
        } else if (h6215_parse_parameter_response(
                       &wheel_frame, &wheel_parameter)) {
            update_parameter(&manager->motors[MOTOR_ROLE_WHEEL],
                             wheel_parameter.register_id,
                             wheel_parameter.raw_value,
                             wheel_parameter.float_value);
            manager->motors[MOTOR_ROLE_WHEEL].rx_count++;
        } else if (h6215_parse_feedback(&wheel_frame, &wheel_feedback)) {
            update_feedback(&manager->motors[MOTOR_ROLE_WHEEL],
                            wheel_feedback.state,
                            wheel_feedback.position_millirad,
                            wheel_feedback.velocity_millirad_s,
                            wheel_feedback.torque_millinewton_m,
                            wheel_feedback.mos_temperature_c,
                            wheel_feedback.rotor_temperature_c, received_ms);
        } else {
            manager->unknown_rx++;
        }
    }
}

void motor_manager_update_online(MotorManager *manager, uint32_t now_ms)
{
    MotorRole role;

    if (manager == NULL) return;
    for (role = MOTOR_ROLE_JOINT_A; role < MOTOR_ROLE_COUNT; ++role) {
        MotorState *motor = &manager->motors[role];

        motor->online = motor->feedback_valid &&
                        now_ms - motor->last_rx_ms <= ONLINE_TIMEOUT_MS;
    }
}

bool motor_manager_parameters_valid(const MotorManager *manager,
                                    MotorRole role)
{
    const MotorState *motor = motor_manager_get_const(manager, role);

    if (motor == NULL ||
        motor->parameter_mask != MOTOR_PARAMETER_MASK_COMPLETE ||
        motor->p_max_milli != 12500 || motor->t_max_milli != 10000)
        return false;
    if (motor->type == MOTOR_TYPE_H6215)
        return motor->control_mode == 3u && motor->v_max_milli == 45000;
    return motor->control_mode == 1u && motor->v_max_milli == 30000;
}

bool motor_manager_transmit(MotorManager *manager, MotorRole role,
                            uint32_t id, const uint8_t data[8], uint8_t dlc)
{
    MotorState *motor = motor_manager_get(manager, role);

    if (motor == NULL || !can_bus_transmit(id, data, dlc)) {
        if (motor != NULL) motor->tx_failed++;
        return false;
    }
    motor->tx_ok++;
    return true;
}

bool motor_manager_send_disable_all(MotorManager *manager)
{
    Dm4310CanFrame dm_frame;
    H6215CanFrame wheel_frame;
    bool success = true;

    if (!dm4310_build_mit_command_for(JOINT_A_CAN_ID, 0, 0, 0,
                                      MIT_KD_MILLI, 0, &dm_frame) ||
        !motor_manager_transmit(manager, MOTOR_ROLE_JOINT_A, dm_frame.id,
                                dm_frame.data, dm_frame.dlc))
        success = false;
    if (!dm4310_build_disable_command_for(JOINT_A_CAN_ID, &dm_frame) ||
        !motor_manager_transmit(manager, MOTOR_ROLE_JOINT_A, dm_frame.id,
                                dm_frame.data, dm_frame.dlc))
        success = false;
    if (!dm4310_build_mit_command_for(JOINT_B_CAN_ID, 0, 0, 0,
                                      MIT_KD_MILLI, 0, &dm_frame) ||
        !motor_manager_transmit(manager, MOTOR_ROLE_JOINT_B, dm_frame.id,
                                dm_frame.data, dm_frame.dlc))
        success = false;
    if (!dm4310_build_disable_command_for(JOINT_B_CAN_ID, &dm_frame) ||
        !motor_manager_transmit(manager, MOTOR_ROLE_JOINT_B, dm_frame.id,
                                dm_frame.data, dm_frame.dlc))
        success = false;
    if (!h6215_build_zero_velocity_command(&wheel_frame) ||
        !motor_manager_transmit(manager, MOTOR_ROLE_WHEEL, wheel_frame.id,
                                wheel_frame.data, wheel_frame.dlc))
        success = false;
    if (!h6215_build_disable_command(&wheel_frame) ||
        !motor_manager_transmit(manager, MOTOR_ROLE_WHEEL, wheel_frame.id,
                                wheel_frame.data, wheel_frame.dlc))
        success = false;
    return success;
}

bool motor_manager_send_enable_all(MotorManager *manager)
{
    Dm4310CanFrame dm_frame;
    H6215CanFrame wheel_frame;

    return dm4310_build_enable_command_for(JOINT_A_CAN_ID, &dm_frame) &&
           motor_manager_transmit(manager, MOTOR_ROLE_JOINT_A, dm_frame.id,
                                  dm_frame.data, dm_frame.dlc) &&
           dm4310_build_enable_command_for(JOINT_B_CAN_ID, &dm_frame) &&
           motor_manager_transmit(manager, MOTOR_ROLE_JOINT_B, dm_frame.id,
                                  dm_frame.data, dm_frame.dlc) &&
           h6215_build_enable_command(&wheel_frame) &&
           motor_manager_transmit(manager, MOTOR_ROLE_WHEEL, wheel_frame.id,
                                  wheel_frame.data, wheel_frame.dlc);
}

bool motor_manager_send_velocity_all(MotorManager *manager,
                                     int8_t direction)
{
    Dm4310CanFrame dm_frame;
    H6215CanFrame wheel_frame;
    int32_t joint_velocity = (int32_t)direction * 400;
    int8_t wheel_step = (int8_t)(direction * 2);

    return dm4310_build_mit_command_for(
               JOINT_A_CAN_ID, 0, joint_velocity, 0, MIT_KD_MILLI, 0,
               &dm_frame) &&
           motor_manager_transmit(manager, MOTOR_ROLE_JOINT_A, dm_frame.id,
                                  dm_frame.data, dm_frame.dlc) &&
           dm4310_build_mit_command_for(
               JOINT_B_CAN_ID, 0, joint_velocity, 0, MIT_KD_MILLI, 0,
               &dm_frame) &&
           motor_manager_transmit(manager, MOTOR_ROLE_JOINT_B, dm_frame.id,
                                  dm_frame.data, dm_frame.dlc) &&
           h6215_build_velocity_step(wheel_step, &wheel_frame) &&
           motor_manager_transmit(manager, MOTOR_ROLE_WHEEL, wheel_frame.id,
                                  wheel_frame.data, wheel_frame.dlc);
}

bool motor_manager_send_feedback_request(MotorManager *manager,
                                         MotorRole role)
{
    if (role == MOTOR_ROLE_WHEEL) {
        H6215CanFrame frame;

        return h6215_build_disable_command(&frame) &&
               motor_manager_transmit(manager, role, frame.id, frame.data,
                                      frame.dlc);
    } else {
        Dm4310CanFrame frame;

        return dm4310_build_feedback_request_for(motor_role_can_id(role),
                                                  &frame) &&
               motor_manager_transmit(manager, role, frame.id, frame.data,
                                      frame.dlc);
    }
}

bool motor_manager_send_parameter_request(MotorManager *manager,
                                          MotorRole role,
                                          uint8_t register_id)
{
    if (role == MOTOR_ROLE_WHEEL) {
        H6215CanFrame frame;

        return h6215_build_read_request(register_id, &frame) &&
               motor_manager_transmit(manager, role, frame.id, frame.data,
                                      frame.dlc);
    } else {
        Dm4310CanFrame frame;

        return dm4310_build_read_request_for(motor_role_can_id(role),
                                              register_id, &frame) &&
               motor_manager_transmit(manager, role, frame.id, frame.data,
                                      frame.dlc);
    }
}

uint8_t motor_role_can_id(MotorRole role)
{
    return role == MOTOR_ROLE_JOINT_A ? JOINT_A_CAN_ID
         : role == MOTOR_ROLE_JOINT_B ? JOINT_B_CAN_ID
                                      : WHEEL_CAN_ID;
}

uint8_t motor_role_mst_id(MotorRole role)
{
    return role == MOTOR_ROLE_JOINT_A ? JOINT_A_MST_ID
         : role == MOTOR_ROLE_JOINT_B ? JOINT_B_MST_ID
                                      : WHEEL_MST_ID;
}

const char *motor_role_name(MotorRole role)
{
    switch (role) {
    case MOTOR_ROLE_JOINT_A: return "JOINT_A";
    case MOTOR_ROLE_JOINT_B: return "JOINT_B";
    case MOTOR_ROLE_WHEEL: return "WHEEL";
    default: return "UNKNOWN";
    }
}

const char *motor_state_name(const MotorState *motor)
{
    if (motor == NULL) return "NO_FEEDBACK";
    return motor->type == MOTOR_TYPE_H6215
               ? h6215_state_name(motor->state)
               : dm4310_state_name(motor->state);
}
