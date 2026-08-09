#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_state.h"

#define JOINT_A_CAN_ID 6u
#define JOINT_A_MST_ID 3u
#define JOINT_B_CAN_ID 8u
#define JOINT_B_MST_ID 4u
#define WHEEL_CAN_ID 1u
#define WHEEL_MST_ID 0u
#define MOTOR_PARAMETER_MASK_COMPLETE 0x1fu

typedef struct {
    MotorState motors[MOTOR_ROLE_COUNT];
    uint32_t unknown_rx;
} MotorManager;

void motor_manager_init(MotorManager *manager);
MotorState *motor_manager_get(MotorManager *manager, MotorRole role);
const MotorState *motor_manager_get_const(const MotorManager *manager,
                                          MotorRole role);
void motor_manager_receive(MotorManager *manager, uint32_t now_ms,
                           uint32_t loop_period_ms);
void motor_manager_update_online(MotorManager *manager, uint32_t now_ms);
bool motor_manager_parameters_valid(const MotorManager *manager,
                                    MotorRole role);
bool motor_manager_transmit(MotorManager *manager, MotorRole role,
                            uint32_t id, const uint8_t data[8], uint8_t dlc);
bool motor_manager_send_disable_all(MotorManager *manager);
bool motor_manager_send_enable_all(MotorManager *manager);
bool motor_manager_send_velocity_all(MotorManager *manager,
                                     int8_t direction);
bool motor_manager_send_feedback_request(MotorManager *manager,
                                         MotorRole role);
bool motor_manager_send_parameter_request(MotorManager *manager,
                                          MotorRole role,
                                          uint8_t register_id);
uint8_t motor_role_can_id(MotorRole role);
uint8_t motor_role_mst_id(MotorRole role);
const char *motor_role_name(MotorRole role);
const char *motor_state_name(const MotorState *motor);

#endif
