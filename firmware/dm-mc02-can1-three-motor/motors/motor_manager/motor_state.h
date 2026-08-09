#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_TYPE_DM4310 = 0,
    MOTOR_TYPE_H6215,
} MotorType;

typedef enum {
    MOTOR_ROLE_JOINT_A = 0,
    MOTOR_ROLE_JOINT_B,
    MOTOR_ROLE_WHEEL,
    MOTOR_ROLE_COUNT,
} MotorRole;

typedef struct {
    MotorType type;
    MotorRole role;
    uint8_t can_id;
    uint8_t mst_id;
    bool online;
    bool feedback_valid;
    uint8_t state;
    int32_t position_millirad;
    int32_t velocity_millirad_s;
    int32_t torque_millinewton_m;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    uint32_t last_rx_ms;
    uint32_t rx_count;
    uint32_t tx_ok;
    uint32_t tx_failed;
    uint8_t parameter_mask;
    uint8_t control_mode;
    int32_t p_max_milli;
    int32_t v_max_milli;
    int32_t t_max_milli;
    char software_version[5];
} MotorState;

#endif
