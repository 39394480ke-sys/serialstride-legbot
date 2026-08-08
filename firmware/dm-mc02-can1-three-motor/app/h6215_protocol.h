#ifndef H6215_PROTOCOL_H
#define H6215_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define H6215_CAN_ID 1u
#define H6215_MASTER_ID 0u

typedef enum {
    H6215_REGISTER_CONTROL_MODE = 10u,
    H6215_REGISTER_SOFTWARE_VERSION = 14u,
    H6215_REGISTER_P_MAX = 21u,
    H6215_REGISTER_V_MAX = 22u,
    H6215_REGISTER_T_MAX = 23u,
} H6215Register;

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} H6215CanFrame;

typedef struct {
    uint8_t register_id;
    uint32_t raw_value;
    float float_value;
} H6215ParameterResponse;

typedef struct {
    uint8_t motor_id;
    uint8_t state;
    int32_t position_millirad;
    int32_t velocity_millirad_s;
    int32_t torque_millinewton_m;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
} H6215Feedback;

bool h6215_build_read_request(uint8_t register_id, H6215CanFrame *frame);
bool h6215_build_feedback_request(H6215CanFrame *frame);
bool h6215_build_disable_command(H6215CanFrame *frame);
bool h6215_build_enable_command(H6215CanFrame *frame);
bool h6215_build_positive_velocity_command(H6215CanFrame *frame);
bool h6215_build_zero_velocity_command(H6215CanFrame *frame);
bool h6215_parse_parameter_response(const H6215CanFrame *frame,
                                     H6215ParameterResponse *response);
bool h6215_parse_feedback(const H6215CanFrame *frame, H6215Feedback *feedback);
bool h6215_decode_software_version(uint32_t raw_value, char version[5]);
const char *h6215_state_name(uint8_t state);

#endif
