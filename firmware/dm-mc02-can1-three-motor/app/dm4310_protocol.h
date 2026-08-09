#ifndef DM4310_PROTOCOL_H
#define DM4310_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define DM4310_CAN_ID 6u
#define DM4310_MASTER_ID 3u
#define DM4310_P_MAX_MILLIRAD 12500
#define DM4310_V_MAX_MILLIRAD_S 30000
#define DM4310_T_MAX_MILLINEWTON_M 10000

typedef enum {
    DM4310_REGISTER_CONTROL_MODE = 10u,
    DM4310_REGISTER_SOFTWARE_VERSION = 14u,
    DM4310_REGISTER_P_MAX = 21u,
    DM4310_REGISTER_V_MAX = 22u,
    DM4310_REGISTER_T_MAX = 23u,
} Dm4310Register;

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} Dm4310CanFrame;

typedef struct {
    uint8_t register_id;
    uint32_t raw_value;
    float float_value;
} Dm4310ParameterResponse;

typedef struct {
    uint8_t motor_id;
    uint8_t state;
    int32_t position_millirad;
    int32_t velocity_millirad_s;
    int32_t torque_millinewton_m;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
} Dm4310Feedback;

bool dm4310_build_read_request(uint8_t register_id, Dm4310CanFrame *frame);
bool dm4310_build_feedback_request(Dm4310CanFrame *frame);
bool dm4310_build_enable_command(Dm4310CanFrame *frame);
bool dm4310_build_disable_command(Dm4310CanFrame *frame);
bool dm4310_build_mit_command(int32_t position_millirad,
                              int32_t velocity_millirad_s,
                              int32_t kp_milli, int32_t kd_milli,
                              int32_t torque_millinewton_m,
                              Dm4310CanFrame *frame);
bool dm4310_parse_parameter_response(const Dm4310CanFrame *frame,
                                     Dm4310ParameterResponse *response);
bool dm4310_parse_feedback(const Dm4310CanFrame *frame,
                           Dm4310Feedback *feedback);
const char *dm4310_state_name(uint8_t state);

#endif
