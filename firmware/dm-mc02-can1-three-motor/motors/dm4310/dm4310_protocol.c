#include "dm4310_protocol.h"

#include <stddef.h>
#include <string.h>

static bool readable_register(uint8_t register_id)
{
    return register_id == DM4310_REGISTER_CONTROL_MODE ||
           register_id == DM4310_REGISTER_SOFTWARE_VERSION ||
           register_id == DM4310_REGISTER_P_MAX ||
           register_id == DM4310_REGISTER_V_MAX ||
           register_id == DM4310_REGISTER_T_MAX;
}

static uint32_t unpack_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint32_t encode_signed_range(int32_t value, int32_t limit,
                                    uint32_t maximum)
{
    int64_t numerator = (int64_t)(value + limit) * maximum;

    return (uint32_t)(numerator / ((int64_t)limit * 2));
}

static uint32_t encode_unsigned_range(int32_t value, int32_t maximum_value,
                                      uint32_t maximum_raw)
{
    return (uint32_t)(((int64_t)value * maximum_raw) / maximum_value);
}

static int32_t decode_signed_range(uint32_t raw, int32_t limit,
                                   uint32_t maximum)
{
    return (int32_t)(((int64_t)raw * (int64_t)limit * 2) / maximum) - limit;
}

static bool build_system_command(uint8_t motor_id, uint8_t opcode,
                                 Dm4310CanFrame *frame)
{
    if (frame == NULL || motor_id > 0x0fu) {
        return false;
    }
    frame->id = motor_id;
    frame->dlc = 8u;
    memset(frame->data, 0xff, 7u);
    frame->data[7] = opcode;
    return true;
}

bool dm4310_build_read_request_for(uint8_t motor_id, uint8_t register_id,
                                   Dm4310CanFrame *frame)
{
    if (frame == NULL || motor_id > 0x0fu ||
        !readable_register(register_id)) {
        return false;
    }
    frame->id = 0x7ffu;
    frame->dlc = 8u;
    frame->data[0] = motor_id;
    frame->data[1] = 0u;
    frame->data[2] = 0x33u;
    frame->data[3] = register_id;
    memset(&frame->data[4], 0, 4u);
    return true;
}

bool dm4310_build_read_request(uint8_t register_id, Dm4310CanFrame *frame)
{
    return dm4310_build_read_request_for(DM4310_CAN_ID, register_id, frame);
}

bool dm4310_build_feedback_request_for(uint8_t motor_id,
                                       Dm4310CanFrame *frame)
{
    if (frame == NULL || motor_id > 0x0fu) {
        return false;
    }
    frame->id = 0x7ffu;
    frame->dlc = 8u;
    frame->data[0] = motor_id;
    frame->data[1] = 0u;
    frame->data[2] = 0xccu;
    memset(&frame->data[3], 0, 5u);
    return true;
}

bool dm4310_build_feedback_request(Dm4310CanFrame *frame)
{
    return dm4310_build_feedback_request_for(DM4310_CAN_ID, frame);
}

bool dm4310_build_enable_command(Dm4310CanFrame *frame)
{
    return dm4310_build_enable_command_for(DM4310_CAN_ID, frame);
}

bool dm4310_build_disable_command(Dm4310CanFrame *frame)
{
    return dm4310_build_disable_command_for(DM4310_CAN_ID, frame);
}

bool dm4310_build_enable_command_for(uint8_t motor_id,
                                     Dm4310CanFrame *frame)
{
    return build_system_command(motor_id, 0xfcu, frame);
}

bool dm4310_build_disable_command_for(uint8_t motor_id,
                                      Dm4310CanFrame *frame)
{
    return build_system_command(motor_id, 0xfdu, frame);
}

bool dm4310_build_mit_command(int32_t position_millirad,
                              int32_t velocity_millirad_s,
                              int32_t kp_milli, int32_t kd_milli,
                              int32_t torque_millinewton_m,
                              Dm4310CanFrame *frame)
{
    return dm4310_build_mit_command_for(
        DM4310_CAN_ID, position_millirad, velocity_millirad_s, kp_milli,
        kd_milli, torque_millinewton_m, frame);
}

bool dm4310_build_mit_command_for(uint8_t motor_id,
                                  int32_t position_millirad,
                                  int32_t velocity_millirad_s,
                                  int32_t kp_milli, int32_t kd_milli,
                                  int32_t torque_millinewton_m,
                                  Dm4310CanFrame *frame)
{
    uint32_t position;
    uint32_t velocity;
    uint32_t kp;
    uint32_t kd;
    uint32_t torque;

    if (frame == NULL || motor_id > 0x0fu ||
        position_millirad < -DM4310_P_MAX_MILLIRAD ||
        position_millirad > DM4310_P_MAX_MILLIRAD ||
        velocity_millirad_s < -DM4310_V_MAX_MILLIRAD_S ||
        velocity_millirad_s > DM4310_V_MAX_MILLIRAD_S ||
        kp_milli < 0 || kp_milli > 500000 || kd_milli < 0 ||
        kd_milli > 5000 ||
        torque_millinewton_m < -DM4310_T_MAX_MILLINEWTON_M ||
        torque_millinewton_m > DM4310_T_MAX_MILLINEWTON_M) {
        return false;
    }

    position = encode_signed_range(position_millirad,
                                   DM4310_P_MAX_MILLIRAD, 65535u);
    velocity = encode_signed_range(velocity_millirad_s,
                                   DM4310_V_MAX_MILLIRAD_S, 4095u);
    kp = encode_unsigned_range(kp_milli, 500000, 4095u);
    kd = encode_unsigned_range(kd_milli, 5000, 4095u);
    torque = encode_signed_range(torque_millinewton_m,
                                 DM4310_T_MAX_MILLINEWTON_M, 4095u);

    frame->id = motor_id;
    frame->dlc = 8u;
    frame->data[0] = (uint8_t)(position >> 8);
    frame->data[1] = (uint8_t)position;
    frame->data[2] = (uint8_t)(velocity >> 4);
    frame->data[3] = (uint8_t)((velocity << 4) | (kp >> 8));
    frame->data[4] = (uint8_t)kp;
    frame->data[5] = (uint8_t)(kd >> 4);
    frame->data[6] = (uint8_t)((kd << 4) | (torque >> 8));
    frame->data[7] = (uint8_t)torque;
    return true;
}

bool dm4310_parse_parameter_response_for(
    uint8_t motor_id, uint8_t master_id, const Dm4310CanFrame *frame,
    Dm4310ParameterResponse *response)
{
    uint32_t raw_value;

    if (motor_id > 0x0fu || master_id > 0x7fu || frame == NULL ||
        response == NULL || frame->id != master_id ||
        frame->dlc != 8u || frame->data[0] != motor_id ||
        frame->data[1] != 0u || frame->data[2] != 0x33u ||
        !readable_register(frame->data[3])) {
        return false;
    }
    raw_value = unpack_u32_le(&frame->data[4]);
    response->register_id = frame->data[3];
    response->raw_value = raw_value;
    memcpy(&response->float_value, &raw_value, sizeof(response->float_value));
    return true;
}

bool dm4310_parse_parameter_response(const Dm4310CanFrame *frame,
                                     Dm4310ParameterResponse *response)
{
    return dm4310_parse_parameter_response_for(
        DM4310_CAN_ID, DM4310_MASTER_ID, frame, response);
}

bool dm4310_parse_feedback_for(uint8_t motor_id, uint8_t master_id,
                               const Dm4310CanFrame *frame,
                               Dm4310Feedback *feedback)
{
    uint32_t position;
    uint32_t velocity;
    uint32_t torque;

    if (motor_id > 0x0fu || master_id > 0x7fu || frame == NULL ||
        feedback == NULL || frame->id != master_id || frame->dlc != 8u ||
        (frame->data[0] & 0x0fu) != motor_id) {
        return false;
    }
    position = ((uint32_t)frame->data[1] << 8) | frame->data[2];
    velocity = ((uint32_t)frame->data[3] << 4) | (frame->data[4] >> 4);
    torque = ((uint32_t)(frame->data[4] & 0x0fu) << 8) | frame->data[5];

    feedback->motor_id = frame->data[0] & 0x0fu;
    feedback->state = frame->data[0] >> 4;
    feedback->position_millirad = decode_signed_range(
        position, DM4310_P_MAX_MILLIRAD, 65535u);
    feedback->velocity_millirad_s = decode_signed_range(
        velocity, DM4310_V_MAX_MILLIRAD_S, 4095u);
    feedback->torque_millinewton_m = decode_signed_range(
        torque, DM4310_T_MAX_MILLINEWTON_M, 4095u);
    feedback->mos_temperature_c = frame->data[6];
    feedback->rotor_temperature_c = frame->data[7];
    return true;
}

bool dm4310_parse_feedback(const Dm4310CanFrame *frame,
                           Dm4310Feedback *feedback)
{
    return dm4310_parse_feedback_for(DM4310_CAN_ID, DM4310_MASTER_ID,
                                     frame, feedback);
}

const char *dm4310_state_name(uint8_t state)
{
    switch (state) {
    case 0u: return "DISABLED";
    case 1u: return "ENABLED";
    case 2u: return "MOTOR_UNRECOGNIZED";
    case 3u: return "OUTPUT_ENCODER_UNRECOGNIZED";
    case 5u: return "ENCODER_ERROR";
    case 8u: return "OVERVOLTAGE";
    case 9u: return "UNDERVOLTAGE";
    case 10u: return "OVERCURRENT";
    case 11u: return "MOS_OVERTEMP";
    case 12u: return "MOTOR_OVERTEMP";
    case 13u: return "COMM_LOST";
    case 14u: return "OVERLOAD";
    default: return "UNKNOWN";
    }
}
