#include "h6215_protocol.h"

#include <string.h>

static bool is_readable_register(uint8_t register_id)
{
    return register_id == H6215_REGISTER_CONTROL_MODE ||
           register_id == H6215_REGISTER_SOFTWARE_VERSION ||
           register_id == H6215_REGISTER_P_MAX ||
           register_id == H6215_REGISTER_V_MAX ||
           register_id == H6215_REGISTER_T_MAX;
}

static uint32_t unpack_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static bool build_system_command(uint8_t opcode, H6215CanFrame *frame)
{
    if (frame == NULL) {
        return false;
    }

    frame->id = H6215_CAN_ID;
    frame->dlc = 8u;
    memset(frame->data, 0xff, 7u);
    frame->data[7] = opcode;
    return true;
}

bool h6215_build_read_request(uint8_t register_id, H6215CanFrame *frame)
{
    if (frame == NULL || !is_readable_register(register_id)) {
        return false;
    }

    frame->id = 0x7ffu;
    frame->dlc = 8u;
    frame->data[0] = (uint8_t)(H6215_CAN_ID & 0xffu);
    frame->data[1] = (uint8_t)(H6215_CAN_ID >> 8);
    frame->data[2] = 0x33u;
    frame->data[3] = register_id;
    memset(&frame->data[4], 0, 4u);
    return true;
}

bool h6215_build_feedback_request(H6215CanFrame *frame)
{
    if (frame == NULL) {
        return false;
    }

    frame->id = 0x7ffu;
    frame->dlc = 8u;
    frame->data[0] = (uint8_t)(H6215_CAN_ID & 0xffu);
    frame->data[1] = (uint8_t)(H6215_CAN_ID >> 8);
    frame->data[2] = 0xccu;
    memset(&frame->data[3], 0, 5u);
    return true;
}

bool h6215_build_disable_command(H6215CanFrame *frame)
{
    return build_system_command(0xfdu, frame);
}

bool h6215_build_enable_command(H6215CanFrame *frame)
{
    return build_system_command(0xfcu, frame);
}

bool h6215_build_velocity_step(int8_t step, H6215CanFrame *frame)
{
    static const uint8_t values[11][4] = {
        {0x00u, 0x00u, 0x00u, 0xbfu}, {0xcdu, 0xccu, 0xccu, 0xbeu},
        {0x9au, 0x99u, 0x99u, 0xbeu}, {0xcdu, 0xccu, 0x4cu, 0xbeu},
        {0xcdu, 0xccu, 0xccu, 0xbdu}, {0x00u, 0x00u, 0x00u, 0x00u},
        {0xcdu, 0xccu, 0xccu, 0x3du}, {0xcdu, 0xccu, 0x4cu, 0x3eu},
        {0x9au, 0x99u, 0x99u, 0x3eu}, {0xcdu, 0xccu, 0xccu, 0x3eu},
        {0x00u, 0x00u, 0x00u, 0x3fu},
    };

    if (frame == NULL || step < -5 || step > 5) {
        return false;
    }

    /* CAN ID 0x200 + motor ID 1, classic CAN, four-byte payload. */
    frame->id = 0x200u + H6215_CAN_ID;
    frame->dlc = 4u;
    memset(frame->data, 0, sizeof(frame->data));
    memcpy(frame->data, values[step + 5], 4u);
    return true;
}

bool h6215_build_positive_velocity_command(H6215CanFrame *frame)
{
    return h6215_build_velocity_step(2, frame);
}

bool h6215_build_zero_velocity_command(H6215CanFrame *frame)
{
    return h6215_build_velocity_step(0, frame);
}

bool h6215_parse_parameter_response(const H6215CanFrame *frame,
                                     H6215ParameterResponse *response)
{
    uint32_t raw_value;

    if (frame == NULL || response == NULL || frame->id != H6215_MASTER_ID ||
        frame->dlc != 8u || frame->data[0] != H6215_CAN_ID ||
        frame->data[1] != 0u || frame->data[2] != 0x33u ||
        !is_readable_register(frame->data[3])) {
        return false;
    }

    raw_value = unpack_u32_le(&frame->data[4]);
    response->register_id = frame->data[3];
    response->raw_value = raw_value;
    memcpy(&response->float_value, &raw_value, sizeof(response->float_value));
    return true;
}

bool h6215_parse_feedback(const H6215CanFrame *frame, H6215Feedback *feedback)
{
    uint32_t position_raw;
    uint32_t velocity_raw;
    uint32_t torque_raw;

    if (frame == NULL || feedback == NULL || frame->id != H6215_MASTER_ID ||
        frame->dlc != 8u || (frame->data[0] & 0x0fu) != H6215_CAN_ID) {
        return false;
    }

    position_raw = ((uint32_t)frame->data[1] << 8) | frame->data[2];
    velocity_raw = ((uint32_t)frame->data[3] << 4) | (frame->data[4] >> 4);
    torque_raw = ((uint32_t)(frame->data[4] & 0x0fu) << 8) | frame->data[5];

    feedback->motor_id = frame->data[0] & 0x0fu;
    feedback->state = frame->data[0] >> 4;
    feedback->position_millirad =
        (int32_t)((position_raw * 25000u) / 65535u) - 12500;
    feedback->velocity_millirad_s =
        (int32_t)((velocity_raw * 90000u) / 4095u) - 45000;
    feedback->torque_millinewton_m =
        (int32_t)((torque_raw * 20000u) / 4095u) - 10000;
    feedback->mos_temperature_c = frame->data[6];
    feedback->rotor_temperature_c = frame->data[7];
    return true;
}

bool h6215_decode_software_version(uint32_t raw_value, char version[5])
{
    uint8_t index;

    if (version == NULL) {
        return false;
    }
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

const char *h6215_state_name(uint8_t state)
{
    switch (state) {
    case 0u:
        return "DISABLED";
    case 1u:
        return "ENABLED";
    case 8u:
        return "OVERVOLTAGE";
    case 9u:
        return "UNDERVOLTAGE";
    case 10u:
        return "OVERCURRENT";
    case 11u:
        return "MOS_OVERTEMP";
    case 12u:
        return "MOTOR_OVERTEMP";
    case 13u:
        return "COMM_LOST";
    case 14u:
        return "OVERLOAD";
    default:
        return "UNKNOWN";
    }
}
