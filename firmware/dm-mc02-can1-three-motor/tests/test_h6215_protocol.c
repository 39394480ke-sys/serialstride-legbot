#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "h6215_protocol.h"

static void test_builds_read_only_parameter_request(void)
{
    H6215CanFrame frame = {0};

    assert(h6215_build_read_request(H6215_REGISTER_SOFTWARE_VERSION, &frame));
    assert(frame.id == 0x7ffu);
    assert(frame.dlc == 8u);
    assert(frame.data[0] == 0x01u);
    assert(frame.data[1] == 0x00u);
    assert(frame.data[2] == 0x33u);
    assert(frame.data[3] == H6215_REGISTER_SOFTWARE_VERSION);
    assert(frame.data[4] == 0u);
    assert(frame.data[5] == 0u);
    assert(frame.data[6] == 0u);
    assert(frame.data[7] == 0u);
    assert(!h6215_build_read_request(0xffu, &frame));
}

static void test_builds_read_only_feedback_request(void)
{
    H6215CanFrame frame = {0};

    assert(h6215_build_feedback_request(&frame));
    assert(frame.id == 0x7ffu);
    assert(frame.dlc == 8u);
    assert(frame.data[0] == 0x01u);
    assert(frame.data[1] == 0x00u);
    assert(frame.data[2] == 0xccu);
    for (uint8_t index = 3u; index < 8u; ++index) {
        assert(frame.data[index] == 0u);
    }
}

static void test_builds_disable_probe(void)
{
    H6215CanFrame frame = {0};

    assert(h6215_build_disable_command(&frame));
    assert(frame.id == H6215_CAN_ID);
    assert(frame.dlc == 8u);
    for (uint8_t index = 0u; index < 7u; ++index) {
        assert(frame.data[index] == 0xffu);
    }
    assert(frame.data[7] == 0xfdu);
}

static void test_parses_parameter_response(void)
{
    const H6215CanFrame frame = {
        .id = H6215_MASTER_ID,
        .dlc = 8u,
        .data = {0x01u, 0x00u, 0x33u, H6215_REGISTER_P_MAX,
                 0x00u, 0x00u, 0x48u, 0x41u},
    };
    H6215ParameterResponse response = {0};

    assert(h6215_parse_parameter_response(&frame, &response));
    assert(response.register_id == H6215_REGISTER_P_MAX);
    assert(response.raw_value == 0x41480000u);
    assert(fabsf(response.float_value - 12.5f) < 0.001f);
}

static void test_rejects_parameter_response_for_another_motor(void)
{
    const H6215CanFrame frame = {
        .id = H6215_MASTER_ID,
        .dlc = 8u,
        .data = {0x02u, 0x00u, 0x33u, H6215_REGISTER_CONTROL_MODE,
                 0x03u, 0x00u, 0x00u, 0x00u},
    };
    H6215ParameterResponse response = {0};

    assert(!h6215_parse_parameter_response(&frame, &response));
}

static void test_decodes_ascii_software_version(void)
{
    char version[5];

    assert(h6215_decode_software_version(0x36303435u, version));
    assert(strcmp(version, "5406") == 0);
    assert(!h6215_decode_software_version(0x01020304u, version));
}

static void test_parses_feedback_without_emitting_a_command(void)
{
    const H6215CanFrame frame = {
        .id = H6215_MASTER_ID,
        .dlc = 8u,
        .data = {0x11u, 0xffu, 0xffu, 0xffu,
                 0xffu, 0xffu, 28u, 26u},
    };
    H6215Feedback feedback = {0};

    assert(h6215_parse_feedback(&frame, &feedback));
    assert(feedback.motor_id == H6215_CAN_ID);
    assert(feedback.state == 1u);
    assert(strcmp(h6215_state_name(feedback.state), "ENABLED") == 0);
    assert(feedback.position_millirad == 12500);
    assert(feedback.velocity_millirad_s == 45000);
    assert(feedback.torque_millinewton_m == 10000);
    assert(feedback.mos_temperature_c == 28u);
    assert(feedback.rotor_temperature_c == 26u);
}

int main(void)
{
    test_builds_read_only_parameter_request();
    test_builds_read_only_feedback_request();
    test_builds_disable_probe();
    test_parses_parameter_response();
    test_rejects_parameter_response_for_another_motor();
    test_decodes_ascii_software_version();
    test_parses_feedback_without_emitting_a_command();
    return 0;
}
