#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "dm4310_protocol.h"

static void test_system_and_read_frames(void)
{
    Dm4310CanFrame frame;

    assert(dm4310_build_enable_command(&frame));
    assert(frame.id == 8u && frame.dlc == 8u && frame.data[7] == 0xfcu);
    for (uint8_t index = 0u; index < 7u; ++index) assert(frame.data[index] == 0xffu);
    assert(dm4310_build_disable_command(&frame));
    assert(frame.data[7] == 0xfdu);

    assert(dm4310_build_feedback_request(&frame));
    assert(frame.id == 0x7ffu && frame.data[0] == 8u &&
           frame.data[2] == 0xccu);
    assert(dm4310_build_read_request(DM4310_REGISTER_V_MAX, &frame));
    assert(frame.id == 0x7ffu && frame.data[0] == 8u &&
           frame.data[2] == 0x33u && frame.data[3] == 22u);
    assert(!dm4310_build_read_request(0xffu, &frame));
}

static void test_mit_known_vectors_and_bounds(void)
{
    Dm4310CanFrame frame;
    static const uint8_t zero_expected[8] = {
        0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x19, 0x97, 0xff,
    };

    assert(dm4310_build_mit_command(0, 0, 0, 500, 0, &frame));
    assert(frame.id == 8u && frame.dlc == 8u);
    assert(memcmp(frame.data, zero_expected, sizeof(zero_expected)) == 0);

    assert(dm4310_build_mit_command(0, 200, 0, 500, 0, &frame));
    assert(frame.data[2] == 0x80u && frame.data[3] == 0xd0u);
    assert(dm4310_build_mit_command(0, -200, 0, 500, 0, &frame));
    assert(frame.data[2] == 0x7fu && frame.data[3] == 0x10u);
    assert(dm4310_build_mit_command_for(6u, 0, 200, 0, 500, 0, &frame));
    assert(frame.id == 6u && frame.data[2] == 0x80u &&
           frame.data[3] == 0xd0u);
    assert(!dm4310_build_mit_command_for(16u, 0, 0, 0, 500, 0, &frame));

    assert(dm4310_build_mit_command(-12500, -30000, 0, 0, -10000,
                                    &frame));
    assert(dm4310_build_mit_command(12500, 30000, 500000, 5000, 10000,
                                    &frame));
    assert(!dm4310_build_mit_command(12501, 0, 0, 500, 0, &frame));
    assert(!dm4310_build_mit_command(0, 30001, 0, 500, 0, &frame));
    assert(!dm4310_build_mit_command(0, 0, 0, 5001, 0, &frame));
    assert(!dm4310_build_mit_command(0, 0, 0, 500, 10001, &frame));
}

static void test_parameter_and_feedback_parsing(void)
{
    Dm4310CanFrame parameter = {
        .id = 4u,
        .dlc = 8u,
        .data = {8u, 0u, 0x33u, 21u, 0x00u, 0x00u, 0x48u, 0x41u},
    };
    Dm4310ParameterResponse response;
    Dm4310CanFrame feedback_frame = {
        .id = 4u,
        .dlc = 8u,
        .data = {0x18u, 0x7fu, 0xffu, 0x7fu, 0xf7u, 0xffu, 31u, 29u},
    };
    Dm4310Feedback feedback;

    assert(dm4310_parse_parameter_response(&parameter, &response));
    assert(response.register_id == 21u);
    assert(fabsf(response.float_value - 12.5f) < 0.001f);

    assert(dm4310_parse_feedback(&feedback_frame, &feedback));
    assert(feedback.motor_id == 8u && feedback.state == 1u);
    assert(feedback.position_millirad >= -2 && feedback.position_millirad <= 0);
    assert(feedback.velocity_millirad_s >= -10 &&
           feedback.velocity_millirad_s <= 0);
    assert(feedback.torque_millinewton_m >= -5 &&
           feedback.torque_millinewton_m <= 0);
    assert(feedback.mos_temperature_c == 31u);
    assert(feedback.rotor_temperature_c == 29u);

    parameter.id = 3u;
    assert(!dm4310_parse_parameter_response(&parameter, &response));
    feedback_frame.data[0] = 0x15u;
    assert(!dm4310_parse_feedback(&feedback_frame, &feedback));
}

static void test_two_joint_addressing_is_isolated(void)
{
    Dm4310CanFrame request;
    Dm4310ParameterResponse parameter_response;
    Dm4310Feedback feedback;
    Dm4310CanFrame parameter = {
        .id = 3u,
        .dlc = 8u,
        .data = {6u, 0u, 0x33u, 10u, 1u, 0u, 0u, 0u},
    };
    Dm4310CanFrame feedback_frame = {
        .id = 4u,
        .dlc = 8u,
        .data = {0x08u, 0x7fu, 0xffu, 0x7fu, 0xf7u, 0xffu, 30u, 29u},
    };

    assert(dm4310_build_feedback_request_for(6u, &request));
    assert(request.id == 0x7ffu && request.data[0] == 6u);
    assert(dm4310_build_read_request_for(8u, DM4310_REGISTER_T_MAX,
                                         &request));
    assert(request.data[0] == 8u && request.data[3] == 23u);
    assert(!dm4310_build_feedback_request_for(16u, &request));
    assert(dm4310_build_disable_command_for(6u, &request));
    assert(request.id == 6u && request.data[7] == 0xfdu);

    assert(dm4310_parse_parameter_response_for(
        6u, 3u, &parameter, &parameter_response));
    assert(!dm4310_parse_parameter_response_for(
        8u, 4u, &parameter, &parameter_response));
    assert(dm4310_parse_feedback_for(8u, 4u, &feedback_frame, &feedback));
    assert(feedback.motor_id == 8u);
    assert(!dm4310_parse_feedback_for(6u, 3u, &feedback_frame, &feedback));
}

int main(void)
{
    test_system_and_read_frames();
    test_mit_known_vectors_and_bounds();
    test_parameter_and_feedback_parsing();
    test_two_joint_addressing_is_isolated();
    return 0;
}
