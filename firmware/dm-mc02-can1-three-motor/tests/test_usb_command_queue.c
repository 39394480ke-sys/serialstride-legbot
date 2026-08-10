#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "usb_command_queue.h"

static void test_preserves_fifo_order(void)
{
    static const uint8_t input[] = {'G', 'S'};
    uint8_t command = 0u;

    usb_command_queue_init();
    usb_command_queue_push_from_isr(input, sizeof(input));

    assert(usb_command_queue_pop(&command) && command == 'G');
    assert(usb_command_queue_pop(&command) && command == 'S');
    assert(!usb_command_queue_pop(&command));
}

static void test_keeps_packet_bytes_individual(void)
{
    static const uint8_t input[] = {'A', '\r', '\n', 'G'};
    uint8_t command = 0u;

    usb_command_queue_init();
    usb_command_queue_push_from_isr(input, sizeof(input));

    assert(usb_command_queue_pop(&command) && command == 'A');
    assert(usb_command_queue_pop(&command) && command == '\r');
    assert(usb_command_queue_pop(&command) && command == '\n');
    assert(usb_command_queue_pop(&command) && command == 'G');
    assert(!usb_command_queue_pop(&command));
}

static void test_wraparound_preserves_order(void)
{
    uint8_t input[32];
    uint8_t command = 0u;

    usb_command_queue_init();
    for (uint8_t index = 0u; index < 32u; ++index) {
        input[index] = index;
    }
    usb_command_queue_push_from_isr(input, sizeof(input));
    for (uint8_t expected = 0u; expected < 20u; ++expected) {
        assert(usb_command_queue_pop(&command) && command == expected);
    }

    for (uint8_t index = 0u; index < 20u; ++index) {
        input[index] = (uint8_t)(32u + index);
    }
    usb_command_queue_push_from_isr(input, 20u);

    for (uint8_t expected = 20u; expected < 52u; ++expected) {
        assert(usb_command_queue_pop(&command) && command == expected);
    }
    assert(!usb_command_queue_pop(&command));
    assert(usb_command_queue_dropped() == 0u);
}

static void test_null_arguments_are_ignored(void)
{
    static const uint8_t input[] = {'A'};
    uint8_t command = 0u;

    usb_command_queue_init();
    usb_command_queue_push_from_isr(NULL, sizeof(input));
    usb_command_queue_push_from_isr(input, 0u);
    assert(!usb_command_queue_pop(NULL));
    assert(!usb_command_queue_pop(&command));
    assert(usb_command_queue_dropped() == 0u);

    usb_command_queue_push_from_isr(input, sizeof(input));
    assert(!usb_command_queue_pop(NULL));
    assert(usb_command_queue_pop(&command) && command == 'A');
}

static void test_full_queue_drops_newest_bytes_deterministically(void)
{
    uint8_t input[67];
    uint8_t command = 0u;

    usb_command_queue_init();
    for (uint8_t index = 0u; index < 67u; ++index) {
        input[index] = index;
    }
    usb_command_queue_push_from_isr(input, sizeof(input));

    assert(usb_command_queue_dropped() == 3u);
    for (uint8_t expected = 0u; expected < 64u; ++expected) {
        assert(usb_command_queue_pop(&command) && command == expected);
    }
    assert(!usb_command_queue_pop(&command));

    usb_command_queue_push_from_isr(&input[64], 3u);
    assert(usb_command_queue_dropped() == 3u);
    for (uint8_t expected = 64u; expected < 67u; ++expected) {
        assert(usb_command_queue_pop(&command) && command == expected);
    }
}

static void test_emergency_stop_bypasses_full_queue_and_flushes_commands(void)
{
    uint8_t input[33];
    uint8_t command = 0u;

    usb_command_queue_init();
    for (uint8_t index = 0u; index < 32u; ++index) {
        input[index] = index == 0u ? (uint8_t)'A' : (uint8_t)'\n';
    }
    input[32] = (uint8_t)'X';
    usb_command_queue_push_from_isr(input, sizeof(input));

    assert(usb_command_queue_take_emergency_stop());
    assert(!usb_command_queue_take_emergency_stop());
    assert(!usb_command_queue_pop(&command));
}

static void test_lowercase_x_inside_extend_command_is_not_emergency(void)
{
    static const uint8_t input[] = "capture-extend\n";
    uint8_t command = 0u;
    size_t index;

    usb_command_queue_init();
    usb_command_queue_push_from_isr(input, sizeof(input) - 1u);

    assert(!usb_command_queue_take_emergency_stop());
    for (index = 0u; index < sizeof(input) - 1u; ++index)
        assert(usb_command_queue_pop(&command) && command == input[index]);
}

static void test_standalone_lowercase_x_remains_emergency_stop(void)
{
    static const uint8_t input[] = {'A', 'G', 'x', 'G'};

    usb_command_queue_init();
    usb_command_queue_push_from_isr(input, sizeof(input));
    assert(usb_command_queue_take_emergency_stop());
}

static void test_stop_all_word_has_priority_across_usb_packets(void)
{
    static const uint8_t first[] = {'A', 'G', 's', 't', 'o'};
    static const uint8_t second[] = {'p', '-', 'a', 'l', 'l', '\n', 'G'};
    uint8_t command = 0u;

    usb_command_queue_init();
    usb_command_queue_push_from_isr(first, sizeof(first));
    assert(!usb_command_queue_take_emergency_stop());
    usb_command_queue_push_from_isr(second, sizeof(second));
    assert(usb_command_queue_take_emergency_stop());
    assert(!usb_command_queue_pop(&command));
}

int main(void)
{
    test_preserves_fifo_order();
    test_keeps_packet_bytes_individual();
    test_wraparound_preserves_order();
    test_null_arguments_are_ignored();
    test_full_queue_drops_newest_bytes_deterministically();
    test_emergency_stop_bypasses_full_queue_and_flushes_commands();
    test_lowercase_x_inside_extend_command_is_not_emergency();
    test_standalone_lowercase_x_remains_emergency_stop();
    test_stop_all_word_has_priority_across_usb_packets();
    return 0;
}
