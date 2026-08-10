#include "usb_command_queue.h"

#include <stddef.h>

#define USB_COMMAND_QUEUE_CAPACITY 64u
#define USB_COMMAND_QUEUE_MASK (USB_COMMAND_QUEUE_CAPACITY - 1u)

static uint8_t queue[USB_COMMAND_QUEUE_CAPACITY];
static volatile uint8_t read_index;
static volatile uint8_t write_index;
static volatile uint8_t emergency_stop_pending;
static volatile uint8_t stop_all_match_index;
static volatile uint8_t line_length;
static volatile uint32_t dropped_count;
static const uint8_t stop_all_command[] = "stop-all";

void usb_command_queue_init(void)
{
    __atomic_store_n(&read_index, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&write_index, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&emergency_stop_pending, 0u, __ATOMIC_RELAXED);
    stop_all_match_index = 0u;
    line_length = 0u;
    dropped_count = 0u;
}

void usb_command_queue_push_from_isr(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if (data == NULL) {
        return;
    }

    for (index = 0u; index < length; ++index) {
        uint8_t match = stop_all_match_index;

        if (data[index] == (uint8_t)'X' ||
            (data[index] == (uint8_t)'x' && line_length == 0u)) {
            __atomic_store_n(&emergency_stop_pending, 1u, __ATOMIC_RELEASE);
            continue;
        }
        if (data[index] == stop_all_command[match]) {
            ++match;
            if (match == sizeof(stop_all_command) - 1u) {
                __atomic_store_n(&emergency_stop_pending, 1u,
                                 __ATOMIC_RELEASE);
                match = 0u;
            }
        } else {
            match = data[index] == stop_all_command[0] ? 1u : 0u;
        }
        stop_all_match_index = match;
        if (data[index] == (uint8_t)'\r' || data[index] == (uint8_t)'\n')
            line_length = 0u;
        else if ((data[index] >= (uint8_t)'a' &&
                  data[index] <= (uint8_t)'z') ||
                 data[index] == (uint8_t)'-')
            line_length = line_length < UINT8_MAX
                              ? (uint8_t)(line_length + 1u)
                              : UINT8_MAX;
        else
            line_length = 0u;

        uint8_t write = __atomic_load_n(&write_index, __ATOMIC_RELAXED);
        uint8_t read = __atomic_load_n(&read_index, __ATOMIC_ACQUIRE);

        if ((uint8_t)(write - read) >= USB_COMMAND_QUEUE_CAPACITY) {
            ++dropped_count;
            continue;
        }

        queue[write & USB_COMMAND_QUEUE_MASK] = data[index];
        __atomic_store_n(&write_index, (uint8_t)(write + 1u),
                         __ATOMIC_RELEASE);
    }
}

bool usb_command_queue_take_emergency_stop(void)
{
    uint8_t write;

    if (__atomic_exchange_n(&emergency_stop_pending, 0u,
                            __ATOMIC_ACQ_REL) == 0u) {
        return false;
    }

    write = __atomic_load_n(&write_index, __ATOMIC_ACQUIRE);
    __atomic_store_n(&read_index, write, __ATOMIC_RELEASE);
    return true;
}

bool usb_command_queue_pop(uint8_t *command)
{
    uint8_t read;

    if (command == NULL) {
        return false;
    }

    read = __atomic_load_n(&read_index, __ATOMIC_RELAXED);
    if (read == __atomic_load_n(&write_index, __ATOMIC_ACQUIRE)) {
        return false;
    }

    *command = queue[read & USB_COMMAND_QUEUE_MASK];
    __atomic_store_n(&read_index, (uint8_t)(read + 1u), __ATOMIC_RELEASE);
    return true;
}

uint32_t usb_command_queue_dropped(void)
{
    return dropped_count;
}
