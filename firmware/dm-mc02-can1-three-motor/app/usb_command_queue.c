#include "usb_command_queue.h"

#include <stddef.h>

#define USB_COMMAND_QUEUE_CAPACITY 32u
#define USB_COMMAND_QUEUE_MASK (USB_COMMAND_QUEUE_CAPACITY - 1u)

static uint8_t queue[USB_COMMAND_QUEUE_CAPACITY];
static volatile uint8_t read_index;
static volatile uint8_t write_index;
static volatile uint32_t dropped_count;

void usb_command_queue_init(void)
{
    __atomic_store_n(&read_index, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&write_index, 0u, __ATOMIC_RELAXED);
    dropped_count = 0u;
}

void usb_command_queue_push_from_isr(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if (data == NULL) {
        return;
    }

    for (index = 0u; index < length; ++index) {
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
