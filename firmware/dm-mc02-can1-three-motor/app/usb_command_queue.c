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
    read_index = 0u;
    write_index = 0u;
    dropped_count = 0u;
}

void usb_command_queue_push_from_isr(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if (data == NULL) {
        return;
    }

    for (index = 0u; index < length; ++index) {
        uint8_t write = write_index;

        if ((uint8_t)(write - read_index) >= USB_COMMAND_QUEUE_CAPACITY) {
            ++dropped_count;
            continue;
        }

        queue[write & USB_COMMAND_QUEUE_MASK] = data[index];
        write_index = (uint8_t)(write + 1u);
    }
}

bool usb_command_queue_pop(uint8_t *command)
{
    uint8_t read;

    if (command == NULL) {
        return false;
    }

    read = read_index;
    if (read == write_index) {
        return false;
    }

    *command = queue[read & USB_COMMAND_QUEUE_MASK];
    read_index = (uint8_t)(read + 1u);
    return true;
}

uint32_t usb_command_queue_dropped(void)
{
    return dropped_count;
}
