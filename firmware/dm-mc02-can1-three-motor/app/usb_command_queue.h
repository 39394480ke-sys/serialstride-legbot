#ifndef USB_COMMAND_QUEUE_H
#define USB_COMMAND_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

void usb_command_queue_init(void);
void usb_command_queue_push_from_isr(const uint8_t *data, uint32_t length);
bool usb_command_queue_pop(uint8_t *command);
bool usb_command_queue_take_emergency_stop(void);
uint32_t usb_command_queue_dropped(void);

#endif
