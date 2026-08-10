#ifndef TEST_USBD_CDC_IF_H
#define TEST_USBD_CDC_IF_H

#include <stdint.h>

#define USBD_OK 0u
#define USBD_BUSY 1u
#define USBD_FAIL 2u

uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t length);

#endif
