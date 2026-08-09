#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

typedef Can1Status CanBusStatus;

bool can_bus_init(void);
bool can_bus_status(CanBusStatus *status);
bool can_bus_transmit(uint32_t standard_id, const uint8_t data[8],
                      uint8_t dlc);
bool can_bus_receive(uint32_t *standard_id, uint8_t data[8], uint8_t *dlc,
                     uint32_t *age_us);

#endif
