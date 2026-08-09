#include "can_bus.h"

bool can_bus_init(void)
{
    return board_can1_init();
}

bool can_bus_status(CanBusStatus *status)
{
    return board_can1_get_status(status);
}

bool can_bus_transmit(uint32_t standard_id, const uint8_t data[8],
                      uint8_t dlc)
{
    return board_can1_transmit(standard_id, data, dlc);
}

bool can_bus_receive(uint32_t *standard_id, uint8_t data[8], uint8_t *dlc,
                     uint32_t *age_us)
{
    return board_can1_receive(standard_id, data, dlc, age_us);
}
