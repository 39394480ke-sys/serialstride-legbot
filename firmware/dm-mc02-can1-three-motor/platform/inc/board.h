#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    uint8_t last_error_code;
    bool error_passive;
    bool warning;
    bool bus_off;
} Can1Status;

void board_clock_init(void);
void board_motor_power_init(void);
void board_motor_power_set(bool enabled);
bool board_motor_power_is_enabled(void);
bool board_can1_init(void);
bool board_can1_get_status(Can1Status *status);
bool board_can1_recover(void);
bool board_can1_transmit(uint32_t standard_id, const uint8_t data[8], uint8_t dlc);
bool board_can1_receive(uint32_t *standard_id, uint8_t data[8], uint8_t *dlc,
                        uint32_t *age_us);

#endif
