#ifndef POWER_QUIET_CONTROLLER_H
#define POWER_QUIET_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#define POWER_QUIET_ARM_WINDOW_MS 10000u

typedef enum {
    POWER_QUIET_OFF = 0,
    POWER_QUIET_ARMED,
    POWER_QUIET_ON,
} PowerQuietState;

typedef enum {
    POWER_QUIET_EVENT_NONE = 0,
    POWER_QUIET_EVENT_STATUS,
    POWER_QUIET_EVENT_ARMED,
    POWER_QUIET_EVENT_ARM_TIMEOUT,
    POWER_QUIET_EVENT_ON,
    POWER_QUIET_EVENT_OFF,
    POWER_QUIET_EVENT_REJECTED,
} PowerQuietEvent;

typedef struct {
    PowerQuietEvent event;
    const char *reason;
    bool set_output;
    bool output_on;
} PowerQuietDecision;

typedef struct {
    PowerQuietState state;
    uint32_t armed_at_ms;
} PowerQuietController;

void power_quiet_controller_init(PowerQuietController *controller);
PowerQuietDecision power_quiet_controller_command(
    PowerQuietController *controller, uint8_t command, uint32_t now_ms);
PowerQuietDecision power_quiet_controller_step(
    PowerQuietController *controller, uint32_t now_ms);
const char *power_quiet_state_name(PowerQuietState state);

#endif
