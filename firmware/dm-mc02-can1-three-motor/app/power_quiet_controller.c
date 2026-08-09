#include "power_quiet_controller.h"

#include <stddef.h>

static PowerQuietDecision no_decision(void)
{
    PowerQuietDecision decision = {0};

    return decision;
}

static bool arm_expired(const PowerQuietController *controller,
                        uint32_t now_ms)
{
    return controller->state == POWER_QUIET_ARMED &&
           (uint32_t)(now_ms - controller->armed_at_ms) >=
               POWER_QUIET_ARM_WINDOW_MS;
}

void power_quiet_controller_init(PowerQuietController *controller)
{
    if (controller == NULL) {
        return;
    }
    controller->state = POWER_QUIET_OFF;
    controller->armed_at_ms = 0u;
}

PowerQuietDecision power_quiet_controller_command(
    PowerQuietController *controller, uint8_t command, uint32_t now_ms)
{
    PowerQuietDecision decision = no_decision();

    if (controller == NULL) {
        return decision;
    }
    if (command >= (uint8_t)'a' && command <= (uint8_t)'z') {
        command = (uint8_t)(command - ((uint8_t)'a' - (uint8_t)'A'));
    }

    if (command == (uint8_t)'X') {
        controller->state = POWER_QUIET_OFF;
        decision.event = POWER_QUIET_EVENT_OFF;
        decision.set_output = true;
        decision.output_on = false;
        return decision;
    }
    if (arm_expired(controller, now_ms)) {
        controller->state = POWER_QUIET_OFF;
        if (command == (uint8_t)'P') {
            decision.event = POWER_QUIET_EVENT_REJECTED;
            decision.reason = "ARM_TIMEOUT";
            return decision;
        }
    }

    switch (command) {
    case (uint8_t)'S':
        decision.event = POWER_QUIET_EVENT_STATUS;
        break;
    case (uint8_t)'A':
        if (controller->state == POWER_QUIET_ON) {
            decision.event = POWER_QUIET_EVENT_REJECTED;
            decision.reason = "ALREADY_POWERED";
        } else {
            controller->state = POWER_QUIET_ARMED;
            controller->armed_at_ms = now_ms;
            decision.event = POWER_QUIET_EVENT_ARMED;
        }
        break;
    case (uint8_t)'P':
        if (controller->state != POWER_QUIET_ARMED) {
            decision.event = POWER_QUIET_EVENT_REJECTED;
            decision.reason = controller->state == POWER_QUIET_ON
                                  ? "ALREADY_POWERED"
                                  : "SEND_A_FIRST";
        } else {
            controller->state = POWER_QUIET_ON;
            decision.event = POWER_QUIET_EVENT_ON;
            decision.set_output = true;
            decision.output_on = true;
        }
        break;
    case (uint8_t)'\r':
    case (uint8_t)'\n':
        break;
    default:
        decision.event = POWER_QUIET_EVENT_REJECTED;
        decision.reason = "UNKNOWN_COMMAND";
        break;
    }
    return decision;
}

PowerQuietDecision power_quiet_controller_step(
    PowerQuietController *controller, uint32_t now_ms)
{
    PowerQuietDecision decision = no_decision();

    if (controller != NULL && arm_expired(controller, now_ms)) {
        controller->state = POWER_QUIET_OFF;
        decision.event = POWER_QUIET_EVENT_ARM_TIMEOUT;
    }
    return decision;
}

const char *power_quiet_state_name(PowerQuietState state)
{
    switch (state) {
    case POWER_QUIET_ARMED:
        return "ARMED";
    case POWER_QUIET_ON:
        return "QUIET";
    case POWER_QUIET_OFF:
    default:
        return "OFF";
    }
}
