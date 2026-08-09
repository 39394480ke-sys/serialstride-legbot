#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "power_quiet_controller.h"

static void test_default_and_guarded_power_on(void)
{
    PowerQuietController controller;
    PowerQuietDecision decision;

    power_quiet_controller_init(&controller);
    assert(controller.state == POWER_QUIET_OFF);

    decision = power_quiet_controller_command(&controller, 'P', 1u);
    assert(decision.event == POWER_QUIET_EVENT_REJECTED);
    assert(strcmp(decision.reason, "SEND_A_FIRST") == 0);
    assert(!decision.set_output);

    decision = power_quiet_controller_command(&controller, 'a', 10u);
    assert(decision.event == POWER_QUIET_EVENT_ARMED);
    assert(controller.state == POWER_QUIET_ARMED);

    decision = power_quiet_controller_command(&controller, 'p', 10009u);
    assert(decision.event == POWER_QUIET_EVENT_ON);
    assert(decision.set_output && decision.output_on);
    assert(controller.state == POWER_QUIET_ON);
}

static void test_arm_deadline_and_wraparound(void)
{
    PowerQuietController controller;
    PowerQuietDecision decision;

    power_quiet_controller_init(&controller);
    (void)power_quiet_controller_command(&controller, 'A', 20u);
    decision = power_quiet_controller_command(&controller, 'P', 10020u);
    assert(decision.event == POWER_QUIET_EVENT_REJECTED);
    assert(strcmp(decision.reason, "ARM_TIMEOUT") == 0);
    assert(controller.state == POWER_QUIET_OFF);

    (void)power_quiet_controller_command(&controller, 'A', UINT32_MAX - 5u);
    decision = power_quiet_controller_step(&controller, 9993u);
    assert(decision.event == POWER_QUIET_EVENT_NONE);
    decision = power_quiet_controller_step(&controller, 9994u);
    assert(decision.event == POWER_QUIET_EVENT_ARM_TIMEOUT);
    assert(controller.state == POWER_QUIET_OFF);
}

static void test_x_always_forces_output_off(void)
{
    PowerQuietController controller;
    PowerQuietDecision decision;

    power_quiet_controller_init(&controller);
    decision = power_quiet_controller_command(&controller, 'X', 0u);
    assert(decision.set_output && !decision.output_on);

    (void)power_quiet_controller_command(&controller, 'A', 1u);
    decision = power_quiet_controller_command(&controller, 'x', 2u);
    assert(decision.set_output && !decision.output_on);
    assert(controller.state == POWER_QUIET_OFF);

    (void)power_quiet_controller_command(&controller, 'A', 3u);
    (void)power_quiet_controller_command(&controller, 'P', 4u);
    decision = power_quiet_controller_command(&controller, 'X', 5u);
    assert(decision.event == POWER_QUIET_EVENT_OFF);
    assert(decision.set_output && !decision.output_on);
    assert(controller.state == POWER_QUIET_OFF);
}

int main(void)
{
    test_default_and_guarded_power_on();
    test_arm_deadline_and_wraparound();
    test_x_always_forces_output_off();
    puts("power quiet controller tests passed");
    return 0;
}
