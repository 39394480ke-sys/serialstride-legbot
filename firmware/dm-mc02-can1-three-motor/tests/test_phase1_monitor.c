#include <assert.h>
#include <stdint.h>

#include "phase1_monitor.h"

static void test_runs_once_per_new_millisecond(void)
{
    Phase1Monitor monitor;

    phase1_monitor_init(&monitor);

    assert(phase1_monitor_step(&monitor, 100u));
    assert(!phase1_monitor_step(&monitor, 100u));
    assert(phase1_monitor_step(&monitor, 101u));
    assert(monitor.loop_count == 2u);
    assert(monitor.min_period_ms == 1u);
    assert(monitor.max_period_ms == 1u);
    assert(monitor.missed_ticks == 0u);
}

static void test_records_delayed_control_periods(void)
{
    Phase1Monitor monitor;

    phase1_monitor_init(&monitor);
    assert(phase1_monitor_step(&monitor, 10u));
    assert(phase1_monitor_step(&monitor, 13u));
    assert(monitor.min_period_ms == 3u);
    assert(monitor.max_period_ms == 3u);
    assert(monitor.missed_ticks == 2u);
    assert(phase1_monitor_step(&monitor, 14u));
    assert(monitor.min_period_ms == 1u);
    assert(monitor.max_period_ms == 3u);
}

static void test_handles_tick_counter_wraparound(void)
{
    Phase1Monitor monitor;

    phase1_monitor_init(&monitor);
    assert(phase1_monitor_step(&monitor, UINT32_MAX));
    assert(phase1_monitor_step(&monitor, 0u));
    assert(monitor.min_period_ms == 1u);
    assert(monitor.max_period_ms == 1u);
    assert(monitor.missed_ticks == 0u);
}

int main(void)
{
    test_runs_once_per_new_millisecond();
    test_records_delayed_control_periods();
    test_handles_tick_counter_wraparound();
    return 0;
}
