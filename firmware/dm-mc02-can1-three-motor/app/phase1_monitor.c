#include "phase1_monitor.h"

#include <limits.h>

void phase1_monitor_init(Phase1Monitor *monitor)
{
    monitor->last_tick_ms = 0u;
    monitor->loop_count = 0u;
    monitor->missed_ticks = 0u;
    monitor->min_period_ms = UINT_MAX;
    monitor->max_period_ms = 0u;
    monitor->started = false;
}

bool phase1_monitor_step(Phase1Monitor *monitor, uint32_t now_ms)
{
    uint32_t period_ms;

    if (!monitor->started) {
        monitor->last_tick_ms = now_ms;
        monitor->loop_count = 1u;
        monitor->started = true;
        return true;
    }

    period_ms = now_ms - monitor->last_tick_ms;
    if (period_ms == 0u) {
        return false;
    }

    monitor->last_tick_ms = now_ms;
    monitor->loop_count++;
    if (period_ms < monitor->min_period_ms) {
        monitor->min_period_ms = period_ms;
    }
    if (period_ms > monitor->max_period_ms) {
        monitor->max_period_ms = period_ms;
    }
    if (period_ms > 1u) {
        monitor->missed_ticks += period_ms - 1u;
    }

    return true;
}
