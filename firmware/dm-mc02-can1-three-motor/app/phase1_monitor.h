#ifndef PHASE1_MONITOR_H
#define PHASE1_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t last_tick_ms;
    uint32_t loop_count;
    uint32_t missed_ticks;
    uint32_t min_period_ms;
    uint32_t max_period_ms;
    bool started;
} Phase1Monitor;

void phase1_monitor_init(Phase1Monitor *monitor);
bool phase1_monitor_step(Phase1Monitor *monitor, uint32_t now_ms);

#endif
