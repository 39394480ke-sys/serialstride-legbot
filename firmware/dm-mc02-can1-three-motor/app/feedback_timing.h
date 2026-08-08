#ifndef FEEDBACK_TIMING_H
#define FEEDBACK_TIMING_H

#include <stdint.h>

uint32_t feedback_received_at_ms(uint32_t now_ms, uint32_t hardware_age_us,
                                 uint32_t loop_period_ms);

#endif
