#include "feedback_timing.h"

uint32_t feedback_received_at_ms(uint32_t now_ms, uint32_t hardware_age_us,
                                 uint32_t loop_period_ms)
{
    uint32_t hardware_age_ms = (hardware_age_us + 999u) / 1000u;
    uint32_t conservative_age_ms = hardware_age_ms;

    if (loop_period_ms > conservative_age_ms) {
        conservative_age_ms = loop_period_ms;
    }
    return now_ms - conservative_age_ms;
}
