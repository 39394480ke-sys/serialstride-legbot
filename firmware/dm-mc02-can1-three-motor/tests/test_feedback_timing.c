#include <assert.h>
#include <stdint.h>

#include "feedback_timing.h"

static void test_uses_hardware_receive_age(void)
{
    assert(feedback_received_at_ms(1000u, 150000u, 1u) == 850u);
}

static void test_loop_stall_is_a_conservative_lower_bound(void)
{
    assert(feedback_received_at_ms(1000u, 1000u, 150u) == 850u);
}

static void test_rounds_submillisecond_hardware_age_up(void)
{
    assert(feedback_received_at_ms(1000u, 1u, 0u) == 999u);
    assert(feedback_received_at_ms(1000u, 1000u, 0u) == 999u);
    assert(feedback_received_at_ms(1000u, 1001u, 0u) == 998u);
}

static void test_timestamp_wrap_preserves_elapsed_age(void)
{
    uint32_t received_at = feedback_received_at_ms(10u, 0u, 20u);

    assert((uint32_t)(10u - received_at) == 20u);
}

int main(void)
{
    test_uses_hardware_receive_age();
    test_loop_stall_is_a_conservative_lower_bound();
    test_rounds_submillisecond_hardware_age_up();
    test_timestamp_wrap_preserves_elapsed_age();
    return 0;
}
