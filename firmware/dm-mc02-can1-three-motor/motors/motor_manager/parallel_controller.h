#ifndef PARALLEL_CONTROLLER_H
#define PARALLEL_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PARALLEL_DISABLED = 0,
    PARALLEL_ARMED,
    PARALLEL_ENABLE_WAIT,
    PARALLEL_RUNNING,
    PARALLEL_ZERO_HOLD,
    PARALLEL_DISABLE_WAIT,
} ParallelState;

typedef enum {
    PARALLEL_EVENT_NONE = 0,
    PARALLEL_EVENT_ARMED,
    PARALLEL_EVENT_REJECTED,
    PARALLEL_EVENT_ENABLE_WAIT,
    PARALLEL_EVENT_RUNNING,
    PARALLEL_EVENT_ZERO_HOLD,
    PARALLEL_EVENT_DISABLE_WAIT,
    PARALLEL_EVENT_COMPLETE,
    PARALLEL_EVENT_SAFETY_TRIP,
    PARALLEL_EVENT_ARM_TIMEOUT,
} ParallelEvent;

typedef struct {
    uint32_t now_ms;
    bool powered;
    bool probe_active;
    bool parameters_valid;
    bool feedback_fresh;
    bool all_disabled;
    bool all_enabled;
    bool start_speed_zero;
    bool runtime_speed_safe;
    bool runtime_torque_safe;
    bool temperature_safe;
    bool can_active;
    bool state_normal;
} ParallelSafetySnapshot;

typedef struct {
    bool send_enable_all;
    bool send_velocity_all;
    bool send_disable_all;
    bool cut_power;
    int8_t direction;
    ParallelEvent event;
    const char *reason;
} ParallelDecision;

typedef struct {
    ParallelState state;
    int8_t direction;
    uint32_t armed_at_ms;
    uint32_t phase_started_ms;
    uint32_t last_command_ms;
} ParallelController;

void parallel_controller_init(ParallelController *controller);
ParallelDecision parallel_controller_command(
    ParallelController *controller, uint8_t command,
    const ParallelSafetySnapshot *safety);
ParallelDecision parallel_controller_step(
    ParallelController *controller, const ParallelSafetySnapshot *safety);
bool parallel_controller_active(const ParallelController *controller);
const char *parallel_state_name(ParallelState state);

#endif
