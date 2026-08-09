#ifndef DM4310_CONTROLLER_H
#define DM4310_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DM4310_MOTION_DISABLED = 0,
    DM4310_MOTION_ARMED,
    DM4310_MOTION_ENABLE_WAIT,
    DM4310_MOTION_PULSE,
    DM4310_MOTION_ZERO_HOLD,
    DM4310_MOTION_DISABLE_WAIT,
} Dm4310MotionState;

typedef enum {
    DM4310_EVENT_NONE = 0,
    DM4310_EVENT_STATUS,
    DM4310_EVENT_ARMED,
    DM4310_EVENT_ARM_TIMEOUT,
    DM4310_EVENT_REJECTED,
    DM4310_EVENT_ENABLE_REQUESTED,
    DM4310_EVENT_RUNNING,
    DM4310_EVENT_ZERO_HOLD,
    DM4310_EVENT_DISABLE_REQUESTED,
    DM4310_EVENT_COMPLETE,
    DM4310_EVENT_EMERGENCY_STOP,
    DM4310_EVENT_SAFETY_TRIP,
    DM4310_EVENT_TX_FAILURE,
} Dm4310MotionEvent;

typedef struct {
    uint32_t now_ms;
    uint32_t feedback_age_ms;
    uint8_t motor_state;
    int32_t velocity_millirad_s;
    int32_t torque_millinewton_m;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    bool powered;
    bool probe_active;
    bool parameters_valid;
    bool feedback_valid;
    bool can_warning;
    bool can_passive;
    bool can_bus_off;
} Dm4310SafetySnapshot;

typedef struct {
    bool send_enable;
    bool send_mit;
    bool send_disable;
    bool cut_power;
    int32_t target_velocity_millirad_s;
    Dm4310MotionEvent event;
    const char *reason;
} Dm4310MotionDecision;

typedef struct {
    Dm4310MotionState state;
    int32_t target_velocity_millirad_s;
    uint32_t pulse_duration_ms;
    uint32_t armed_at_ms;
    uint32_t phase_started_ms;
    uint32_t last_command_ms;
} Dm4310Controller;

void dm4310_controller_init(Dm4310Controller *controller);
Dm4310MotionDecision dm4310_controller_command(
    Dm4310Controller *controller, uint8_t command,
    const Dm4310SafetySnapshot *safety);
Dm4310MotionDecision dm4310_controller_step(
    Dm4310Controller *controller, const Dm4310SafetySnapshot *safety);
Dm4310MotionDecision dm4310_controller_tx_failed(
    Dm4310Controller *controller);
const char *dm4310_motion_state_name(Dm4310MotionState state);

#endif
