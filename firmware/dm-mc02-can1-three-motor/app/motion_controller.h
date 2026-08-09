#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_IDLE_DISABLED,
    MOTION_ARMED,
    MOTION_ENABLE_WAIT,
    MOTION_RUNNING,
    MOTION_CONTINUOUS,
    MOTION_WATCHDOG_RAMP,
    MOTION_ZERO_HOLD,
    MOTION_DISABLE_WAIT,
    MOTION_FAULT_ZERO,
    MOTION_FAULT_DISABLE
} MotionState;

typedef enum {
    MOTION_ACTION_NONE,
    MOTION_ACTION_ENABLE,
    MOTION_ACTION_VELOCITY,
    MOTION_ACTION_DISABLE
} MotionAction;

#define MOTION_ACTION_POSITIVE_VELOCITY MOTION_ACTION_VELOCITY
#define MOTION_ACTION_ZERO_VELOCITY MOTION_ACTION_VELOCITY

typedef enum {
    MOTION_EVENT_NONE,
    MOTION_EVENT_STATUS,
    MOTION_EVENT_ARMED,
    MOTION_EVENT_ARM_TIMEOUT,
    MOTION_EVENT_START_REJECTED,
    MOTION_EVENT_START_REQUESTED,
    MOTION_EVENT_RUNNING,
    MOTION_EVENT_CONTINUOUS_READY,
    MOTION_EVENT_TARGET_UPDATED,
    MOTION_EVENT_KEEPALIVE,
    MOTION_EVENT_HOST_WATCHDOG,
    MOTION_EVENT_ZERO_HOLD,
    MOTION_EVENT_DISABLE_REQUESTED,
    MOTION_EVENT_COMPLETE,
    MOTION_EVENT_EMERGENCY_STOP,
    MOTION_EVENT_SAFETY_TRIP,
    MOTION_EVENT_TX_FAILURE
} MotionEvent;

typedef struct {
    uint32_t now_ms;
    uint32_t feedback_age_ms;
    uint8_t parameter_mask;
    uint8_t control_mode;
    uint8_t motor_state;
    int32_t velocity_millirad_s;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    bool feedback_valid;
    bool can_warning;
    bool can_passive;
    bool can_bus_off;
} MotionSafetySnapshot;

typedef struct {
    MotionAction action;
    MotionEvent event;
    const char *reason;
    int8_t velocity_step;
} MotionDecision;

typedef struct {
    MotionState state;
    bool armed;
    bool continuous_requested;
    bool watchdog_stop;
    int8_t pulse_step;
    int8_t current_step;
    int8_t target_step;
    uint32_t armed_at_ms;
    uint32_t phase_started_ms;
    uint32_t last_action_ms;
    uint32_t last_control_ms;
    uint32_t last_ramp_ms;
} MotionController;

void motion_controller_init(MotionController *controller);
MotionDecision motion_controller_command(MotionController *controller,
                                          uint8_t command,
                                          const MotionSafetySnapshot *safety);
MotionDecision motion_controller_step(MotionController *controller,
                                       const MotionSafetySnapshot *safety);
MotionDecision motion_controller_tx_failed(MotionController *controller);
const char *motion_controller_state_name(MotionState state);
uint32_t motion_controller_watchdog_age(const MotionController *controller,
                                        uint32_t now_ms);

#endif
