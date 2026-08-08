#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_IDLE_DISABLED,
    MOTION_ARMED,
    MOTION_ENABLE_WAIT,
    MOTION_RUNNING,
    MOTION_ZERO_HOLD,
    MOTION_FAULT_ZERO,
    MOTION_FAULT_DISABLE
} MotionState;

typedef enum {
    MOTION_ACTION_NONE,
    MOTION_ACTION_ENABLE,
    MOTION_ACTION_POSITIVE_VELOCITY,
    MOTION_ACTION_ZERO_VELOCITY,
    MOTION_ACTION_DISABLE
} MotionAction;

typedef enum {
    MOTION_EVENT_NONE,
    MOTION_EVENT_STATUS,
    MOTION_EVENT_ARMED,
    MOTION_EVENT_ARM_TIMEOUT,
    MOTION_EVENT_START_REJECTED,
    MOTION_EVENT_START_REQUESTED,
    MOTION_EVENT_RUNNING,
    MOTION_EVENT_ZERO_HOLD,
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
} MotionDecision;

typedef struct {
    MotionState state;
    bool armed;
    uint32_t armed_at_ms;
    uint32_t phase_started_ms;
    uint32_t last_action_ms;
} MotionController;

void motion_controller_init(MotionController *controller);
MotionDecision motion_controller_command(MotionController *controller,
                                          uint8_t command,
                                          const MotionSafetySnapshot *safety);
MotionDecision motion_controller_step(MotionController *controller,
                                       const MotionSafetySnapshot *safety);
MotionDecision motion_controller_tx_failed(MotionController *controller);

#endif
