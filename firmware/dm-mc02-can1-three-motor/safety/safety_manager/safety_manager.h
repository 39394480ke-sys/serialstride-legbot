#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "can_bus.h"
#include "dm4310_controller.h"
#include "motion_controller.h"
#include "motor_manager.h"
#include "parallel_controller.h"

const char *safety_manager_global_fault(const MotorManager *manager,
                                        MotorRole selected,
                                        bool motion_active,
                                        bool parallel_active,
                                        uint32_t now_ms);
Dm4310SafetySnapshot safety_manager_joint_snapshot(
    const MotorManager *manager, MotorRole role,
    const CanBusStatus *can_status, bool can_status_valid,
    bool probe_active, uint32_t now_ms);
MotionSafetySnapshot safety_manager_wheel_snapshot(
    const MotorManager *manager, const CanBusStatus *can_status,
    bool can_status_valid, uint32_t now_ms);
ParallelSafetySnapshot safety_manager_parallel_snapshot(
    const MotorManager *manager, const CanBusStatus *can_status,
    bool can_status_valid, bool probe_active, uint32_t now_ms);

#endif
