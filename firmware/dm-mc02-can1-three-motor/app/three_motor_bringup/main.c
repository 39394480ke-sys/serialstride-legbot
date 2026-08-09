#include <stdio.h>
#include <string.h>

#include "board.h"
#include "can_bus.h"
#include "dm4310_controller.h"
#include "dm4310_protocol.h"
#include "gpio.h"
#include "h6215_protocol.h"
#include "main.h"
#include "motion_controller.h"
#include "motor_manager.h"
#include "parallel_controller.h"
#include "phase1_monitor.h"
#include "power_quiet_controller.h"
#include "safety_manager.h"
#include "serial_logger.h"
#include "usb_command_queue.h"
#include "usb_device.h"

#define BOOT_LOG_DELAY_MS 1500u
#define TELEMETRY_MS 1000u
#define FEEDBACK_SLOT_MS 10u
#define PARAMETER_SLOT_MS 30u
#define USB_COMMANDS_PER_LOOP 32u
#define MIT_KD_MILLI 1000

static const uint8_t parameter_registers[] = {
    DM4310_REGISTER_SOFTWARE_VERSION,
    DM4310_REGISTER_CONTROL_MODE,
    DM4310_REGISTER_P_MAX,
    DM4310_REGISTER_V_MAX,
    DM4310_REGISTER_T_MAX,
};

static ParallelController parallel_motion;
static SerialLogger serial_logger;

static bool enqueue_log(MotionLogQueue *queue, uint32_t *dropped_logs,
                        const char *record)
{
    (void)queue;
    (void)dropped_logs;
    return serial_logger_write(&serial_logger, record);
}

static void service_log_queue(MotionLogQueue *queue, uint32_t *dropped_logs)
{
    (void)queue;
    serial_logger_service(&serial_logger);
    *dropped_logs = serial_logger_dropped(&serial_logger);
}

static void format_milli(char *buffer, size_t capacity, int32_t value)
{
    uint32_t magnitude = value < 0 ? (uint32_t)(-value) : (uint32_t)value;

    (void)snprintf(buffer, capacity, "%s%lu.%03lu", value < 0 ? "-" : "",
                   (unsigned long)(magnitude / 1000u),
                   (unsigned long)(magnitude % 1000u));
}

static void power_off(PowerQuietController *power, bool *probe_active,
                      Dm4310Controller *joint_motion,
                      MotionController *wheel_motion)
{
    board_motor_power_set(false);
    power_quiet_controller_init(power);
    *probe_active = false;
    dm4310_controller_init(joint_motion);
    motion_controller_init(wheel_motion);
    parallel_controller_init(&parallel_motion);
}

static void emergency_stop(MotorManager *manager,
                           PowerQuietController *power, bool *probe_active,
                           Dm4310Controller *joint_motion,
                           MotionController *wheel_motion,
                           MotionLogQueue *queue, uint32_t *dropped_logs)
{
    if (board_motor_power_is_enabled())
        (void)motor_manager_send_disable_all(manager);
    power_off(power, probe_active, joint_motion, wheel_motion);
    (void)enqueue_log(queue, dropped_logs,
                      "EMERGENCY_STOP_REQUESTED DISABLE_ALL POWER_OFF\r\n");
}

static void enqueue_power_status(MotionLogQueue *queue,
                                 uint32_t *dropped_logs,
                                 const PowerQuietController *power,
                                 bool probe_active)
{
    char record[160];

    (void)snprintf(record, sizeof(record),
                   "[POWER] VCC_OUT1=%s MODE=%s STATE=%s\r\n",
                   board_motor_power_is_enabled() ? "ON" : "OFF",
                   probe_active ? "READ_ONLY" : "QUIET",
                   power_quiet_state_name(power->state));
    (void)enqueue_log(queue, dropped_logs, record);
}

static size_t append_motor_status(char *record, size_t capacity, size_t used,
                                  const MotorManager *manager,
                                  const MotorState *motor, MotorRole role,
                                  uint32_t now_ms)
{
    char position[16];
    char velocity[16];
    char torque[16];
    char p_max[16];
    char v_max[16];
    char t_max[16];
    char age[16];
    int written;

    if (used >= capacity) return used;

    format_milli(position, sizeof(position), motor->position_millirad);
    format_milli(velocity, sizeof(velocity), motor->velocity_millirad_s);
    format_milli(torque, sizeof(torque), motor->torque_millinewton_m);
    format_milli(p_max, sizeof(p_max), motor->p_max_milli);
    format_milli(v_max, sizeof(v_max), motor->v_max_milli);
    format_milli(t_max, sizeof(t_max), motor->t_max_milli);
    if (motor->feedback_valid)
        (void)snprintf(age, sizeof(age), "%lu",
                       (unsigned long)(now_ms - motor->last_rx_ms));
    else
        (void)snprintf(age, sizeof(age), "NA");

    written = snprintf(
        record + used, capacity - used,
        "[%s] ONLINE=%u ID=%u MST=%u ST=%s AGE=%s SW=%s MODE=%u "
        "LIM=%s/%s/%s PARAM_OK=%u RX=%lu POS=%s V=%s T=%s "
        "TEMP=%u/%u TXF=%lu\r\n",
        motor_role_name(role), motor->online, motor->can_id, motor->mst_id,
        motor->feedback_valid ? motor_state_name(motor) : "NO_FEEDBACK",
        age, motor->software_version[0] != '\0' ? motor->software_version
                                                : "UNKNOWN",
        motor->control_mode, p_max, v_max, t_max,
        motor_manager_parameters_valid(manager, role),
        (unsigned long)motor->rx_count,
        motor->feedback_valid ? position : "NA",
        motor->feedback_valid ? velocity : "NA",
        motor->feedback_valid ? torque : "NA",
        motor->feedback_valid ? motor->mos_temperature_c : 0u,
        motor->feedback_valid ? motor->rotor_temperature_c : 0u,
        (unsigned long)motor->tx_failed);
    if (written < 0) return used;
    if ((size_t)written >= capacity - used) return capacity - 1u;
    return used + (size_t)written;
}

static void enqueue_status(MotionLogQueue *queue, uint32_t *dropped_logs,
                           const PowerQuietController *power,
                           const MotorManager *manager, bool probe_active,
                           const Can1Status *can_status,
                           bool can_status_valid, uint32_t now_ms,
                           bool requested)
{
    MotorRole role;
    char record[MOTION_LOG_RECORD_CAPACITY];
    size_t used;
    int written;

    used = (size_t)snprintf(
        record, sizeof(record), "%s[POWER] VCC_OUT1=%s MODE=%s STATE=%s\r\n",
        requested ? "STATUS_REQUESTED\r\n" : "",
        board_motor_power_is_enabled() ? "ON" : "OFF",
        probe_active ? "READ_ONLY" : "QUIET",
        power_quiet_state_name(power->state));
    for (role = MOTOR_ROLE_JOINT_A; role < MOTOR_ROLE_COUNT; ++role)
        used = append_motor_status(record, sizeof(record), used,
                                   manager, &manager->motors[role], role,
                                   now_ms);
    written = snprintf(
        record + used, sizeof(record) - used,
        "[CAN1] ACTIVE=%u TEC=%u REC=%u LEC=%u WARN=%u PASSIVE=%u "
        "BUS_OFF=%u UNKNOWN_RX=%lu MOTION=ONE_AT_A_TIME\r\n",
        can_status_valid, can_status->tx_error_count,
        can_status->rx_error_count, can_status->last_error_code,
        can_status->warning, can_status->error_passive, can_status->bus_off,
        (unsigned long)manager->unknown_rx);
    if (written > 0 && (size_t)written < sizeof(record) - used)
        used += (size_t)written;
    if (used >= sizeof(record)) used = sizeof(record) - 1u;
    record[used] = '\0';
    (void)enqueue_log(queue, dropped_logs, record);
}

static bool joint_motion_active(const Dm4310Controller *motion)
{
    return motion->state == DM4310_MOTION_ENABLE_WAIT ||
           motion->state == DM4310_MOTION_PULSE ||
           motion->state == DM4310_MOTION_ZERO_HOLD ||
           motion->state == DM4310_MOTION_DISABLE_WAIT;
}

static bool wheel_motion_active(const MotionController *motion)
{
    return motion->state != MOTION_IDLE_DISABLED &&
           motion->state != MOTION_ARMED;
}

static bool any_motion_active(const Dm4310Controller *joint_motion,
                              const MotionController *wheel_motion)
{
    return joint_motion_active(joint_motion) ||
           wheel_motion_active(wheel_motion) ||
           parallel_controller_active(&parallel_motion);
}

static bool execute_parallel_decision(
    ParallelDecision decision, MotorManager *manager,
    PowerQuietController *power, Dm4310Controller *joint_motion,
    MotionController *wheel_motion, bool *probe_active,
    MotionLogQueue *queue, uint32_t *dropped_logs)
{
    bool success = true;
    char record[192];

    if (decision.send_enable_all)
        success = motor_manager_send_enable_all(manager);
    if (success && decision.send_velocity_all)
        success = motor_manager_send_velocity_all(manager,
                                                  decision.direction);
    if (success && decision.send_disable_all)
        success = motor_manager_send_disable_all(manager);
    if (!success || decision.cut_power) {
        (void)motor_manager_send_disable_all(manager);
        power_off(power, probe_active, joint_motion, wheel_motion);
    }
    if (!success) {
        (void)enqueue_log(queue, dropped_logs,
                          "[PARALLEL_MOTION] EVENT=TX_FAILURE POWER_OFF\r\n");
        return false;
    }
    if (decision.event != PARALLEL_EVENT_NONE) {
        const char *event =
            decision.event == PARALLEL_EVENT_ARMED ? "ARMED" :
            decision.event == PARALLEL_EVENT_ENABLE_WAIT ? "ENABLE_WAIT" :
            decision.event == PARALLEL_EVENT_RUNNING ? "RUNNING" :
            decision.event == PARALLEL_EVENT_ZERO_HOLD ? "ZERO_HOLD" :
            decision.event == PARALLEL_EVENT_DISABLE_WAIT ? "DISABLE_WAIT" :
            decision.event == PARALLEL_EVENT_COMPLETE ? "COMPLETE" :
            decision.event == PARALLEL_EVENT_REJECTED ? "REJECTED" :
            decision.event == PARALLEL_EVENT_SAFETY_TRIP ? "SAFETY_TRIP" :
            decision.event == PARALLEL_EVENT_ARM_TIMEOUT ? "ARM_TIMEOUT" :
            "EVENT";

        (void)snprintf(record, sizeof(record),
                       "[PARALLEL_MOTION] EVENT=%s DIRECTION=%d%s%s\r\n",
                       event, decision.direction,
                       decision.reason != NULL ? " REASON=" : "",
                       decision.reason != NULL ? decision.reason : "");
        (void)enqueue_log(queue, dropped_logs, record);
    }
    return true;
}

static void enqueue_motion_log(MotionLogQueue *queue, uint32_t *dropped_logs,
                               MotorRole role, const char *event,
                               const char *reason, int32_t target)
{
    char value[16];
    char record[192];

    format_milli(value, sizeof(value), target);
    (void)snprintf(record, sizeof(record),
                   "[%s_MOTION] EVENT=%s TARGET=%s%s%s\r\n",
                   motor_role_name(role), event, value,
                   reason != NULL ? " REASON=" : "",
                   reason != NULL ? reason : "");
    (void)enqueue_log(queue, dropped_logs, record);
}

static bool execute_joint_decision(
    Dm4310MotionDecision decision, MotorRole role,
    MotorManager *manager, PowerQuietController *power,
    Dm4310Controller *joint_motion, MotionController *wheel_motion,
    bool *probe_active, MotionLogQueue *queue, uint32_t *dropped_logs)
{
    Dm4310CanFrame frame;
    bool success = true;

    if (decision.send_enable)
        success = dm4310_build_enable_command_for(motor_role_can_id(role), &frame) &&
                  motor_manager_transmit(manager, role, frame.id, frame.data,
                                         frame.dlc);
    if (success && decision.send_mit)
        success = dm4310_build_mit_command_for(
                      motor_role_can_id(role), 0, decision.target_velocity_millirad_s,
                      0, MIT_KD_MILLI, 0, &frame) &&
                  motor_manager_transmit(manager, role, frame.id, frame.data,
                                         frame.dlc);
    if (success && decision.send_disable)
        success = dm4310_build_disable_command_for(motor_role_can_id(role), &frame) &&
                  motor_manager_transmit(manager, role, frame.id, frame.data,
                                         frame.dlc);
    if (!success || decision.cut_power) {
        (void)motor_manager_send_disable_all(manager);
        power_off(power, probe_active, joint_motion, wheel_motion);
    }
    if (!success) {
        enqueue_motion_log(queue, dropped_logs, role, "TX_FAILURE",
                           "CAN_TX_FAILED", 0);
        return false;
    }
    if (decision.event != DM4310_EVENT_NONE) {
        const char *event =
            decision.event == DM4310_EVENT_ARMED ? "ARMED" :
            decision.event == DM4310_EVENT_ENABLE_REQUESTED ? "ENABLE_WAIT" :
            decision.event == DM4310_EVENT_RUNNING ? "RUNNING" :
            decision.event == DM4310_EVENT_ZERO_HOLD ? "ZERO_HOLD" :
            decision.event == DM4310_EVENT_DISABLE_REQUESTED ? "DISABLE_WAIT" :
            decision.event == DM4310_EVENT_COMPLETE ? "COMPLETE" :
            decision.event == DM4310_EVENT_ARM_TIMEOUT ? "ARM_TIMEOUT" :
            decision.event == DM4310_EVENT_REJECTED ? "REJECTED" :
            decision.event == DM4310_EVENT_SAFETY_TRIP ? "SAFETY_TRIP" :
            "EVENT";
        enqueue_motion_log(queue, dropped_logs, role, event,
                           decision.reason,
                           decision.target_velocity_millirad_s);
    }
    return true;
}

static bool execute_wheel_decision(
    MotionDecision decision, MotorManager *manager,
    PowerQuietController *power, Dm4310Controller *joint_motion,
    MotionController *wheel_motion, bool *probe_active,
    MotionLogQueue *queue, uint32_t *dropped_logs)
{
    H6215CanFrame frame;
    bool success = true;

    if (decision.action == MOTION_ACTION_ENABLE)
        success = h6215_build_enable_command(&frame);
    else if (decision.action == MOTION_ACTION_VELOCITY)
        success = h6215_build_velocity_step(decision.velocity_step, &frame);
    else if (decision.action == MOTION_ACTION_DISABLE)
        success = h6215_build_disable_command(&frame);
    if (success && decision.action != MOTION_ACTION_NONE)
        success = motor_manager_transmit(manager, MOTOR_ROLE_WHEEL, frame.id,
                                         frame.data, frame.dlc);
    if (!success || decision.event == MOTION_EVENT_SAFETY_TRIP ||
        decision.event == MOTION_EVENT_TX_FAILURE) {
        (void)motor_manager_send_disable_all(manager);
        power_off(power, probe_active, joint_motion, wheel_motion);
    }
    if (decision.event != MOTION_EVENT_NONE) {
        const char *event =
            decision.event == MOTION_EVENT_ARMED ? "ARMED" :
            decision.event == MOTION_EVENT_START_REQUESTED ? "ENABLE_WAIT" :
            decision.event == MOTION_EVENT_RUNNING ? "RUNNING" :
            decision.event == MOTION_EVENT_ZERO_HOLD ? "ZERO_HOLD" :
            decision.event == MOTION_EVENT_DISABLE_REQUESTED ? "DISABLE_WAIT" :
            decision.event == MOTION_EVENT_COMPLETE ? "COMPLETE" :
            decision.event == MOTION_EVENT_ARM_TIMEOUT ? "ARM_TIMEOUT" :
            decision.event == MOTION_EVENT_START_REJECTED ? "REJECTED" :
            decision.event == MOTION_EVENT_SAFETY_TRIP ? "SAFETY_TRIP" :
            "EVENT";
        enqueue_motion_log(queue, dropped_logs, MOTOR_ROLE_WHEEL, event,
                           decision.reason,
                           (int32_t)decision.velocity_step * 100);
    }
    return success;
}

int main(void)
{
    PowerQuietController power;
    Dm4310Controller joint_motion;
    MotionController wheel_motion;
    Phase1Monitor monitor;
    MotorManager motor_manager;
    bool can1_ok;
    bool boot_queued = false;
    bool probe_active = false;
    bool parallel_selected = false;
    MotorRole selected = MOTOR_ROLE_COUNT;
    uint32_t next_telemetry_ms = BOOT_LOG_DELAY_MS + TELEMETRY_MS;
    uint32_t next_feedback_ms = 0u;
    uint32_t next_parameter_ms = 0u;
    uint32_t dropped_logs = 0u;
    uint32_t last_loop_ms = 0u;
    uint8_t feedback_role = 0u;
    uint8_t parameter_role = 0u;
    uint8_t parameter_index = 0u;

    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    board_motor_power_init();
    board_clock_init();
    MX_GPIO_Init();
    can1_ok = can_bus_init();
    power_quiet_controller_init(&power);
    dm4310_controller_init(&joint_motion);
    motion_controller_init(&wheel_motion);
    parallel_controller_init(&parallel_motion);
    motor_manager_init(&motor_manager);
    serial_logger_init(&serial_logger);
    usb_command_queue_init();
    MX_USB_DEVICE_Init();
    phase1_monitor_init(&monitor);

    while (1) {
        Can1Status can_status = {0};
        bool can_status_valid;
        uint32_t command_count;
        uint32_t now_ms = HAL_GetTick();
        uint32_t loop_period_ms = (uint32_t)(now_ms - last_loop_ms);
        const char *feedback_fault;

        if (loop_period_ms == 0u) loop_period_ms = 1u;
        last_loop_ms = now_ms;
        (void)phase1_monitor_step(&monitor, now_ms);
        motor_manager_receive(&motor_manager, now_ms, loop_period_ms);
        motor_manager_update_online(&motor_manager, now_ms);
        can_status_valid = can1_ok && can_bus_status(&can_status);

        if (usb_command_queue_take_emergency_stop())
            emergency_stop(&motor_manager, &power, &probe_active, &joint_motion,
                           &wheel_motion, &serial_logger.queue, &dropped_logs);

        for (command_count = 0u; command_count < USB_COMMANDS_PER_LOOP;
             ++command_count) {
            uint8_t command;

            if (!usb_command_queue_pop(&command)) break;
            if (command >= (uint8_t)'a' && command <= (uint8_t)'z')
                command = (uint8_t)(command - ((uint8_t)'a' - (uint8_t)'A'));
            if (command == (uint8_t)'\r' || command == (uint8_t)'\n') continue;
            if (command == (uint8_t)'X') {
                emergency_stop(&motor_manager, &power, &probe_active,
                               &joint_motion,
                               &wheel_motion, &serial_logger.queue, &dropped_logs);
                selected = MOTOR_ROLE_COUNT;
                parallel_selected = false;
                continue;
            }
            if (command == (uint8_t)'S') {
                enqueue_status(&serial_logger.queue, &dropped_logs, &power,
                               &motor_manager, probe_active, &can_status,
                               can_status_valid, now_ms, true);
                continue;
            }
            if (power.state != POWER_QUIET_ON) {
                PowerQuietDecision decision =
                    power_quiet_controller_command(&power, command, now_ms);

                if (decision.set_output)
                    board_motor_power_set(decision.output_on);
                if (decision.event == POWER_QUIET_EVENT_ARMED)
                    (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                      "POWER_ARMED EXPIRES_MS=10000\r\n");
                else if (decision.event == POWER_QUIET_EVENT_ON)
                    (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                      "POWER_ON_REQUESTED MODE=QUIET\r\n");
                else if (decision.event == POWER_QUIET_EVENT_REJECTED) {
                    char record[128];

                    (void)snprintf(record, sizeof(record),
                                   "POWER_REJECTED REASON=%s\r\n",
                                   decision.reason);
                    (void)enqueue_log(&serial_logger.queue, &dropped_logs, record);
                }
                enqueue_power_status(&serial_logger.queue, &dropped_logs, &power,
                                     probe_active);
                continue;
            }
            if (command == (uint8_t)'R') {
                if (any_motion_active(&joint_motion, &wheel_motion)) {
                    (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                      "PROBE_REJECTED REASON=MOTION_ACTIVE\r\n");
                    continue;
                }
                motor_manager_init(&motor_manager);
                if (!motor_manager_send_disable_all(&motor_manager)) {
                    power_off(&power, &probe_active, &joint_motion,
                              &wheel_motion);
                    (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                      "TX_FAILURE DISABLE_ALL POWER_OFF\r\n");
                } else {
                    probe_active = true;
                    feedback_role = 0u;
                    parameter_role = 0u;
                    parameter_index = 0u;
                    next_feedback_ms = now_ms;
                    next_parameter_ms = now_ms;
                    (void)enqueue_log(
                        &serial_logger.queue, &dropped_logs,
                        "PROBE_STARTED DEVICES=3 MOTION=GUARDED\r\n");
                }
                continue;
            }
            if (command >= (uint8_t)'1' && command <= (uint8_t)'4') {
                MotorRole requested = command == (uint8_t)'4'
                                          ? MOTOR_ROLE_COUNT
                                          : (MotorRole)(command -
                                                        (uint8_t)'1');

                if (!probe_active || any_motion_active(&joint_motion,
                                                       &wheel_motion)) {
                    (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                      "SELECT_REJECTED REASON=SEND_R_FIRST_OR_MOTION_ACTIVE\r\n");
                } else {
                    (void)motor_manager_send_disable_all(&motor_manager);
                    dm4310_controller_init(&joint_motion);
                    motion_controller_init(&wheel_motion);
                    parallel_controller_init(&parallel_motion);
                    selected = requested;
                    parallel_selected = command == (uint8_t)'4';
                    {
                        char record[96];

                        if (parallel_selected)
                            (void)snprintf(record, sizeof(record),
                                           "MOTOR_SELECTED ROLE=ALL COUNT=3\r\n");
                        else
                            (void)snprintf(record, sizeof(record),
                                           "MOTOR_SELECTED ROLE=%s ID=%u\r\n",
                                           motor_role_name(selected),
                                           motor_role_can_id(selected));
                        (void)enqueue_log(&serial_logger.queue, &dropped_logs, record);
                    }
                }
                continue;
            }
            if (!parallel_selected && selected == MOTOR_ROLE_COUNT) {
                (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                  "MOTION_REJECTED REASON=SELECT_1_2_3_4_FIRST\r\n");
                continue;
            }
            if (parallel_selected) {
                ParallelSafetySnapshot safety =
                    safety_manager_parallel_snapshot(
                        &motor_manager, &can_status, can_status_valid,
                        probe_active, now_ms);
                ParallelDecision decision = parallel_controller_command(
                    &parallel_motion, command, &safety);

                (void)execute_parallel_decision(
                    decision, &motor_manager, &power, &joint_motion,
                    &wheel_motion,
                    &probe_active, &serial_logger.queue, &dropped_logs);
            } else if (selected == MOTOR_ROLE_WHEEL) {
                MotionSafetySnapshot safety =
                    safety_manager_wheel_snapshot(
                        &motor_manager, &can_status, can_status_valid,
                        now_ms);
                MotionDecision decision;

                if (command == (uint8_t)'N') {
                    (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                      "MOTION_REJECTED REASON=N_NOT_SUPPORTED_FOR_WHEEL\r\n");
                    continue;
                }
                decision = motion_controller_command(&wheel_motion, command,
                                                     &safety);
                (void)execute_wheel_decision(
                    decision, &motor_manager, &power, &joint_motion,
                    &wheel_motion,
                    &probe_active, &serial_logger.queue, &dropped_logs);
            } else {
                Dm4310SafetySnapshot safety =
                    safety_manager_joint_snapshot(
                        &motor_manager, selected, &can_status,
                        can_status_valid, probe_active, now_ms);
                Dm4310MotionDecision decision = dm4310_controller_command(
                    &joint_motion, command, &safety);

                (void)execute_joint_decision(
                    decision, selected, &motor_manager, &power, &joint_motion,
                    &wheel_motion, &probe_active, &serial_logger.queue, &dropped_logs);
            }
        }

        if (power.state != POWER_QUIET_ON) {
            PowerQuietDecision decision =
                power_quiet_controller_step(&power, now_ms);

            if (decision.event == POWER_QUIET_EVENT_ARM_TIMEOUT)
                (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                  "POWER_ARM_TIMEOUT\r\n");
        }

        if (probe_active && selected != MOTOR_ROLE_COUNT) {
            if (selected == MOTOR_ROLE_WHEEL) {
                MotionSafetySnapshot safety =
                    safety_manager_wheel_snapshot(
                        &motor_manager, &can_status, can_status_valid,
                        now_ms);
                MotionDecision decision = motion_controller_step(
                    &wheel_motion, &safety);

                (void)execute_wheel_decision(
                    decision, &motor_manager, &power, &joint_motion,
                    &wheel_motion,
                    &probe_active, &serial_logger.queue, &dropped_logs);
            } else {
                Dm4310SafetySnapshot safety =
                    safety_manager_joint_snapshot(
                        &motor_manager, selected, &can_status,
                        can_status_valid, probe_active, now_ms);
                Dm4310MotionDecision decision = dm4310_controller_step(
                    &joint_motion, &safety);

                (void)execute_joint_decision(
                    decision, selected, &motor_manager, &power, &joint_motion,
                    &wheel_motion, &probe_active, &serial_logger.queue, &dropped_logs);
            }
        }

        if (probe_active && parallel_selected) {
            ParallelSafetySnapshot safety =
                safety_manager_parallel_snapshot(
                    &motor_manager, &can_status, can_status_valid,
                    probe_active, now_ms);
            ParallelDecision decision = parallel_controller_step(
                &parallel_motion, &safety);

            (void)execute_parallel_decision(
                decision, &motor_manager, &power, &joint_motion, &wheel_motion,
                &probe_active, &serial_logger.queue, &dropped_logs);
        }

        if (probe_active && board_motor_power_is_enabled()) {
            if ((int32_t)(now_ms - next_feedback_ms) >= 0) {
                next_feedback_ms = now_ms + FEEDBACK_SLOT_MS;
                if (!(parallel_controller_active(&parallel_motion) ||
                      ((MotorRole)feedback_role == selected &&
                       any_motion_active(&joint_motion, &wheel_motion))) &&
                    !motor_manager_send_feedback_request(
                        &motor_manager, (MotorRole)feedback_role)) {
                    power_off(&power, &probe_active, &joint_motion,
                              &wheel_motion);
                    (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                      "TX_FAILURE FEEDBACK_POLL POWER_OFF\r\n");
                }
                feedback_role = (uint8_t)((feedback_role + 1u) % MOTOR_ROLE_COUNT);
            }
            if (probe_active && (int32_t)(now_ms - next_parameter_ms) >= 0) {
                next_parameter_ms = now_ms + PARAMETER_SLOT_MS;
                if (!(parallel_controller_active(&parallel_motion) ||
                      ((MotorRole)parameter_role == selected &&
                       any_motion_active(&joint_motion, &wheel_motion))) &&
                    !motor_manager_send_parameter_request(
                        &motor_manager, (MotorRole)parameter_role,
                        parameter_registers[parameter_index])) {
                    power_off(&power, &probe_active, &joint_motion,
                              &wheel_motion);
                    (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                                      "TX_FAILURE PARAMETER_POLL POWER_OFF\r\n");
                }
                parameter_role = (uint8_t)(parameter_role + 1u);
                if (parameter_role >= MOTOR_ROLE_COUNT) {
                    parameter_role = 0u;
                    parameter_index = (uint8_t)(
                        (parameter_index + 1u) %
                        (sizeof(parameter_registers) /
                         sizeof(parameter_registers[0])));
                }
            }
        }

        if (probe_active &&
            (!can_status_valid || can_status.error_passive ||
             can_status.bus_off)) {
            (void)motor_manager_send_disable_all(&motor_manager);
            power_off(&power, &probe_active, &joint_motion, &wheel_motion);
            (void)enqueue_log(&serial_logger.queue, &dropped_logs,
                              "SAFETY_TRIP CAN_NOT_ACTIVE POWER_OFF\r\n");
        }
        feedback_fault = safety_manager_global_fault(
            &motor_manager, selected,
            any_motion_active(&joint_motion, &wheel_motion),
            parallel_controller_active(&parallel_motion), now_ms);
        if (probe_active && feedback_fault != NULL) {
            char record[128];

            (void)motor_manager_send_disable_all(&motor_manager);
            power_off(&power, &probe_active, &joint_motion, &wheel_motion);
            (void)snprintf(record, sizeof(record),
                           "SAFETY_TRIP %s POWER_OFF\r\n", feedback_fault);
            (void)enqueue_log(&serial_logger.queue, &dropped_logs, record);
        }

        if (!boot_queued && now_ms >= BOOT_LOG_DELAY_MS) {
            const char *banner = can1_ok
                ? "MC02_BOOT\r\nPHASE35_THREE_MOTOR_PARALLEL\r\n"
                  "CAN1_INIT_OK BITRATE=1000000 TX=GUARDED\r\n"
                  "JOINT_A ID=6 MST_ID=3\r\n"
                  "JOINT_B ID=8 MST_ID=4\r\n"
                  "WHEEL ID=1 MST_ID=0\r\n"
                  "COMMANDS=S,A,P,R,1,2,3,4,N,G,B,X MODE4=PARALLEL\r\n"
                  "DEFAULT_POWER=OFF DEFAULT_MOTORS=DISABLED\r\n"
                : "MC02_BOOT\r\nPHASE35_THREE_MOTOR_PARALLEL\r\n"
                  "CAN1_INIT_ERROR\r\nDEFAULT_POWER=OFF\r\n";

            boot_queued = enqueue_log(&serial_logger.queue, &dropped_logs, banner);
        }

        if ((int32_t)(now_ms - next_telemetry_ms) >= 0) {
            next_telemetry_ms = now_ms + TELEMETRY_MS;
            enqueue_status(&serial_logger.queue, &dropped_logs, &power,
                           &motor_manager, probe_active, &can_status,
                           can_status_valid, now_ms, false);
        }
        service_log_queue(&serial_logger.queue, &dropped_logs);
    }
}

void Error_Handler(void)
{
    board_motor_power_set(false);
    __disable_irq();
    while (1) {
    }
}
