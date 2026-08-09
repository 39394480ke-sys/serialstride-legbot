#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "dm4310_controller.h"
#include "dm4310_protocol.h"
#include "feedback_timing.h"
#include "gpio.h"
#include "main.h"
#include "motion_io.h"
#include "phase1_monitor.h"
#include "power_quiet_controller.h"
#include "usb_command_queue.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define BOOT_LOG_DELAY_MS 1500u
#define IDLE_TELEMETRY_MS 1000u
#define ACTIVE_TELEMETRY_MS 100u
#define FEEDBACK_POLL_MS 50u
#define PARAMETER_POLL_MS 100u
#define ONLINE_TIMEOUT_MS 500u
#define USB_COMMANDS_PER_LOOP 32u
#define PARAMETER_MASK_COMPLETE 0x1fu

typedef struct {
    uint32_t last_any_rx_ms;
    uint32_t last_feedback_ms;
    uint32_t rx_count;
    uint32_t tx_ok;
    uint32_t tx_failed;
    uint8_t parameter_mask;
    uint8_t control_mode;
    int32_t p_max_milli;
    int32_t v_max_milli;
    int32_t t_max_milli;
    char software_version[5];
    Dm4310Feedback feedback;
    bool feedback_valid;
} JointStatus;

static const uint8_t parameter_registers[] = {
    DM4310_REGISTER_SOFTWARE_VERSION,
    DM4310_REGISTER_CONTROL_MODE,
    DM4310_REGISTER_P_MAX,
    DM4310_REGISTER_V_MAX,
    DM4310_REGISTER_T_MAX,
};

static bool enqueue_log(MotionLogQueue *queue, uint32_t *dropped_logs,
                        const char *record)
{
    size_t length = strlen(record);

    if (!motion_log_queue_push(queue, record, length)) {
        (*dropped_logs)++;
        return false;
    }
    return true;
}

static void service_log_queue(MotionLogQueue *queue, uint32_t *dropped_logs)
{
    static uint8_t tx_buffers[2][MOTION_LOG_RECORD_CAPACITY];
    static uint8_t tx_buffer_index;
    const uint8_t *record;
    uint16_t length;
    bool accepted;

    if (!motion_log_queue_peek(queue, &record, &length)) return;
    memcpy(tx_buffers[tx_buffer_index], record, length);
    accepted = CDC_Transmit_HS(tx_buffers[tx_buffer_index], length) == USBD_OK;
    motion_log_queue_finish_attempt(queue, accepted, dropped_logs);
    if (accepted) tx_buffer_index ^= 1u;
}

static int32_t float_to_milli(float value)
{
    float scaled = value * 1000.0f;

    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void format_milli(char *buffer, size_t capacity, int32_t value)
{
    uint32_t magnitude = value < 0 ? (uint32_t)(-value) : (uint32_t)value;

    (void)snprintf(buffer, capacity, "%s%lu.%03lu", value < 0 ? "-" : "",
                   (unsigned long)(magnitude / 1000u),
                   (unsigned long)(magnitude % 1000u));
}

static bool decode_version(uint32_t raw_value, char version[5])
{
    uint8_t index;

    for (index = 0u; index < 4u; ++index) {
        uint8_t character = (uint8_t)(raw_value >> (index * 8u));

        if (character < 0x20u || character > 0x7eu) {
            version[0] = '\0';
            return false;
        }
        version[index] = (char)character;
    }
    version[4] = '\0';
    return true;
}

static void update_parameter(JointStatus *joint,
                             const Dm4310ParameterResponse *response)
{
    switch (response->register_id) {
    case DM4310_REGISTER_SOFTWARE_VERSION:
        if (decode_version(response->raw_value, joint->software_version))
            joint->parameter_mask |= 1u << 0;
        break;
    case DM4310_REGISTER_CONTROL_MODE:
        joint->control_mode = (uint8_t)response->raw_value;
        joint->parameter_mask |= 1u << 1;
        break;
    case DM4310_REGISTER_P_MAX:
        joint->p_max_milli = float_to_milli(response->float_value);
        joint->parameter_mask |= 1u << 2;
        break;
    case DM4310_REGISTER_V_MAX:
        joint->v_max_milli = float_to_milli(response->float_value);
        joint->parameter_mask |= 1u << 3;
        break;
    case DM4310_REGISTER_T_MAX:
        joint->t_max_milli = float_to_milli(response->float_value);
        joint->parameter_mask |= 1u << 4;
        break;
    default:
        break;
    }
}

static void receive_frames(JointStatus *joint, uint32_t now_ms,
                           uint32_t loop_period_ms)
{
    Dm4310CanFrame frame;
    uint32_t age_us;

    while (board_can1_receive(&frame.id, frame.data, &frame.dlc, &age_us)) {
        Dm4310ParameterResponse parameter;
        Dm4310Feedback feedback;

        if (dm4310_parse_parameter_response(&frame, &parameter)) {
            update_parameter(joint, &parameter);
            joint->last_any_rx_ms = now_ms;
            joint->rx_count++;
        } else if (dm4310_parse_feedback(&frame, &feedback)) {
            joint->feedback = feedback;
            joint->feedback_valid = true;
            joint->last_any_rx_ms = now_ms;
            joint->last_feedback_ms = feedback_received_at_ms(
                now_ms, age_us, loop_period_ms);
            joint->rx_count++;
        }
    }
}

static bool parameters_valid(const JointStatus *joint)
{
    return joint->parameter_mask == PARAMETER_MASK_COMPLETE &&
           joint->control_mode == 1u &&
           joint->p_max_milli == DM4310_P_MAX_MILLIRAD &&
           joint->v_max_milli == DM4310_V_MAX_MILLIRAD_S &&
           joint->t_max_milli == DM4310_T_MAX_MILLINEWTON_M;
}

static bool transmit(JointStatus *joint, const Dm4310CanFrame *frame)
{
    if (!board_can1_transmit(frame->id, frame->data, frame->dlc)) {
        joint->tx_failed++;
        return false;
    }
    joint->tx_ok++;
    return true;
}

static void enqueue_power_status(MotionLogQueue *queue,
                                 uint32_t *dropped_logs,
                                 const PowerQuietController *power,
                                 bool active)
{
    char record[160];

    (void)snprintf(record, sizeof(record),
                   "[POWER] VCC_OUT1=%s MODE=%s STATE=%s\r\n",
                   board_motor_power_is_enabled() ? "ON" : "OFF",
                   active ? "ACTIVE" : "QUIET",
                   power_quiet_state_name(power->state));
    (void)enqueue_log(queue, dropped_logs, record);
}

static void enqueue_joint_status(MotionLogQueue *queue,
                                 uint32_t *dropped_logs,
                                 const JointStatus *joint,
                                 const Dm4310Controller *motion,
                                 bool probe_active, bool status_requested,
                                 uint32_t now_ms)
{
    char record[MOTION_LOG_RECORD_CAPACITY];
    char position[16];
    char velocity[16];
    char torque[16];
    char p_max[16];
    char v_max[16];
    char t_max[16];
    char feedback_age[16];
    bool online = joint->feedback_valid &&
                  (uint32_t)(now_ms - joint->last_feedback_ms) <=
                      ONLINE_TIMEOUT_MS;

    format_milli(position, sizeof(position), joint->feedback.position_millirad);
    format_milli(velocity, sizeof(velocity),
                 joint->feedback.velocity_millirad_s);
    format_milli(torque, sizeof(torque),
                 joint->feedback.torque_millinewton_m);
    format_milli(p_max, sizeof(p_max), joint->p_max_milli);
    format_milli(v_max, sizeof(v_max), joint->v_max_milli);
    format_milli(t_max, sizeof(t_max), joint->t_max_milli);
    if (joint->feedback_valid)
        (void)snprintf(feedback_age, sizeof(feedback_age), "%lu",
                       (unsigned long)(now_ms - joint->last_feedback_ms));
    else
        (void)snprintf(feedback_age, sizeof(feedback_age), "NA");

    (void)snprintf(
        record, sizeof(record),
        "%s[POWER] VCC_OUT1=%s MODE=%s\r\n"
        "[JOINT_A] ONLINE=%u ID=6 MST_ID=3 STATE=%s FB_AGE_MS=%s "
        "SW=%s MODE=%u P_MAX=%s V_MAX=%s T_MAX=%s PARAM_MASK=0x%02X "
        "PARAM_OK=%u PROBE=%u RX=%lu P=%s V=%s T=%s TMOS=%u "
        "TROTOR=%u TX_OK=%lu TX_FAIL=%lu\r\n"
        "[JOINT_A_MOTION] STATE=%s TARGET=%ld.%03ld\r\n",
        status_requested ? "STATUS_REQUESTED\r\n" : "",
        board_motor_power_is_enabled() ? "ON" : "OFF",
        probe_active ? "ACTIVE" : "QUIET", online,
        joint->feedback_valid ? dm4310_state_name(joint->feedback.state)
                              : "NO_FEEDBACK",
        feedback_age,
        joint->software_version[0] != '\0' ? joint->software_version : "UNKNOWN",
        joint->control_mode, p_max, v_max, t_max, joint->parameter_mask,
        parameters_valid(joint), probe_active, (unsigned long)joint->rx_count,
        joint->feedback_valid ? position : "NA",
        joint->feedback_valid ? velocity : "NA",
        joint->feedback_valid ? torque : "NA",
        joint->feedback_valid ? joint->feedback.mos_temperature_c : 0u,
        joint->feedback_valid ? joint->feedback.rotor_temperature_c : 0u,
        (unsigned long)joint->tx_ok, (unsigned long)joint->tx_failed,
        dm4310_motion_state_name(motion->state),
        (long)(motion->target_velocity_millirad_s / 1000),
        (long)(motion->target_velocity_millirad_s < 0
                   ? -(motion->target_velocity_millirad_s % 1000)
                   : motion->target_velocity_millirad_s % 1000));
    (void)enqueue_log(queue, dropped_logs, record);
}

static Dm4310SafetySnapshot safety_snapshot(
    const JointStatus *joint, const Can1Status *can_status,
    bool can_status_valid, bool probe_active, uint32_t now_ms)
{
    Dm4310SafetySnapshot safety = {
        .now_ms = now_ms,
        .feedback_age_ms = joint->feedback_valid
                               ? (uint32_t)(now_ms - joint->last_feedback_ms)
                               : UINT32_MAX,
        .motor_state = joint->feedback.state,
        .velocity_millirad_s = joint->feedback.velocity_millirad_s,
        .torque_millinewton_m = joint->feedback.torque_millinewton_m,
        .mos_temperature_c = joint->feedback.mos_temperature_c,
        .rotor_temperature_c = joint->feedback.rotor_temperature_c,
        .powered = board_motor_power_is_enabled(),
        .probe_active = probe_active,
        .parameters_valid = parameters_valid(joint),
        .feedback_valid = joint->feedback_valid,
        .can_warning = !can_status_valid || can_status->warning,
        .can_passive = !can_status_valid || can_status->error_passive,
        .can_bus_off = !can_status_valid || can_status->bus_off,
    };

    return safety;
}

static void reset_after_power_cut(PowerQuietController *power,
                                  Dm4310Controller *motion,
                                  bool *probe_active)
{
    board_motor_power_set(false);
    power_quiet_controller_init(power);
    dm4310_controller_init(motion);
    *probe_active = false;
}

static bool send_zero_and_disable(JointStatus *joint)
{
    Dm4310CanFrame frame;
    bool success = dm4310_build_mit_command(0, 0, 0, 500, 0, &frame) &&
                   transmit(joint, &frame);

    if (!dm4310_build_disable_command(&frame) || !transmit(joint, &frame))
        success = false;
    return success;
}

static void enqueue_motion_event(MotionLogQueue *queue,
                                 uint32_t *dropped_logs,
                                 const Dm4310MotionDecision *decision)
{
    char record[192];
    const char *format = NULL;

    switch (decision->event) {
    case DM4310_EVENT_STATUS: format = "STATUS_REQUESTED\r\n"; break;
    case DM4310_EVENT_ARMED: format = "MOTION_ARMED EXPIRES_MS=10000\r\n"; break;
    case DM4310_EVENT_ARM_TIMEOUT: format = "ARM_TIMEOUT\r\n"; break;
    case DM4310_EVENT_ENABLE_REQUESTED:
        (void)snprintf(record, sizeof(record),
                       "[JOINT_A_MOTION] STATE=ENABLE_WAIT TARGET=%ld.%03ld\r\n",
                       (long)(decision->target_velocity_millirad_s / 1000),
                       (long)(decision->target_velocity_millirad_s < 0
                                  ? -(decision->target_velocity_millirad_s % 1000)
                                  : decision->target_velocity_millirad_s % 1000));
        format = record;
        break;
    case DM4310_EVENT_RUNNING:
        (void)snprintf(record, sizeof(record),
                       "[JOINT_A_MOTION] STATE=PULSE TARGET=%ld.%03ld DURATION_MS=500\r\n",
                       (long)(decision->target_velocity_millirad_s / 1000),
                       (long)(decision->target_velocity_millirad_s < 0
                                  ? -(decision->target_velocity_millirad_s % 1000)
                                  : decision->target_velocity_millirad_s % 1000));
        format = record;
        break;
    case DM4310_EVENT_ZERO_HOLD:
        format = "[JOINT_A_MOTION] STATE=ZERO_HOLD TARGET=0.000 DURATION_MS=200\r\n";
        break;
    case DM4310_EVENT_COMPLETE:
        format = "[JOINT_A_MOTION] STATE=DISABLED TARGET=0.000 COMPLETE=1\r\n";
        break;
    case DM4310_EVENT_EMERGENCY_STOP:
        format = "EMERGENCY_STOP_REQUESTED POWER_OFF\r\n";
        break;
    case DM4310_EVENT_REJECTED:
        (void)snprintf(record, sizeof(record), "START_REJECTED REASON=%s\r\n",
                       decision->reason != NULL ? decision->reason : "UNKNOWN");
        format = record;
        break;
    case DM4310_EVENT_SAFETY_TRIP:
        (void)snprintf(record, sizeof(record), "SAFETY_TRIP REASON=%s POWER_OFF\r\n",
                       decision->reason != NULL ? decision->reason : "UNKNOWN");
        format = record;
        break;
    case DM4310_EVENT_TX_FAILURE:
        format = "TX_FAILURE REASON=CAN_TX_FAILED POWER_OFF\r\n";
        break;
    case DM4310_EVENT_NONE:
    default:
        break;
    }
    if (format != NULL) (void)enqueue_log(queue, dropped_logs, format);
}

static bool execute_motion_decision(
    Dm4310MotionDecision decision, JointStatus *joint,
    PowerQuietController *power, Dm4310Controller *motion,
    bool *probe_active, MotionLogQueue *queue, uint32_t *dropped_logs)
{
    Dm4310CanFrame frame;
    bool success = true;

    if (decision.send_enable)
        success = dm4310_build_enable_command(&frame) && transmit(joint, &frame);
    if (success && decision.send_mit)
        success = dm4310_build_mit_command(
                      0, decision.target_velocity_millirad_s, 0, 500, 0,
                      &frame) &&
                  transmit(joint, &frame);
    if (success && decision.send_disable)
        success = dm4310_build_disable_command(&frame) && transmit(joint, &frame);

    if (!success) {
        Dm4310MotionDecision failure = dm4310_controller_tx_failed(motion);

        (void)send_zero_and_disable(joint);
        reset_after_power_cut(power, motion, probe_active);
        enqueue_motion_event(queue, dropped_logs, &failure);
        return false;
    }
    enqueue_motion_event(queue, dropped_logs, &decision);
    if (decision.cut_power) {
        reset_after_power_cut(power, motion, probe_active);
        enqueue_power_status(queue, dropped_logs, power, false);
    }
    return true;
}

static void emergency_stop(JointStatus *joint, PowerQuietController *power,
                           Dm4310Controller *motion, bool *probe_active,
                           MotionLogQueue *queue, uint32_t *dropped_logs)
{
    Dm4310MotionDecision stop = {0};

    stop.event = DM4310_EVENT_EMERGENCY_STOP;
    stop.reason = "EMERGENCY_STOP";
    stop.send_mit = board_motor_power_is_enabled();
    stop.send_disable = board_motor_power_is_enabled();
    stop.cut_power = true;
    (void)execute_motion_decision(stop, joint, power, motion, probe_active,
                                  queue, dropped_logs);
}

int main(void)
{
    static MotionLogQueue log_queue;
    PowerQuietController power;
    Dm4310Controller motion;
    Phase1Monitor monitor;
    JointStatus joint = {0};
    bool can1_ok;
    bool boot_queued = false;
    bool probe_active = false;
    uint32_t next_telemetry_ms = BOOT_LOG_DELAY_MS + IDLE_TELEMETRY_MS;
    uint32_t next_feedback_poll_ms = 0u;
    uint32_t next_parameter_poll_ms = 0u;
    uint32_t dropped_logs = 0u;
    uint32_t last_loop_ms = 0u;
    size_t parameter_index = 0u;

    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    board_motor_power_init();
    board_clock_init();
    MX_GPIO_Init();
    can1_ok = board_can1_init();
    power_quiet_controller_init(&power);
    dm4310_controller_init(&motion);
    motion_log_queue_init(&log_queue);
    usb_command_queue_init();
    MX_USB_DEVICE_Init();
    phase1_monitor_init(&monitor);

    while (1) {
        Can1Status can_status = {0};
        Dm4310SafetySnapshot safety;
        Dm4310MotionDecision motion_decision;
        bool can_status_valid;
        uint32_t command_count;
        uint32_t now_ms = HAL_GetTick();
        uint32_t loop_period_ms = (uint32_t)(now_ms - last_loop_ms);

        if (loop_period_ms == 0u) loop_period_ms = 1u;
        last_loop_ms = now_ms;
        (void)phase1_monitor_step(&monitor, now_ms);

        receive_frames(&joint, now_ms, loop_period_ms);
        can_status_valid = can1_ok && board_can1_get_status(&can_status);
        safety = safety_snapshot(&joint, &can_status, can_status_valid,
                                 probe_active, now_ms);

        if (usb_command_queue_take_emergency_stop())
            emergency_stop(&joint, &power, &motion, &probe_active, &log_queue,
                           &dropped_logs);

        for (command_count = 0u; command_count < USB_COMMANDS_PER_LOOP;
             ++command_count) {
            uint8_t command;

            if (!usb_command_queue_pop(&command)) break;
            if (command >= (uint8_t)'a' && command <= (uint8_t)'z')
                command = (uint8_t)(command - ((uint8_t)'a' - (uint8_t)'A'));
            if (command == (uint8_t)'\r' || command == (uint8_t)'\n') continue;
            if (command == (uint8_t)'X') {
                emergency_stop(&joint, &power, &motion, &probe_active,
                               &log_queue, &dropped_logs);
                continue;
            }
            if (command == (uint8_t)'S') {
                enqueue_joint_status(&log_queue, &dropped_logs, &joint,
                                     &motion, probe_active, true, now_ms);
                continue;
            }
            if (power.state != POWER_QUIET_ON) {
                PowerQuietDecision power_decision =
                    power_quiet_controller_command(&power, command, now_ms);

                if (power_decision.set_output)
                    board_motor_power_set(power_decision.output_on);
                if (power_decision.event == POWER_QUIET_EVENT_ARMED)
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "POWER_ARMED EXPIRES_MS=10000\r\n");
                else if (power_decision.event == POWER_QUIET_EVENT_ON)
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "POWER_ON_REQUESTED MODE=QUIET\r\n");
                else if (power_decision.event == POWER_QUIET_EVENT_REJECTED) {
                    char record[128];

                    (void)snprintf(record, sizeof(record),
                                   "POWER_REJECTED REASON=%s\r\n",
                                   power_decision.reason);
                    (void)enqueue_log(&log_queue, &dropped_logs, record);
                }
                enqueue_power_status(&log_queue, &dropped_logs, &power,
                                     probe_active);
                continue;
            }
            if (command == (uint8_t)'R') {
                if (motion.state != DM4310_MOTION_DISABLED) {
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "PROBE_REJECTED REASON=MOTION_ACTIVE\r\n");
                } else {
                    probe_active = true;
                    joint = (JointStatus){0};
                    parameter_index = 0u;
                    next_feedback_poll_ms = now_ms;
                    next_parameter_poll_ms = now_ms;
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "READ_ONLY_PROBE_STARTED ID=6 MST_ID=3\r\n");
                }
                continue;
            }

            safety = safety_snapshot(&joint, &can_status, can_status_valid,
                                     probe_active, now_ms);
            motion_decision = dm4310_controller_command(
                &motion, command, &safety);
            (void)execute_motion_decision(
                motion_decision, &joint, &power, &motion, &probe_active,
                &log_queue, &dropped_logs);
        }

        if (power.state != POWER_QUIET_ON) {
            PowerQuietDecision power_decision =
                power_quiet_controller_step(&power, now_ms);

            if (power_decision.event == POWER_QUIET_EVENT_ARM_TIMEOUT) {
                (void)enqueue_log(&log_queue, &dropped_logs,
                                  "POWER_ARM_TIMEOUT\r\n");
                enqueue_power_status(&log_queue, &dropped_logs, &power,
                                     probe_active);
            }
        }

        safety = safety_snapshot(&joint, &can_status, can_status_valid,
                                 probe_active, now_ms);
        motion_decision = dm4310_controller_step(&motion, &safety);
        (void)execute_motion_decision(motion_decision, &joint, &power, &motion,
                                      &probe_active, &log_queue,
                                      &dropped_logs);

        if (probe_active && board_motor_power_is_enabled() &&
            motion.state != DM4310_MOTION_ENABLE_WAIT &&
            motion.state != DM4310_MOTION_PULSE &&
            motion.state != DM4310_MOTION_ZERO_HOLD) {
            Dm4310CanFrame frame;

            if ((int32_t)(now_ms - next_feedback_poll_ms) >= 0) {
                next_feedback_poll_ms = now_ms + FEEDBACK_POLL_MS;
                if (!dm4310_build_feedback_request(&frame) ||
                    !transmit(&joint, &frame)) {
                    Dm4310MotionDecision failure =
                        dm4310_controller_tx_failed(&motion);
                    reset_after_power_cut(&power, &motion, &probe_active);
                    enqueue_motion_event(&log_queue, &dropped_logs, &failure);
                }
            }
            if (probe_active &&
                (int32_t)(now_ms - next_parameter_poll_ms) >= 0) {
                next_parameter_poll_ms = now_ms + PARAMETER_POLL_MS;
                if (!dm4310_build_read_request(
                        parameter_registers[parameter_index], &frame) ||
                    !transmit(&joint, &frame)) {
                    Dm4310MotionDecision failure =
                        dm4310_controller_tx_failed(&motion);
                    reset_after_power_cut(&power, &motion, &probe_active);
                    enqueue_motion_event(&log_queue, &dropped_logs, &failure);
                }
                parameter_index = (parameter_index + 1u) %
                                  (sizeof(parameter_registers) /
                                   sizeof(parameter_registers[0]));
            }
        }

        if (probe_active && can_status_valid &&
            (can_status.error_passive || can_status.bus_off)) {
            Dm4310MotionDecision trip = {0};

            trip.event = DM4310_EVENT_SAFETY_TRIP;
            trip.reason = "CAN_NOT_ACTIVE";
            trip.send_mit = true;
            trip.send_disable = true;
            trip.cut_power = true;
            (void)execute_motion_decision(trip, &joint, &power, &motion,
                                          &probe_active, &log_queue,
                                          &dropped_logs);
        }

        if (probe_active && board_motor_power_is_enabled() &&
            joint.feedback_valid &&
            (uint32_t)(now_ms - joint.last_feedback_ms) <=
                ONLINE_TIMEOUT_MS &&
            (joint.feedback.state > 1u ||
             joint.feedback.mos_temperature_c >= 60u ||
             joint.feedback.rotor_temperature_c >= 60u)) {
            Dm4310MotionDecision trip = {0};

            trip.event = DM4310_EVENT_SAFETY_TRIP;
            trip.reason = joint.feedback.state > 1u ? "STATE_ABNORMAL"
                                                    : "TEMPERATURE_LIMIT";
            trip.send_mit = true;
            trip.send_disable = true;
            trip.cut_power = true;
            (void)execute_motion_decision(trip, &joint, &power, &motion,
                                          &probe_active, &log_queue,
                                          &dropped_logs);
        }

        if (!boot_queued && now_ms >= BOOT_LOG_DELAY_MS) {
            const char *banner = can1_ok
                ? "MC02_BOOT\r\nPHASE31_JOINT_A_GUARDED\r\n"
                  "CAN1_INIT_OK BITRATE=1000000 TX=GUARDED\r\n"
                  "JOINT_A ID=6 MST_ID=3 MODE=MIT COMMANDS=S,A,P,R,N,G,B,X\r\n"
                  "DEFAULT_POWER=OFF DEFAULT_MOTOR=DISABLED\r\n"
                : "MC02_BOOT\r\nPHASE31_JOINT_A_GUARDED\r\n"
                  "CAN1_INIT_ERROR\r\nDEFAULT_POWER=OFF\r\n";

            boot_queued = enqueue_log(&log_queue, &dropped_logs, banner);
            enqueue_power_status(&log_queue, &dropped_logs, &power,
                                 probe_active);
        }

        {
            uint32_t telemetry_period =
                motion.state == DM4310_MOTION_ENABLE_WAIT ||
                        motion.state == DM4310_MOTION_PULSE ||
                        motion.state == DM4310_MOTION_ZERO_HOLD
                    ? ACTIVE_TELEMETRY_MS
                    : IDLE_TELEMETRY_MS;

            if ((int32_t)(now_ms - next_telemetry_ms) >= 0) {
                char health[320];

                next_telemetry_ms = now_ms + telemetry_period;
                enqueue_joint_status(&log_queue, &dropped_logs, &joint,
                                     &motion, probe_active, false, now_ms);
                (void)snprintf(
                    health, sizeof(health),
                    "HEALTH uptime_ms=%lu loops=%lu missed=%lu can1_tec=%u "
                    "can1_rec=%u can1_lec=%u warning=%u passive=%u "
                    "bus_off=%u dropped_logs=%lu dropped_commands=%lu\r\n",
                    (unsigned long)now_ms, (unsigned long)monitor.loop_count,
                    (unsigned long)monitor.missed_ticks,
                    can_status.tx_error_count, can_status.rx_error_count,
                    can_status.last_error_code, can_status.warning,
                    can_status.error_passive, can_status.bus_off,
                    (unsigned long)dropped_logs,
                    (unsigned long)usb_command_queue_dropped());
                (void)enqueue_log(&log_queue, &dropped_logs, health);
            }
        }
        service_log_queue(&log_queue, &dropped_logs);
    }
}

void Error_Handler(void)
{
    board_motor_power_set(false);
    __disable_irq();
    while (1) {
    }
}
