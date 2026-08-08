#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "gpio.h"
#include "h6215_protocol.h"
#include "main.h"
#include "motion_controller.h"
#include "motion_io.h"
#include "phase1_monitor.h"
#include "usb_device.h"
#include "usb_command_queue.h"
#include "usbd_cdc_if.h"

#define BOOT_LOG_DELAY_MS 1500u
#define H6215_POLL_START_MS 2000u
#define H6215_POLL_PERIOD_MS 200u
#define H6215_ONLINE_TIMEOUT_MS 1000u
#define USB_COMMANDS_PER_LOOP 32u

typedef struct {
    uint32_t last_any_rx_ms;
    uint32_t last_feedback_ms;
    uint32_t rx_count;
    uint32_t tx_ok;
    uint32_t tx_failed;
    uint32_t can_recoveries;
    bool disable_probe_sent;
    char software_version[5];
    uint8_t control_mode;
    uint8_t parameter_mask;
    int32_t p_max_milli;
    int32_t v_max_milli;
    int32_t t_max_milli;
    H6215Feedback feedback;
    bool feedback_valid;
} WheelStatus;

static const uint8_t h6215_poll_registers[] = {
    H6215_REGISTER_SOFTWARE_VERSION,
    H6215_REGISTER_CONTROL_MODE,
    H6215_REGISTER_P_MAX,
    H6215_REGISTER_V_MAX,
    H6215_REGISTER_T_MAX,
};

static bool append_log_format(char *record, size_t capacity, size_t *used,
                              const char *format, ...)
{
    va_list arguments;
    int length;

    if (*used >= capacity) {
        return false;
    }
    va_start(arguments, format);
    length = vsnprintf(&record[*used], capacity - *used, format, arguments);
    va_end(arguments);
    if (length <= 0 || length >= (int)(capacity - *used)) {
        return false;
    }
    *used += (size_t)length;
    return true;
}

static bool enqueue_log_record(MotionLogQueue *queue, uint32_t *dropped_logs,
                               const char *record, size_t length)
{
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
    uint8_t result;
    bool accepted;

    if (!motion_log_queue_peek(queue, &record, &length)) {
        return;
    }
    memcpy(tx_buffers[tx_buffer_index], record, length);
    result = CDC_Transmit_HS(tx_buffers[tx_buffer_index], length);
    accepted = result == USBD_OK;
    motion_log_queue_finish_attempt(queue, accepted, dropped_logs);
    if (accepted) {
        tx_buffer_index ^= 1u;
    }
}

static bool enqueue_boot_banner(bool can1_ok, MotionLogQueue *queue,
                                uint32_t *dropped_logs)
{
    static const char banner[] =
        "MC02_BOOT\r\n"
        "CLOCK_OK SYSCLK=480000000 HCLK=240000000\r\n"
        "TIMER_OK TICK_HZ=1000\r\n"
        "CAN1_INIT_OK BITRATE=1000000 TX=GUARDED_MOTION\r\n"
        "H6215_GUARDED_MOTION COMMANDS=S,A,G,X TARGET=+0.200rad/s\r\n"
        "DEFAULT_STATE=DISABLED AUTO_MOTION=OFF\r\n";
    static const char failure_banner[] =
        "MC02_BOOT\r\n"
        "CLOCK_OK SYSCLK=480000000 HCLK=240000000\r\n"
        "TIMER_OK TICK_HZ=1000\r\n"
        "CAN1_INIT_ERROR\r\n"
        "H6215_GUARDED_MOTION COMMANDS=S,A,G,X TARGET=+0.200rad/s\r\n"
        "DEFAULT_STATE=DISABLED AUTO_MOTION=OFF\r\n";
    const char *message = can1_ok ? banner : failure_banner;
    size_t length = can1_ok ? sizeof(banner) - 1u
                            : sizeof(failure_banner) - 1u;

    return enqueue_log_record(queue, dropped_logs, message, length);
}

static int32_t float_to_milli(float value)
{
    float scaled = value * 1000.0f;

    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void format_milli(char *buffer, size_t size, int32_t value)
{
    uint32_t magnitude = value < 0 ? (uint32_t)(-value) : (uint32_t)value;

    snprintf(buffer, size, "%s%lu.%03lu", value < 0 ? "-" : "",
             (unsigned long)(magnitude / 1000u),
             (unsigned long)(magnitude % 1000u));
}

static void update_parameter(WheelStatus *wheel,
                             const H6215ParameterResponse *response)
{
    switch (response->register_id) {
    case H6215_REGISTER_SOFTWARE_VERSION:
        if (h6215_decode_software_version(response->raw_value,
                                          wheel->software_version)) {
            wheel->parameter_mask |= 1u << 0;
        }
        break;
    case H6215_REGISTER_CONTROL_MODE:
        wheel->control_mode = (uint8_t)response->raw_value;
        wheel->parameter_mask |= 1u << 1;
        break;
    case H6215_REGISTER_P_MAX:
        wheel->p_max_milli = float_to_milli(response->float_value);
        wheel->parameter_mask |= 1u << 2;
        break;
    case H6215_REGISTER_V_MAX:
        wheel->v_max_milli = float_to_milli(response->float_value);
        wheel->parameter_mask |= 1u << 3;
        break;
    case H6215_REGISTER_T_MAX:
        wheel->t_max_milli = float_to_milli(response->float_value);
        wheel->parameter_mask |= 1u << 4;
        break;
    default:
        break;
    }
}

static void receive_h6215_frames(WheelStatus *wheel, uint32_t now_ms)
{
    H6215CanFrame frame;

    while (board_can1_receive(&frame.id, frame.data, &frame.dlc)) {
        H6215ParameterResponse parameter;
        H6215Feedback feedback;

        if (h6215_parse_parameter_response(&frame, &parameter)) {
            update_parameter(wheel, &parameter);
            wheel->last_any_rx_ms = now_ms;
            wheel->rx_count++;
        } else if (h6215_parse_feedback(&frame, &feedback)) {
            wheel->feedback = feedback;
            wheel->feedback_valid = true;
            wheel->last_any_rx_ms = now_ms;
            wheel->last_feedback_ms = now_ms;
            wheel->rx_count++;
        }
    }
}

static void poll_h6215(WheelStatus *wheel, uint32_t *next_poll_ms,
                       uint32_t now_ms, size_t *register_index)
{
    H6215CanFrame request;
    const size_t register_count = sizeof(h6215_poll_registers) /
                                  sizeof(h6215_poll_registers[0]);

    if ((int32_t)(now_ms - *next_poll_ms) < 0) {
        return;
    }
    *next_poll_ms += H6215_POLL_PERIOD_MS;
    if (!h6215_build_read_request(h6215_poll_registers[*register_index],
                                  &request)) {
        wheel->tx_failed++;
        return;
    }

    if (board_can1_transmit(request.id, request.data, request.dlc)) {
        wheel->tx_ok++;
    } else {
        wheel->tx_failed++;
    }
    *register_index = (*register_index + 1u) % register_count;
}

static void send_disable_feedback_probe(WheelStatus *wheel)
{
    H6215CanFrame command;

    if (!h6215_build_disable_command(&command)) {
        wheel->tx_failed++;
        return;
    }
    if (board_can1_transmit(command.id, command.data, command.dlc)) {
        wheel->disable_probe_sent = true;
        wheel->tx_ok++;
    } else {
        wheel->tx_failed++;
    }
}

static void recover_can1_if_needed(WheelStatus *wheel,
                                   uint32_t *next_recovery_check_ms,
                                   uint32_t now_ms,
                                   const Can1Status *status,
                                   bool status_valid)
{
    if ((int32_t)(now_ms - *next_recovery_check_ms) < 0) {
        return;
    }
    *next_recovery_check_ms = now_ms + 100u;
    if (status_valid && status->bus_off &&
        board_can1_recover()) {
        wheel->can_recoveries++;
    }
}

static int format_wheel_status(char *buffer, size_t capacity,
                               const WheelStatus *wheel, uint32_t now_ms)
{
    char p_max[16];
    char v_max[16];
    char t_max[16];
    char position[16];
    char velocity[16];
    char torque[16];
    char feedback_age[16];
    bool online = wheel->last_any_rx_ms != 0u &&
                  (uint32_t)(now_ms - wheel->last_any_rx_ms) <=
                      H6215_ONLINE_TIMEOUT_MS;
    int length;

    format_milli(p_max, sizeof(p_max), wheel->p_max_milli);
    format_milli(v_max, sizeof(v_max), wheel->v_max_milli);
    format_milli(t_max, sizeof(t_max), wheel->t_max_milli);
    format_milli(position, sizeof(position), wheel->feedback.position_millirad);
    format_milli(velocity, sizeof(velocity),
                 wheel->feedback.velocity_millirad_s);
    format_milli(torque, sizeof(torque),
                 wheel->feedback.torque_millinewton_m);
    if (wheel->feedback_valid) {
        snprintf(feedback_age, sizeof(feedback_age), "%lu",
                 (unsigned long)(now_ms - wheel->last_feedback_ms));
    } else {
        snprintf(feedback_age, sizeof(feedback_age), "NA");
    }
    length = snprintf(
        buffer, capacity,
        "[WHEEL] ONLINE=%u ID=1 MST_ID=0 STATE=%s FB_AGE_MS=%s SW=%s MODE=%u "
        "P_MAX=%s V_MAX=%s T_MAX=%s PARAM_MASK=0x%02X RX=%lu "
        "P=%s V=%s T=%s TMOS=%u TROTOR=%u DISABLE_PROBE=%u "
        "TX_OK=%lu TX_FAIL=%lu CAN_RECOVERIES=%lu\r\n",
        online, wheel->feedback_valid ? h6215_state_name(wheel->feedback.state)
                                      : "NO_FEEDBACK",
        feedback_age,
        wheel->software_version[0] != '\0' ? wheel->software_version : "UNKNOWN",
        wheel->control_mode, p_max,
        v_max, t_max, wheel->parameter_mask, (unsigned long)wheel->rx_count,
        wheel->feedback_valid ? position : "NA",
        wheel->feedback_valid ? velocity : "NA",
        wheel->feedback_valid ? torque : "NA",
        wheel->feedback_valid ? wheel->feedback.mos_temperature_c : 0u,
        wheel->feedback_valid ? wheel->feedback.rotor_temperature_c : 0u,
        wheel->disable_probe_sent,
        (unsigned long)wheel->tx_ok, (unsigned long)wheel->tx_failed,
        (unsigned long)wheel->can_recoveries);
    return length;
}

static int format_diagnostics(char *buffer, size_t capacity,
                              const Phase1Monitor *monitor, bool can1_ok,
                              const Can1Status *can_status,
                              bool can_status_valid, uint32_t uptime_ms,
                              uint32_t dropped_logs)
{
    int length;

    if (can1_ok && can_status_valid) {
        length = snprintf(buffer, capacity,
                          "HEALTH uptime_ms=%lu loops=%lu missed=%lu period_min_ms=%lu "
                          "period_max_ms=%lu can1_tec=%u can1_rec=%u can1_lec=%u "
                          "warning=%u passive=%u bus_off=%u dropped_logs=%lu\r\n",
                          (unsigned long)uptime_ms,
                          (unsigned long)monitor->loop_count,
                          (unsigned long)monitor->missed_ticks,
                          (unsigned long)(monitor->min_period_ms == UINT_MAX
                                              ? 0u : monitor->min_period_ms),
                          (unsigned long)monitor->max_period_ms,
                          can_status->tx_error_count,
                          can_status->rx_error_count,
                          can_status->last_error_code, can_status->warning,
                          can_status->error_passive, can_status->bus_off,
                          (unsigned long)dropped_logs);
    } else {
        length = snprintf(buffer, capacity,
                          "HEALTH uptime_ms=%lu loops=%lu missed=%lu "
                          "CAN1_STATUS_ERROR dropped_logs=%lu\r\n",
                          (unsigned long)uptime_ms,
                          (unsigned long)monitor->loop_count,
                          (unsigned long)monitor->missed_ticks,
                          (unsigned long)dropped_logs);
    }
    return length;
}

static bool append_formatted_line(char *record, size_t capacity, size_t *used,
                                  int length)
{
    if (length <= 0 || length >= (int)(capacity - *used)) {
        return false;
    }
    *used += (size_t)length;
    return true;
}

static void enqueue_status_report(MotionLogQueue *queue,
                                  uint32_t *dropped_logs,
                                  const WheelStatus *wheel,
                                  const Phase1Monitor *monitor, bool can1_ok,
                                  const Can1Status *can_status,
                                  bool can_status_valid, uint32_t now_ms)
{
    char record[MOTION_LOG_RECORD_CAPACITY];
    size_t used = 0u;

    if (!append_log_format(record, sizeof(record), &used,
                           "STATUS_REQUESTED\r\n") ||
        !append_formatted_line(
            record, sizeof(record), &used,
            format_wheel_status(&record[used], sizeof(record) - used, wheel,
                                now_ms)) ||
        !append_formatted_line(
            record, sizeof(record), &used,
            format_diagnostics(&record[used], sizeof(record) - used, monitor,
                               can1_ok, can_status, can_status_valid, now_ms,
                               *dropped_logs))) {
        (*dropped_logs)++;
        return;
    }
    (void)enqueue_log_record(queue, dropped_logs, record, used);
}

static void enqueue_periodic_telemetry(
    MotionLogQueue *queue, uint32_t *dropped_logs, bool wheel_due,
    bool health_due, const WheelStatus *wheel, const Phase1Monitor *monitor,
    bool can1_ok, const Can1Status *can_status, bool can_status_valid,
    uint32_t now_ms)
{
    char record[MOTION_LOG_RECORD_CAPACITY];
    size_t used = 0u;
    bool formatted = true;

    if (wheel_due) {
        formatted = append_formatted_line(
            record, sizeof(record), &used,
            format_wheel_status(&record[used], sizeof(record) - used, wheel,
                                now_ms));
    }
    if (formatted && health_due) {
        formatted = append_formatted_line(
            record, sizeof(record), &used,
            format_diagnostics(&record[used], sizeof(record) - used, monitor,
                               can1_ok, can_status, can_status_valid, now_ms,
                               *dropped_logs));
    }
    if (!formatted || used == 0u) {
        (*dropped_logs)++;
        return;
    }
    (void)enqueue_log_record(queue, dropped_logs, record, used);
}

static MotionSafetySnapshot build_motion_safety_snapshot(
    const WheelStatus *wheel, const Can1Status *can_status,
    bool can_status_valid, uint32_t now_ms)
{
    MotionSafetySnapshot safety = {
        .now_ms = now_ms,
        .feedback_age_ms = wheel->feedback_valid
                               ? (uint32_t)(now_ms - wheel->last_feedback_ms)
                               : UINT32_MAX,
        .parameter_mask = wheel->parameter_mask,
        .control_mode = wheel->control_mode,
        .motor_state = wheel->feedback.state,
        .velocity_millirad_s = wheel->feedback.velocity_millirad_s,
        .mos_temperature_c = wheel->feedback.mos_temperature_c,
        .rotor_temperature_c = wheel->feedback.rotor_temperature_c,
        .feedback_valid = wheel->feedback_valid,
        .can_warning = !can_status_valid || can_status->warning,
        .can_passive = !can_status_valid || can_status->error_passive,
        .can_bus_off = !can_status_valid || can_status->bus_off,
    };

    return safety;
}

static bool execute_motion_action(MotionAction action, WheelStatus *wheel)
{
    H6215CanFrame frame;
    bool built;

    switch (action) {
    case MOTION_ACTION_ENABLE:
        built = h6215_build_enable_command(&frame);
        break;
    case MOTION_ACTION_POSITIVE_VELOCITY:
        built = h6215_build_positive_velocity_command(&frame);
        break;
    case MOTION_ACTION_ZERO_VELOCITY:
        built = h6215_build_zero_velocity_command(&frame);
        break;
    case MOTION_ACTION_DISABLE:
        built = h6215_build_disable_command(&frame);
        break;
    case MOTION_ACTION_NONE:
    default:
        return true;
    }

    if (!built || !board_can1_transmit(frame.id, frame.data, frame.dlc)) {
        wheel->tx_failed++;
        return false;
    }
    wheel->tx_ok++;
    return true;
}

static const char *motion_action_name(MotionAction action)
{
    switch (action) {
    case MOTION_ACTION_ENABLE:
        return "ENABLE";
    case MOTION_ACTION_POSITIVE_VELOCITY:
        return "POSITIVE_VELOCITY";
    case MOTION_ACTION_ZERO_VELOCITY:
        return "ZERO_VELOCITY";
    case MOTION_ACTION_DISABLE:
        return "DISABLE";
    case MOTION_ACTION_NONE:
    default:
        return "NONE";
    }
}

static bool append_motion_event(char *record, size_t capacity, size_t *used,
                                MotionEvent event, const char *reason)
{
    switch (event) {
    case MOTION_EVENT_STATUS:
        return append_log_format(record, capacity, used,
                                 "STATUS_REQUESTED\r\n");
    case MOTION_EVENT_ARMED:
        return append_log_format(record, capacity, used,
                                 "MOTION_ARMED EXPIRES_MS=10000\r\n");
    case MOTION_EVENT_ARM_TIMEOUT:
        return append_log_format(record, capacity, used, "ARM_TIMEOUT\r\n");
    case MOTION_EVENT_START_REJECTED:
        return append_log_format(record, capacity, used,
                                 "START_REJECTED REASON=%s\r\n",
                                 reason != NULL ? reason : "UNKNOWN");
    case MOTION_EVENT_START_REQUESTED:
        return append_log_format(
            record, capacity, used,
            "MOTION_START_REQUESTED TARGET=+0.200rad/s\r\n");
    case MOTION_EVENT_RUNNING:
        return append_log_format(
            record, capacity, used,
            "MOTION_RUNNING TARGET=+0.200rad/s DURATION_MS=1000\r\n");
    case MOTION_EVENT_ZERO_HOLD:
        return append_log_format(record, capacity, used,
                                 "ZERO_SPEED_HOLD DURATION_MS=200\r\n");
    case MOTION_EVENT_COMPLETE:
        return append_log_format(record, capacity, used,
                                 "TEST_COMPLETE MOTOR_DISABLED\r\n");
    case MOTION_EVENT_EMERGENCY_STOP:
        return append_log_format(record, capacity, used,
                                 "EMERGENCY_STOP_REQUESTED\r\n");
    case MOTION_EVENT_SAFETY_TRIP:
        return append_log_format(record, capacity, used,
                                 "SAFETY_TRIP REASON=%s\r\n",
                                 reason != NULL ? reason : "UNKNOWN");
    case MOTION_EVENT_TX_FAILURE:
        return append_log_format(record, capacity, used,
                                 "TX_FAILURE REASON=CAN_TX_FAILED\r\n");
    case MOTION_EVENT_NONE:
    default:
        return true;
    }
}

static bool append_successful_motion_action(char *record, size_t capacity,
                                            size_t *used, MotionAction action)
{
    switch (action) {
    case MOTION_ACTION_ENABLE:
        return append_log_format(record, capacity, used,
                                 "MOTOR_ENABLE_TX_OK\r\n");
    case MOTION_ACTION_ZERO_VELOCITY:
        return append_log_format(record, capacity, used,
                                 "ZERO_SPEED_TX_OK\r\n");
    case MOTION_ACTION_DISABLE:
        return append_log_format(record, capacity, used,
                                 "MOTOR_DISABLE_TX_OK\r\n");
    case MOTION_ACTION_POSITIVE_VELOCITY:
    case MOTION_ACTION_NONE:
    default:
        return true;
    }
}

static bool motion_event_follows_action(MotionEvent event)
{
    return event == MOTION_EVENT_RUNNING || event == MOTION_EVENT_ZERO_HOLD ||
           event == MOTION_EVENT_COMPLETE;
}

static void enqueue_motion_decision_log(MotionLogQueue *queue,
                                        uint32_t *dropped_logs,
                                        MotionDecision decision,
                                        bool action_succeeded)
{
    char record[MOTION_LOG_RECORD_CAPACITY];
    size_t used = 0u;
    bool formatted = true;

    if (!motion_event_follows_action(decision.event)) {
        formatted = append_motion_event(record, sizeof(record), &used,
                                        decision.event, decision.reason);
    }
    if (formatted && decision.action != MOTION_ACTION_NONE) {
        if (action_succeeded) {
            formatted = append_successful_motion_action(
                record, sizeof(record), &used, decision.action);
        } else {
            formatted = append_log_format(
                record, sizeof(record), &used,
                "CAN_TX_FAILED ACTION=%s\r\n",
                motion_action_name(decision.action));
        }
    }
    if (formatted && action_succeeded &&
        motion_event_follows_action(decision.event)) {
        formatted = append_motion_event(record, sizeof(record), &used,
                                        decision.event, decision.reason);
    }
    if (!formatted) {
        (*dropped_logs)++;
        return;
    }
    if (used != 0u) {
        (void)enqueue_log_record(queue, dropped_logs, record, used);
    }
}

static bool attempt_motion_decision(MotionDecision decision,
                                    WheelStatus *wheel,
                                    MotionLogQueue *queue,
                                    uint32_t *dropped_logs)
{
    bool succeeded = decision.action == MOTION_ACTION_NONE ||
                     execute_motion_action(decision.action, wheel);

    enqueue_motion_decision_log(queue, dropped_logs, decision, succeeded);
    return succeeded;
}

static void enqueue_motion_event_only(MotionDecision decision,
                                      MotionLogQueue *queue,
                                      uint32_t *dropped_logs)
{
    decision.action = MOTION_ACTION_NONE;
    enqueue_motion_decision_log(queue, dropped_logs, decision, true);
}

static void execute_motion_decision(MotionController *controller,
                                    MotionDecision decision,
                                    WheelStatus *wheel,
                                    MotionLogQueue *queue,
                                    PendingMotionAction *pending,
                                    uint32_t *dropped_logs)
{
    MotionDecision recovery;
    MotionDecision retained;

    if (attempt_motion_decision(decision, wheel, queue, dropped_logs)) {
        return;
    }

    recovery = motion_controller_tx_failed(controller);
    if (attempt_motion_decision(recovery, wheel, queue, dropped_logs)) {
        return;
    }

    retained = motion_controller_tx_failed(controller);
    enqueue_motion_event_only(retained, queue, dropped_logs);
    pending_motion_action_failed(pending, recovery.action, retained.action);
}

static bool service_pending_motion_action(MotionController *controller,
                                          WheelStatus *wheel,
                                          MotionLogQueue *queue,
                                          PendingMotionAction *pending,
                                          uint32_t *dropped_logs)
{
    MotionDecision decision = {0};
    MotionDecision recovery;
    MotionAction attempted;

    if (!pending_motion_action_has_value(pending)) {
        return true;
    }
    attempted = pending_motion_action_begin_attempt(pending);
    decision.action = attempted;
    if (attempt_motion_decision(decision, wheel, queue, dropped_logs)) {
        pending_motion_action_succeeded(pending);
        return true;
    }

    recovery = motion_controller_tx_failed(controller);
    enqueue_motion_event_only(recovery, queue, dropped_logs);
    pending_motion_action_failed(pending, attempted, recovery.action);
    return false;
}

static bool motion_allows_parameter_polling(MotionState state)
{
    return state == MOTION_IDLE_DISABLED || state == MOTION_ARMED;
}

int main(void)
{
    static MotionLogQueue log_queue;
    MotionController motion;
    PendingMotionAction pending_motion_action;
    MotionFeedbackProbeSchedule feedback_probe_schedule;
    Phase1Monitor monitor;
    bool boot_banner_queued = false;
    bool can1_ok;
    uint32_t telemetry_period_ms = MOTION_TELEMETRY_IDLE_PERIOD_MS;
    uint32_t next_diagnostic_ms =
        BOOT_LOG_DELAY_MS + MOTION_TELEMETRY_IDLE_PERIOD_MS;
    uint32_t next_h6215_poll_ms = H6215_POLL_START_MS;
    uint32_t next_wheel_log_ms =
        H6215_POLL_START_MS + MOTION_TELEMETRY_IDLE_PERIOD_MS + 50u;
    uint32_t next_can_recovery_check_ms = H6215_POLL_START_MS;
    uint32_t dropped_logs = 0u;
    size_t h6215_register_index = 0u;
    WheelStatus wheel = {0};

    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    board_clock_init();
    MX_GPIO_Init();
    can1_ok = board_can1_init();
    motion_controller_init(&motion);
    motion_log_queue_init(&log_queue);
    pending_motion_action_init(&pending_motion_action);
    motion_feedback_probe_schedule_init(&feedback_probe_schedule);
    usb_command_queue_init();
    MX_USB_DEVICE_Init();
    phase1_monitor_init(&monitor);

    while (1) {
        Can1Status can_status = {0};
        MotionSafetySnapshot safety;
        bool can_status_valid;
        bool feedback_probe_due;
        bool health_log_due = false;
        bool wheel_log_due = false;
        uint32_t command_count;
        uint32_t now_ms = HAL_GetTick();

        if (!phase1_monitor_step(&monitor, now_ms)) {
            continue;
        }

        can_status_valid = can1_ok && board_can1_get_status(&can_status);
        if (can1_ok) {
            recover_can1_if_needed(&wheel, &next_can_recovery_check_ms, now_ms,
                                   &can_status, can_status_valid);
            receive_h6215_frames(&wheel, now_ms);
        }
        safety = build_motion_safety_snapshot(&wheel, &can_status,
                                              can_status_valid, now_ms);

        if (service_pending_motion_action(
                &motion, &wheel, &log_queue, &pending_motion_action,
                &dropped_logs)) {
            for (command_count = 0u; command_count < USB_COMMANDS_PER_LOOP;
                 command_count++) {
                MotionDecision decision;
                uint8_t command;

                if (!usb_command_queue_pop(&command)) {
                    break;
                }
                decision = motion_controller_command(&motion, command, &safety);
                if (decision.event == MOTION_EVENT_STATUS) {
                    enqueue_status_report(
                        &log_queue, &dropped_logs, &wheel, &monitor, can1_ok,
                        &can_status, can_status_valid, now_ms);
                } else {
                    execute_motion_decision(
                        &motion, decision, &wheel, &log_queue,
                        &pending_motion_action, &dropped_logs);
                }
                if (pending_motion_action_has_value(&pending_motion_action)) {
                    break;
                }
            }
            if (!pending_motion_action_has_value(&pending_motion_action)) {
                execute_motion_decision(
                    &motion, motion_controller_step(&motion, &safety), &wheel,
                    &log_queue, &pending_motion_action, &dropped_logs);
            }
        }

        feedback_probe_due = motion_feedback_probe_should_send(
            &feedback_probe_schedule, motion.state,
            pending_motion_action_has_value(&pending_motion_action), now_ms);
        if (can1_ok && feedback_probe_due) {
            send_disable_feedback_probe(&wheel);
        }

        if (can1_ok && !pending_motion_action_has_value(&pending_motion_action) &&
            motion_allows_parameter_polling(motion.state)) {
            poll_h6215(&wheel, &next_h6215_poll_ms, now_ms,
                       &h6215_register_index);
        } else {
            next_h6215_poll_ms = now_ms + H6215_POLL_PERIOD_MS;
        }

        if (!boot_banner_queued && now_ms >= BOOT_LOG_DELAY_MS) {
            boot_banner_queued = enqueue_boot_banner(
                can1_ok, &log_queue, &dropped_logs);
        }
        (void)motion_telemetry_reschedule(
            motion.state, now_ms, &telemetry_period_ms, &next_diagnostic_ms,
            &next_wheel_log_ms);
        if (boot_banner_queued &&
            (int32_t)(now_ms - next_diagnostic_ms) >= 0) {
            health_log_due = true;
            next_diagnostic_ms += telemetry_period_ms;
        }
        if (boot_banner_queued &&
            (int32_t)(now_ms - next_wheel_log_ms) >= 0) {
            wheel_log_due = true;
            next_wheel_log_ms += telemetry_period_ms;
        }
        if (wheel_log_due || health_log_due) {
            enqueue_periodic_telemetry(
                &log_queue, &dropped_logs, wheel_log_due, health_log_due,
                &wheel, &monitor, can1_ok, &can_status, can_status_valid,
                now_ms);
        }
        service_log_queue(&log_queue, &dropped_logs);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
