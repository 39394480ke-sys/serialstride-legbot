#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "gpio.h"
#include "h6215_protocol.h"
#include "main.h"
#include "motion_controller.h"
#include "phase1_monitor.h"
#include "usb_device.h"
#include "usb_command_queue.h"
#include "usbd_cdc_if.h"

#define BOOT_LOG_DELAY_MS 1500u
#define DIAGNOSTIC_PERIOD_MS 100u
#define H6215_POLL_START_MS 2000u
#define H6215_POLL_PERIOD_MS 200u
#define WHEEL_IDLE_LOG_PERIOD_MS 1000u
#define WHEEL_MOTION_LOG_PERIOD_MS 100u
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

static void try_send_boot_banner(bool can1_ok, bool *sent)
{
    static uint8_t banner[] =
        "MC02_BOOT\r\n"
        "CLOCK_OK SYSCLK=480000000 HCLK=240000000\r\n"
        "TIMER_OK TICK_HZ=1000\r\n"
        "CAN1_INIT_OK BITRATE=1000000 TX=GUARDED_MOTION\r\n"
        "H6215_GUARDED_MOTION COMMANDS=S,A,G,X TARGET=+0.200rad/s\r\n"
        "DEFAULT_STATE=DISABLED AUTO_MOTION=OFF\r\n";
    static uint8_t failure_banner[] =
        "MC02_BOOT\r\n"
        "CLOCK_OK SYSCLK=480000000 HCLK=240000000\r\n"
        "TIMER_OK TICK_HZ=1000\r\n"
        "CAN1_INIT_ERROR\r\n"
        "H6215_GUARDED_MOTION COMMANDS=S,A,G,X TARGET=+0.200rad/s\r\n"
        "DEFAULT_STATE=DISABLED AUTO_MOTION=OFF\r\n";
    uint8_t *message = can1_ok ? banner : failure_banner;
    uint16_t length = can1_ok ? (uint16_t)(sizeof(banner) - 1u)
                              : (uint16_t)(sizeof(failure_banner) - 1u);

    if (CDC_Transmit_HS(message, length) == USBD_OK) {
        *sent = true;
    }
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

static void send_disable_probe_once(WheelStatus *wheel)
{
    H6215CanFrame command;

    if (wheel->disable_probe_sent || wheel->parameter_mask != 0x1fu ||
        !h6215_build_disable_command(&command)) {
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

static void try_send_wheel_status(const WheelStatus *wheel, uint32_t now_ms,
                                  uint32_t *dropped_logs)
{
    static uint8_t buffers[2][320];
    static uint32_t buffer_index;
    char p_max[16];
    char v_max[16];
    char t_max[16];
    char position[16];
    char velocity[16];
    char torque[16];
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
    length = snprintf(
        (char *)buffers[buffer_index], sizeof(buffers[0]),
        "[WHEEL] ONLINE=%u ID=1 MST_ID=0 STATE=%s SW=%s MODE=%u "
        "P_MAX=%s V_MAX=%s T_MAX=%s PARAM_MASK=0x%02X RX=%lu "
        "P=%s V=%s T=%s TMOS=%u TROTOR=%u DISABLE_PROBE=%u "
        "TX_OK=%lu TX_FAIL=%lu CAN_RECOVERIES=%lu\r\n",
        online, wheel->feedback_valid ? h6215_state_name(wheel->feedback.state)
                                      : "NO_FEEDBACK",
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

    if (length <= 0 || length >= (int)sizeof(buffers[0]) ||
        CDC_Transmit_HS(buffers[buffer_index], (uint16_t)length) != USBD_OK) {
        (*dropped_logs)++;
        return;
    }
    buffer_index ^= 1u;
}

static void try_send_diagnostics(const Phase1Monitor *monitor, bool can1_ok,
                                 const Can1Status *can_status,
                                 bool can_status_valid, uint32_t uptime_ms,
                                 uint32_t *dropped_logs)
{
    static uint8_t buffers[2][192];
    static uint32_t buffer_index;
    int length;

    if (can1_ok && can_status_valid) {
        length = snprintf((char *)buffers[buffer_index], sizeof(buffers[0]),
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
                          (unsigned long)*dropped_logs);
    } else {
        length = snprintf((char *)buffers[buffer_index], sizeof(buffers[0]),
                          "HEALTH uptime_ms=%lu loops=%lu missed=%lu "
                          "CAN1_STATUS_ERROR dropped_logs=%lu\r\n",
                          (unsigned long)uptime_ms,
                          (unsigned long)monitor->loop_count,
                          (unsigned long)monitor->missed_ticks,
                          (unsigned long)*dropped_logs);
    }

    if (length <= 0 || length >= (int)sizeof(buffers[0]) ||
        CDC_Transmit_HS(buffers[buffer_index], (uint16_t)length) != USBD_OK) {
        (*dropped_logs)++;
        return;
    }
    buffer_index ^= 1u;
}

static void try_send_motion_log(uint32_t *dropped_logs, const char *format, ...)
{
    static uint8_t buffers[2][128];
    static uint32_t buffer_index;
    va_list arguments;
    int length;

    va_start(arguments, format);
    length = vsnprintf((char *)buffers[buffer_index], sizeof(buffers[0]),
                       format, arguments);
    va_end(arguments);
    if (length <= 0 || length >= (int)sizeof(buffers[0]) ||
        CDC_Transmit_HS(buffers[buffer_index], (uint16_t)length) != USBD_OK) {
        (*dropped_logs)++;
        return;
    }
    buffer_index ^= 1u;
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

static void log_motion_event(MotionEvent event, const char *reason,
                             uint32_t *dropped_logs)
{
    switch (event) {
    case MOTION_EVENT_STATUS:
        try_send_motion_log(dropped_logs, "STATUS_REQUESTED\r\n");
        break;
    case MOTION_EVENT_ARMED:
        try_send_motion_log(dropped_logs,
                            "MOTION_ARMED EXPIRES_MS=10000\r\n");
        break;
    case MOTION_EVENT_ARM_TIMEOUT:
        try_send_motion_log(dropped_logs, "ARM_TIMEOUT\r\n");
        break;
    case MOTION_EVENT_START_REJECTED:
        try_send_motion_log(dropped_logs, "START_REJECTED REASON=%s\r\n",
                            reason != NULL ? reason : "UNKNOWN");
        break;
    case MOTION_EVENT_START_REQUESTED:
        try_send_motion_log(
            dropped_logs,
            "MOTION_START_REQUESTED TARGET=+0.200rad/s\r\n");
        break;
    case MOTION_EVENT_RUNNING:
        try_send_motion_log(
            dropped_logs,
            "MOTION_RUNNING TARGET=+0.200rad/s DURATION_MS=1000\r\n");
        break;
    case MOTION_EVENT_ZERO_HOLD:
        try_send_motion_log(dropped_logs,
                            "ZERO_SPEED_HOLD DURATION_MS=200\r\n");
        break;
    case MOTION_EVENT_COMPLETE:
        try_send_motion_log(dropped_logs,
                            "TEST_COMPLETE MOTOR_DISABLED\r\n");
        break;
    case MOTION_EVENT_EMERGENCY_STOP:
        try_send_motion_log(dropped_logs, "EMERGENCY_STOP_REQUESTED\r\n");
        break;
    case MOTION_EVENT_SAFETY_TRIP:
        try_send_motion_log(dropped_logs, "SAFETY_TRIP REASON=%s\r\n",
                            reason != NULL ? reason : "UNKNOWN");
        break;
    case MOTION_EVENT_TX_FAILURE:
        try_send_motion_log(dropped_logs,
                            "TX_FAILURE REASON=CAN_TX_FAILED\r\n");
        break;
    case MOTION_EVENT_NONE:
    default:
        break;
    }
}

static void log_successful_motion_action(MotionAction action,
                                         uint32_t *dropped_logs)
{
    switch (action) {
    case MOTION_ACTION_ENABLE:
        try_send_motion_log(dropped_logs, "MOTOR_ENABLE_TX_OK\r\n");
        break;
    case MOTION_ACTION_ZERO_VELOCITY:
        try_send_motion_log(dropped_logs, "ZERO_SPEED_TX_OK\r\n");
        break;
    case MOTION_ACTION_DISABLE:
        try_send_motion_log(dropped_logs, "MOTOR_DISABLE_TX_OK\r\n");
        break;
    case MOTION_ACTION_POSITIVE_VELOCITY:
    case MOTION_ACTION_NONE:
    default:
        break;
    }
}

static bool motion_event_follows_action(MotionEvent event)
{
    return event == MOTION_EVENT_RUNNING || event == MOTION_EVENT_ZERO_HOLD ||
           event == MOTION_EVENT_COMPLETE;
}

static void execute_motion_decision(MotionController *controller,
                                    MotionDecision decision,
                                    WheelStatus *wheel,
                                    uint32_t *dropped_logs)
{
    MotionDecision recovery;

    if (!motion_event_follows_action(decision.event)) {
        log_motion_event(decision.event, decision.reason, dropped_logs);
    }
    if (decision.action == MOTION_ACTION_NONE) {
        return;
    }
    if (execute_motion_action(decision.action, wheel)) {
        log_successful_motion_action(decision.action, dropped_logs);
        if (motion_event_follows_action(decision.event)) {
            log_motion_event(decision.event, decision.reason, dropped_logs);
        }
        return;
    }

    try_send_motion_log(dropped_logs, "CAN_TX_FAILED ACTION=%s\r\n",
                        motion_action_name(decision.action));
    recovery = motion_controller_tx_failed(controller);
    log_motion_event(recovery.event, recovery.reason, dropped_logs);
    if (recovery.action == MOTION_ACTION_NONE) {
        return;
    }
    if (execute_motion_action(recovery.action, wheel)) {
        log_successful_motion_action(recovery.action, dropped_logs);
        return;
    }

    try_send_motion_log(dropped_logs, "CAN_TX_FAILED ACTION=%s\r\n",
                        motion_action_name(recovery.action));
    recovery = motion_controller_tx_failed(controller);
    log_motion_event(recovery.event, recovery.reason, dropped_logs);
}

static bool motion_allows_parameter_polling(MotionState state)
{
    return state == MOTION_IDLE_DISABLED || state == MOTION_ARMED;
}

static bool motion_uses_fast_wheel_log(MotionState state)
{
    return state != MOTION_IDLE_DISABLED;
}

int main(void)
{
    MotionController motion;
    Phase1Monitor monitor;
    bool boot_banner_sent = false;
    bool can1_ok;
    uint32_t next_diagnostic_ms = BOOT_LOG_DELAY_MS + DIAGNOSTIC_PERIOD_MS;
    uint32_t next_h6215_poll_ms = H6215_POLL_START_MS;
    uint32_t next_wheel_log_ms =
        H6215_POLL_START_MS + WHEEL_IDLE_LOG_PERIOD_MS + 50u;
    uint32_t next_can_recovery_check_ms = H6215_POLL_START_MS;
    uint32_t dropped_logs = 0u;
    size_t h6215_register_index = 0u;
    bool fast_wheel_log = false;
    WheelStatus wheel = {0};

    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    board_clock_init();
    MX_GPIO_Init();
    can1_ok = board_can1_init();
    motion_controller_init(&motion);
    usb_command_queue_init();
    MX_USB_DEVICE_Init();
    phase1_monitor_init(&monitor);

    while (1) {
        Can1Status can_status = {0};
        MotionSafetySnapshot safety;
        bool can_status_valid;
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
            send_disable_probe_once(&wheel);
        }
        safety = build_motion_safety_snapshot(&wheel, &can_status,
                                              can_status_valid, now_ms);

        for (command_count = 0u; command_count < USB_COMMANDS_PER_LOOP;
             command_count++) {
            MotionDecision decision;
            uint8_t command;

            if (!usb_command_queue_pop(&command)) {
                break;
            }
            decision = motion_controller_command(&motion, command, &safety);
            execute_motion_decision(&motion, decision, &wheel, &dropped_logs);
            if (decision.event == MOTION_EVENT_STATUS) {
                try_send_wheel_status(&wheel, now_ms, &dropped_logs);
                try_send_diagnostics(&monitor, can1_ok, &can_status,
                                     can_status_valid, now_ms, &dropped_logs);
            }
        }
        execute_motion_decision(&motion, motion_controller_step(&motion, &safety),
                                &wheel, &dropped_logs);

        if (can1_ok && motion_allows_parameter_polling(motion.state)) {
            poll_h6215(&wheel, &next_h6215_poll_ms, now_ms,
                       &h6215_register_index);
        } else {
            next_h6215_poll_ms = now_ms + H6215_POLL_PERIOD_MS;
        }

        if (!boot_banner_sent && now_ms >= BOOT_LOG_DELAY_MS) {
            try_send_boot_banner(can1_ok, &boot_banner_sent);
        }
        if (boot_banner_sent && (int32_t)(now_ms - next_diagnostic_ms) >= 0) {
            try_send_diagnostics(&monitor, can1_ok, &can_status,
                                 can_status_valid, now_ms, &dropped_logs);
            next_diagnostic_ms += DIAGNOSTIC_PERIOD_MS;
        }
        if (motion_uses_fast_wheel_log(motion.state) != fast_wheel_log) {
            fast_wheel_log = motion_uses_fast_wheel_log(motion.state);
            next_wheel_log_ms =
                now_ms + (fast_wheel_log ? WHEEL_MOTION_LOG_PERIOD_MS
                                         : WHEEL_IDLE_LOG_PERIOD_MS);
        }
        if (boot_banner_sent && (int32_t)(now_ms - next_wheel_log_ms) >= 0) {
            try_send_wheel_status(&wheel, now_ms, &dropped_logs);
            next_wheel_log_ms += fast_wheel_log ? WHEEL_MOTION_LOG_PERIOD_MS
                                                : WHEEL_IDLE_LOG_PERIOD_MS;
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
