#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "gpio.h"
#include "main.h"
#include "motion_io.h"
#include "phase1_monitor.h"
#include "power_quiet_controller.h"
#include "usb_command_queue.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define BOOT_LOG_DELAY_MS 1500u
#define TELEMETRY_PERIOD_MS 1000u
#define USB_COMMANDS_PER_LOOP 32u

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

    if (!motion_log_queue_peek(queue, &record, &length)) {
        return;
    }
    memcpy(tx_buffers[tx_buffer_index], record, length);
    accepted = CDC_Transmit_HS(tx_buffers[tx_buffer_index], length) == USBD_OK;
    motion_log_queue_finish_attempt(queue, accepted, dropped_logs);
    if (accepted) {
        tx_buffer_index ^= 1u;
    }
}

static void apply_decision(const PowerQuietDecision *decision)
{
    if (decision->set_output) {
        board_motor_power_set(decision->output_on);
    }
}

static void enqueue_power_status(MotionLogQueue *queue,
                                 uint32_t *dropped_logs,
                                 const PowerQuietController *controller)
{
    char record[160];

    (void)snprintf(record, sizeof(record),
                   "[POWER] VCC_OUT1=%s MODE=QUIET STATE=%s CAN_TX=0\r\n",
                   board_motor_power_is_enabled() ? "ON" : "OFF",
                   power_quiet_state_name(controller->state));
    (void)enqueue_log(queue, dropped_logs, record);
}

static void enqueue_decision(MotionLogQueue *queue, uint32_t *dropped_logs,
                             const PowerQuietController *controller,
                             const PowerQuietDecision *decision)
{
    char record[192];

    switch (decision->event) {
    case POWER_QUIET_EVENT_STATUS:
        (void)enqueue_log(queue, dropped_logs, "STATUS_REQUESTED\r\n");
        enqueue_power_status(queue, dropped_logs, controller);
        break;
    case POWER_QUIET_EVENT_ARMED:
        (void)enqueue_log(queue, dropped_logs,
                          "POWER_ARMED EXPIRES_MS=10000\r\n");
        enqueue_power_status(queue, dropped_logs, controller);
        break;
    case POWER_QUIET_EVENT_ARM_TIMEOUT:
        (void)enqueue_log(queue, dropped_logs, "POWER_ARM_TIMEOUT\r\n");
        enqueue_power_status(queue, dropped_logs, controller);
        break;
    case POWER_QUIET_EVENT_ON:
        (void)enqueue_log(queue, dropped_logs, "POWER_ON_REQUESTED\r\n");
        enqueue_power_status(queue, dropped_logs, controller);
        break;
    case POWER_QUIET_EVENT_OFF:
        (void)enqueue_log(queue, dropped_logs,
                          "EMERGENCY_STOP_REQUESTED POWER_OFF\r\n");
        enqueue_power_status(queue, dropped_logs, controller);
        break;
    case POWER_QUIET_EVENT_REJECTED:
        (void)snprintf(record, sizeof(record),
                       "POWER_REJECTED REASON=%s\r\n",
                       decision->reason != NULL ? decision->reason : "UNKNOWN");
        (void)enqueue_log(queue, dropped_logs, record);
        break;
    case POWER_QUIET_EVENT_NONE:
    default:
        break;
    }
}

static void enqueue_health(MotionLogQueue *queue, uint32_t *dropped_logs,
                           const Phase1Monitor *monitor, bool can1_ok,
                           uint32_t now_ms)
{
    Can1Status can_status = {0};
    bool status_ok = can1_ok && board_can1_get_status(&can_status);
    char record[320];

    if (status_ok) {
        (void)snprintf(
            record, sizeof(record),
            "HEALTH uptime_ms=%lu loops=%lu missed=%lu period_min_ms=%lu "
            "period_max_ms=%lu can1_tec=%u can1_rec=%u can1_lec=%u "
            "warning=%u passive=%u bus_off=%u can_tx=0 dropped_logs=%lu "
            "dropped_commands=%lu\r\n",
            (unsigned long)now_ms, (unsigned long)monitor->loop_count,
            (unsigned long)monitor->missed_ticks,
            (unsigned long)(monitor->min_period_ms == UINT_MAX
                                ? 0u
                                : monitor->min_period_ms),
            (unsigned long)monitor->max_period_ms,
            can_status.tx_error_count, can_status.rx_error_count,
            can_status.last_error_code, can_status.warning,
            can_status.error_passive, can_status.bus_off,
            (unsigned long)*dropped_logs,
            (unsigned long)usb_command_queue_dropped());
    } else {
        (void)snprintf(record, sizeof(record),
                       "HEALTH uptime_ms=%lu loops=%lu CAN1_STATUS_ERROR "
                       "can_tx=0 dropped_logs=%lu dropped_commands=%lu\r\n",
                       (unsigned long)now_ms,
                       (unsigned long)monitor->loop_count,
                       (unsigned long)*dropped_logs,
                       (unsigned long)usb_command_queue_dropped());
    }
    (void)enqueue_log(queue, dropped_logs, record);
}

int main(void)
{
    static MotionLogQueue log_queue;
    PowerQuietController power;
    Phase1Monitor monitor;
    bool can1_ok;
    bool boot_queued = false;
    uint32_t next_telemetry_ms = BOOT_LOG_DELAY_MS + TELEMETRY_PERIOD_MS;
    uint32_t dropped_logs = 0u;

    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    board_motor_power_init();
    board_clock_init();
    MX_GPIO_Init();
    can1_ok = board_can1_init();
    power_quiet_controller_init(&power);
    motion_log_queue_init(&log_queue);
    usb_command_queue_init();
    MX_USB_DEVICE_Init();
    phase1_monitor_init(&monitor);

    while (1) {
        PowerQuietDecision decision;
        uint32_t command_count;
        uint32_t now_ms = HAL_GetTick();

        (void)phase1_monitor_step(&monitor, now_ms);

        if (usb_command_queue_take_emergency_stop()) {
            board_motor_power_set(false);
            decision = power_quiet_controller_command(&power, 'X', now_ms);
            enqueue_decision(&log_queue, &dropped_logs, &power, &decision);
        }

        for (command_count = 0u; command_count < USB_COMMANDS_PER_LOOP;
             ++command_count) {
            uint8_t command;

            if (!usb_command_queue_pop(&command)) {
                break;
            }
            decision = power_quiet_controller_command(&power, command, now_ms);
            apply_decision(&decision);
            enqueue_decision(&log_queue, &dropped_logs, &power, &decision);
        }

        decision = power_quiet_controller_step(&power, now_ms);
        apply_decision(&decision);
        enqueue_decision(&log_queue, &dropped_logs, &power, &decision);

        if (!boot_queued && now_ms >= BOOT_LOG_DELAY_MS) {
            const char *banner = can1_ok
                ? "MC02_BOOT\r\nPHASE31_POWER_QUIET\r\n"
                  "CAN1_INIT_OK BITRATE=1000000 TX=DISABLED\r\n"
                  "COMMANDS=S,A,P,X DEFAULT_POWER=OFF\r\n"
                : "MC02_BOOT\r\nPHASE31_POWER_QUIET\r\n"
                  "CAN1_INIT_ERROR TX=DISABLED\r\n"
                  "COMMANDS=S,A,P,X DEFAULT_POWER=OFF\r\n";

            boot_queued = enqueue_log(&log_queue, &dropped_logs, banner);
            enqueue_power_status(&log_queue, &dropped_logs, &power);
        }

        if ((int32_t)(now_ms - next_telemetry_ms) >= 0) {
            next_telemetry_ms = now_ms + TELEMETRY_PERIOD_MS;
            enqueue_power_status(&log_queue, &dropped_logs, &power);
            enqueue_health(&log_queue, &dropped_logs, &monitor, can1_ok,
                           now_ms);
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
