#include <stdio.h>
#include <string.h>

#include "board.h"
#include "dm4310_controller.h"
#include "dm4310_protocol.h"
#include "feedback_timing.h"
#include "gpio.h"
#include "h6215_protocol.h"
#include "main.h"
#include "motion_io.h"
#include "motion_controller.h"
#include "parallel_controller.h"
#include "phase1_monitor.h"
#include "power_quiet_controller.h"
#include "usb_command_queue.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define BOOT_LOG_DELAY_MS 1500u
#define TELEMETRY_MS 1000u
#define FEEDBACK_SLOT_MS 10u
#define PARAMETER_SLOT_MS 30u
#define ONLINE_TIMEOUT_MS 500u
#define USB_COMMANDS_PER_LOOP 32u
#define PARAMETER_MASK_COMPLETE 0x1fu
#define JOINT_A_ID 6u
#define JOINT_A_MASTER_ID 3u
#define JOINT_B_ID 8u
#define JOINT_B_MASTER_ID 4u
#define MIT_KD_MILLI 1000

typedef enum {
    MOTOR_JOINT_A = 0,
    MOTOR_JOINT_B,
    MOTOR_WHEEL,
    MOTOR_COUNT,
} MotorRole;

typedef struct {
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
    uint8_t state;
    int32_t position_millirad;
    int32_t velocity_millirad_s;
    int32_t torque_millinewton_m;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    bool feedback_valid;
} MotorStatus;

static const uint8_t parameter_registers[] = {
    DM4310_REGISTER_SOFTWARE_VERSION,
    DM4310_REGISTER_CONTROL_MODE,
    DM4310_REGISTER_P_MAX,
    DM4310_REGISTER_V_MAX,
    DM4310_REGISTER_T_MAX,
};

static ParallelController parallel_motion;

static bool enqueue_log(MotionLogQueue *queue, uint32_t *dropped_logs,
                        const char *record)
{
    if (!motion_log_queue_push(queue, record, strlen(record))) {
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

static void update_parameter(MotorStatus *motor, uint8_t register_id,
                             uint32_t raw_value, float float_value)
{
    switch (register_id) {
    case DM4310_REGISTER_SOFTWARE_VERSION:
        if (decode_version(raw_value, motor->software_version))
            motor->parameter_mask |= 1u << 0;
        break;
    case DM4310_REGISTER_CONTROL_MODE:
        motor->control_mode = (uint8_t)raw_value;
        motor->parameter_mask |= 1u << 1;
        break;
    case DM4310_REGISTER_P_MAX:
        motor->p_max_milli = float_to_milli(float_value);
        motor->parameter_mask |= 1u << 2;
        break;
    case DM4310_REGISTER_V_MAX:
        motor->v_max_milli = float_to_milli(float_value);
        motor->parameter_mask |= 1u << 3;
        break;
    case DM4310_REGISTER_T_MAX:
        motor->t_max_milli = float_to_milli(float_value);
        motor->parameter_mask |= 1u << 4;
        break;
    default:
        break;
    }
}

static void update_dm_feedback(MotorStatus *motor,
                               const Dm4310Feedback *feedback,
                               uint32_t received_ms)
{
    motor->state = feedback->state;
    motor->position_millirad = feedback->position_millirad;
    motor->velocity_millirad_s = feedback->velocity_millirad_s;
    motor->torque_millinewton_m = feedback->torque_millinewton_m;
    motor->mos_temperature_c = feedback->mos_temperature_c;
    motor->rotor_temperature_c = feedback->rotor_temperature_c;
    motor->last_feedback_ms = received_ms;
    motor->feedback_valid = true;
    motor->rx_count++;
}

static void update_wheel_feedback(MotorStatus *motor,
                                  const H6215Feedback *feedback,
                                  uint32_t received_ms)
{
    motor->state = feedback->state;
    motor->position_millirad = feedback->position_millirad;
    motor->velocity_millirad_s = feedback->velocity_millirad_s;
    motor->torque_millinewton_m = feedback->torque_millinewton_m;
    motor->mos_temperature_c = feedback->mos_temperature_c;
    motor->rotor_temperature_c = feedback->rotor_temperature_c;
    motor->last_feedback_ms = received_ms;
    motor->feedback_valid = true;
    motor->rx_count++;
}

static void receive_frames(MotorStatus motors[MOTOR_COUNT], uint32_t now_ms,
                           uint32_t loop_period_ms, uint32_t *unknown_rx)
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t age_us;

    while (board_can1_receive(&id, data, &dlc, &age_us)) {
        Dm4310CanFrame dm_frame = {.id = id, .dlc = dlc};
        H6215CanFrame wheel_frame = {.id = id, .dlc = dlc};
        Dm4310ParameterResponse dm_parameter;
        H6215ParameterResponse wheel_parameter;
        Dm4310Feedback dm_feedback;
        H6215Feedback wheel_feedback;
        uint32_t received_ms = feedback_received_at_ms(
            now_ms, age_us, loop_period_ms);

        memcpy(dm_frame.data, data, sizeof(data));
        memcpy(wheel_frame.data, data, sizeof(data));
        if (dm4310_parse_parameter_response_for(
                JOINT_A_ID, JOINT_A_MASTER_ID, &dm_frame, &dm_parameter)) {
            update_parameter(&motors[MOTOR_JOINT_A], dm_parameter.register_id,
                             dm_parameter.raw_value, dm_parameter.float_value);
            motors[MOTOR_JOINT_A].rx_count++;
        } else if (dm4310_parse_feedback_for(
                       JOINT_A_ID, JOINT_A_MASTER_ID, &dm_frame,
                       &dm_feedback)) {
            update_dm_feedback(&motors[MOTOR_JOINT_A], &dm_feedback,
                               received_ms);
        } else if (dm4310_parse_parameter_response_for(
                       JOINT_B_ID, JOINT_B_MASTER_ID, &dm_frame,
                       &dm_parameter)) {
            update_parameter(&motors[MOTOR_JOINT_B], dm_parameter.register_id,
                             dm_parameter.raw_value, dm_parameter.float_value);
            motors[MOTOR_JOINT_B].rx_count++;
        } else if (dm4310_parse_feedback_for(
                       JOINT_B_ID, JOINT_B_MASTER_ID, &dm_frame,
                       &dm_feedback)) {
            update_dm_feedback(&motors[MOTOR_JOINT_B], &dm_feedback,
                               received_ms);
        } else if (h6215_parse_parameter_response(
                       &wheel_frame, &wheel_parameter)) {
            update_parameter(&motors[MOTOR_WHEEL], wheel_parameter.register_id,
                             wheel_parameter.raw_value,
                             wheel_parameter.float_value);
            motors[MOTOR_WHEEL].rx_count++;
        } else if (h6215_parse_feedback(&wheel_frame, &wheel_feedback)) {
            update_wheel_feedback(&motors[MOTOR_WHEEL], &wheel_feedback,
                                  received_ms);
        } else {
            (*unknown_rx)++;
        }
    }
}

static bool parameters_valid(const MotorStatus *motor, MotorRole role)
{
    if (motor->parameter_mask != PARAMETER_MASK_COMPLETE ||
        motor->p_max_milli != 12500 || motor->t_max_milli != 10000)
        return false;
    if (role == MOTOR_WHEEL)
        return motor->control_mode == 3u && motor->v_max_milli == 45000;
    return motor->control_mode == 1u && motor->v_max_milli == 30000;
}

static bool online(const MotorStatus *motor, uint32_t now_ms)
{
    return motor->feedback_valid &&
           (uint32_t)(now_ms - motor->last_feedback_ms) <= ONLINE_TIMEOUT_MS;
}

static bool transmit(MotorStatus *motor, uint32_t id, const uint8_t data[8],
                     uint8_t dlc)
{
    if (!board_can1_transmit(id, data, dlc)) {
        motor->tx_failed++;
        return false;
    }
    motor->tx_ok++;
    return true;
}

static bool send_disable_all(MotorStatus motors[MOTOR_COUNT])
{
    Dm4310CanFrame dm_frame;
    H6215CanFrame wheel_frame;
    bool success = true;

    if (!dm4310_build_mit_command_for(JOINT_A_ID, 0, 0, 0,
                                      MIT_KD_MILLI, 0, &dm_frame) ||
        !transmit(&motors[MOTOR_JOINT_A], dm_frame.id, dm_frame.data,
                  dm_frame.dlc))
        success = false;
    if (!dm4310_build_disable_command_for(JOINT_A_ID, &dm_frame) ||
        !transmit(&motors[MOTOR_JOINT_A], dm_frame.id, dm_frame.data,
                  dm_frame.dlc))
        success = false;
    if (!dm4310_build_mit_command_for(JOINT_B_ID, 0, 0, 0,
                                      MIT_KD_MILLI, 0, &dm_frame) ||
        !transmit(&motors[MOTOR_JOINT_B], dm_frame.id, dm_frame.data,
                  dm_frame.dlc))
        success = false;
    if (!dm4310_build_disable_command_for(JOINT_B_ID, &dm_frame) ||
        !transmit(&motors[MOTOR_JOINT_B], dm_frame.id, dm_frame.data,
                  dm_frame.dlc))
        success = false;
    if (!h6215_build_zero_velocity_command(&wheel_frame) ||
        !transmit(&motors[MOTOR_WHEEL], wheel_frame.id, wheel_frame.data,
                  wheel_frame.dlc))
        success = false;
    if (!h6215_build_disable_command(&wheel_frame) ||
        !transmit(&motors[MOTOR_WHEEL], wheel_frame.id, wheel_frame.data,
                  wheel_frame.dlc))
        success = false;
    return success;
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

static void emergency_stop(MotorStatus motors[MOTOR_COUNT],
                           PowerQuietController *power, bool *probe_active,
                           Dm4310Controller *joint_motion,
                           MotionController *wheel_motion,
                           MotionLogQueue *queue, uint32_t *dropped_logs)
{
    if (board_motor_power_is_enabled()) (void)send_disable_all(motors);
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

static const char *role_name(MotorRole role)
{
    switch (role) {
    case MOTOR_JOINT_A: return "JOINT_A";
    case MOTOR_JOINT_B: return "JOINT_B";
    case MOTOR_WHEEL: return "WHEEL";
    default: return "UNKNOWN";
    }
}

static uint8_t role_id(MotorRole role)
{
    return role == MOTOR_JOINT_A ? JOINT_A_ID
                                : role == MOTOR_JOINT_B ? JOINT_B_ID
                                                       : H6215_CAN_ID;
}

static uint8_t role_master_id(MotorRole role)
{
    return role == MOTOR_JOINT_A ? JOINT_A_MASTER_ID
                                : role == MOTOR_JOINT_B ? JOINT_B_MASTER_ID
                                                       : H6215_MASTER_ID;
}

static const char *state_name(MotorRole role, uint8_t state)
{
    return role == MOTOR_WHEEL ? h6215_state_name(state)
                               : dm4310_state_name(state);
}

static size_t append_motor_status(char *record, size_t capacity, size_t used,
                                  const MotorStatus *motor, MotorRole role,
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
                       (unsigned long)(now_ms - motor->last_feedback_ms));
    else
        (void)snprintf(age, sizeof(age), "NA");

    written = snprintf(
        record + used, capacity - used,
        "[%s] ONLINE=%u ID=%u MST=%u ST=%s AGE=%s SW=%s MODE=%u "
        "LIM=%s/%s/%s PARAM_OK=%u RX=%lu POS=%s V=%s T=%s "
        "TEMP=%u/%u TXF=%lu\r\n",
        role_name(role), online(motor, now_ms), role_id(role),
        role_master_id(role),
        motor->feedback_valid ? state_name(role, motor->state) : "NO_FEEDBACK",
        age, motor->software_version[0] != '\0' ? motor->software_version
                                                : "UNKNOWN",
        motor->control_mode, p_max, v_max, t_max,
        parameters_valid(motor, role),
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
                           const MotorStatus motors[MOTOR_COUNT],
                           bool probe_active, uint32_t unknown_rx,
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
    for (role = MOTOR_JOINT_A; role < MOTOR_COUNT; ++role)
        used = append_motor_status(record, sizeof(record), used,
                                   &motors[role], role, now_ms);
    written = snprintf(
        record + used, sizeof(record) - used,
        "[CAN1] ACTIVE=%u TEC=%u REC=%u LEC=%u WARN=%u PASSIVE=%u "
        "BUS_OFF=%u UNKNOWN_RX=%lu MOTION=ONE_AT_A_TIME\r\n",
        can_status_valid, can_status->tx_error_count,
        can_status->rx_error_count, can_status->last_error_code,
        can_status->warning, can_status->error_passive, can_status->bus_off,
        (unsigned long)unknown_rx);
    if (written > 0 && (size_t)written < sizeof(record) - used)
        used += (size_t)written;
    if (used >= sizeof(record)) used = sizeof(record) - 1u;
    record[used] = '\0';
    (void)enqueue_log(queue, dropped_logs, record);
}

static bool send_feedback_request(MotorStatus motors[MOTOR_COUNT],
                                  MotorRole role)
{
    if (role == MOTOR_WHEEL) {
        H6215CanFrame frame;

        return h6215_build_disable_command(&frame) &&
               transmit(&motors[role], frame.id, frame.data, frame.dlc);
    } else {
        Dm4310CanFrame frame;

        return dm4310_build_feedback_request_for(role_id(role), &frame) &&
               transmit(&motors[role], frame.id, frame.data, frame.dlc);
    }
}

static bool send_parameter_request(MotorStatus motors[MOTOR_COUNT],
                                   MotorRole role, uint8_t register_id)
{
    if (role == MOTOR_WHEEL) {
        H6215CanFrame frame;

        return h6215_build_read_request(register_id, &frame) &&
               transmit(&motors[role], frame.id, frame.data, frame.dlc);
    } else {
        Dm4310CanFrame frame;

        return dm4310_build_read_request_for(role_id(role), register_id,
                                              &frame) &&
               transmit(&motors[role], frame.id, frame.data, frame.dlc);
    }
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

static const char *unsafe_feedback(const MotorStatus motors[MOTOR_COUNT],
                                   MotorRole selected, bool motion_active,
                                   bool parallel_active,
                                   uint32_t now_ms)
{
    MotorRole role;

    for (role = MOTOR_JOINT_A; role < MOTOR_COUNT; ++role) {
        const MotorStatus *motor = &motors[role];

        if (!online(motor, now_ms)) continue;
        if (motor->state != 0u &&
            !(motion_active && (parallel_active || role == selected) &&
              motor->state == 1u))
            return "MOTOR_NOT_DISABLED";
        if (motor->mos_temperature_c >= 60u ||
            motor->rotor_temperature_c >= 60u)
            return "TEMPERATURE_LIMIT";
    }
    return NULL;
}

static Dm4310SafetySnapshot joint_safety(
    const MotorStatus *motor, MotorRole role, const Can1Status *can_status,
    bool can_status_valid, bool probe_active, uint32_t now_ms)
{
    return (Dm4310SafetySnapshot){
        .now_ms = now_ms,
        .feedback_age_ms = motor->feedback_valid
                               ? now_ms - motor->last_feedback_ms
                               : UINT32_MAX,
        .motor_state = motor->state,
        .velocity_millirad_s = motor->velocity_millirad_s,
        .torque_millinewton_m = motor->torque_millinewton_m,
        .mos_temperature_c = motor->mos_temperature_c,
        .rotor_temperature_c = motor->rotor_temperature_c,
        .powered = board_motor_power_is_enabled(),
        .probe_active = probe_active,
        .parameters_valid = parameters_valid(motor, role),
        .feedback_valid = motor->feedback_valid,
        .can_warning = !can_status_valid || can_status->warning,
        .can_passive = !can_status_valid || can_status->error_passive,
        .can_bus_off = !can_status_valid || can_status->bus_off,
    };
}

static MotionSafetySnapshot wheel_safety(
    const MotorStatus *motor, const Can1Status *can_status,
    bool can_status_valid, uint32_t now_ms)
{
    return (MotionSafetySnapshot){
        .now_ms = now_ms,
        .feedback_age_ms = motor->feedback_valid
                               ? now_ms - motor->last_feedback_ms
                               : UINT32_MAX,
        .parameter_mask = motor->parameter_mask,
        .control_mode = motor->control_mode,
        .motor_state = motor->state,
        .velocity_millirad_s = motor->velocity_millirad_s,
        .mos_temperature_c = motor->mos_temperature_c,
        .rotor_temperature_c = motor->rotor_temperature_c,
        .feedback_valid = motor->feedback_valid,
        .can_warning = !can_status_valid || can_status->warning,
        .can_passive = !can_status_valid || can_status->error_passive,
        .can_bus_off = !can_status_valid || can_status->bus_off,
    };
}

static int32_t absolute(int32_t value)
{
    return value < 0 ? -value : value;
}

static ParallelSafetySnapshot parallel_safety(
    const MotorStatus motors[MOTOR_COUNT], const Can1Status *can_status,
    bool can_status_valid, bool probe_active, uint32_t now_ms)
{
    MotorRole role;
    ParallelSafetySnapshot safety = {
        .now_ms = now_ms,
        .powered = board_motor_power_is_enabled(),
        .probe_active = probe_active,
        .parameters_valid = true,
        .feedback_fresh = true,
        .all_disabled = true,
        .all_enabled = true,
        .start_speed_zero = true,
        .runtime_speed_safe = true,
        .runtime_torque_safe = true,
        .temperature_safe = true,
        .can_active = can_status_valid && !can_status->warning &&
                      !can_status->error_passive && !can_status->bus_off,
        .state_normal = true,
    };

    for (role = MOTOR_JOINT_A; role < MOTOR_COUNT; ++role) {
        const MotorStatus *motor = &motors[role];

        safety.parameters_valid &= parameters_valid(motor, role);
        safety.feedback_fresh &=
            motor->feedback_valid && now_ms - motor->last_feedback_ms <= 100u;
        safety.all_disabled &= motor->state == 0u;
        safety.all_enabled &= motor->state == 1u;
        safety.start_speed_zero &= absolute(motor->velocity_millirad_s) < 100;
        safety.runtime_speed_safe &=
            absolute(motor->velocity_millirad_s) <= 800;
        if (role != MOTOR_WHEEL)
            safety.runtime_torque_safe &=
                absolute(motor->torque_millinewton_m) <= 500;
        safety.temperature_safe &= motor->mos_temperature_c < 60u &&
                                   motor->rotor_temperature_c < 60u;
        safety.state_normal &= motor->state <= 1u;
    }
    return safety;
}

static bool send_enable_all(MotorStatus motors[MOTOR_COUNT])
{
    Dm4310CanFrame dm_frame;
    H6215CanFrame wheel_frame;

    return dm4310_build_enable_command_for(JOINT_A_ID, &dm_frame) &&
           transmit(&motors[MOTOR_JOINT_A], dm_frame.id, dm_frame.data,
                    dm_frame.dlc) &&
           dm4310_build_enable_command_for(JOINT_B_ID, &dm_frame) &&
           transmit(&motors[MOTOR_JOINT_B], dm_frame.id, dm_frame.data,
                    dm_frame.dlc) &&
           h6215_build_enable_command(&wheel_frame) &&
           transmit(&motors[MOTOR_WHEEL], wheel_frame.id, wheel_frame.data,
                    wheel_frame.dlc);
}

static bool send_velocity_all(MotorStatus motors[MOTOR_COUNT],
                              int8_t direction)
{
    Dm4310CanFrame dm_frame;
    H6215CanFrame wheel_frame;
    int32_t joint_velocity = (int32_t)direction * 400;
    int8_t wheel_step = (int8_t)(direction * 2);

    return dm4310_build_mit_command_for(
               JOINT_A_ID, 0, joint_velocity, 0, MIT_KD_MILLI, 0,
               &dm_frame) &&
           transmit(&motors[MOTOR_JOINT_A], dm_frame.id, dm_frame.data,
                    dm_frame.dlc) &&
           dm4310_build_mit_command_for(
               JOINT_B_ID, 0, joint_velocity, 0, MIT_KD_MILLI, 0,
               &dm_frame) &&
           transmit(&motors[MOTOR_JOINT_B], dm_frame.id, dm_frame.data,
                    dm_frame.dlc) &&
           h6215_build_velocity_step(wheel_step, &wheel_frame) &&
           transmit(&motors[MOTOR_WHEEL], wheel_frame.id, wheel_frame.data,
                    wheel_frame.dlc);
}

static bool execute_parallel_decision(
    ParallelDecision decision, MotorStatus motors[MOTOR_COUNT],
    PowerQuietController *power, Dm4310Controller *joint_motion,
    MotionController *wheel_motion, bool *probe_active,
    MotionLogQueue *queue, uint32_t *dropped_logs)
{
    bool success = true;
    char record[192];

    if (decision.send_enable_all) success = send_enable_all(motors);
    if (success && decision.send_velocity_all)
        success = send_velocity_all(motors, decision.direction);
    if (success && decision.send_disable_all)
        success = send_disable_all(motors);
    if (!success || decision.cut_power) {
        (void)send_disable_all(motors);
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
                   role_name(role), event, value,
                   reason != NULL ? " REASON=" : "",
                   reason != NULL ? reason : "");
    (void)enqueue_log(queue, dropped_logs, record);
}

static bool execute_joint_decision(
    Dm4310MotionDecision decision, MotorRole role,
    MotorStatus motors[MOTOR_COUNT], PowerQuietController *power,
    Dm4310Controller *joint_motion, MotionController *wheel_motion,
    bool *probe_active, MotionLogQueue *queue, uint32_t *dropped_logs)
{
    Dm4310CanFrame frame;
    bool success = true;

    if (decision.send_enable)
        success = dm4310_build_enable_command_for(role_id(role), &frame) &&
                  transmit(&motors[role], frame.id, frame.data, frame.dlc);
    if (success && decision.send_mit)
        success = dm4310_build_mit_command_for(
                      role_id(role), 0, decision.target_velocity_millirad_s,
                      0, MIT_KD_MILLI, 0, &frame) &&
                  transmit(&motors[role], frame.id, frame.data, frame.dlc);
    if (success && decision.send_disable)
        success = dm4310_build_disable_command_for(role_id(role), &frame) &&
                  transmit(&motors[role], frame.id, frame.data, frame.dlc);
    if (!success || decision.cut_power) {
        (void)send_disable_all(motors);
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
    MotionDecision decision, MotorStatus motors[MOTOR_COUNT],
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
        success = transmit(&motors[MOTOR_WHEEL], frame.id, frame.data,
                           frame.dlc);
    if (!success || decision.event == MOTION_EVENT_SAFETY_TRIP ||
        decision.event == MOTION_EVENT_TX_FAILURE) {
        (void)send_disable_all(motors);
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
        enqueue_motion_log(queue, dropped_logs, MOTOR_WHEEL, event,
                           decision.reason,
                           (int32_t)decision.velocity_step * 100);
    }
    return success;
}

int main(void)
{
    static MotionLogQueue log_queue;
    PowerQuietController power;
    Dm4310Controller joint_motion;
    MotionController wheel_motion;
    Phase1Monitor monitor;
    MotorStatus motors[MOTOR_COUNT] = {0};
    bool can1_ok;
    bool boot_queued = false;
    bool probe_active = false;
    bool parallel_selected = false;
    MotorRole selected = MOTOR_COUNT;
    uint32_t next_telemetry_ms = BOOT_LOG_DELAY_MS + TELEMETRY_MS;
    uint32_t next_feedback_ms = 0u;
    uint32_t next_parameter_ms = 0u;
    uint32_t dropped_logs = 0u;
    uint32_t unknown_rx = 0u;
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
    can1_ok = board_can1_init();
    power_quiet_controller_init(&power);
    dm4310_controller_init(&joint_motion);
    motion_controller_init(&wheel_motion);
    parallel_controller_init(&parallel_motion);
    motion_log_queue_init(&log_queue);
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
        receive_frames(motors, now_ms, loop_period_ms, &unknown_rx);
        can_status_valid = can1_ok && board_can1_get_status(&can_status);

        if (usb_command_queue_take_emergency_stop())
            emergency_stop(motors, &power, &probe_active, &joint_motion,
                           &wheel_motion, &log_queue, &dropped_logs);

        for (command_count = 0u; command_count < USB_COMMANDS_PER_LOOP;
             ++command_count) {
            uint8_t command;

            if (!usb_command_queue_pop(&command)) break;
            if (command >= (uint8_t)'a' && command <= (uint8_t)'z')
                command = (uint8_t)(command - ((uint8_t)'a' - (uint8_t)'A'));
            if (command == (uint8_t)'\r' || command == (uint8_t)'\n') continue;
            if (command == (uint8_t)'X') {
                emergency_stop(motors, &power, &probe_active, &joint_motion,
                               &wheel_motion, &log_queue, &dropped_logs);
                selected = MOTOR_COUNT;
                parallel_selected = false;
                continue;
            }
            if (command == (uint8_t)'S') {
                enqueue_status(&log_queue, &dropped_logs, &power, motors,
                               probe_active, unknown_rx, &can_status,
                               can_status_valid, now_ms, true);
                continue;
            }
            if (power.state != POWER_QUIET_ON) {
                PowerQuietDecision decision =
                    power_quiet_controller_command(&power, command, now_ms);

                if (decision.set_output)
                    board_motor_power_set(decision.output_on);
                if (decision.event == POWER_QUIET_EVENT_ARMED)
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "POWER_ARMED EXPIRES_MS=10000\r\n");
                else if (decision.event == POWER_QUIET_EVENT_ON)
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "POWER_ON_REQUESTED MODE=QUIET\r\n");
                else if (decision.event == POWER_QUIET_EVENT_REJECTED) {
                    char record[128];

                    (void)snprintf(record, sizeof(record),
                                   "POWER_REJECTED REASON=%s\r\n",
                                   decision.reason);
                    (void)enqueue_log(&log_queue, &dropped_logs, record);
                }
                enqueue_power_status(&log_queue, &dropped_logs, &power,
                                     probe_active);
                continue;
            }
            if (command == (uint8_t)'R') {
                if (any_motion_active(&joint_motion, &wheel_motion)) {
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "PROBE_REJECTED REASON=MOTION_ACTIVE\r\n");
                    continue;
                }
                memset(motors, 0, sizeof(motors));
                unknown_rx = 0u;
                if (!send_disable_all(motors)) {
                    power_off(&power, &probe_active, &joint_motion,
                              &wheel_motion);
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "TX_FAILURE DISABLE_ALL POWER_OFF\r\n");
                } else {
                    probe_active = true;
                    feedback_role = 0u;
                    parameter_role = 0u;
                    parameter_index = 0u;
                    next_feedback_ms = now_ms;
                    next_parameter_ms = now_ms;
                    (void)enqueue_log(
                        &log_queue, &dropped_logs,
                        "PROBE_STARTED DEVICES=3 MOTION=GUARDED\r\n");
                }
                continue;
            }
            if (command >= (uint8_t)'1' && command <= (uint8_t)'4') {
                MotorRole requested = command == (uint8_t)'4'
                                          ? MOTOR_COUNT
                                          : (MotorRole)(command -
                                                        (uint8_t)'1');

                if (!probe_active || any_motion_active(&joint_motion,
                                                       &wheel_motion)) {
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "SELECT_REJECTED REASON=SEND_R_FIRST_OR_MOTION_ACTIVE\r\n");
                } else {
                    (void)send_disable_all(motors);
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
                                           role_name(selected),
                                           role_id(selected));
                        (void)enqueue_log(&log_queue, &dropped_logs, record);
                    }
                }
                continue;
            }
            if (!parallel_selected && selected == MOTOR_COUNT) {
                (void)enqueue_log(&log_queue, &dropped_logs,
                                  "MOTION_REJECTED REASON=SELECT_1_2_3_4_FIRST\r\n");
                continue;
            }
            if (parallel_selected) {
                ParallelSafetySnapshot safety = parallel_safety(
                    motors, &can_status, can_status_valid, probe_active,
                    now_ms);
                ParallelDecision decision = parallel_controller_command(
                    &parallel_motion, command, &safety);

                (void)execute_parallel_decision(
                    decision, motors, &power, &joint_motion, &wheel_motion,
                    &probe_active, &log_queue, &dropped_logs);
            } else if (selected == MOTOR_WHEEL) {
                MotionSafetySnapshot safety = wheel_safety(
                    &motors[MOTOR_WHEEL], &can_status, can_status_valid,
                    now_ms);
                MotionDecision decision;

                if (command == (uint8_t)'N') {
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "MOTION_REJECTED REASON=N_NOT_SUPPORTED_FOR_WHEEL\r\n");
                    continue;
                }
                decision = motion_controller_command(&wheel_motion, command,
                                                     &safety);
                (void)execute_wheel_decision(
                    decision, motors, &power, &joint_motion, &wheel_motion,
                    &probe_active, &log_queue, &dropped_logs);
            } else {
                Dm4310SafetySnapshot safety = joint_safety(
                    &motors[selected], selected, &can_status,
                    can_status_valid, probe_active, now_ms);
                Dm4310MotionDecision decision = dm4310_controller_command(
                    &joint_motion, command, &safety);

                (void)execute_joint_decision(
                    decision, selected, motors, &power, &joint_motion,
                    &wheel_motion, &probe_active, &log_queue, &dropped_logs);
            }
        }

        if (power.state != POWER_QUIET_ON) {
            PowerQuietDecision decision =
                power_quiet_controller_step(&power, now_ms);

            if (decision.event == POWER_QUIET_EVENT_ARM_TIMEOUT)
                (void)enqueue_log(&log_queue, &dropped_logs,
                                  "POWER_ARM_TIMEOUT\r\n");
        }

        if (probe_active && selected != MOTOR_COUNT) {
            if (selected == MOTOR_WHEEL) {
                MotionSafetySnapshot safety = wheel_safety(
                    &motors[MOTOR_WHEEL], &can_status, can_status_valid,
                    now_ms);
                MotionDecision decision = motion_controller_step(
                    &wheel_motion, &safety);

                (void)execute_wheel_decision(
                    decision, motors, &power, &joint_motion, &wheel_motion,
                    &probe_active, &log_queue, &dropped_logs);
            } else {
                Dm4310SafetySnapshot safety = joint_safety(
                    &motors[selected], selected, &can_status,
                    can_status_valid, probe_active, now_ms);
                Dm4310MotionDecision decision = dm4310_controller_step(
                    &joint_motion, &safety);

                (void)execute_joint_decision(
                    decision, selected, motors, &power, &joint_motion,
                    &wheel_motion, &probe_active, &log_queue, &dropped_logs);
            }
        }

        if (probe_active && parallel_selected) {
            ParallelSafetySnapshot safety = parallel_safety(
                motors, &can_status, can_status_valid, probe_active, now_ms);
            ParallelDecision decision = parallel_controller_step(
                &parallel_motion, &safety);

            (void)execute_parallel_decision(
                decision, motors, &power, &joint_motion, &wheel_motion,
                &probe_active, &log_queue, &dropped_logs);
        }

        if (probe_active && board_motor_power_is_enabled()) {
            if ((int32_t)(now_ms - next_feedback_ms) >= 0) {
                next_feedback_ms = now_ms + FEEDBACK_SLOT_MS;
                if (!(parallel_controller_active(&parallel_motion) ||
                      ((MotorRole)feedback_role == selected &&
                       any_motion_active(&joint_motion, &wheel_motion))) &&
                    !send_feedback_request(motors,
                                           (MotorRole)feedback_role)) {
                    power_off(&power, &probe_active, &joint_motion,
                              &wheel_motion);
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "TX_FAILURE FEEDBACK_POLL POWER_OFF\r\n");
                }
                feedback_role = (uint8_t)((feedback_role + 1u) % MOTOR_COUNT);
            }
            if (probe_active && (int32_t)(now_ms - next_parameter_ms) >= 0) {
                next_parameter_ms = now_ms + PARAMETER_SLOT_MS;
                if (!(parallel_controller_active(&parallel_motion) ||
                      ((MotorRole)parameter_role == selected &&
                       any_motion_active(&joint_motion, &wheel_motion))) &&
                    !send_parameter_request(
                        motors, (MotorRole)parameter_role,
                        parameter_registers[parameter_index])) {
                    power_off(&power, &probe_active, &joint_motion,
                              &wheel_motion);
                    (void)enqueue_log(&log_queue, &dropped_logs,
                                      "TX_FAILURE PARAMETER_POLL POWER_OFF\r\n");
                }
                parameter_role = (uint8_t)(parameter_role + 1u);
                if (parameter_role >= MOTOR_COUNT) {
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
            (void)send_disable_all(motors);
            power_off(&power, &probe_active, &joint_motion, &wheel_motion);
            (void)enqueue_log(&log_queue, &dropped_logs,
                              "SAFETY_TRIP CAN_NOT_ACTIVE POWER_OFF\r\n");
        }
        feedback_fault = unsafe_feedback(
            motors, selected,
            any_motion_active(&joint_motion, &wheel_motion),
            parallel_controller_active(&parallel_motion), now_ms);
        if (probe_active && feedback_fault != NULL) {
            char record[128];

            (void)send_disable_all(motors);
            power_off(&power, &probe_active, &joint_motion, &wheel_motion);
            (void)snprintf(record, sizeof(record),
                           "SAFETY_TRIP %s POWER_OFF\r\n", feedback_fault);
            (void)enqueue_log(&log_queue, &dropped_logs, record);
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

            boot_queued = enqueue_log(&log_queue, &dropped_logs, banner);
        }

        if ((int32_t)(now_ms - next_telemetry_ms) >= 0) {
            next_telemetry_ms = now_ms + TELEMETRY_MS;
            enqueue_status(&log_queue, &dropped_logs, &power, motors,
                           probe_active, unknown_rx, &can_status,
                           can_status_valid, now_ms, false);
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
