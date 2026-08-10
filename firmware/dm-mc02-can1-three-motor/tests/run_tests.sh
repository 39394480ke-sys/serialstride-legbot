#!/bin/sh
set -eu

root=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
out="${TMPDIR:-/tmp}/dm-mc02-phase1-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_phase1_monitor.c" \
  "$root/app/phase1_monitor.c" \
  -o "$out"

"$out"

protocol_out="${TMPDIR:-/tmp}/dm-mc02-h6215-protocol-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_h6215_protocol.c" \
  "$root/motors/h6215/h6215_protocol.c" \
  -lm \
  -o "$protocol_out"

"$protocol_out"

motion_out="${TMPDIR:-/tmp}/dm-mc02-motion-controller-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_motion_controller.c" \
  "$root/motors/h6215/motion_controller.c" \
  -o "$motion_out"

"$motion_out"

usb_queue_out="${TMPDIR:-/tmp}/dm-mc02-usb-command-queue-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_usb_command_queue.c" \
  "$root/drivers/uart/usb_command_queue.c" \
  -o "$usb_queue_out"

"$usb_queue_out"

feedback_timing_out="${TMPDIR:-/tmp}/dm-mc02-feedback-timing-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_feedback_timing.c" \
  "$root/drivers/can_bus/feedback_timing.c" \
  -o "$feedback_timing_out"

"$feedback_timing_out"

motion_io_out="${TMPDIR:-/tmp}/dm-mc02-motion-io-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_motion_io.c" \
  "$root/drivers/uart/motion_io.c" \
  "$root/motors/h6215/motion_controller.c" \
  -o "$motion_io_out"

"$motion_io_out"

power_quiet_out="${TMPDIR:-/tmp}/dm-mc02-power-quiet-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_power_quiet_controller.c" \
  "$root/app/power_quiet_controller.c" \
  -o "$power_quiet_out"

"$power_quiet_out"

dm4310_protocol_out="${TMPDIR:-/tmp}/dm-mc02-dm4310-protocol-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_dm4310_protocol.c" \
  "$root/motors/dm4310/dm4310_protocol.c" \
  -lm \
  -o "$dm4310_protocol_out"

"$dm4310_protocol_out"

dm4310_controller_out="${TMPDIR:-/tmp}/dm-mc02-dm4310-controller-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_dm4310_controller.c" \
  "$root/motors/dm4310/dm4310_controller.c" \
  -o "$dm4310_controller_out"

"$dm4310_controller_out"

parallel_controller_out="${TMPDIR:-/tmp}/dm-mc02-parallel-controller-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" \
  "$root/tests/test_parallel_controller.c" \
  "$root/motors/motor_manager/parallel_controller.c" \
  -o "$parallel_controller_out"

"$parallel_controller_out"

motor_manager_out="${TMPDIR:-/tmp}/dm-mc02-motor-manager-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" -I"$root/drivers/can_bus" -I"$root/drivers/uart" -I"$root/motors/dm4310" -I"$root/motors/h6215" -I"$root/motors/motor_manager" -I"$root/safety/safety_manager" -I"$root/platform/inc" \
  "$root/tests/test_motor_manager.c" \
  "$root/motors/motor_manager/motor_manager.c" \
  "$root/safety/safety_manager/safety_manager.c" \
  "$root/motors/dm4310/dm4310_protocol.c" \
  "$root/motors/h6215/h6215_protocol.c" \
  "$root/drivers/can_bus/feedback_timing.c" \
  -lm \
  -o "$motor_manager_out"

"$motor_manager_out"

single_leg_out="${TMPDIR:-/tmp}/dm-mc02-single-leg-bringup-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app/single_leg_bringup" -I"$root/motors/motor_manager" \
  "$root/tests/test_single_leg_bringup.c" \
  "$root/app/single_leg_bringup/single_leg_bringup.c" \
  -o "$single_leg_out"

"$single_leg_out"

single_leg_trajectory_out="${TMPDIR:-/tmp}/dm-mc02-single-leg-trajectory-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app/single_leg_bringup" -I"$root/motors/motor_manager" \
  "$root/tests/test_single_leg_trajectory.c" \
  "$root/app/single_leg_bringup/single_leg_trajectory.c" \
  -o "$single_leg_trajectory_out"

"$single_leg_trajectory_out"

serial_logger_out="${TMPDIR:-/tmp}/dm-mc02-serial-logger-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/tests/stubs" -I"$root/drivers/uart" -I"$root/motors/h6215" \
  "$root/tests/test_serial_logger.c" \
  "$root/drivers/uart/serial_logger.c" \
  "$root/drivers/uart/motion_io.c" \
  -o "$serial_logger_out"

"$serial_logger_out"
