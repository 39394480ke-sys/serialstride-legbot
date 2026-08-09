#!/bin/sh
set -eu

root=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
out="${TMPDIR:-/tmp}/dm-mc02-phase1-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" \
  "$root/tests/test_phase1_monitor.c" \
  "$root/app/phase1_monitor.c" \
  -o "$out"

"$out"

protocol_out="${TMPDIR:-/tmp}/dm-mc02-h6215-protocol-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" \
  "$root/tests/test_h6215_protocol.c" \
  "$root/app/h6215_protocol.c" \
  -lm \
  -o "$protocol_out"

"$protocol_out"

motion_out="${TMPDIR:-/tmp}/dm-mc02-motion-controller-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" \
  "$root/tests/test_motion_controller.c" \
  "$root/app/motion_controller.c" \
  -o "$motion_out"

"$motion_out"

usb_queue_out="${TMPDIR:-/tmp}/dm-mc02-usb-command-queue-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" \
  "$root/tests/test_usb_command_queue.c" \
  "$root/app/usb_command_queue.c" \
  -o "$usb_queue_out"

"$usb_queue_out"

feedback_timing_out="${TMPDIR:-/tmp}/dm-mc02-feedback-timing-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" \
  "$root/tests/test_feedback_timing.c" \
  "$root/app/feedback_timing.c" \
  -o "$feedback_timing_out"

"$feedback_timing_out"

motion_io_out="${TMPDIR:-/tmp}/dm-mc02-motion-io-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" \
  "$root/tests/test_motion_io.c" \
  "$root/app/motion_io.c" \
  "$root/app/motion_controller.c" \
  -o "$motion_io_out"

"$motion_io_out"

power_quiet_out="${TMPDIR:-/tmp}/dm-mc02-power-quiet-tests"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/app" \
  "$root/tests/test_power_quiet_controller.c" \
  "$root/app/power_quiet_controller.c" \
  -o "$power_quiet_out"

"$power_quiet_out"
