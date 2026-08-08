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
