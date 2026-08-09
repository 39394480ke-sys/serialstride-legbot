# Phase 3.5 Three-Motor Parallel Validation

Date: 2026-08-09

## Verified software facts

- Firmware mode: `PHASE35_THREE_MOTOR_PARALLEL`.
- CAN1 devices: JOINT_A ID 6/MST 3, JOINT_B ID 8/MST 4, and H6215 ID 1/MST 0.
- Parallel command sequence: `4`, `A`, `G`.
- The bounded pulse commands both DM4310 motors at +0.4 rad/s and H6215 at +0.2 rad/s for 1 second, then commands zero and waits for all three Disabled confirmations.
- Host tests, a clean ARM build, and `git diff --check` passed before flashing.
- STM32CubeProgrammer download verification passed.
- Flashed binary SHA256: `7f387f05c9872602ba066715058fd438a32a3ad23e40be891c9c37439fbe6d72`.

## User observation

- The three-motor parallel `G` test passed.
- All three motors moved concurrently in the expected counterclockwise direction and completed the bounded motion.

## Not verified in this test

- Parallel `B` direction was not tested.
- No serial log export was supplied for this run.
- Long-duration parallel motion, loaded operation, and stability soak were not tested.
