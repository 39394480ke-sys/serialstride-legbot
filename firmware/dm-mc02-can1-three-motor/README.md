# DM-MC02 CAN1 three-motor firmware

This directory is the firmware base for the single-side three-motor CAN1
network. It contains the completed DM-MC02 bring-up and the guarded
three-motor controls for two DM4310 joints and one H6215 wheel. Motion is off
by default and every start requires an explicit arming command.

## Architecture

The active firmware is split by ownership rather than CAN ID branches in the
application entry point:

```text
drivers/
  can_bus/                 CAN transport and receive timing
  uart/                    USB command queue and non-blocking serial logger
motors/
  dm4310/                  DM4310 protocol and single-motor controller
  h6215/                   H6215 protocol and single-motor controller
  motor_manager/           unified states, routing, group commands, parallel control
safety/
  safety_manager/          single and parallel safety snapshots and global trips
app/
  three_motor_bringup/     command orchestration and telemetry
  single_leg_bringup/      calibration, guarded jogs, and pair trajectories
legacy/                    uncompiled historical bring-up entry points
```

`MotorState` is the single runtime representation for every motor. It owns
the motor type and role, CAN/MST IDs, online state, feedback values and age,
parameters, temperatures, and TX/RX counters. `MotorManager` is the only
module that parses incoming device frames or emits group commands.
`SafetyManager` derives controller snapshots from those unified states.

## Phase 1 behavior

- STM32H723VGT6 clocked from the board's 24 MHz HSE at 480 MHz SYSCLK;
- HAL SysTick provides the 1 kHz timebase;
- USB CDC emits boot markers and rate-limited health records;
- FDCAN1 uses PD0/PD1 in classic CAN mode at 1 Mbps;
- FDCAN1 starts in normal mode and uses only the documented H6215 read,
  feedback-probe, and guarded motion frames;
- the application always boots in `DEFAULT_STATE=DISABLED`.

Expected boot markers:

```text
MC02_BOOT
CLOCK_OK SYSCLK=480000000 HCLK=240000000
TIMER_OK TICK_HZ=1000
CAN1_INIT_OK BITRATE=1000000 TX=GUARDED_MOTION
H6215_GUARDED_MOTION COMMANDS=S,A,G,B,C,+,-,0,K,X LIMIT=+/-0.500rad/s
DEFAULT_STATE=DISABLED AUTO_MOTION=OFF
```

## Phase 2.1 behavior

- polls software version, control mode, `P_MAX`, `V_MAX`, and `T_MAX` using
  the read-register opcode only;
- sends an explicit Disable feedback probe every 200 ms only while the guarded
  controller is idle-disabled or armed and no recovery action is pending;
- parses state, position, velocity, torque, MOS temperature, and rotor
  temperature;
- keeps the probe path limited to parameter reads and the feedback-producing
  Disable command, with no catch-up probes after motion or fault states;
- marks the wheel offline when no valid response is received for one second.

Expected hardware status after the safe probe:

```text
[WHEEL] ONLINE=1 ID=1 MST_ID=0 STATE=DISABLED FB_AGE_MS=1 SW=5406 MODE=3
P_MAX=12.500 V_MAX=45.000 T_MAX=10.000 PARAM_MASK=0x1F
```

## Guarded motion controls

The current firmware accepts one ASCII command character at a time over USB
CDC:

- `S` requests all motor and CAN status;
- while power is off, `A`, then `P`, enables VCC_OUT1 in quiet mode;
- `R` starts the guarded three-device probe;
- `1`, `2`, and `3` select JOINT_A, JOINT_B, and WHEEL respectively;
- `4` selects all three motors for a bounded parallel test;
- after selection, `A` opens a 10-second one-shot motion arm;
- `G` and `B` request the bounded positive and negative tests;
- `X` has a priority path that zeros and Disables all motors and turns
  VCC_OUT1 off.

Both bounded pulses send velocity commands for 1000 ms, hold zero for 200 ms,
and then Disable. Continuous mode refreshes the command every 10 ms and ramps
one `0.100rad/s` step per 100 ms. If no `+`, `-`, `0`, or `K` arrives for five
seconds, it irreversibly ramps to zero, holds zero for 200 ms, and Disables.
Feedback freshness, enabled state, `0.800rad/s` overspeed, 60 C temperature,
CAN passive/bus-off, and transmit-failure checks remain active throughout.
`X` uses a priority path independent of a full ordinary USB command queue.
Parameter polling pauses while motion or fault shutdown is active.

## Single-leg bring-up commands

`APP_MODE_SINGLE_LEG_BRINGUP` adds lowercase line commands without replacing
the existing uppercase single-character interface. Send each command with CR,
LF, or CRLF. Uppercase `X`, standalone lowercase `x`, and the complete
`stop-all` word use the priority emergency path and flush queued commands.

Power and read-only setup:

```text
power-arm
power-on
probe
status
disable-all
stop-all
```

Selection, raw capture, and guarded jog:

```text
read-raw
capture-stand
capture-crouch
capture-extend
select-a
select-b
select-wheel
arm
jog-positive
jog-negative
phase10-arm
move-stand
move-mid-crouch
move-mid-extend
```

Phase 10 joint-pair commands require a fresh `phase10-arm` before every move.
They keep `WHEEL` disabled and interpolate both DM4310 raw-position targets
with the same four-second cubic progress `3r^2 - 2r^3`:

```text
STAND       A=+1.008 rad  B=-1.307 rad
MID_CROUCH  A=+0.835 rad  B=-1.129 rad
MID_EXTEND  A=+1.181 rad  B=-1.484 rad
```

The midpoint targets are explicitly recorded in `single_leg_calibration.h`,
remain inside the Phase 8 software limits, and do not imply validated
load-bearing poses. Here `CROUCH` and `EXTEND` mean observed mechanical-contact
endpoints, `STAND` means their temporary arithmetic midpoint, and
`MID_CROUCH`/`MID_EXTEND` are conservative suspended-motion targets. Any stale
feedback, CAN fault, wheel enable, joint state fault, software-limit crossing,
excessive speed, torque, or temperature causes Disable All and power-off.
`move-mid-crouch` and `move-mid-extend` additionally require both joints to
start within `0.100 rad` of STAND; arbitrary hand-positioned starts are
rejected.

`read-raw` records the MCU timestamp, role, CAN ID, online/state, raw position,
calibrated `q`, mechanical direction, velocity, torque, temperatures, feedback
timestamp, and feedback age. Pose captures require both joints to be online,
stationary, and Disabled; captures are stored in RAM only.

Each `arm` is one-shot and expires after 3 seconds. The first arm after a
selection or power/probe reset anchors a temporary `+/-0.100 rad`
commissioning envelope. Re-arming does not move that envelope. A jog uses
the role-specific commissioning profile in `single_leg_bringup.h`, then
commands zero and Disable. Feedback torque above `0.500 Nm`, stale feedback,
CAN faults, abnormal state, 60 C temperature, overspeed, or a position-limit
violation causes a fail-closed stop.

The 2026-08-10 mechanical-contact captures define calibrated global raw limits
with 10% of total travel reserved at each end: JOINT_A is `0.731..1.285 rad`
and JOINT_B is `-1.591..-1.022 rad`. Every joint arm and jog is constrained by
both its global range and the temporary commissioning envelope. Pulse duration
is target-limited, speed is reduced inside the final `0.050 rad`, and motion
toward a boundary is rejected inside the final `0.010 rad`. A manually
back-driven joint outside the range may move only toward the safe range.
Calibration data and the accepted test boundaries are in
`evidence/2026-08-10-single-leg-phase7-phase8.md`.

The 2026-08-10 suspended sequence completed in both directions with WHEEL
Disabled. This does not validate load bearing, stability, endurance, or
forward kinematics. Low-gain final tracking residuals of about `0.04..0.07 rad`
remain, and one isolated JOINT_B velocity sample of `+0.212 rad/s` was observed
during Disable before the next frame returned to normal.

## Build and test

STM32CubeIDE must be installed in `/Applications`. CMake discovers its bundled
GNU Arm toolchain so it does not accidentally use a Homebrew compiler without
Newlib.

```sh
tests/run_tests.sh
cmake -S . -B build -G Ninja
cmake --build build
```

The generated files are:

```text
build/dm_mc02_phase35_three_motor_parallel.elf
build/dm_mc02_phase35_three_motor_parallel.bin
build/dm_mc02_phase35_three_motor_parallel.map
```

## Flash and observe

Keep the USB-C cable connected for board power and USB CDC. Connect ST-Link to
the four-pin SWD connector using `PA13/SWDIO`, `PA14/SWCLK`, `GND`, and the
appropriate target-voltage reference.

Use STM32CubeProgrammer to write the binary at `0x08000000`, enable verify, and
reset the MCU. The USB CDC port currently enumerates as an STM32 Virtual
ComPort; its `/dev/cu.usbmodem*` suffix may change.

```sh
screen /dev/cu.usbmodemXXXXXXXX 115200
```

For [serial.baud-dance.com](https://serial.baud-dance.com/), use these exact
settings:

```text
Port: STM32 Virtual ComPort (cu.usbmodem...)
Baud rate: 115200
Data bits: 8
Parity: None
Stop bits: 1
Flow control: None / Off
Receive format: ASCII
Send format: ASCII
HEX: Off
Automatic send / Loop send: Off
Line ending: None, CRLF, or LF
```

USB CDC does not actually depend on the selected baud rate; `115200` 8N1
matches the prior F103 bench. Send one command character at a time: `S`, `A`,
wait for `MOTION_ARMED EXPIRES_MS=10000`, then send exactly one of `G`, `B`, or
`C`. In continuous mode, send `+`, `-`, `0`, or `K` as individual ASCII
characters. A start command without a fresh `A` is rejected.

## Source provenance

The board configuration follows the public DM-MC02 V1.1 manual and reference
projects for the STM32H723VGT6, 24 MHz HSE, USB device port, and FDCAN1 pins.
CMSIS, STM32H7 HAL, startup, USB Device Library, and generated USB glue retain
their upstream STMicroelectronics and Arm license files under `vendor/`.
Project-specific application and board integration code is original to this
repository.

`platform/mc02-usb-reference.ioc` preserves the upstream USB pin/clock
reference only. The checked-in CMake sources are authoritative for this Phase
1 build, including the independently integrated FDCAN1 configuration.

## Bring-up evidence

The hardware verification records and compressed USB CDC logs are in
[`docs/bring-up/dm-mc02/`](../../docs/bring-up/dm-mc02/README.md). The physical
power-cycle check is intentionally tracked separately from debugger reset.
