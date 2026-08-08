# DM-MC02 CAN1 three-motor firmware

This directory is the firmware base for the single-side three-motor CAN1
network. It contains the completed DM-MC02 Phase 1 bring-up and the Phase 2.1
H6215 parameter/feedback probe, plus the Phase 2.2a guarded positive-velocity
test. Motion is off by default and is available only through the explicit
`A`, then `G`, serial command sequence.

## Phase 1 behavior

- STM32H723VGT6 clocked from the board's 24 MHz HSE at 480 MHz SYSCLK;
- HAL SysTick provides the 1 kHz timebase;
- USB CDC emits boot markers and 100 ms health records;
- FDCAN1 uses PD0/PD1 in classic CAN mode at 1 Mbps;
- FDCAN1 starts in normal mode but transmits no frames;
- the application always boots in `DEFAULT_STATE=DISABLED`.

Expected boot markers:

```text
MC02_BOOT
CLOCK_OK SYSCLK=480000000 HCLK=240000000
TIMER_OK TICK_HZ=1000
CAN1_INIT_OK BITRATE=1000000 TX=GUARDED_MOTION
H6215_GUARDED_MOTION COMMANDS=S,A,G,X TARGET=+0.200rad/s
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

## Phase 2.2a guarded motion test

The firmware accepts one ASCII command character at a time over USB CDC:

- `S` requests the current wheel and health status;
- `A` opens a 10-second arming window;
- `G` requests the guarded `+0.200rad/s` test only while freshly armed and
  while all safety checks pass;
- `X` requests an immediate zero-speed and Disable shutdown.

The bounded test sends positive velocity commands for 1000 ms, holds zero
speed for 200 ms, and then sends Disable. Enable, positive velocity, zero
velocity, and Disable are the only motion-path CAN actions. Parameter polling
pauses while motion or fault shutdown is active so it cannot contend with the
10 ms control frames. A CAN transmit failure enters the controller's zero and
Disable recovery path. An idle feedback-probe transmit failure is counted but
does not enter that motion-fault path; a later `G` still requires fresh
feedback and active CAN. `G` without a fresh `A` is rejected.

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
build/dm_mc02_phase1.elf
build/dm_mc02_phase1.bin
build/dm_mc02_phase1.map
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
wait for `MOTION_ARMED EXPIRES_MS=10000`, then `G`. `G` without a fresh `A` is
rejected.

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
