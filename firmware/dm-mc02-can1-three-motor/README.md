# DM-MC02 CAN1 three-motor firmware

This directory is the firmware base for the single-side three-motor CAN1
network. It contains the completed DM-MC02 Phase 1 bring-up and the Phase 2.1
H6215 parameter/feedback probe. It does not contain enable, velocity, torque,
or other motion commands.

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
CAN1_INIT_OK BITRATE=1000000 TX=READ_AND_DISABLE_ONLY
H6215_SAFE_PROBE CAN_ID=1 MST_ID=0 ENABLE=ABSENT MOTION=ABSENT DISABLE=ONCE
MAIN_LOOP_RUNNING DEFAULT_STATE=DISABLED
```

## Phase 2.1 behavior

- polls software version, control mode, `P_MAX`, `V_MAX`, and `T_MAX` using
  the read-register opcode only;
- waits until all five parameters are valid, then sends exactly one explicit
  Disable command to request a feedback frame while enforcing zero output;
- parses state, position, velocity, torque, MOS temperature, and rotor
  temperature;
- never implements or sends Enable, velocity, torque, or position commands;
- marks the wheel offline when no valid response is received for one second.

Expected hardware status after the safe probe:

```text
[WHEEL] ONLINE=1 ID=1 MST_ID=0 STATE=DISABLED SW=5406 MODE=3
P_MAX=12.500 V_MAX=45.000 T_MAX=10.000 PARAM_MASK=0x1F
```

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

USB CDC ignores the selected baud rate, but `115200` keeps the command
consistent with the earlier bench workflow.

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
