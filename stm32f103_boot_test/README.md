# STM32F103 H6215 guarded velocity test

Minimal bare-metal firmware for a Blue Pill board:

- Uses the 8 MHz external crystal and runs the CPU at 72 MHz.
- Runs APB1 at 36 MHz and bxCAN at 1 Mbps.
- Uses PA11 as CAN RX and PA12 as CAN TX.
- Runs a silent internal bxCAN loopback test before entering normal mode.
- Toggles the active-low PC13 LED every 500 ms.
- Uses USART1 on PA9/PA10 at 115200 baud for logs and read-only commands:
  - `s`: print CAN error state, TEC, REC, and LEC.
  - `r`: read the H6215 software version.
  - `m`: read the active control mode.
  - `p`: read `P_MAX`.
  - `v`: read `V_MAX`.
  - `t`: read `T_MAX`.
  - `f`: print the most recently decoded motor feedback.
- Provides one deliberately constrained velocity-mode motion test:
  - `a`: arm motion for 10 seconds, only after mode 3 was read with `m`.
  - `g`: enable, run at `+0.2 rad/s` for one second, hold zero speed for
    200 ms, and disable.
  - `b`: enable, run at `-0.2 rad/s` for one second, hold zero speed for
    200 ms, and disable.
  - `x`: immediately command zero speed and disable.
- Provides a guarded interactive speed controller:
  - `a`, then `c`: start enabled at zero speed.
  - `+` / `-`: change target by `0.1 rad/s`, limited to `+/-0.5 rad/s`.
  - `0`: request zero speed.
  - `k`: refresh the host watchdog without changing the target.
  - No serial control input for 5 seconds ramps to zero and disables.
  - The command is sent every 10 ms and the speed ramp changes by
    `0.1 rad/s` every 100 ms.
- Targets the rated 64 KiB flash and 20 KiB RAM of STM32F103C8T6.

The read request is standard CAN ID `0x7FF` with:

```text
01 00 33 0E 00 00 00 00
```

No position or torque command is implemented. The velocity test is rejected
unless the controller has observed a mode-3 response from the motor after
boot.

## Build

```bash
cmake -S . -B build -G Ninja
cmake --build build
```

## Flash

Keep the H6215 24 V supply off. Flash while `BOOT0=1`, `BOOT1=0`:

```bash
stm32flash -b 115200 \
  -w build/stm32f103_boot_test.bin \
  -v /dev/cu.usbserial-140
```

The macOS device name can change after moving the adapter to another USB
port or hub. Run `ls /dev/cu.usbserial-*` and use the device currently shown.

After flashing, power the board off, set `BOOT0=0` and `BOOT1=0`, then
power it on again.

## Expected startup log

```text
H6215_VELOCITY_TEST_START
CLOCK_OK SYSCLK=72MHz APB1=36MHz
CAN_LOOPBACK_PASS
CAN_NORMAL_MODE_READY 1Mbps
READ: s=status, r=version, m=mode, p=P_MAX, v=V_MAX, t=T_MAX, f=last feedback
PULSE: a then g=+0.2rad/s or b=-0.2rad/s; x=stop
LIVE: a then c=start; +=faster, -=slower, 0=zero, k=keepalive
```

During motion, decoded feedback is printed at up to 10 Hz:

```text
FB ID=1 STATE=ENABLED P=0.123rad V=0.197rad/s T=0.005Nm TMOS=28C TROTOR=27C
```

The conversion constants assume the verified motor values
`P_MAX=12.5`, `V_MAX=45`, and `T_MAX=10`.

While a test is active, feedback protection stops and disables the motor if:

- the motor is no longer in state `ENABLED`;
- measured speed exceeds `0.8 rad/s` in either direction;
- MOS or rotor temperature reaches `60 C`.

## Wiring

```text
STM32 PA12 (CAN_TX) -> TJA1050 TXD
STM32 PA11 (CAN_RX)  <- TJA1050 RXD
STM32 5V             -> TJA1050 VCC
STM32 GND            -> TJA1050 GND
TJA1050 CANH         -> H6215 CANH
TJA1050 CANL         -> H6215 CANL
```

Keep the motor suspended. The CAN bus must have one 120 ohm terminator at
each physical end, measuring about 60 ohms between CANH and CANL while all
power is off.

Before sending `a` and `g`, rigidly secure the stationary side of the motor,
leave the rotor completely unobstructed, and keep the physical 24 V cutoff
within reach. Software stop is not a substitute for cutting motor power.
