# Single-leg mechanical calibration and suspended-motion evidence

Date: 2026-08-10

## Scope and observation basis

- The operator identified and captured the fully crouched and fully extended
  mechanical-contact poses. The raw photos are retained privately because
  they include desktop, screen, and purchase information; they are not part
  of this public repository.
- Both endpoint captures used 100 samples while JOINT_A, JOINT_B, and WHEEL
  remained Disabled. At the logged 0.001 rad resolution, each endpoint's raw
  samples were constant.
- STAND is defined for this stage as the arithmetic midpoint of the two raw
  endpoint captures. It is not a physically captured, load-tested, or
  stability-validated standing pose.

## Mechanical calibration

| Pose | JOINT_A raw (rad) | JOINT_B raw (rad) | qA (rad) | qB (rad) | Source |
| --- | ---: | ---: | ---: | ---: | --- |
| CROUCH | +0.662 | -0.951 | -0.346 | -0.356 | 100-sample mechanical-contact capture |
| STAND | +1.008 | -1.307 | 0.000 | 0.000 | Computed endpoint midpoint |
| EXTEND | +1.354 | -1.662 | +0.346 | +0.355 | 100-sample mechanical-contact capture |

The direction convention is mechanical extension positive:

```text
qA = +(A_raw - 1.008)
qB = -(B_raw + 1.307)
```

## Software limits

A conservative margin equal to 10% of total captured travel is reserved at
each mechanical end. This leaves 80% of the captured travel commandable.

| Joint | Mechanical raw range (rad) | Per-end margin (rad) | Software raw range (rad) | Software q range (rad) |
| --- | --- | ---: | --- | --- |
| JOINT_A | +0.662 to +1.354 | 0.069 | +0.731 to +1.285 | -0.277 to +0.277 |
| JOINT_B | -1.662 to -0.951 | 0.071 | -1.591 to -1.022 | -0.285 to +0.284 |

The global software ranges are enforced before and during joint jogs. The
existing per-arm 0.100 rad commissioning window remains active and is clamped
to these global limits. The commanded pulse duration is shortened so its
worst-case velocity target remains inside the active window. Inside the final
0.050 rad before a software limit, velocity is reduced proportionally; inside
the final 0.010 rad, motion farther toward the limit is rejected. If manual
back-driving starts outside a software limit, only motion toward the safe range
is permitted. A boundary violation in the dangerous direction requests STOP
ALL.

The authoritative compile-time values are separated from the command logic in
[`single_leg_calibration.h`](../app/single_leg_bringup/single_leg_calibration.h).
Mechanical-contact endpoints, software limits, and suspended-motion targets
are deliberately named as different categories.

## Suspended low-speed coordinated motion

The guarded pair trajectory was run through the complete accepted sequence
using these targets:

| Target | JOINT_A raw (rad) | JOINT_B raw (rad) | Meaning |
| --- | ---: | ---: | --- |
| `MID_CROUCH` | +0.835 | -1.129 | Midpoint between temporary STAND and CROUCH contact |
| `STAND` | +1.008 | -1.307 | Arithmetic endpoint midpoint; temporary reference only |
| `MID_EXTEND` | +1.181 | -1.484 | Midpoint between temporary STAND and EXTEND contact |

The operator confirmed that JOINT_A and JOINT_B moved in the intended
mechanical direction, smoothly, and without abnormal sound. WHEEL remained
Disabled throughout. No CAN loss of control or safety abort was observed.
MCU restart still defaults to Disabled, and the existing STOP ALL, explicit
arming, timeout, feedback freshness, software-limit, and fault checks remain
active.

Low-gain final tracking residuals of approximately `0.04..0.07 rad` were
observed and accepted for this suspended bring-up. On one Disable transition,
JOINT_B briefly reported `+0.212 rad/s`; the next frame returned to the normal
state. The cause has not been established.

## Power-cycle observation

A physical 24 V power-cycle repeatability check returned the same raw pair in
both accepted readings:

```text
R0: JOINT_A=+1.054 rad, JOINT_B=-1.367 rad
R1: JOINT_A=+1.054 rad, JOINT_B=-1.367 rad
```

This is a two-reading observation at one mechanism pose. It is not a fixture-
based repeatability study and does not replace the explicitly waived three-run
mechanical calibration.

## Accepted exceptions and unverified boundaries

- STAND mechanical repeatability without a fixture.
- Suitability of the computed STAND pose for load bearing or vehicle stability.
- Three repeated mechanical calibrations and ten consecutive coordinated
  cycles were explicitly waived for this stage.
- Long-duration endurance, load bearing, stability, and forward kinematics
  remain unverified.
- The isolated JOINT_B Disable-transition velocity sample and low-gain final
  tracking residuals remain unresolved.
- The original serial transcript was not committed because no complete,
  sanitized local export was available. This record contains the accepted
  engineering conclusions only.
