# Full-Chassis MuJoCo Model

This directory contains the first stable geometric-debug model for Track B.

## Files

- `robot_source.xml`: unmodified MJCF from the local CAD export.
- `robot.xml`: curated MuJoCo model used for geometric debugging.
- `meshes/`: unmodified per-link STL meshes from the export.
- `user_model.json`, `parts.json`, `mjcf_export_report.json`: unmodified export metadata.
- `validate_model.py`: static and MuJoCo 3.9 runtime checks.

## Evidence status

- MuJoCo can compile both the source and curated MJCF with the bundled meshes.
- The declared link masses sum to `3.76954478618 kg`.
- The user confirmed that the simplified CAD model has the intended total weight.
- Individual link mass allocation, centers of mass, and inertias have not been
  independently validated.
- The gas-spring links are passive geometry only. No spring force model is present.
- Joint A/B identity, left/right mapping, wheel-axis alignment, gas-spring
  connections, and all seven keyframes were visually confirmed by the user.

## Curated model conventions

The CAD export axes do not match the maintained robot frame despite its metadata
label. After visual checks in MuJoCo, `robot.xml` applies a combined `+90 deg`
rotation about X and `180 deg` rotation about Z so the maintained convention is:

```text
+X: forward
+Y: left
+Z: up
```

The user visually confirmed the final axis requirements as `+X` forward and
`+Z` up. The user-confirmed side mapping is: the `link_002/link_003` branch is
the left leg and the `link_014/link_015` branch is the right leg. The chassis is
fixed to the world and gravity is zero for this geometric-debug phase.

## Open in MuJoCo

Drag `robot.xml` into MuJoCo, or run:

```sh
/Applications/MuJoCo.app/Contents/MacOS/simulate \
  "$(pwd)/simulation/mujoco/full_chassis/robot.xml"
```

The curated window title is `MuJoCo : serial_stride_full_chassis_debug`, which
distinguishes it from the untouched `generated_robot` source model.

The Control panel intentionally contains only four leg inputs:

| Control | CAD link | Side and input |
| --- | --- | --- |
| `left_joint_a_position` | `link_002` | left A |
| `left_joint_b_position` | `link_003` | left B |
| `right_joint_a_position` | `link_014` | right A |
| `right_joint_b_position` | `link_015` | right B |

The active interface is frozen as `left_joint_a`, `left_joint_b`,
`right_joint_a`, and `right_joint_b`; wheel axes are
`left_wheel_joint` and `right_wheel_joint`. The A/B and left/right mapping was
user-observed in the MuJoCo viewer. The joint axes are normalized so positive
control means extension on all four inputs:

```text
left:  qA = -exported native A, qB = +exported native B
right: qA = +exported native A, qB = -exported native B
```

The CAD assembly was exported at an active-link angle of `33.065472 deg`, while
the Track A mechanical-limit midpoint is `25.484200 deg`. The four active
joints therefore use a `0.132318159 rad` reference offset. After this offset,
control and qpos zero are `CALIB_MID` and correspond to Track A `qA = qB = 0`.
In the Track A X-Z plane, positive qA rotates AG counterclockwise and positive
qB rotates AB clockwise. Both positive coordinates extend the leg.

The control ranges now use the previous physical bring-up mechanical limits:
`qA = [-0.346, 0.346] rad` and `qB = [-0.356, 0.355] rad`. Track A software
limits remain narrower and should be used by future normal controllers.

The user currently treats `CALIB_MID` as a normal standing pose. It is not named
`NOMINAL_STAND` here because support force, Jacobian, VMC, and gas-spring working
range have not yet been evaluated.

The two wheel joints remain continuous and unactuated, so they do not appear in
the Control panel.

Seven named keyframes are available in this order:

```text
0: CALIB_MID
1: CROUCH
2: EXTEND
3: INTERMEDIATE_N025   (qA = qB = -0.250 rad)
4: INTERMEDIATE_N0125  (qA = qB = -0.125 rad)
5: INTERMEDIATE_P0125  (qA = qB = +0.125 rad)
6: INTERMEDIATE_P025   (qA = qB = +0.250 rad)
```

Enter the key number in the Simulation panel and select `Load key`. `CROUCH`
and `EXTEND` use the symmetric CAD reference angles from Track A. The four
intermediate poses are model-derived and are automatically checked against the
Track A FK wheel position. These are geometry-validation poses, not normal
command targets for hardware.

## Sites and frames

In the desktop viewer:

1. Open `Group enable` and enable site group 3 (`Shift+3`).
2. Open `Rendering` and set `Label` to `Site`.
3. Set `Frame` to `World` for the global axes or `Site` for site frames.
4. `F6` cycles frame visualization.

Blue sites are loop-closure anchors, red sites are left/right base references,
and green sites are wheel centers. The Sensor panel exposes the wheel-center
position relative to the corresponding base reference.

## Validation

Run:

```sh
python3 simulation/mujoco/full_chassis/validate_model.py
```

The check loads the model through the MuJoCo 3.9 framework bundled in
`/Applications/MuJoCo.app`, verifies model structure and initial equality
residuals, and runs bounded single-joint and paired two-second step tests.

The accepted model contains 25 bodies, 24 joints, 6 equality/connect
constraints, and 4 actuators, with a declared total mass of
`3.76954478618 kg`. Across the seven keyframes, closed-chain residuals are on
the order of `1e-9 m`; the largest residual velocity in the bounded step tests
is approximately `2.01e-6 rad/s`. Track A and Track B agree on the three CAD
reference-pose heights and four intermediate-pose heights.

This model does not establish ground contact, standing control, real motor
dynamics, gas-spring dynamics, or Track A physical FK accuracy. The user's
observation that the overall weight is correct does not independently validate
per-link mass allocation, centers of mass, or inertia tensors.
