# Single-Leg Kinematics

This directory contains the degenerate-five-bar model for Track A.

## Evidence status

- Link lengths and the equivalent symmetric topology are user-confirmed from CAD.
- Raw zero, direction, and limit values are verified as repository state from the
  previous physical bring-up Stage.
- The three symmetric CAD poses are geometrically self-consistent reference data.
- Phase 4 independent paired-pose validation was skipped by user decision on
  2026-08-11. No `rawA/rawB -> Hx/Hz` accuracy claim is made.
- Workspace results are model-derived and are not hardware verification.

## Verified model results

The maintained geometry is:

```text
l1 = l4 = 0.110 m
l2 = l3 = 0.132 m
l5 = 0 m
```

Both FK and raw-angle conversion tests pass. Sampling both software-limit
ranges at 181 points per joint produces 32,761 valid states:

```text
Hx = -37.33 .. +37.07 mm
Hz = -176.16 .. -93.07 mm
```

The three CAD reference poses and four intermediate poses agree with the
wheel-center heights used by the full-chassis MuJoCo model. This consistency
is between the two digital models; it is not an independent physical accuracy
measurement.

## Commands

```sh
python3 -m unittest -v test_forward_kinematics.py
python3 forward_kinematics.py 0 0
python3 plot_workspace.py
```
