#!/usr/bin/env python3
"""Forward kinematics for the single-leg degenerate five-bar linkage."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml


class KinematicsError(ValueError):
    """Raised when the requested joint state has no unique closed-chain solution."""


_DIRECTORY = Path(__file__).resolve().parent


def _load_yaml(name: str) -> dict[str, Any]:
    with (_DIRECTORY / name).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise KinematicsError(f"{name} must contain a YAML mapping")
    return data


_GEOMETRY = _load_yaml("geometry.yaml")
_CALIBRATION = _load_yaml("calibration.yaml")


def raw_to_joint_angles(raw_a: float, raw_b: float) -> tuple[float, float]:
    """Convert raw motor angles to extension-positive relative joint angles."""
    joints = _CALIBRATION["joints"]
    q_a = float(joints["joint_a"]["direction"]) * (
        raw_a - float(joints["joint_a"]["zero_raw"])
    )
    q_b = float(joints["joint_b"]["direction"]) * (
        raw_b - float(joints["joint_b"]["zero_raw"])
    )
    return q_a, q_b


def _circle_intersections(
    center_a: tuple[float, float],
    radius_a: float,
    center_b: tuple[float, float],
    radius_b: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    dx = center_b[0] - center_a[0]
    dz = center_b[1] - center_a[1]
    distance = math.hypot(dx, dz)
    tolerance = 1e-12

    if distance <= tolerance:
        raise KinematicsError("distal-circle centers coincide; the solution is not unique")
    if distance > radius_a + radius_b + tolerance:
        raise KinematicsError("distal links cannot reach each other")
    if distance < abs(radius_a - radius_b) - tolerance:
        raise KinematicsError("one distal-link circle lies inside the other")

    along = (
        radius_a * radius_a
        - radius_b * radius_b
        + distance * distance
    ) / (2.0 * distance)
    height_squared = radius_a * radius_a - along * along
    if height_squared < -tolerance:
        raise KinematicsError("closed-chain solution is numerically invalid")
    height = math.sqrt(max(0.0, height_squared))

    unit_x = dx / distance
    unit_z = dz / distance
    base_x = center_a[0] + along * unit_x
    base_z = center_a[1] + along * unit_z
    perpendicular_x = -unit_z
    perpendicular_z = unit_x

    return (
        (base_x + height * perpendicular_x, base_z + height * perpendicular_z),
        (base_x - height * perpendicular_x, base_z - height * perpendicular_z),
    )


def forward_kinematics(q_a: float, q_b: float) -> dict[str, float]:
    """Return wheel position and equivalent-leg state for extension-positive q."""
    links = _GEOMETRY["links"]
    mapping = _CALIBRATION["planar_angle_mapping"]

    if not math.isclose(float(links["l5"]), 0.0, abs_tol=1e-12):
        raise KinematicsError("this implementation requires the degenerate l5 = 0 model")

    angle_a = (
        float(mapping["joint_a"]["angle_at_calib_mid"])
        + float(mapping["joint_a"]["q_sign"]) * q_a
    )
    angle_b = (
        float(mapping["joint_b"]["angle_at_calib_mid"])
        + float(mapping["joint_b"]["q_sign"]) * q_b
    )

    proximal_a = float(links["l1"])
    distal_a = float(links["l2"])
    distal_b = float(links["l3"])
    proximal_b = float(links["l4"])

    knee_a = (
        proximal_a * math.cos(angle_a),
        proximal_a * math.sin(angle_a),
    )
    knee_b = (
        proximal_b * math.cos(angle_b),
        proximal_b * math.sin(angle_b),
    )
    candidates = _circle_intersections(knee_a, distal_a, knee_b, distal_b)

    branch = _GEOMETRY.get("assembly_branch")
    if branch != "lower_z":
        raise KinematicsError(f"unsupported assembly branch: {branch!r}")
    wheel_x, wheel_z = min(candidates, key=lambda point: point[1])

    leg_length = math.hypot(wheel_x, wheel_z)
    leg_angle = math.atan2(wheel_x, -wheel_z)
    return {
        "wheel_x": wheel_x,
        "wheel_z": wheel_z,
        "leg_length": leg_length,
        "leg_angle": leg_angle,
    }


def _main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("q_a", type=float, help="JOINT_A relative angle in radians")
    parser.add_argument("q_b", type=float, help="JOINT_B relative angle in radians")
    args = parser.parse_args()
    print(json.dumps(forward_kinematics(args.q_a, args.q_b), indent=2))


if __name__ == "__main__":
    _main()
