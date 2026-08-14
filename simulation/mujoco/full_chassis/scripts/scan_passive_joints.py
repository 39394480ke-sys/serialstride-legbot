#!/usr/bin/env python3
"""Scan both legs over the software-safe rotation/shape grid in MuJoCo."""

from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from joint_limits import joint_qpos_indices, load_config
from sync_inertials import DYNAMICS_PATH, ROOT, require

sys.path.insert(0, str(ROOT))
from validate_model import MuJoCo, _numbers_in_section, _variant  # noqa: E402


RESULTS = ROOT / "results"
ROTATION_GRID_SIZE = 25
SHAPE_GRID_SIZE = 21
SETTLE_STEPS = 2000
RESIDUAL_LIMIT_M = 1e-6
VELOCITY_LIMIT = 1e-4
RADIUS_SPAN_LIMIT_M = 1e-7
MODAL_TRACKING_LIMIT_RAD = 1e-6


def linspace(start: float, stop: float, count: int) -> list[float]:
    return [start + (stop - start) * index / (count - 1) for index in range(count)]


def wrapped_angle_error(actual: float, expected: float) -> float:
    return math.atan2(math.sin(actual - expected), math.cos(actual - expected))


def main() -> None:
    config = load_config()
    tree = ET.parse(DYNAMICS_PATH)
    root = tree.getroot()
    joint_names = [
        joint.get("name", "") for joint in root.findall("./worldbody//joint")
    ]
    indices = joint_qpos_indices(root)
    sensors = [node.get("name", "") for node in root.findall("./sensor/*")]
    require(
        sensors == ["right_wheel_position", "left_wheel_position"],
        f"unexpected sensor order: {sensors}",
    )
    passive = {
        side: list(config["passive_hinges"][side]) for side in ("left", "right")
    }
    slides = {
        side: config["gas_spring_slides"][side]["joint"]
        for side in ("left", "right")
    }
    modal = config["modal_coordinates"]
    rotation_values = linspace(
        *modal["rotation"]["observation_range"], ROTATION_GRID_SIZE
    )
    shape_values = linspace(*modal["shape"]["software_range"], SHAPE_GRID_SIZE)
    runtime = MuJoCo()
    rows: list[dict[str, object]] = []
    observed: dict[str, list[float]] = defaultdict(lambda: [math.inf, -math.inf])
    radii_by_shape: dict[tuple[str, int], list[float]] = defaultdict(list)
    maximum_residual = 0.0
    maximum_velocity = 0.0
    maximum_rotation_error = 0.0
    maximum_shape_error = 0.0
    continuous_indices = {indices[name] for name in config["continuous_joints"]}

    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        for side in ("left", "right"):
            for shape_index, target_shape in enumerate(shape_values):
                for target_rotation in rotation_values:
                    q_a = target_shape + target_rotation
                    q_b = target_shape - target_rotation
                    controls = (
                        (target_rotation, target_shape, 0.0, 0.0)
                        if side == "left"
                        else (0.0, 0.0, target_rotation, target_shape)
                    )
                    variant = ET.ElementTree(ET.fromstring(ET.tostring(root)))
                    path = _variant(variant, controls, directory)
                    text = runtime.formatted_data(
                        path, steps=SETTLE_STEPS, keyframe=0
                    )
                    qpos = _numbers_in_section(text, "QPOS")
                    qvel = _numbers_in_section(text, "QVEL")
                    residuals = _numbers_in_section(text, "EFC_POS")
                    sensor_values = _numbers_in_section(text, "SENSOR")
                    require(len(sensor_values) == 6, "unexpected sensor output")
                    velocity = max(
                        (
                            abs(value)
                            for index, value in enumerate(qvel)
                            if index not in continuous_indices
                        ),
                        default=0.0,
                    )
                    residual = max(map(abs, residuals), default=0.0)
                    maximum_velocity = max(maximum_velocity, velocity)
                    maximum_residual = max(maximum_residual, residual)
                    require(all(math.isfinite(value) for value in qpos), "non-finite qpos")
                    require(
                        velocity < VELOCITY_LIMIT,
                        f"{side} {target_rotation=} {target_shape=}: not settled",
                    )
                    require(
                        residual < RESIDUAL_LIMIT_M,
                        f"{side} {target_rotation=} {target_shape=}: residual {residual}",
                    )
                    wheel = sensor_values[3:6] if side == "left" else sensor_values[0:3]
                    other = sensor_values[0:3] if side == "left" else sensor_values[3:6]
                    actual_q_a = qpos[indices[f"{side}_joint_a"]]
                    actual_q_b = qpos[indices[f"{side}_joint_b"]]
                    actual_rotation = (actual_q_a - actual_q_b) / 2
                    actual_shape = (actual_q_a + actual_q_b) / 2
                    wheel_radius = math.hypot(wheel[0], wheel[2])
                    wheel_angle = math.atan2(wheel[0], -wheel[2])
                    rotation_error = wrapped_angle_error(
                        wheel_angle, target_rotation
                    )
                    shape_error = actual_shape - target_shape
                    maximum_rotation_error = max(
                        maximum_rotation_error, abs(rotation_error)
                    )
                    maximum_shape_error = max(maximum_shape_error, abs(shape_error))
                    radii_by_shape[(side, shape_index)].append(wheel_radius)
                    result: dict[str, object] = {
                        "side": side,
                        "target_rotation_rad": target_rotation,
                        "target_shape_rad": target_shape,
                        "q_a_command_rad": q_a,
                        "q_b_command_rad": q_b,
                        "actual_rotation_rad": actual_rotation,
                        "actual_shape_rad": actual_shape,
                        "rotation_tracking_error_rad": rotation_error,
                        "shape_tracking_error_rad": shape_error,
                        "wheel_hx_m": wheel[0],
                        "wheel_hz_m": wheel[2],
                        "wheel_radius_m": wheel_radius,
                        "wheel_angle_rad": wheel_angle,
                        "other_wheel_hx_m": other[0],
                        "other_wheel_hz_m": other[2],
                        "gas_slide_q_m": qpos[indices[slides[side]]],
                        "max_abs_nonwheel_qvel": velocity,
                        "max_abs_equality_residual_m": residual,
                    }
                    for name in passive[side]:
                        value = qpos[indices[name]]
                        result[f"q_{name}_rad"] = value
                        observed[name][0] = min(observed[name][0], value)
                        observed[name][1] = max(observed[name][1], value)
                    slide_value = result["gas_slide_q_m"]
                    observed[slides[side]][0] = min(observed[slides[side]][0], slide_value)
                    observed[slides[side]][1] = max(observed[slides[side]][1], slide_value)
                    rows.append(result)

    radius_spans = {
        f"{side}:shape_index_{shape_index}": max(values) - min(values)
        for (side, shape_index), values in radii_by_shape.items()
    }
    maximum_radius_span = max(radius_spans.values(), default=0.0)
    require(
        maximum_radius_span <= RADIUS_SPAN_LIMIT_M,
        f"constant-shape radius span {maximum_radius_span} exceeds limit",
    )
    require(
        maximum_rotation_error <= MODAL_TRACKING_LIMIT_RAD,
        f"rotation tracking error {maximum_rotation_error} exceeds limit",
    )
    require(
        maximum_shape_error <= MODAL_TRACKING_LIMIT_RAD,
        f"shape tracking error {maximum_shape_error} exceeds limit",
    )

    padding_fraction = config["passive_hinges"][
        "required_padding_fraction_of_observed_span"
    ]
    passive_report = {}
    stale_passive_ranges = {}
    for side in ("left", "right"):
        for name, settings in config["passive_hinges"][side].items():
            low, high = observed[name]
            span = high - low
            padding = span * padding_fraction
            model_low, model_high = settings["model_range"]
            sufficient = low - padding >= model_low and high + padding <= model_high
            require(sufficient, f"{name}: model range lacks 15% scan padding")
            recorded = settings.get("scan_observed_range")
            require(recorded is not None, f"{name}: scan range is not recorded in YAML")
            if not (
                math.isclose(low, recorded[0], abs_tol=1e-8)
                and math.isclose(high, recorded[1], abs_tol=1e-8)
            ):
                stale_passive_ranges[name] = [low, high]
            passive_report[name] = {
                "side": side,
                "observed_range_rad": [low, high],
                "observed_span_rad": span,
                "required_padding_each_side_rad": padding,
                "model_range_rad": settings["model_range"],
                "padding_check": "PASS",
                "physical_limit_status": "UNVERIFIED",
            }
    require(
        not stale_passive_ranges,
        "recorded YAML passive ranges are stale: "
        + json.dumps(stale_passive_ranges, sort_keys=True),
    )

    slide_report = {}
    for side in ("left", "right"):
        name = slides[side]
        low, high = observed[name]
        allowed_low, allowed_high = config["gas_spring_slides"][side]["range"]
        within = low >= allowed_low - 1e-9 and high <= allowed_high + 1e-9
        touches = low <= allowed_low + 1e-6 or high >= allowed_high - 1e-6
        require(within, f"{name}: scan left configured slide range")
        require(not touches, f"{name}: software scan touches a slide limit")
        slide_report[name] = {
            "side": side,
            "observed_range_m": [low, high],
            "configured_range_m": [allowed_low, allowed_high],
            "range_check": "PASS_NO_LIMIT_CONTACT",
        }

    RESULTS.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0])
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with (RESULTS / "modal_scan.csv").open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    keyframe_report = []
    for index, key in enumerate(root.findall("./keyframe/key")):
        text = runtime.formatted_data(DYNAMICS_PATH, keyframe=index)
        qpos = _numbers_in_section(text, "QPOS")
        sensor_values = _numbers_in_section(text, "SENSOR")
        residual = max(map(abs, _numbers_in_section(text, "EFC_POS")), default=0.0)
        keyframe_report.append(
            {
                "name": key.get("name"),
                "left_wheel_hx_hz_m": [sensor_values[3], sensor_values[5]],
                "right_wheel_hx_hz_m": [sensor_values[0], sensor_values[2]],
                "left_slide_q_m": qpos[indices[slides["left"]]],
                "right_slide_q_m": qpos[indices[slides["right"]]],
                "qpos_by_joint": {
                    name: qpos[indices[name]] for name in joint_names
                },
                "max_abs_equality_residual_m": residual,
            }
        )

    report = {
        "schema_version": 1,
        "status": "PASS",
        "grid": {
            "coordinates": ["rotation", "shape"],
            "size_per_leg": [ROTATION_GRID_SIZE, SHAPE_GRID_SIZE],
            "samples_per_leg": ROTATION_GRID_SIZE * SHAPE_GRID_SIZE,
            "total_samples": len(rows),
            "rotation_range_rad": modal["rotation"]["observation_range"],
            "shape_range_rad": modal["shape"]["software_range"],
            "active_domain": "full_rotation_observation_x_software_safe_shape",
            "settle_steps": SETTLE_STEPS,
        },
        "sensor_order_verified": ["right_wheel_position", "left_wheel_position"],
        "maximum_abs_nonwheel_qvel": maximum_velocity,
        "maximum_equality_residual_m": maximum_residual,
        "modal_tracking": {
            "maximum_abs_rotation_error_rad": maximum_rotation_error,
            "maximum_abs_shape_error_rad": maximum_shape_error,
            "maximum_constant_shape_radius_span_m": maximum_radius_span,
            "rotation_error_limit_rad": MODAL_TRACKING_LIMIT_RAD,
            "shape_error_limit_rad": MODAL_TRACKING_LIMIT_RAD,
            "radius_span_limit_m": RADIUS_SPAN_LIMIT_M,
            "radius_spans_by_shape": radius_spans,
        },
        "passive_hinges": passive_report,
        "gas_spring_slides": slide_report,
        "keyframes": keyframe_report,
        "evidence": {
            "verified": [
                "MuJoCo grid states converged without NaN",
                "closed-chain residual limit",
                "rotation tracks wheel polar angle over a complete revolution",
                "wheel radius remains constant while rotation changes at fixed shape",
                "shape tracks the software-safe modal command",
                "configured passive ranges include 15% scan padding",
                "software-range scan does not touch gas-spring slide limits",
            ],
            "user_estimated": ["physical gas-spring stroke is approximately 23 mm"],
            "user_verified": [
                "the physical mechanism supports continuous unlimited 360-degree leg rotation"
            ],
            "unverified": [
                "physical passive-joint mechanical limits",
                "ground-contact behavior and gas-spring forces",
            ],
        },
    }
    (RESULTS / "modal_scan_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(f"PASS: {len(rows)} converged modal grid states")
    print(f"maximum |qvel|: {maximum_velocity:.3g}")
    print(f"maximum equality residual: {maximum_residual:.3g} m")
    print(f"maximum rotation error: {maximum_rotation_error:.3g} rad")
    print(f"maximum shape error: {maximum_shape_error:.3g} rad")
    print(f"maximum constant-shape radius span: {maximum_radius_span:.3g} m")
    for name, values in slide_report.items():
        print(f"{name}: {values['observed_range_m']}")


if __name__ == "__main__":
    main()
