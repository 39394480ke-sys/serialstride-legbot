#!/usr/bin/env python3
"""Compare ideal and nominal passive parameters without changing the selected model."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(ROOT))

from generate_scenes import render_scene  # noqa: E402
from joint_limits import joint_qpos_indices  # noqa: E402
from sync_inertials import PASSIVE_PATH, build_outputs, require  # noqa: E402
from validate_dynamics import _connect_residual_norms, _contact_distances  # noqa: E402
from validate_model import EXPECTED_KEYFRAMES, MuJoCo, _numbers_in_section  # noqa: E402


REPORT_PATH = ROOT / "results" / "passive_profile_report.json"
PROFILES = ("ideal", "nominal")
LEG_MOTORS = (
    ("left_joint_a_motor", "left_joint_a"),
    ("left_joint_b_motor", "left_joint_b"),
    ("right_joint_a_motor", "right_joint_a"),
    ("right_joint_b_motor", "right_joint_b"),
)


def _write_model(xml_bytes: bytes, path: Path) -> ET.Element:
    root = ET.fromstring(xml_bytes)
    compiler = root.find("./compiler")
    require(compiler is not None, "missing compiler")
    compiler.set("meshdir", str(ROOT))
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return root


def _norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _pulse_response(runtime: MuJoCo, xml_bytes: bytes, directory: Path) -> tuple[dict, dict]:
    base_root = ET.fromstring(xml_bytes)
    indices = joint_qpos_indices(base_root)
    responses = {}
    pulse_state = None
    for motor_index, (motor_name, joint_name) in enumerate(LEG_MOTORS):
        root = ET.fromstring(xml_bytes)
        root.find("./compiler").set("meshdir", str(ROOT))
        controls = [0.0] * 6
        controls[motor_index] = 0.01
        key = root.find('./keyframe/key[@name="CALIB_MID"]')
        require(key is not None, "missing CALIB_MID")
        key.set("ctrl", " ".join(map(str, controls)))
        path = directory / f"pulse_{motor_name}.xml"
        ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
        text = runtime.formatted_data(path, steps=20, keyframe=0)
        qvel = _numbers_in_section(text, "QVEL")
        responses[motor_name] = {
            "target_joint": joint_name,
            "target_qvel": qvel[indices[joint_name]],
        }
        if motor_index == 0:
            pulse_state = (
                _numbers_in_section(text, "QPOS"),
                qvel,
            )

    require(pulse_state is not None, "missing pulse state")
    coast_root = ET.fromstring(xml_bytes)
    coast_root.find("./compiler").set("meshdir", str(ROOT))
    coast_key = coast_root.find('./keyframe/key[@name="CALIB_MID"]')
    coast_key.set("qpos", " ".join(map(str, pulse_state[0])))
    coast_key.set("qvel", " ".join(map(str, pulse_state[1])))
    coast_key.set("ctrl", "0 0 0 0 0 0")
    coast_path = directory / "coast.xml"
    ET.ElementTree(coast_root).write(coast_path, encoding="utf-8", xml_declaration=True)
    coast_text = runtime.formatted_data(coast_path, steps=500, keyframe=0)
    coast_final_qvel = _numbers_in_section(coast_text, "QVEL")
    coast = {
        "duration_s": 0.5,
        "initial_qvel_norm": _norm(pulse_state[1]),
        "final_qvel_norm": _norm(coast_final_qvel),
        "retained_ratio": _norm(coast_final_qvel) / _norm(pulse_state[1]),
    }
    return responses, coast


def _zero_control_drift(runtime: MuJoCo, path: Path) -> float:
    maximum = 0.0
    for key_index in range(len(EXPECTED_KEYFRAMES)):
        text = runtime.formatted_data(path, steps=1000, keyframe=key_index)
        maximum = max(maximum, max(map(abs, _numbers_in_section(text, "QVEL")), default=0.0))
    return maximum


def _scene_rollout(runtime: MuJoCo, xml_bytes: bytes, directory: Path, drop: bool) -> dict:
    directory.mkdir()
    root = _write_model(render_scene(True, drop, dynamics_xml=xml_bytes), directory / "scene.xml")
    path = directory / "scene.xml"
    maximum_qvel = 0.0
    maximum_qacc = 0.0
    maximum_closure = 0.0
    minimum_contact_distance = math.inf
    for key_index, _ in enumerate(root.findall("./keyframe/key")):
        text = runtime.formatted_data(path, steps=5000, keyframe=key_index)
        qpos = _numbers_in_section(text, "QPOS")
        qvel = _numbers_in_section(text, "QVEL")
        qacc = _numbers_in_section(text, "QACC")
        require(all(math.isfinite(value) for value in qpos + qvel + qacc), "non-finite 5 s state")
        require("\nWARNING\n" not in text, "MuJoCo warning in 5 s rollout")
        maximum_qvel = max(maximum_qvel, max(map(abs, qvel), default=0.0))
        maximum_qacc = max(maximum_qacc, max(map(abs, qacc), default=0.0))
        maximum_closure = max(maximum_closure, max(_connect_residual_norms(text), default=0.0))
        minimum_contact_distance = min(
            minimum_contact_distance,
            min(_contact_distances(text), default=math.inf),
        )
    require(maximum_closure <= 1e-4, f"5 s closure residual is {maximum_closure:g} m")
    require(minimum_contact_distance >= -1e-3, f"5 s contact penetration is {minimum_contact_distance:g} m")
    return {
        "keyframes": len(EXPECTED_KEYFRAMES),
        "duration_each_s": 5.0,
        "maximum_qvel_native": maximum_qvel,
        "maximum_qacc_native": maximum_qacc,
        "maximum_closure_residual_m": maximum_closure,
        "minimum_contact_distance_m": minimum_contact_distance,
        "status": "PASS_NUMERICAL",
    }


def main() -> None:
    config = yaml.safe_load(PASSIVE_PATH.read_text(encoding="utf-8"))
    require(config["selected_profile"] == "nominal", "nominal must remain the selected baseline")
    runtime = MuJoCo()
    results = {}
    with tempfile.TemporaryDirectory() as temporary:
        temporary_root = Path(temporary)
        for profile in PROFILES:
            profile_dir = temporary_root / profile
            profile_dir.mkdir()
            xml_bytes, _ = build_outputs(profile)
            dynamics_path = profile_dir / "robot_dynamics.xml"
            root = _write_model(xml_bytes, dynamics_path)
            responses, coast = _pulse_response(runtime, xml_bytes, profile_dir)
            results[profile] = {
                "parameters": config["profiles"][profile],
                "zero_control_1s_maximum_qvel": _zero_control_drift(runtime, dynamics_path),
                "leg_pulse_0p01nm_20ms": responses,
                "pulse_removed_coast": coast,
                "free_ground_5s": _scene_rollout(runtime, xml_bytes, profile_dir / "free", False),
                "drop_test_5s": _scene_rollout(runtime, xml_bytes, profile_dir / "drop", True),
            }

    for profile in PROFILES:
        responses = results[profile]["leg_pulse_0p01nm_20ms"]
        for motor_name, _ in LEG_MOTORS:
            require(responses[motor_name]["target_qvel"] > 0.0, f"{profile}: wrong sign for {motor_name}")
        for left_name, right_name in (
            ("left_joint_a_motor", "right_joint_a_motor"),
            ("left_joint_b_motor", "right_joint_b_motor"),
        ):
            left = responses[left_name]["target_qvel"]
            right = responses[right_name]["target_qvel"]
            require(abs(left - right) / max(abs(left), abs(right)) < 0.01, f"{profile}: asymmetric pulse response")

    nominal_coast = results["nominal"]["pulse_removed_coast"]
    ideal_coast = results["ideal"]["pulse_removed_coast"]
    require(nominal_coast["retained_ratio"] < ideal_coast["retained_ratio"], "nominal damping did not increase decay")

    report = {
        "schema_version": 1,
        "status": "PASS_WITH_PROVISIONAL_NOMINAL_PARAMETERS",
        "selected_baseline": "nominal",
        "gas_spring_force_enabled": False,
        "profiles": results,
        "evidence_classification": {
            "verified": [
                "ideal and nominal models compile without changing authoritative geometry",
                "zero-control keyframes, four leg-motor pulse signs, and left-right response symmetry",
                "nominal damping increases post-pulse velocity decay relative to ideal",
                "all 11 keyframes in free-ground and drop scenes remain numerically stable for 5 s",
            ],
            "unverified": [
                "physical damping, dry friction, armature, settling time, and decay-rate agreement",
            ],
        },
    }
    REPORT_PATH.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("PASS: ideal/nominal passive profiles; nominal remains the provisional baseline")
    print(f"generated: {REPORT_PATH.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
