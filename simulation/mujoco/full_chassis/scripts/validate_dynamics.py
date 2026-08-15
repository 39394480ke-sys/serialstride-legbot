#!/usr/bin/env python3
"""Validate the geometry-to-dynamics migration and write its evidence report."""

from __future__ import annotations

import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(ROOT))

from generate_scenes import SCENE_DIR, SCENES, render_scene  # noqa: E402
from sync_inertials import (  # noqa: E402
    ACTUATOR_PATH,
    DYNAMICS_PATH,
    GEOMETRY_PATH,
    GEOMETRY_SHA256,
    build_outputs,
    require,
)
from validate_model import (  # noqa: E402
    EXPECTED_KEYFRAMES,
    EXPECTED_SOURCE_SHA256,
    MuJoCo,
    SOURCE_PATH,
    _numbers_in_section,
)


REPORT_PATH = ROOT / "results" / "dynamics_migration_report.json"
EXPECTED_MASS_KG = 3.76954478618
EXPECTED_MOTORS = (
    ("left_joint_a_motor", "left_joint_a"),
    ("left_joint_b_motor", "left_joint_b"),
    ("right_joint_a_motor", "right_joint_a"),
    ("right_joint_b_motor", "right_joint_b"),
    ("left_wheel_motor", "left_wheel_joint"),
    ("right_wheel_motor", "right_wheel_joint"),
)
EXPECTED_LINK_COLLISIONS = 22


def _contact_distances(text: str) -> list[float]:
    match = re.search(r"^CONTACT\n(.*?)(?=\nEFC_TYPE\n|\Z)", text, re.MULTILINE | re.DOTALL)
    if not match:
        return []
    return [float(value) for value in re.findall(r"^\s+dist\s+([-+0-9.eE]+)$", match.group(1), re.MULTILINE)]


def _constraint_rows(text: str) -> list[tuple[float, int, int]]:
    positions = _numbers_in_section(text, "EFC_POS")
    types = [int(value) for value in _numbers_in_section(text, "EFC_TYPE")]
    ids = [int(value) for value in _numbers_in_section(text, "EFC_ID")]
    require(len(positions) == len(types) == len(ids), "inconsistent constraint diagnostics")
    return list(zip(positions, types, ids))


def _connect_residual_norms(text: str) -> list[float]:
    components: dict[int, list[float]] = {}
    for position, constraint_type, constraint_id in _constraint_rows(text):
        if constraint_type == 0:  # mjCNSTR_EQUALITY; all six equalities are 3D connects.
            components.setdefault(constraint_id, []).append(position)
    require(len(components) == 6, f"expected six active connect constraints, got {len(components)}")
    require(all(len(values) == 3 for values in components.values()), "connect residual is not 3D")
    return [math.sqrt(sum(value * value for value in values)) for values in components.values()]


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_attributes(node: ET.Element) -> tuple:
    return (
        node.tag,
        tuple(sorted(node.attrib.items())),
        tuple(_canonical_attributes(child) for child in list(node)),
    )


def _static_checks() -> tuple[ET.ElementTree, ET.ElementTree, dict]:
    source_digest = _digest(SOURCE_PATH)
    geometry_digest = _digest(GEOMETRY_PATH)
    require(source_digest == EXPECTED_SOURCE_SHA256, "immutable CAD source hash changed")
    require(geometry_digest == GEOMETRY_SHA256, "authoritative geometry hash changed")

    first_xml, first_yaml = build_outputs()
    second_xml, second_yaml = build_outputs()
    require(first_xml == second_xml and first_yaml == second_yaml, "generator is nondeterministic")
    require(DYNAMICS_PATH.read_bytes() == first_xml, "robot_dynamics.xml is stale")

    geometry = ET.parse(GEOMETRY_PATH)
    dynamics = ET.parse(DYNAMICS_PATH)
    g_root = geometry.getroot()
    d_root = dynamics.getroot()
    counts = {
        "bodies": len(d_root.findall(".//body")),
        "joints": len(d_root.findall("./worldbody//joint")),
        "connects": len(d_root.findall("./equality/connect")),
        "tendons": len(d_root.findall("./tendon/fixed")),
        "motors": len(d_root.findall("./actuator/motor")),
        "position_actuators": len(d_root.findall("./actuator/position")),
        "keyframes": len(d_root.findall("./keyframe/key")),
        "sensors": len(list(d_root.find("./sensor") or [])),
    }
    require(
        counts
        == {
            "bodies": 25,
            "joints": 24,
            "connects": 6,
            "tendons": 4,
            "motors": 6,
            "position_actuators": 0,
            "keyframes": 11,
            "sensors": 2,
        },
        f"unexpected dynamics structure: {counts}",
    )
    require(len(g_root.findall("./actuator/position")) == 4, "geometry debug actuators changed")

    actuator_config = yaml.safe_load(ACTUATOR_PATH.read_text(encoding="utf-8"))
    motors = tuple(
        (node.get("name"), node.get("joint"))
        for node in d_root.findall("./actuator/motor")
    )
    require(motors == EXPECTED_MOTORS, f"unexpected motor interface: {motors}")
    require(all(node.get("gear") == "1" for node in d_root.findall("./actuator/motor")), "motor gear changed")
    for index, node in enumerate(d_root.findall("./actuator/motor")):
        role = actuator_config["motors"][index]["role"]
        limits = actuator_config["limits"][role]
        expected_control_range = [
            -float(limits["torque_command_limit_nm"]),
            float(limits["torque_command_limit_nm"]),
        ]
        expected_force_range = [-float(limits["peak_torque_nm"]), float(limits["peak_torque_nm"])]
        require(node.get("ctrllimited") == "true", f"{node.get('name')}: control limit disabled")
        require(node.get("forcelimited") == "true", f"{node.get('name')}: force limit disabled")
        require([float(value) for value in node.get("ctrlrange", "").split()] == expected_control_range, f"{node.get('name')}: wrong control range")
        require([float(value) for value in node.get("forcerange", "").split()] == expected_force_range, f"{node.get('name')}: wrong force range")

    g_tendons = g_root.findall("./tendon/fixed")
    d_tendons = d_root.findall("./tendon/fixed")
    require(
        [_canonical_attributes(node) for node in g_tendons]
        == [_canonical_attributes(node) for node in d_tendons],
        "dynamics tendon interface differs from geometry",
    )
    g_sensors = list(g_root.find("./sensor") or [])
    d_sensors = list(d_root.find("./sensor") or [])
    require(
        [_canonical_attributes(node) for node in g_sensors]
        == [_canonical_attributes(node) for node in d_sensors],
        "wheel sensor interface changed",
    )
    require(
        [node.get("name") for node in d_sensors]
        == ["right_wheel_position", "left_wheel_position"],
        "wheel sensor order changed",
    )

    g_keys = g_root.findall("./keyframe/key")
    d_keys = d_root.findall("./keyframe/key")
    require([key.get("name") for key in d_keys] == list(EXPECTED_KEYFRAMES), "keyframe names or order changed")
    for g_key, d_key in zip(g_keys, d_keys):
        require(g_key.get("qpos") == d_key.get("qpos"), f"{g_key.get('name')}: qpos changed")
        require(d_key.get("ctrl") == "0 0 0 0 0 0", f"{g_key.get('name')}: motor control is not zero")
        require("act" not in d_key.attrib, f"{g_key.get('name')}: motor keyframe has activation state")

    mass = sum(float(node.get("mass", "0")) for node in d_root.findall(".//inertial"))
    require(math.isclose(mass, EXPECTED_MASS_KG, abs_tol=1e-12), f"dynamics mass is {mass}")
    compiler = d_root.find("./compiler")
    require(compiler is not None and compiler.get("balanceinertia") == "false", "balanceinertia must be false")
    return geometry, dynamics, {"counts": counts, "mass_kg": mass}


def _runtime_checks(geometry: ET.ElementTree, dynamics: ET.ElementTree) -> dict:
    runtime = MuJoCo()
    maximum_residual = 0.0
    maximum_sensor_delta = 0.0
    for index, key in enumerate(geometry.getroot().findall("./keyframe/key")):
        _, geometry_residual, geometry_sensors = runtime.evaluate(GEOMETRY_PATH, 0, index)
        _, dynamics_residual, dynamics_sensors = runtime.evaluate(DYNAMICS_PATH, 0, index)
        maximum_residual = max(maximum_residual, dynamics_residual)
        require(dynamics_residual < 1e-6, f"{key.get('name')}: closure residual too large")
        for old, new in zip(geometry_sensors, dynamics_sensors):
            maximum_sensor_delta = max(maximum_sensor_delta, abs(old - new))
    require(maximum_sensor_delta <= 1e-9, f"wheel sensor migration delta is {maximum_sensor_delta:g} m")

    fixed_path = SCENE_DIR / "fixed_base.xml"
    fixed_sensor_outputs = [
        _numbers_in_section(runtime.formatted_data(fixed_path, steps=0, keyframe=index), "SENSOR")
        for index in range(len(EXPECTED_KEYFRAMES))
    ]
    scene_results = {}
    for name, (free, drop) in SCENES.items():
        path = SCENE_DIR / name
        require(path.read_bytes() == render_scene(free, drop), f"{name} is stale")
        scene_root = ET.parse(path).getroot()
        base_collision = scene_root.find('.//geom[@name="base_collision"]')
        require(base_collision is not None and base_collision.get("type") == "box", f"{name}: missing base collision")
        link_collisions = scene_root.findall('.//geom[@type="mesh"][@group="3"]')
        require(
            len(link_collisions) == EXPECTED_LINK_COLLISIONS,
            f"{name}: expected {EXPECTED_LINK_COLLISIONS} link collision hulls",
        )
        require(not scene_root.findall('.//geom[@type="capsule"][@group="3"]'), f"{name}: stale link capsules")
        ground_pairs = scene_root.findall('./contact/pair[@geom1="ground"]')
        require(len(ground_pairs) == EXPECTED_LINK_COLLISIONS + 3, f"{name}: incomplete ground pairs")
        initial_text = runtime.formatted_data(path, steps=0, keyframe=0)
        residual = max(_connect_residual_norms(initial_text), default=0.0)
        sensors = _numbers_in_section(initial_text, "SENSOR")
        require(all(math.isfinite(value) for value in sensors), f"{name}: non-finite sensor")
        require(residual < 1e-6, f"{name}: initial closure residual {residual:g} m")
        scene_results[name] = {"compile": "PASS", "initial_closure_residual_m": residual}
        if free:
            minimum_base_z = math.inf
            maximum_rollout_residual = 0.0
            maximum_tendon_limit_residual = 0.0
            minimum_contact_distance = math.inf
            for key_index, _ in enumerate(scene_root.findall("./keyframe/key")):
                text = runtime.formatted_data(path, steps=500, keyframe=key_index)
                qpos = _numbers_in_section(text, "QPOS")
                constraint_rows = _constraint_rows(text)
                contact_distances = _contact_distances(text)
                minimum_base_z = min(minimum_base_z, qpos[2])
                minimum_contact_distance = min(
                    minimum_contact_distance,
                    min(contact_distances, default=math.inf),
                )
                maximum_rollout_residual = max(
                    maximum_rollout_residual,
                    max(_connect_residual_norms(text), default=0.0),
                )
                maximum_tendon_limit_residual = max(
                    maximum_tendon_limit_residual,
                    max(
                        (abs(position) for position, constraint_type, _ in constraint_rows if constraint_type == 4),
                        default=0.0,
                    ),
                )
                scene_sensors = _numbers_in_section(
                    runtime.formatted_data(path, steps=0, keyframe=key_index), "SENSOR"
                )
                require(
                    max(
                        (abs(actual - expected) for actual, expected in zip(scene_sensors, fixed_sensor_outputs[key_index])),
                        default=0.0,
                    )
                    <= 1e-9,
                    f"{name}: free-base orientation does not preserve fixed-base pose",
                )
            require(
                minimum_contact_distance >= -1e-3,
                f"{name}: ground contact penetrated {minimum_contact_distance:g} m",
            )
            scene_results[name]["all_keyframes_0p5s_collision_contact_check"] = "PASS"
            scene_results[name]["minimum_base_origin_z_m"] = minimum_base_z
            scene_results[name]["minimum_contact_distance_m"] = minimum_contact_distance
            scene_results[name]["maximum_rollout_closure_residual_m"] = maximum_rollout_residual
            scene_results[name]["maximum_tendon_limit_residual_rad"] = maximum_tendon_limit_residual
            scene_results[name]["dynamic_closure_interpretation"] = "MEASURED_CONNECT_SITE_SEPARATION"

            long_rollout_maximum_qvel = 0.0
            long_rollout_maximum_qacc = 0.0
            long_rollout_maximum_closure = 0.0
            long_rollout_minimum_contact_distance = math.inf
            for key_index, _ in enumerate(scene_root.findall("./keyframe/key")):
                text = runtime.formatted_data(path, steps=5000, keyframe=key_index)
                qpos = _numbers_in_section(text, "QPOS")
                qvel = _numbers_in_section(text, "QVEL")
                qacc = _numbers_in_section(text, "QACC")
                require(
                    all(math.isfinite(value) for value in qpos + qvel + qacc),
                    f"{name}: non-finite state after 5 s",
                )
                require("\nWARNING\n" not in text, f"{name}: MuJoCo warning after 5 s")
                long_rollout_maximum_qvel = max(
                    long_rollout_maximum_qvel, max(map(abs, qvel), default=0.0)
                )
                long_rollout_maximum_qacc = max(
                    long_rollout_maximum_qacc, max(map(abs, qacc), default=0.0)
                )
                long_rollout_maximum_closure = max(
                    long_rollout_maximum_closure,
                    max(_connect_residual_norms(text), default=0.0),
                )
                long_rollout_minimum_contact_distance = min(
                    long_rollout_minimum_contact_distance,
                    min(_contact_distances(text), default=math.inf),
                )
            require(
                long_rollout_maximum_closure <= 1e-4,
                f"{name}: 5 s closure residual {long_rollout_maximum_closure:g} m",
            )
            require(
                long_rollout_minimum_contact_distance >= -1e-3,
                f"{name}: 5 s ground penetration {long_rollout_minimum_contact_distance:g} m",
            )
            scene_results[name]["all_keyframes_5s_numerical_stability_check"] = "PASS"
            scene_results[name]["all_keyframes_5s_maximum_qvel_native"] = long_rollout_maximum_qvel
            scene_results[name]["all_keyframes_5s_maximum_qacc_native"] = long_rollout_maximum_qacc
            scene_results[name]["all_keyframes_5s_maximum_closure_residual_m"] = long_rollout_maximum_closure
            scene_results[name]["all_keyframes_5s_minimum_contact_distance_m"] = long_rollout_minimum_contact_distance
            scene_results[name]["physical_settling_acceptance"] = "UNVERIFIED_PROVISIONAL_PASSIVE_PARAMETERS"
    return {
        "mujoco_version": runtime.version,
        "maximum_keyframe_closure_residual_m": maximum_residual,
        "maximum_geometry_dynamics_sensor_delta_m": maximum_sensor_delta,
        "scenes": scene_results,
    }


def main() -> None:
    geometry, dynamics, static = _static_checks()
    runtime = _runtime_checks(geometry, dynamics)
    report = {
        "schema_version": 1,
        "status": "PASS_WITH_PROVISIONAL_INPUTS",
        "authoritative_geometry_sha256": GEOMETRY_SHA256,
        "source_sha256": EXPECTED_SOURCE_SHA256,
        "generated_dynamics_sha256": _digest(DYNAMICS_PATH),
        "static": static,
        "runtime": runtime,
        "interfaces": {
            "geometry_debug_actuators": 4,
            "dynamics_torque_motors": [name for name, _ in EXPECTED_MOTORS],
            "keyframes": list(EXPECTED_KEYFRAMES),
            "wheel_sensor_order": ["right_wheel_position", "left_wheel_position"],
            "world_forward_axis": "+X",
            "forward_wheel_torque_signs": {"left": -1, "right": 1},
            "motor_limits": {
                role: {
                    "model": limits["model"],
                    "rated_torque_nm": limits["rated_torque_nm"],
                    "peak_torque_nm": limits["peak_torque_nm"],
                    "rated_velocity_rpm": limits["rated_velocity_rpm"],
                    "no_load_max_velocity_rpm": limits["no_load_max_velocity_rpm"],
                    "source_status": limits["status"],
                }
                for role, limits in yaml.safe_load(
                    ACTUATOR_PATH.read_text(encoding="utf-8")
                )["limits"].items()
            },
        },
        "evidence_classification": {
            "verified": [
                "source and geometry byte hashes",
                "deterministic geometry-to-dynamics generation",
                "mass/inertia overlay and MuJoCo compilation",
                "four geometry debug actuators replaced by six dynamics torque motors",
                "datasheet peak-torque limits emitted as motor control and force limits",
                "tendons, keyframe qpos, slide convention, closed chains, and wheel sensors preserved",
                "fixed-base, free-ground, and low-drop scene compilation",
                "free/drop keyframes preserve the authoritative fixed-base orientation and sensor pose",
                "left-negative/right-positive wheel torque moves the base toward world +X",
                "all CAD link visuals have separate ground-only convex-hull collision proxies",
                "all free/drop keyframes stay within 1 mm ground-contact penetration after 0.5 s",
                "all free/drop keyframes remain finite with sub-0.1 mm connect residual over 5 s",
            ],
            "user_observed": [
                "SolidWorks-assigned main-body masses approximately agree with the physical build",
                "gas-spring physical stroke is approximately 23 mm",
                "gas-spring nominal force is 60 N",
                "the CAD wheel model radius and width match the physical tire",
                "free/drop scenes previously showed link visuals sinking below the ground",
                "all keyframes in the current free-ground and drop scenes showed no ground penetration, abnormal jitter, numerical explosion, or visible closed-chain separation in the MuJoCo Viewer",
            ],
            "ai_inference": [
                "none for the disabled gas-spring force model",
            ],
            "unverified": [
                "datasheet torque and speed limits under the installed supply and drive configuration",
                "motor thermal derating and duration-dependent continuous-torque behavior",
                "physical joint damping, dry friction, and rotor inertia",
                "gas-spring nominal-force tolerance, physical stiffness, damping, and force curve",
                "gas-spring dynamic effect, intentionally disabled at the user's request",
                "ground friction and load-bearing contact behavior",
                "physical settling time with identified damping and friction",
                "precise physical whole-robot mass and pose-matched COM",
            ],
        },
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"PASS: dynamics migration, maximum closure residual {runtime['maximum_keyframe_closure_residual_m']:.3g} m")
    print(f"generated: {REPORT_PATH.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
