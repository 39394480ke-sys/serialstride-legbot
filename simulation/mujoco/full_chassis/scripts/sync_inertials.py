#!/usr/bin/env python3
"""Generate the dynamics MJCF from frozen geometry and editable parameters."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml

from joint_limits import load_config, validate_authoritative_geometry


ROOT = Path(__file__).resolve().parents[1]
CSV_PATH = ROOT / "params" / "body_mass_map.csv"
YAML_PATH = ROOT / "params" / "inertials.yaml"
GEOMETRY_PATH = ROOT / "robot_geometry.xml"
DYNAMICS_PATH = ROOT / "robot_dynamics.xml"
ACTUATOR_PATH = ROOT / "params" / "actuator_params.yaml"
PASSIVE_PATH = ROOT / "params" / "passive_params.yaml"
GAS_SPRING_PATH = ROOT / "params" / "gas_spring.yaml"
GEOMETRY_SHA256 = "e7e1f21c22bbcc8846be8548e071819e72ccfad49df203704b56205417b800ea"

VECTOR_FIELDS = {
    "com": ("com_x_m", "com_y_m", "com_z_m"),
    "quat": (
        "inertia_quat_w",
        "inertia_quat_x",
        "inertia_quat_y",
        "inertia_quat_z",
    ),
    "principal": (
        "principal_i1_kg_m2",
        "principal_i2_kg_m2",
        "principal_i3_kg_m2",
    ),
}


@dataclass(frozen=True)
class Inertial:
    body_name: str
    mass: float
    com: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    principal: tuple[float, float, float]
    row: dict[str, str]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def format_number(value: float) -> str:
    return f"{value:.12g}"


def parse_vector(row: dict[str, str], name: str) -> tuple[float, ...]:
    try:
        return tuple(float(row[field]) for field in VECTOR_FIELDS[name])
    except (KeyError, ValueError) as error:
        raise RuntimeError(
            f"{row.get('body_name', '<unknown>')}: invalid {name} vector"
        ) from error


def geometry_digest() -> str:
    return hashlib.sha256(GEOMETRY_PATH.read_bytes()).hexdigest()


def check_frozen_geometry() -> None:
    digest = geometry_digest()
    require(
        digest == GEOMETRY_SHA256,
        "robot_geometry.xml is not the read-only authoritative input; "
        f"expected {GEOMETRY_SHA256}, got {digest}",
    )


def load_rows() -> tuple[list[str], dict[str, dict[str, str]]]:
    with CSV_PATH.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        require(reader.fieldnames is not None, "mass-property CSV has no header")
        rows: dict[str, dict[str, str]] = {}
        order: list[str] = []
        for row in reader:
            name = row.get("body_name", "").strip()
            require(name, "mass-property CSV contains an unnamed body")
            require(name not in rows, f"duplicate body row: {name}")
            rows[name] = {key: (value or "").strip() for key, value in row.items()}
            order.append(name)
    return order, rows


def _validate_inertial(item: Inertial) -> None:
    require(item.mass > 0.0, f"{item.body_name}: mass must be positive")
    require(
        all(value > 0.0 and math.isfinite(value) for value in item.principal),
        f"{item.body_name}: principal inertias must be finite and positive",
    )
    i1, i2, i3 = item.principal
    tolerance = max(item.principal) * 1e-9
    require(i1 + i2 + tolerance >= i3, f"{item.body_name}: I1 + I2 < I3")
    require(i1 + i3 + tolerance >= i2, f"{item.body_name}: I1 + I3 < I2")
    require(i2 + i3 + tolerance >= i1, f"{item.body_name}: I2 + I3 < I1")
    norm = math.sqrt(sum(value * value for value in item.quat))
    require(abs(norm - 1.0) <= 1e-5, f"{item.body_name}: inertial quat norm is {norm}")
    require(
        all(math.isfinite(value) for value in item.com),
        f"{item.body_name}: COM contains a non-finite value",
    )


def resolve_rows(
    order: list[str], rows: dict[str, dict[str, str]]
) -> dict[str, Inertial]:
    resolved: dict[str, Inertial] = {}
    resolving: set[str] = set()

    def resolve(name: str) -> Inertial:
        if name in resolved:
            return resolved[name]
        require(name in rows, f"unknown mirror source: {name}")
        require(name not in resolving, f"mirror cycle involving {name}")
        resolving.add(name)
        row = rows[name]
        mode = row.get("property_mode", "")
        mirror_of = row.get("mirror_of", "")
        com = parse_vector(row, "com")
        quat = parse_vector(row, "quat")

        if mode == "mirror":
            require(mirror_of, f"{name}: mirror mode requires mirror_of")
            source = resolve(mirror_of)
            mass = source.mass
            principal = source.principal
        else:
            require(not mirror_of, f"{name}: non-mirror row has mirror_of")
            require(
                mode in {"explicit", "uniform_mass_scale"},
                f"{name}: unsupported property_mode {mode!r}",
            )
            try:
                mass = float(row["mass_kg"])
                reference_mass = float(row["inertia_reference_mass_kg"])
            except (KeyError, ValueError) as error:
                raise RuntimeError(f"{name}: invalid mass fields") from error
            principal = parse_vector(row, "principal")
            require(reference_mass > 0.0, f"{name}: reference mass must be positive")
            if mode == "uniform_mass_scale":
                if not math.isclose(mass, reference_mass, rel_tol=1e-12):
                    require(
                        row.get("confidence") in {"C", "D"},
                        f"{name}: uniformly scaled inertia must use confidence C or D",
                    )
                scale = mass / reference_mass
                principal = tuple(value * scale for value in principal)

        item = Inertial(name, mass, com, quat, principal, row)
        _validate_inertial(item)
        resolving.remove(name)
        resolved[name] = item
        return item

    for body_name in order:
        resolve(body_name)
    return resolved


def _model_body_names(root: ET.Element) -> list[str]:
    return [node.get("name", "") for node in root.findall(".//body")]


def _load_yaml(path: Path, schema_version: int = 1) -> dict:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    require(config.get("schema_version") == schema_version, f"{path.name}: unsupported schema")
    return config


def _set_nonnegative_attribute(node: ET.Element, name: str, value: float) -> None:
    require(math.isfinite(value) and value >= 0.0, f"{node.get('name')}: invalid {name}")
    if value == 0.0:
        node.attrib.pop(name, None)
    else:
        node.set(name, format_number(value))


def _joint_role(name: str) -> str:
    if name in {"left_joint_a", "left_joint_b", "right_joint_a", "right_joint_b"}:
        return "active_hinge"
    if name in {"left_wheel_joint", "right_wheel_joint"}:
        return "wheel_hinge"
    if name in {"link_012_joint", "link_023_joint"}:
        return "gas_slide"
    return "passive_hinge"


def _apply_passive_parameters(root: ET.Element, config: dict, profile_name: str) -> None:
    profiles = config.get("profiles", {})
    require(profile_name in profiles, f"unknown passive profile {profile_name!r}")
    profile = profiles[profile_name]
    for joint in root.findall("./worldbody//joint"):
        settings = profile[_joint_role(joint.get("name", ""))]
        for attribute in ("damping", "frictionloss", "armature"):
            _set_nonnegative_attribute(joint, attribute, float(settings[attribute]))
        joint.attrib.pop("actuatorfrcrange", None)
        joint.attrib.pop("actuatorfrclimited", None)


def _replace_actuators(root: ET.Element, config: dict) -> None:
    actuator = root.find("./actuator")
    require(actuator is not None, "geometry is missing actuator section")
    motors = config.get("motors", [])
    require(len(motors) == 6, "actuator_params.yaml must define exactly six motors")
    names: set[str] = set()
    joints: set[str] = set()
    for node in list(actuator):
        actuator.remove(node)
    for settings in motors:
        name = settings["name"]
        joint_name = settings["joint"]
        require(name not in names, f"duplicate actuator name {name}")
        require(joint_name not in joints, f"duplicate actuator target {joint_name}")
        require(root.find(f'.//joint[@name="{joint_name}"]') is not None, f"missing motor joint {joint_name}")
        require(float(settings["gear"]) == 1.0, f"{name}: migration requires 1:1 gear")
        limits = config["limits"][settings["role"]]
        require(limits.get("status") in {"PROVISIONAL", "DATASHEET", "MEASURED", "VERIFIED"}, f"{name}: invalid limit status")
        attributes = {"name": name, "joint": joint_name, "gear": "1"}
        command_limit = limits.get("torque_command_limit_nm")
        peak_limit = limits.get("peak_torque_nm")
        if command_limit is not None:
            command_limit = float(command_limit)
            require(command_limit > 0.0, f"{name}: torque command limit must be positive")
            attributes.update(
                ctrllimited="true",
                ctrlrange=f"-{format_number(command_limit)} {format_number(command_limit)}",
            )
        if peak_limit is not None:
            peak_limit = float(peak_limit)
            require(peak_limit > 0.0, f"{name}: peak torque must be positive")
            attributes.update(
                forcelimited="true",
                forcerange=f"-{format_number(peak_limit)} {format_number(peak_limit)}",
            )
        ET.SubElement(actuator, "motor", **attributes)
        names.add(name)
        joints.add(joint_name)

    zeros = " ".join("0" for _ in motors)
    for key in root.findall("./keyframe/key"):
        key.set("ctrl", zeros)
        key.attrib.pop("act", None)


def _apply_gas_springs(root: ET.Element, config: dict) -> None:
    if not config.get("enabled", False):
        return
    required = ("force_extended_n", "stiffness_n_per_m", "damping_n_s_per_m")
    require(all(config.get(name) is not None for name in required), "enabled gas spring has null force parameters")
    force = float(config["force_extended_n"])
    stiffness = float(config["stiffness_n_per_m"])
    damping = float(config["damping_n_s_per_m"])
    require(force >= 0.0 and stiffness > 0.0 and damping >= 0.0, "invalid gas-spring parameters")
    for name in config["slides"]:
        joint = root.find(f'.//joint[@name="{name}"]')
        require(joint is not None, f"missing gas-spring slide {name}")
        joint.set("stiffness", format_number(stiffness))
        joint.set("springref", format_number(force / stiffness))
        existing_damping = float(joint.get("damping", "0"))
        _set_nonnegative_attribute(joint, "damping", existing_damping + damping)


def update_tree(
    tree: ET.ElementTree,
    resolved: dict[str, Inertial],
    actuator_config: dict,
    passive_config: dict,
    gas_spring_config: dict,
    profile_name: str,
) -> None:
    root = tree.getroot()
    validate_authoritative_geometry(tree, load_config())
    root.set("model", "serial_stride_full_chassis_dynamics")
    model_names = _model_body_names(root)
    require(set(model_names) == set(resolved), "CSV and MJCF body sets differ")
    compiler = root.find("compiler")
    require(compiler is not None, "MJCF is missing compiler")
    compiler.set("balanceinertia", "false")

    for body in root.findall(".//body"):
        name = body.get("name", "")
        inertial = body.find("inertial")
        require(inertial is not None, f"{name}: missing explicit inertial")
        item = resolved[name]
        inertial.set("pos", " ".join(map(format_number, item.com)))
        inertial.set("mass", format_number(item.mass))
        inertial.set("quat", " ".join(map(format_number, item.quat)))
        inertial.set("diaginertia", " ".join(map(format_number, item.principal)))

    _apply_passive_parameters(root, passive_config, profile_name)
    _apply_gas_springs(root, gas_spring_config)
    _replace_actuators(root, actuator_config)


def render_yaml(order: list[str], resolved: dict[str, Inertial]) -> str:
    lines = [
        "# GENERATED by scripts/sync_inertials.py from body_mass_map.csv.",
        "# DO NOT EDIT: body_mass_map.csv is the only hand-edited source.",
        "schema_version: 1",
        'source: "params/body_mass_map.csv"',
        f'geometry_sha256: "{GEOMETRY_SHA256}"',
        "bodies:",
    ]
    for name in order:
        item = resolved[name]
        row = item.row
        lines.extend(
            [
                f"  {name}:",
                f"    category: {json.dumps(row['category'])}",
                f"    side: {json.dumps(row['side'])}",
                f"    mirror_of: {json.dumps(row['mirror_of'])}",
                f"    mass_kg: {format_number(item.mass)}",
                f"    com_m: [{', '.join(map(format_number, item.com))}]",
                f"    inertia_quat_wxyz: [{', '.join(map(format_number, item.quat))}]",
                "    principal_inertia_kg_m2: "
                f"[{', '.join(map(format_number, item.principal))}]",
                f"    confidence: {json.dumps(row['confidence'])}",
                f"    status: {json.dumps(row['status'])}",
            ]
        )
    return "\n".join(lines) + "\n"


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, delete=False
    ) as output:
        output.write(content)
        temporary = Path(output.name)
    os.replace(temporary, path)


def _tree_bytes(tree: ET.ElementTree) -> bytes:
    return ET.tostring(tree.getroot(), encoding="utf-8", xml_declaration=True)


def build_outputs(profile_name: str | None = None) -> tuple[bytes, str]:
    check_frozen_geometry()
    order, rows = load_rows()
    resolved = resolve_rows(order, rows)
    actuator_config = _load_yaml(ACTUATOR_PATH)
    passive_config = _load_yaml(PASSIVE_PATH)
    gas_spring_config = _load_yaml(GAS_SPRING_PATH)
    profile_name = profile_name or passive_config["selected_profile"]
    tree = ET.parse(GEOMETRY_PATH)
    update_tree(
        tree,
        resolved,
        actuator_config,
        passive_config,
        gas_spring_config,
        profile_name,
    )
    return _tree_bytes(tree), render_yaml(order, resolved)


def synchronize(check_only: bool, profile_name: str | None = None) -> None:
    expected_xml, expected_yaml = build_outputs(profile_name)

    if check_only:
        require(DYNAMICS_PATH.exists(), "robot_dynamics.xml does not exist")
        require(DYNAMICS_PATH.read_bytes() == expected_xml, "robot_dynamics.xml is stale")
        require(YAML_PATH.exists(), "inertials.yaml does not exist")
        require(YAML_PATH.read_text(encoding="utf-8") == expected_yaml, "inertials.yaml is stale")
        print("PASS: authoritative geometry and dynamics parameters are synchronized")
        return

    _atomic_write(DYNAMICS_PATH, expected_xml.decode("utf-8"))
    _atomic_write(YAML_PATH, expected_yaml)
    print(f"updated: {DYNAMICS_PATH.relative_to(ROOT)}")
    print(f"generated: {YAML_PATH.relative_to(ROOT)}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check", action="store_true", help="verify generated files without writing"
    )
    parser.add_argument(
        "--profile", choices=("ideal", "nominal"), help="passive-parameter profile"
    )
    args = parser.parse_args()
    synchronize(args.check, args.profile)


if __name__ == "__main__":
    main()
