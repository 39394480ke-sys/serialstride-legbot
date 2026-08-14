"""Apply modal leg-limit conventions to a geometry-baseline MJCF tree."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "params" / "joint_limits.yaml"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def format_number(value: float) -> str:
    return f"{value:.15g}"


def load_config() -> dict:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    require(config.get("schema_version") == 2, "unsupported joint-limit schema")
    return config


def joint_qpos_indices(root: ET.Element) -> dict[str, int]:
    widths = {"hinge": 1, "slide": 1, "ball": 4, "free": 7}
    indices: dict[str, int] = {}
    offset = 0
    for joint in root.findall("./worldbody//joint"):
        name = joint.get("name", "")
        joint_type = joint.get("type", "hinge")
        require(joint_type in widths, f"{name}: unsupported joint type {joint_type}")
        indices[name] = offset
        offset += widths[joint_type]
    return indices


def _set_range(node: ET.Element, values: list[float]) -> None:
    require(len(values) == 2 and values[0] < values[1], "invalid joint range")
    node.set("range", " ".join(format_number(value) for value in values))
    node.set("limited", "true")


def _set_unlimited(node: ET.Element) -> None:
    node.attrib.pop("range", None)
    node.set("limited", "false")


def _format_values(values: list[float]) -> str:
    return " ".join(format_number(value) for value in values)


def _apply_modal_active_limits(root: ET.Element, config: dict) -> None:
    active_names: set[str] = set()
    for settings in config["active_joint_types"].values():
        mechanical = settings["calibration_mechanical_range"]
        software = settings["calibration_software_range"]
        require(
            mechanical[0] <= software[0] < software[1] <= mechanical[1],
            "calibration software range must lie inside mechanical range",
        )
        for name in settings["model_joints"]:
            active_names.add(name)
            joint = root.find(f'.//joint[@name="{name}"]')
            require(joint is not None, f"missing active joint {name}")
            _set_unlimited(joint)

    modal = config["modal_coordinates"]
    shape = modal["shape"]
    require(
        shape["mechanical_range"][0]
        <= shape["software_range"][0]
        < shape["software_range"][1]
        <= shape["mechanical_range"][1],
        "shape software range must lie inside mechanical range",
    )
    tendon = root.find("./tendon")
    if tendon is None:
        tendon = ET.Element("tendon")
        actuator = root.find("./actuator")
        require(actuator is not None, "MJCF is missing actuator section")
        root.insert(list(root).index(actuator), tendon)

    rotation = modal["rotation"]
    configured_tendons = {
        settings["name"]
        for coordinate in (shape, rotation)
        for settings in coordinate["tendons"].values()
    }
    for node in list(tendon.findall("./fixed")):
        if node.get("name") in configured_tendons:
            tendon.remove(node)

    for coordinate_name, coordinate in (("rotation", rotation), ("shape", shape)):
        coefficients = coordinate["joint_coefficients"]
        for side in ("left", "right"):
            settings = coordinate["tendons"][side]
            joints = settings["joints"]
            require(
                len(joints) == 2,
                f"{side}: {coordinate_name} tendon requires two joints",
            )
            require(
                set(joints) <= active_names,
                f"{side}: unknown active tendon joint",
            )
            attributes = {
                "name": settings["name"],
                "limited": "true" if coordinate_name == "shape" else "false",
            }
            if coordinate_name == "shape":
                attributes["range"] = _format_values(shape["mechanical_range"])
            fixed = ET.SubElement(tendon, "fixed", **attributes)
            ET.SubElement(
                fixed,
                "joint",
                joint=joints[0],
                coef=format_number(coefficients["joint_a"]),
            )
            ET.SubElement(
                fixed,
                "joint",
                joint=joints[1],
                coef=format_number(coefficients["joint_b"]),
            )

    actuator = root.find("./actuator")
    require(actuator is not None, "MJCF is missing actuator section")
    template = actuator.find("./position")
    require(template is not None, "MJCF is missing position actuator template")
    tuning = {
        key: template.get(key, "")
        for key in ("kp", "dampratio", "timeconst")
    }
    tuning["kp"] = format_number(2 * float(tuning["kp"]))
    for node in list(actuator):
        actuator.remove(node)
    for side in ("left", "right"):
        for coordinate_name, coordinate, control_range in (
            ("rotation", rotation, rotation["observation_range"]),
            ("shape", shape, shape["software_range"]),
        ):
            ET.SubElement(
                actuator,
                "position",
                name=f"{side}_leg_{coordinate_name}_position",
                tendon=coordinate["tendons"][side]["name"],
                **tuning,
                ctrlrange=_format_values(control_range),
                ctrllimited="true",
            )


def _add_rotation_keyframes(root: ET.Element, config: dict, indices: dict[str, int]) -> None:
    keyframe = root.find("./keyframe")
    require(keyframe is not None, "MJCF is missing keyframe section")
    base = keyframe.find('./key[@name="CALIB_MID"]')
    require(base is not None, "missing CALIB_MID keyframe")
    configured = config["modal_coordinates"]["viewer_keyframes"]
    for node in list(keyframe.findall("./key")):
        if node.get("name") in configured:
            keyframe.remove(node)

    actuators = list(root.findall("./actuator/*"))
    actuator_indices = {
        node.get("name", ""): index for index, node in enumerate(actuators)
    }
    for key in keyframe.findall("./key"):
        qpos = [float(value) for value in key.get("qpos", "").split()]
        controls = [0.0] * len(actuators)
        for side in ("left", "right"):
            q_a = qpos[indices[f"{side}_joint_a"]]
            q_b = qpos[indices[f"{side}_joint_b"]]
            values = {
                "rotation": (q_a - q_b) / 2,
                "shape": (q_a + q_b) / 2,
            }
            for coordinate_name, value in values.items():
                actuator_name = f"{side}_leg_{coordinate_name}_position"
                require(
                    actuator_name in actuator_indices,
                    f"missing modal actuator {actuator_name}",
                )
                controls[actuator_indices[actuator_name]] = value
        key.set("ctrl", _format_values(controls))
        key.set("act", _format_values(controls))

    for name, rotation in configured.items():
        node = ET.fromstring(ET.tostring(base))
        node.set("name", name)
        qpos = [float(value) for value in node.get("qpos", "").split()]
        controls = [0.0] * len(actuators)
        for side in ("left", "right"):
            targets = {
                f"{side}_joint_a": rotation,
                f"{side}_joint_b": -rotation,
            }
            for joint, value in targets.items():
                require(joint in indices, f"missing keyframe joint {joint}")
                qpos[indices[joint]] = value
            actuator_name = f"{side}_leg_rotation_position"
            controls[actuator_indices[actuator_name]] = rotation
        node.set("qpos", _format_values(qpos))
        node.set("ctrl", _format_values(controls))
        node.set("act", _format_values(controls))
        keyframe.append(node)


def apply_joint_limits(tree: ET.ElementTree, config: dict | None = None) -> None:
    config = config or load_config()
    root = tree.getroot()
    indices = joint_qpos_indices(root)

    _apply_modal_active_limits(root, config)

    keyframes = {node.get("name", ""): node for node in root.findall("./keyframe/key")}
    for side in ("left", "right"):
        settings = config["gas_spring_slides"][side]
        name = settings["joint"]
        pose = settings["normalization_pose"]
        require(name in indices, f"missing gas-spring slide {name}")
        require(pose in keyframes, f"missing normalization pose {pose}")
        index = indices[name]
        reference_qpos = float(keyframes[pose].get("qpos", "").split()[index])
        joint = root.find(f'.//joint[@name="{name}"]')
        require(joint is not None, f"missing slide joint {name}")
        joint.set("ref", format_number(-reference_qpos))
        _set_range(joint, settings["range"])
        for key in keyframes.values():
            qpos = [float(value) for value in key.get("qpos", "").split()]
            require(index < len(qpos), f"{key.get('name')}: incomplete qpos")
            qpos[index] -= reference_qpos
            if math.isclose(qpos[index], 0.0, abs_tol=1e-14):
                qpos[index] = 0.0
            key.set("qpos", " ".join(format_number(value) for value in qpos))

    for side in ("left", "right"):
        for name, settings in config["passive_hinges"][side].items():
            joint = root.find(f'.//joint[@name="{name}"]')
            require(joint is not None, f"missing passive joint {name}")
            _set_range(joint, settings["model_range"])

    for name in config["continuous_joints"]:
        joint = root.find(f'.//joint[@name="{name}"]')
        require(joint is not None, f"missing continuous joint {name}")
        _set_unlimited(joint)

    _add_rotation_keyframes(root, config, indices)
