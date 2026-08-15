"""Validate Phase 3 conventions inherited from authoritative geometry."""

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


def _float_pair(value: str | None, label: str) -> tuple[float, float]:
    require(value is not None, f"{label}: missing range")
    values = tuple(float(item) for item in value.split())
    require(len(values) == 2, f"{label}: expected a two-value range")
    return values  # type: ignore[return-value]


def validate_authoritative_geometry(
    tree: ET.ElementTree, config: dict | None = None
) -> None:
    """Check Phase 3 interfaces without modifying the MJCF tree."""

    config = config or load_config()
    root = tree.getroot()
    indices = joint_qpos_indices(root)

    active_names: set[str] = set()
    for short_name, settings in config["active_joint_types"].items():
        mechanical = settings["calibration_mechanical_range"]
        software = settings["calibration_software_range"]
        require(
            mechanical[0] <= software[0] < software[1] <= mechanical[1],
            f"{short_name}: software range must lie inside mechanical range",
        )
        for name in settings["model_joints"]:
            active_names.add(name)
            joint = root.find(f'.//joint[@name="{name}"]')
            require(joint is not None, f"missing active joint {name}")
            require(
                joint.get("limited") == "false" and joint.get("range") is None,
                f"{name}: authoritative geometry joint must remain continuous",
            )

    modal = config["modal_coordinates"]
    for coordinate_name in ("rotation", "shape"):
        coordinate = modal[coordinate_name]
        coefficients = coordinate["joint_coefficients"]
        for side in ("left", "right"):
            settings = coordinate["tendons"][side]
            tendon = root.find(f'./tendon/fixed[@name="{settings["name"]}"]')
            require(tendon is not None, f"missing tendon {settings['name']}")
            require(
                [node.get("joint") for node in tendon.findall("./joint")]
                == settings["joints"],
                f"{settings['name']}: joint mapping changed",
            )
            require(
                [float(node.get("coef", "nan")) for node in tendon.findall("./joint")]
                == [coefficients["joint_a"], coefficients["joint_b"]],
                f"{settings['name']}: coefficients changed",
            )
            if coordinate_name == "shape":
                require(tendon.get("limited") == "true", f"{settings['name']}: must be limited")
                require(
                    _float_pair(tendon.get("range"), settings["name"])
                    == tuple(coordinate["mechanical_range"]),
                    f"{settings['name']}: mechanical range changed",
                )
            else:
                require(tendon.get("limited") == "false", f"{settings['name']}: must be continuous")

    keyframes = {node.get("name", ""): node for node in root.findall("./keyframe/key")}
    for side in ("left", "right"):
        settings = config["gas_spring_slides"][side]
        name = settings["joint"]
        require(name in indices, f"missing gas-spring slide {name}")
        joint = root.find(f'.//joint[@name="{name}"]')
        require(joint is not None, f"missing gas-spring slide {name}")
        actual_range = _float_pair(joint.get("range"), name)
        require(actual_range == tuple(settings["range"]), f"{name}: slide range changed")
        index = indices[name]
        require("EXTEND" in keyframes and "CROUCH" in keyframes, "missing slide keyframes")
        extend = float(keyframes["EXTEND"].get("qpos", "").split()[index])
        crouch = float(keyframes["CROUCH"].get("qpos", "").split()[index])
        require(math.isclose(extend, 0.0, abs_tol=1e-12), f"{name}: EXTEND is not zero")
        expected = -config["gas_spring_slides"]["model_derived_pose_span_m"]
        require(math.isclose(crouch, expected, abs_tol=1e-9), f"{name}: CROUCH compression changed")

    for side in ("left", "right"):
        for name, settings in config["passive_hinges"][side].items():
            joint = root.find(f'.//joint[@name="{name}"]')
            require(joint is not None, f"missing passive joint {name}")
            require(
                _float_pair(joint.get("range"), name) == tuple(settings["model_range"]),
                f"{name}: passive range changed",
            )

    for name in config["continuous_joints"]:
        joint = root.find(f'.//joint[@name="{name}"]')
        require(joint is not None, f"missing continuous joint {name}")
        require(
            joint.get("limited") == "false" and joint.get("range") is None,
            f"{name}: wheel joint must remain continuous",
        )

    require(active_names == {"left_joint_a", "left_joint_b", "right_joint_a", "right_joint_b"}, "unexpected active-joint interface")
