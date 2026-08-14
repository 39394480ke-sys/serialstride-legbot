#!/usr/bin/env python3
"""Validate the immutable CAD source and authoritative geometry MJCF."""

from __future__ import annotations

import ctypes
import hashlib
import math
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


DIRECTORY = Path(__file__).resolve().parent
SOURCE_PATH = DIRECTORY / "robot_source.xml"
MODEL_PATH = DIRECTORY / "robot_geometry.xml"
FRAMEWORK_PATH = Path(
    "/Applications/MuJoCo.app/Contents/Frameworks/"
    "mujoco.framework/Versions/A/libmujoco.3.9.0.dylib"
)

EXPECTED_SOURCE_SHA256 = "9387924d3dad5ea2097e8953228b8de0cfd61661b36b10f039596c25b4e14d55"
EXPECTED_MODEL_SHA256 = "e7e1f21c22bbcc8846be8548e071819e72ccfad49df203704b56205417b800ea"
EXPECTED_MASS_KG = 3.76954478618
EXPECTED_COUNTS = {
    "bodies": 25,
    "joints": 24,
    "connects": 6,
    "tendons": 4,
    "actuators": 4,
    "keyframes": 11,
}
EXPECTED_KEYFRAMES = (
    "CALIB_MID",
    "CROUCH",
    "EXTEND",
    "INTERMEDIATE_N025",
    "INTERMEDIATE_N0125",
    "INTERMEDIATE_P0125",
    "INTERMEDIATE_P025",
    "ROTATION_N180",
    "ROTATION_N090",
    "ROTATION_P090",
    "ROTATION_P180",
)
ROTATION_KEYFRAMES = {
    "ROTATION_N180": -math.pi,
    "ROTATION_N090": -math.pi / 2,
    "ROTATION_P090": math.pi / 2,
    "ROTATION_P180": math.pi,
}
ACTIVE_JOINTS = {
    "left_joint_a": "0 0 -1",
    "left_joint_b": "0 0 1",
    "right_joint_a": "0 0 1",
    "right_joint_b": "0 0 -1",
}
EXPECTED_ACTIVE_REF = "0.132318159422"
SHAPE_MECHANICAL_RANGE = (-0.351, 0.3505)
SHAPE_SOFTWARE_RANGE = (-0.281, 0.2805)
ROTATION_RANGE = (-math.pi, math.pi)
FORBIDDEN_JOINT_ATTRIBUTES = {
    "damping",
    "frictionloss",
    "armature",
    "actuatorfrcrange",
    "actuatorfrclimited",
    "stiffness",
    "springref",
}
RESIDUAL_LIMIT_M = 1e-6
RADIUS_VARIATION_LIMIT_M = 1e-7
MODAL_ERROR_LIMIT_RAD = 1e-6


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _numbers_in_section(text: str, name: str) -> list[float]:
    match = re.search(
        rf"^{name}\n(.*?)(?=\n[A-Z][A-Z0-9_ ]+\n|\Z)",
        text,
        re.MULTILINE | re.DOTALL,
    )
    if not match:
        raise RuntimeError(f"MuJoCo data output is missing {name}")
    pattern = r"[-+]?(?:\d*\.\d+|\d+\.?)(?:e[-+]?\d+)?"
    return [float(value) for value in re.findall(pattern, match.group(1), re.I)]


def _float_pair(value: str | None) -> tuple[float, float]:
    _require(value is not None, "missing numeric pair")
    values = tuple(float(item) for item in value.split())
    _require(len(values) == 2, f"expected numeric pair, got {value!r}")
    return values  # type: ignore[return-value]


def _wrapped_error(actual: float, expected: float) -> float:
    return math.atan2(math.sin(actual - expected), math.cos(actual - expected))


def _joint_qpos_indices(root: ET.Element) -> dict[str, int]:
    widths = {"hinge": 1, "slide": 1, "ball": 4, "free": 7}
    result: dict[str, int] = {}
    offset = 0
    for node in root.findall("./worldbody//joint"):
        joint_type = node.get("type", "hinge")
        _require(joint_type in widths, f"unsupported joint type {joint_type}")
        result[node.get("name", "")] = offset
        offset += widths[joint_type]
    return result


class MuJoCo:
    def __init__(self) -> None:
        _require(FRAMEWORK_PATH.exists(), f"MuJoCo framework not found: {FRAMEWORK_PATH}")
        self.lib = ctypes.CDLL(str(FRAMEWORK_PATH))
        self.lib.mj_versionString.restype = ctypes.c_char_p
        self.lib.mj_loadXML.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        self.lib.mj_loadXML.restype = ctypes.c_void_p
        self.lib.mj_makeData.argtypes = [ctypes.c_void_p]
        self.lib.mj_makeData.restype = ctypes.c_void_p
        self.lib.mj_resetDataKeyframe.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
        self.lib.mj_forward.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.lib.mj_step.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.lib.mj_printFormattedData.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p]
        self.lib.mj_deleteData.argtypes = [ctypes.c_void_p]
        self.lib.mj_deleteModel.argtypes = [ctypes.c_void_p]

    @property
    def version(self) -> str:
        return self.lib.mj_versionString().decode("ascii")

    def formatted_data(self, path: Path, steps: int = 0, keyframe: int | None = None) -> str:
        error = ctypes.create_string_buffer(4096)
        model = self.lib.mj_loadXML(str(path).encode(), None, error, len(error))
        _require(bool(model), error.value.decode() or f"failed to load {path}")
        data = self.lib.mj_makeData(model)
        _require(bool(data), "mj_makeData failed")
        try:
            if keyframe is not None:
                self.lib.mj_resetDataKeyframe(model, data, keyframe)
            self.lib.mj_forward(model, data)
            for _ in range(steps):
                self.lib.mj_step(model, data)
            with tempfile.NamedTemporaryFile(suffix=".txt") as output:
                self.lib.mj_printFormattedData(model, data, output.name.encode(), b"%.15g")
                return Path(output.name).read_text(encoding="utf-8")
        finally:
            self.lib.mj_deleteData(data)
            self.lib.mj_deleteModel(model)

    def evaluate(self, path: Path, steps: int, keyframe: int | None) -> tuple[float, float, list[float]]:
        text = self.formatted_data(path, steps=steps, keyframe=keyframe)
        velocities = _numbers_in_section(text, "QVEL")
        residuals = _numbers_in_section(text, "EFC_POS")
        sensors = _numbers_in_section(text, "SENSOR")
        _require(all(math.isfinite(value) for value in velocities + residuals + sensors), "non-finite simulation state")
        return max(map(abs, velocities), default=0.0), max(map(abs, residuals), default=0.0), sensors


def _variant(tree: ET.ElementTree, controls: tuple[float, ...], directory: Path) -> Path:
    root = tree.getroot()
    compiler = root.find("compiler")
    _require(compiler is not None, "missing compiler element")
    compiler.set("meshdir", str(DIRECTORY))
    key = root.find('./keyframe/key[@name="CALIB_MID"]')
    _require(key is not None, "missing CALIB_MID keyframe")
    values = " ".join(str(value) for value in controls)
    key.set("ctrl", values)
    key.set("act", values)
    path = directory / "robot_step_test.xml"
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def _static_checks() -> ET.ElementTree:
    source_digest = hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest()
    geometry_digest = hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()
    _require(source_digest == EXPECTED_SOURCE_SHA256, f"immutable CAD source hash changed: {source_digest}")
    _require(geometry_digest == EXPECTED_MODEL_SHA256, f"authoritative geometry hash changed: {geometry_digest}")

    tree = ET.parse(MODEL_PATH)
    root = tree.getroot()
    counts = {
        "bodies": len(root.findall(".//body")),
        "joints": len(root.findall("./worldbody//joint")),
        "connects": len(root.findall("./equality/connect")),
        "tendons": len(root.findall("./tendon/fixed")),
        "actuators": len(list(root.find("actuator") or [])),
        "keyframes": len(root.findall("./keyframe/key")),
    }
    _require(counts == EXPECTED_COUNTS, f"unexpected geometry counts: {counts}")
    _require([key.get("name") for key in root.findall("./keyframe/key")] == list(EXPECTED_KEYFRAMES), "unexpected keyframes")
    mass = sum(float(node.get("mass", "0")) for node in root.findall(".//inertial"))
    _require(math.isclose(mass, EXPECTED_MASS_KG, abs_tol=1e-10), f"geometry debug mass is {mass}")
    _require(root.find("option").get("gravity") == "0 0 0", "geometry must use zero gravity")

    joints = {node.get("name"): node for node in root.findall("./worldbody//joint")}
    for name, axis in ACTIVE_JOINTS.items():
        node = joints[name]
        _require(node.get("axis") == axis, f"unexpected axis for {name}")
        _require(node.get("ref") == EXPECTED_ACTIVE_REF, f"unexpected ref for {name}")
        _require(node.get("limited") == "false" and node.get("range") is None, f"{name} is not continuous")
    for name in ("left_wheel_joint", "right_wheel_joint"):
        _require(joints[name].get("limited") == "false" and joints[name].get("range") is None, f"{name} is not continuous")
    for node in joints.values():
        forbidden = FORBIDDEN_JOINT_ATTRIBUTES & set(node.attrib)
        _require(not forbidden, f"{node.get('name')}: forbidden dynamics attributes {sorted(forbidden)}")

    expected_tendons = {
        "left_leg_rotation": ("false", None, (("left_joint_a", 0.5), ("left_joint_b", -0.5))),
        "right_leg_rotation": ("false", None, (("right_joint_a", 0.5), ("right_joint_b", -0.5))),
        "left_leg_shape": ("true", SHAPE_MECHANICAL_RANGE, (("left_joint_a", 0.5), ("left_joint_b", 0.5))),
        "right_leg_shape": ("true", SHAPE_MECHANICAL_RANGE, (("right_joint_a", 0.5), ("right_joint_b", 0.5))),
    }
    for name, (limited, tendon_range, expected_joints) in expected_tendons.items():
        node = root.find(f'./tendon/fixed[@name="{name}"]')
        _require(node is not None and node.get("limited") == limited, f"invalid tendon {name}")
        if tendon_range is not None:
            _require(_float_pair(node.get("range")) == tendon_range, f"invalid range for {name}")
        actual = tuple((joint.get("joint"), float(joint.get("coef", "nan"))) for joint in node.findall("./joint"))
        _require(actual == expected_joints, f"invalid coefficients for {name}: {actual}")

    expected_actuators = []
    for side in ("left", "right"):
        for mode, control_range in (("rotation", ROTATION_RANGE), ("shape", SHAPE_SOFTWARE_RANGE)):
            name = f"{side}_leg_{mode}_position"
            expected_actuators.append(name)
            node = root.find(f'./actuator/position[@name="{name}"]')
            _require(node is not None, f"missing modal actuator {name}")
            _require(node.get("tendon") == f"{side}_leg_{mode}", f"wrong tendon for {name}")
            actual_range = _float_pair(node.get("ctrlrange"))
            _require(all(abs(a - b) < 1e-12 for a, b in zip(actual_range, control_range)), f"wrong control range for {name}")
            _require(node.get("ctrllimited") == "true", f"{name} must be control-limited")
    _require([node.get("name") for node in root.findall("./actuator/position")] == expected_actuators, "unexpected modal actuator order")

    indices = _joint_qpos_indices(root)
    keys = {node.get("name"): node for node in root.findall("./keyframe/key")}
    for side, joint_name in (("left", "link_012_joint"), ("right", "link_023_joint")):
        node = joints[joint_name]
        _require(_float_pair(node.get("range")) == (-0.0231, 0.0), f"wrong {side} gas-slide range")
        index = indices[joint_name]
        _require(abs(float(keys["EXTEND"].get("qpos").split()[index])) < 1e-12, f"{side} EXTEND is not zero")
        _require(abs(float(keys["CROUCH"].get("qpos").split()[index]) + 0.023008515) < 1e-9, f"{side} CROUCH compression is wrong")
        for key in keys.values():
            value = float(key.get("qpos").split()[index])
            _require(-0.0231 - 1e-12 <= value <= 1e-12, f"{key.get('name')}: {side} gas slide is out of range")

    print(f"static: {counts}, source={source_digest}, geometry={geometry_digest}")
    return tree


def _runtime_checks(tree: ET.ElementTree) -> None:
    runtime = MuJoCo()
    root = tree.getroot()
    keys = root.findall("./keyframe/key")
    radii: dict[str, list[float]] = {"left": [], "right": []}
    for index, key in enumerate(keys):
        _, residual, sensors = runtime.evaluate(MODEL_PATH, 0, index)
        name = key.get("name", "")
        _require(residual < RESIDUAL_LIMIT_M, f"{name}: closure residual {residual:g} m")
        _require(len(sensors) == 6, f"{name}: unexpected wheel-axis sensors")
        if name == "CALIB_MID" or name in ROTATION_KEYFRAMES:
            for side, offset in (("right", 0), ("left", 3)):
                hx, hz = sensors[offset], sensors[offset + 2]
                radii[side].append(math.hypot(hx, hz))
                if name in ROTATION_KEYFRAMES:
                    angle = math.atan2(hx, -hz)
                    error = _wrapped_error(angle, ROTATION_KEYFRAMES[name])
                    _require(abs(error) < 1e-6, f"{name}/{side}: rotation error {error:g} rad")
    for side, values in radii.items():
        variation = max(values) - min(values)
        _require(variation <= RADIUS_VARIATION_LIMIT_M, f"{side}: radius variation {variation:g} m")

    indices = _joint_qpos_indices(root)
    baseline = runtime.evaluate(MODEL_PATH, 0, 0)[2]
    cases = {
        "left_rotation": ((0.2, 0.0, 0.0, 0.0), "left", 0.2, 0.0),
        "left_shape": ((0.0, 0.2, 0.0, 0.0), "left", 0.0, 0.2),
        "right_rotation": ((0.0, 0.0, 0.2, 0.0), "right", 0.2, 0.0),
        "right_shape": ((0.0, 0.0, 0.0, 0.2), "right", 0.0, 0.2),
    }
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        for name, (controls, side, target_rotation, target_shape) in cases.items():
            variant_tree = ET.ElementTree(ET.fromstring(ET.tostring(root)))
            path = _variant(variant_tree, controls, directory)
            text = runtime.formatted_data(path, steps=5000, keyframe=0)
            residual = max(map(abs, _numbers_in_section(text, "EFC_POS")), default=0.0)
            sensors = _numbers_in_section(text, "SENSOR")
            qpos = _numbers_in_section(text, "QPOS")
            q_a = qpos[indices[f"{side}_joint_a"]]
            q_b = qpos[indices[f"{side}_joint_b"]]
            actual_rotation = (q_a - q_b) / 2
            actual_shape = (q_a + q_b) / 2
            _require(abs(actual_rotation - target_rotation) < MODAL_ERROR_LIMIT_RAD, f"{name}: rotation target error")
            _require(abs(actual_shape - target_shape) < MODAL_ERROR_LIMIT_RAD, f"{name}: shape target error")
            _require(residual < RESIDUAL_LIMIT_M, f"{name}: closure residual {residual:g} m")
            untouched_offset = 0 if side == "left" else 3
            for component in range(3):
                _require(abs(sensors[untouched_offset + component] - baseline[untouched_offset + component]) < 1e-7, f"{name}: moved the other leg")

    print(f"runtime: MuJoCo {runtime.version}, 11 keyframes and four modal sliders PASS")


def main() -> None:
    tree = _static_checks()
    _runtime_checks(tree)
    print("PASS: authoritative full-chassis geometry")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1) from error
