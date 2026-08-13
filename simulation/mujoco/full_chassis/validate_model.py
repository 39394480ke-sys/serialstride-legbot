#!/usr/bin/env python3
"""Validate the curated full-chassis MJCF with the bundled MuJoCo framework."""

from __future__ import annotations

import ctypes
import math
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


DIRECTORY = Path(__file__).resolve().parent
MODEL_PATH = DIRECTORY / "robot.xml"
FRAMEWORK_PATH = Path(
    "/Applications/MuJoCo.app/Contents/Frameworks/"
    "mujoco.framework/Versions/A/libmujoco.3.9.0.dylib"
)

EXPECTED_MASS_KG = 3.76954478618
EXPECTED_COUNTS = {
    "bodies": 25,
    "joints": 24,
    "connects": 6,
    "actuators": 4,
}
EXPECTED_ACTUATED_JOINTS = {
    "left_joint_a",
    "left_joint_b",
    "right_joint_a",
    "right_joint_b",
}
EXPECTED_ACTIVE_JOINTS = {
    "left_joint_a": ("0 0 -1", "-0.346 0.346"),
    "left_joint_b": ("0 0 1", "-0.356 0.355"),
    "right_joint_a": ("0 0 1", "-0.346 0.346"),
    "right_joint_b": ("0 0 -1", "-0.356 0.355"),
}
EXPECTED_ACTIVE_REF = "0.132318159422"
EXPECTED_KEYFRAME_HZ_M = {
    "CALIB_MID": -0.1343,
    "CROUCH": -0.0849,
    "EXTEND": -0.1837,
    "INTERMEDIATE_N025": -0.09729949628693883,
    "INTERMEDIATE_N0125": -0.11532467989702938,
    "INTERMEDIATE_P0125": -0.15338820925747548,
    "INTERMEDIATE_P025": -0.17183192494453448,
}
INITIAL_RESIDUAL_LIMIT_M = 1e-8
SETTLED_VELOCITY_LIMIT = 1e-4
SETTLED_RESIDUAL_LIMIT_M = 1e-6
STEP_COUNT = 1000


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


class MuJoCo:
    def __init__(self) -> None:
        _require(FRAMEWORK_PATH.exists(), f"MuJoCo framework not found: {FRAMEWORK_PATH}")
        self.lib = ctypes.CDLL(str(FRAMEWORK_PATH))
        self.lib.mj_loadXML.argtypes = [
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        self.lib.mj_loadXML.restype = ctypes.c_void_p
        self.lib.mj_makeData.argtypes = [ctypes.c_void_p]
        self.lib.mj_makeData.restype = ctypes.c_void_p
        self.lib.mj_resetDataKeyframe.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
        ]
        self.lib.mj_forward.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.lib.mj_step.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.lib.mj_printFormattedData.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
        ]
        self.lib.mj_deleteData.argtypes = [ctypes.c_void_p]
        self.lib.mj_deleteModel.argtypes = [ctypes.c_void_p]

    def evaluate(
        self, path: Path, steps: int, keyframe: int | None
    ) -> tuple[float, float, list[float]]:
        error = ctypes.create_string_buffer(4096)
        model = self.lib.mj_loadXML(str(path).encode(), None, error, len(error))
        _require(bool(model), error.value.decode() or f"failed to load {path}")
        data = self.lib.mj_makeData(model)
        _require(bool(data), "mj_makeData failed")

        try:
            if keyframe is not None:
                self.lib.mj_resetDataKeyframe(model, data, keyframe)
                self.lib.mj_forward(model, data)
            else:
                self.lib.mj_forward(model, data)
            for _ in range(steps):
                self.lib.mj_step(model, data)

            with tempfile.NamedTemporaryFile(suffix=".txt") as output:
                self.lib.mj_printFormattedData(
                    model, data, output.name.encode(), b"%.15g"
                )
                text = Path(output.name).read_text(encoding="utf-8")
            velocities = _numbers_in_section(text, "QVEL")
            residuals = _numbers_in_section(text, "EFC_POS")
            sensors = _numbers_in_section(text, "SENSOR")
            _require(
                all(math.isfinite(value) for value in velocities + residuals),
                "non-finite simulation state",
            )
            return (
                max(map(abs, velocities), default=0.0),
                max(map(abs, residuals), default=0.0),
                sensors,
            )
        finally:
            self.lib.mj_deleteData(data)
            self.lib.mj_deleteModel(model)


def _static_checks() -> ET.ElementTree:
    tree = ET.parse(MODEL_PATH)
    root = tree.getroot()
    counts = {
        "bodies": len(root.findall(".//body")),
        "joints": len(root.findall(".//joint")),
        "connects": len(root.findall("./equality/connect")),
        "actuators": len(list(root.find("actuator") or [])),
    }
    _require(counts == EXPECTED_COUNTS, f"unexpected model counts: {counts}")

    mass = sum(float(node.get("mass", "0")) for node in root.findall(".//inertial"))
    _require(math.isclose(mass, EXPECTED_MASS_KG, abs_tol=1e-10), f"mass is {mass}")

    option = root.find("option")
    _require(option is not None, "missing option element")
    _require(option.get("gravity") == "0 0 0", "gravity is not zero")
    _require(option.get("integrator") == "implicitfast", "integrator is not implicitfast")

    pairs = [
        (node.get("site1"), node.get("site2"))
        for node in root.findall("./equality/connect")
    ]
    _require(len(pairs) == len(set(pairs)), "duplicate equality/connect pair")
    keys = root.findall("./keyframe/key")
    _require(
        [node.get("name") for node in keys] == list(EXPECTED_KEYFRAME_HZ_M),
        "unexpected standard or intermediate keyframes",
    )

    actuated_joints = {
        node.get("joint") for node in root.findall("./actuator/position")
    }
    _require(
        actuated_joints == EXPECTED_ACTUATED_JOINTS,
        f"unexpected actuated joints: {actuated_joints}",
    )
    joint_names = {node.get("name") for node in root.findall(".//joint")}
    _require(
        {"left_wheel_joint", "right_wheel_joint"}.issubset(joint_names),
        "missing a named wheel joint",
    )
    joint_nodes = {node.get("name"): node for node in root.findall(".//joint")}
    for name, (axis, joint_range) in EXPECTED_ACTIVE_JOINTS.items():
        node = joint_nodes.get(name)
        _require(node is not None, f"missing active joint {name}")
        _require(node.get("axis") == axis, f"unexpected axis for {name}")
        _require(node.get("range") == joint_range, f"unexpected range for {name}")
        _require(node.get("ref") == EXPECTED_ACTIVE_REF, f"unexpected ref for {name}")
    base = root.find('./worldbody/body[@name="base_link"]')
    _require(base is not None, "missing base_link")
    _require(
        base.get("euler") == "1.5707963267948966 0 3.141592653589793",
        "unexpected world-frame correction",
    )

    print(f"static: {counts}, mass={mass:.11f} kg")
    return tree


def _variant(tree: ET.ElementTree, controls: tuple[float, ...], directory: Path) -> Path:
    root = tree.getroot()
    compiler = root.find("compiler")
    _require(compiler is not None, "missing compiler element")
    compiler.set("meshdir", str(DIRECTORY))
    key = root.find("./keyframe/key")
    _require(key is not None, "missing home keyframe")
    key.set("ctrl", " ".join(str(value) for value in controls))
    path = directory / "robot_step_test.xml"
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def main() -> None:
    tree = _static_checks()
    mujoco = MuJoCo()
    initial_velocity, initial_residual, _ = mujoco.evaluate(MODEL_PATH, 0, None)
    _require(
        initial_residual <= INITIAL_RESIDUAL_LIMIT_M,
        f"initial equality residual {initial_residual:g} m exceeds limit",
    )
    print(f"initial: max|qvel|={initial_velocity:.3g}, max|efc_pos|={initial_residual:.3g} m")

    for index, name in enumerate(EXPECTED_KEYFRAME_HZ_M):
        velocity, residual, sensors = mujoco.evaluate(MODEL_PATH, 0, index)
        _require(
            residual < SETTLED_RESIDUAL_LIMIT_M,
            f"keyframe {name}: max|efc_pos|={residual:g} m exceeds limit",
        )
        _require(len(sensors) == 6, f"keyframe {name}: unexpected sensor data")
        left_hx, _, left_hz, right_hx, _, right_hz = sensors
        expected_hz = EXPECTED_KEYFRAME_HZ_M[name]
        _require(abs(left_hx) < 1e-6 and abs(right_hx) < 1e-6, f"keyframe {name}: Hx is not zero")
        _require(
            abs(left_hz - expected_hz) < 1e-4
            and abs(right_hz - expected_hz) < 1e-4,
            f"keyframe {name}: unexpected Hz {left_hz}, {right_hz}",
        )
        print(
            f"keyframe {name}: max|qvel|={velocity:.3g}, "
            f"max|efc_pos|={residual:.3g} m, Hz={left_hz:.4f}/{right_hz:.4f} m"
        )

    scenarios = {
        "left_joint_a_pos": (0.2, 0.0, 0.0, 0.0),
        "left_joint_a_neg": (-0.2, 0.0, 0.0, 0.0),
        "left_joint_b_pos": (0.0, 0.2, 0.0, 0.0),
        "left_joint_b_neg": (0.0, -0.2, 0.0, 0.0),
        "right_joint_a_pos": (0.0, 0.0, 0.2, 0.0),
        "right_joint_a_neg": (0.0, 0.0, -0.2, 0.0),
        "right_joint_b_pos": (0.0, 0.0, 0.0, 0.2),
        "right_joint_b_neg": (0.0, 0.0, 0.0, -0.2),
        "paired_joint_a": (0.2, 0.0, 0.2, 0.0),
        "paired_joint_b": (0.0, 0.2, 0.0, 0.2),
    }

    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        for name, controls in scenarios.items():
            variant_tree = ET.ElementTree(ET.fromstring(ET.tostring(tree.getroot())))
            path = _variant(variant_tree, controls, directory)
            velocity, residual, _ = mujoco.evaluate(path, STEP_COUNT, 0)
            _require(
                velocity < SETTLED_VELOCITY_LIMIT,
                f"{name}: max|qvel|={velocity:g} exceeds settled limit",
            )
            _require(
                residual < SETTLED_RESIDUAL_LIMIT_M,
                f"{name}: max|efc_pos|={residual:g} m exceeds limit",
            )
            print(
                f"{name}: max|qvel|={velocity:.3g}, "
                f"max|efc_pos|={residual:.3g} m"
            )

    print("PASS: full-chassis geometric-debug model")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1) from error
