#!/usr/bin/env python3
"""Regression tests for authoritative geometry modal conventions."""

from __future__ import annotations

import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from joint_limits import CONFIG_PATH, joint_qpos_indices, load_config
from sync_inertials import DYNAMICS_PATH, GEOMETRY_PATH

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from validate_model import MuJoCo, _variant  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_PATH = ROOT.parents[1] / "kinematics" / "single_leg" / "calibration.yaml"


class JointLimitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = load_config()
        cls.geometry = ET.parse(GEOMETRY_PATH)
        cls.dynamics = ET.parse(DYNAMICS_PATH)
        cls.runtime = MuJoCo()

    def test_modal_ranges_match_physical_calibration(self) -> None:
        calibration = yaml.safe_load(CALIBRATION_PATH.read_text(encoding="utf-8"))
        root = self.geometry.getroot()
        for short_name in ("joint_a", "joint_b"):
            settings = self.config["active_joint_types"][short_name]
            self.assertEqual(
                settings["calibration_mechanical_range"],
                calibration["joints"][short_name]["mechanical_limits_q"],
            )
            self.assertEqual(
                settings["calibration_software_range"],
                calibration["joints"][short_name]["soft_limits_q"],
            )
            for name in settings["model_joints"]:
                joint = root.find(f'.//joint[@name="{name}"]')
                self.assertEqual(joint.get("limited"), "false")
                self.assertIsNone(joint.get("range"))

        calibration_joints = calibration["joints"]
        expected_mechanical = [
            (
                calibration_joints["joint_a"]["mechanical_limits_q"][index]
                + calibration_joints["joint_b"]["mechanical_limits_q"][index]
            )
            / 2
            for index in range(2)
        ]
        expected_software = [
            (
                calibration_joints["joint_a"]["soft_limits_q"][index]
                + calibration_joints["joint_b"]["soft_limits_q"][index]
            )
            / 2
            for index in range(2)
        ]
        modal = self.config["modal_coordinates"]
        for actual, expected in zip(
            modal["shape"]["mechanical_range"], expected_mechanical
        ):
            self.assertAlmostEqual(actual, expected, places=12)
        for actual, expected in zip(
            modal["shape"]["software_range"], expected_software
        ):
            self.assertAlmostEqual(actual, expected, places=12)

        expected_actuators = []
        for side in ("left", "right"):
            for coordinate_name in ("rotation", "shape"):
                coordinate = modal[coordinate_name]
                settings = coordinate["tendons"][side]
                tendon = root.find(f'./tendon/fixed[@name="{settings["name"]}"]')
                self.assertIsNotNone(tendon)
                self.assertEqual(
                    tendon.get("limited"),
                    "true" if coordinate_name == "shape" else "false",
                )
                if coordinate_name == "shape":
                    self.assertEqual(
                        [float(value) for value in tendon.get("range").split()],
                        coordinate["mechanical_range"],
                    )
                joints = tendon.findall("./joint")
                self.assertEqual(
                    [joint.get("joint") for joint in joints], settings["joints"]
                )
                self.assertEqual(
                    [float(joint.get("coef")) for joint in joints],
                    list(coordinate["joint_coefficients"].values()),
                )

                actuator_name = f"{side}_leg_{coordinate_name}_position"
                expected_actuators.append(actuator_name)
                actuator = root.find(f'./actuator/position[@name="{actuator_name}"]')
                self.assertIsNotNone(actuator)
                self.assertEqual(actuator.get("tendon"), settings["name"])
                self.assertEqual(actuator.get("ctrllimited"), "true")
                expected_range = (
                    coordinate["software_range"]
                    if coordinate_name == "shape"
                    else coordinate["observation_range"]
                )
                for actual, expected in zip(
                    [float(value) for value in actuator.get("ctrlrange").split()],
                    expected_range,
                ):
                    self.assertAlmostEqual(actual, expected, places=12)
        self.assertEqual(
            [node.get("name") for node in root.findall("./actuator/position")],
            expected_actuators,
        )

    def test_pre_migration_dynamics_preserves_geometry_keyframes(self) -> None:
        geometry_keys = self.geometry.getroot().findall("./keyframe/key")
        dynamics_keys = self.dynamics.getroot().findall("./keyframe/key")
        dynamics_indices = {
            key.get("name"): index for index, key in enumerate(dynamics_keys)
        }
        self.assertTrue(
            set(key.get("name") for key in geometry_keys) <= set(dynamics_indices)
        )
        for geometry_index, key in enumerate(geometry_keys):
            dynamics_index = dynamics_indices[key.get("name")]
            _, geometry_residual, geometry_sensors = self.runtime.evaluate(
                GEOMETRY_PATH, 0, geometry_index
            )
            _, dynamics_residual, dynamics_sensors = self.runtime.evaluate(
                DYNAMICS_PATH, 0, dynamics_index
            )
            self.assertLess(dynamics_residual, 1e-8, key.get("name"))
            self.assertAlmostEqual(geometry_residual, dynamics_residual, places=12)
            for old, new in zip(geometry_sensors, dynamics_sensors):
                self.assertAlmostEqual(old, new, places=10, msg=key.get("name"))

    def test_gas_spring_slide_coordinates(self) -> None:
        root = self.geometry.getroot()
        indices = joint_qpos_indices(root)
        keys = {key.get("name"): key for key in root.findall("./keyframe/key")}
        expected_crouch = -self.config["gas_spring_slides"][
            "model_derived_pose_span_m"
        ]
        for side in ("left", "right"):
            settings = self.config["gas_spring_slides"][side]
            index = indices[settings["joint"]]
            extend = float(keys["EXTEND"].get("qpos").split()[index])
            crouch = float(keys["CROUCH"].get("qpos").split()[index])
            self.assertAlmostEqual(extend, 0.0, places=12)
            self.assertAlmostEqual(crouch, expected_crouch, places=9)
            for key in keys.values():
                value = float(key.get("qpos").split()[index])
                self.assertGreaterEqual(value, settings["range"][0] - 1e-12)
                self.assertLessEqual(value, settings["range"][1] + 1e-12)

    def test_rotation_keyframes_preserve_mid_leg_radius(self) -> None:
        root = self.geometry.getroot()
        keys = root.findall("./keyframe/key")
        indices = {key.get("name"): index for index, key in enumerate(keys)}
        _, _, baseline = self.runtime.evaluate(
            GEOMETRY_PATH, 0, indices["CALIB_MID"]
        )
        expected_radius = math.hypot(baseline[3], baseline[5])
        configured = self.config["modal_coordinates"]["viewer_keyframes"]
        for name, expected_rotation in configured.items():
            _, residual, sensors = self.runtime.evaluate(
                GEOMETRY_PATH, 0, indices[name]
            )
            self.assertLess(residual, 1e-6, name)
            for hx, hz in ((sensors[0], sensors[2]), (sensors[3], sensors[5])):
                radius = math.hypot(hx, hz)
                rotation = math.atan2(hx, -hz)
                error = math.atan2(
                    math.sin(rotation - expected_rotation),
                    math.cos(rotation - expected_rotation),
                )
                self.assertAlmostEqual(radius, expected_radius, places=8, msg=name)
                self.assertAlmostEqual(error, 0.0, places=7, msg=name)

    def test_modal_controls_move_only_selected_leg(self) -> None:
        cases = {
            "shape": ((0.0, 0.2, 0.0, 0.0), 0.0),
            "positive_rotation": ((0.2, 0.0, 0.0, 0.0), 1.0),
            "negative_rotation": ((-0.2, 0.0, 0.0, 0.0), -1.0),
        }
        baseline = self.runtime.evaluate(GEOMETRY_PATH, 0, 0)[2]
        with tempfile.TemporaryDirectory() as temporary:
            for name, (controls, expected_sign) in cases.items():
                tree = ET.ElementTree(
                    ET.fromstring(ET.tostring(self.geometry.getroot()))
                )
                path = _variant(tree, controls, Path(temporary))
                velocity, residual, sensors = self.runtime.evaluate(path, 5000, 0)
                right_hx, right_hy, right_hz, left_hx, _, _ = sensors
                self.assertLess(velocity, 1e-4, name)
                self.assertLess(residual, 1e-6, name)
                self.assertAlmostEqual(right_hx, baseline[0], places=7, msg=name)
                self.assertAlmostEqual(right_hy, baseline[1], places=7, msg=name)
                self.assertAlmostEqual(right_hz, baseline[2], places=7, msg=name)
                if expected_sign == 0.0:
                    self.assertAlmostEqual(left_hx, 0.0, places=6, msg=name)
                else:
                    self.assertEqual(math.copysign(1.0, left_hx), expected_sign, name)
                    self.assertGreater(abs(left_hx), 0.005, name)


if __name__ == "__main__":
    unittest.main()
