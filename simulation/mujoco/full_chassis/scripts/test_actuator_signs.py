#!/usr/bin/env python3
"""Check six direct-torque motor targets and positive-coordinate response."""

from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from joint_limits import joint_qpos_indices
from generate_scenes import SCENE_DIR
from sync_inertials import ACTUATOR_PATH, DYNAMICS_PATH, ROOT

import sys

sys.path.insert(0, str(ROOT))
from validate_model import MuJoCo, _numbers_in_section  # noqa: E402


class ActuatorSignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runtime = MuJoCo()
        cls.tree = ET.parse(DYNAMICS_PATH)
        cls.root = cls.tree.getroot()
        cls.indices = joint_qpos_indices(cls.root)
        cls.motors = cls.root.findall("./actuator/motor")
        cls.config = yaml.safe_load(ACTUATOR_PATH.read_text(encoding="utf-8"))

    def test_datasheet_peak_torque_limits(self) -> None:
        for index, motor in enumerate(self.motors):
            role = self.config["motors"][index]["role"]
            limits = self.config["limits"][role]
            command_limit = float(limits["torque_command_limit_nm"])
            peak_limit = float(limits["peak_torque_nm"])
            self.assertEqual(motor.get("ctrllimited"), "true")
            self.assertEqual(motor.get("forcelimited"), "true")
            self.assertEqual(
                [float(value) for value in motor.get("ctrlrange").split()],
                [-command_limit, command_limit],
            )
            self.assertEqual(
                [float(value) for value in motor.get("forcerange").split()],
                [-peak_limit, peak_limit],
            )

    def test_commands_above_peak_are_saturated(self) -> None:
        leg_limit = float(self.config["limits"]["leg"]["peak_torque_nm"])
        wheel_limit = float(self.config["limits"]["wheel"]["peak_torque_nm"])
        tree = ET.ElementTree(ET.fromstring(ET.tostring(self.root)))
        tree.getroot().find("./compiler").set("meshdir", str(ROOT))
        tree.getroot().find('./keyframe/key[@name="CALIB_MID"]').set(
            "ctrl", f"{2 * leg_limit} {-2 * leg_limit} 0 0 {2 * wheel_limit} {-2 * wheel_limit}"
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "saturation.xml"
            tree.write(path, encoding="utf-8", xml_declaration=True)
            text = self.runtime.formatted_data(path, steps=0, keyframe=0)
        self.assertEqual(
            _numbers_in_section(text, "ACTUATOR_FORCE"),
            [leg_limit, -leg_limit, 0.0, 0.0, wheel_limit, -wheel_limit],
        )

    def test_positive_motor_torque_increases_target_coordinate(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            for motor_index, motor in enumerate(self.motors):
                tree = ET.ElementTree(ET.fromstring(ET.tostring(self.root)))
                tree.getroot().find("./compiler").set("meshdir", str(ROOT))
                # Isolate actuator-coordinate sign from gas-spring preload.
                for joint_name in ("link_012_joint", "link_023_joint"):
                    joint = tree.getroot().find(f'.//joint[@name="{joint_name}"]')
                    joint.attrib.pop("stiffness", None)
                    joint.attrib.pop("springref", None)
                controls = [0.0] * len(self.motors)
                controls[motor_index] = 0.01
                tree.getroot().find('./keyframe/key[@name="CALIB_MID"]').set(
                    "ctrl", " ".join(map(str, controls))
                )
                path = Path(temporary) / f"{motor.get('name')}.xml"
                tree.write(path, encoding="utf-8", xml_declaration=True)
                text = self.runtime.formatted_data(path, steps=5, keyframe=0)
                qvel = _numbers_in_section(text, "QVEL")
                target = self.indices[motor.get("joint")]
                self.assertGreater(qvel[target], 0.0, motor.get("name"))

    def test_leg_motor_signs_match_modal_coordinates(self) -> None:
        cases = (
            (0, "left_joint_a", "left_joint_b", 1.0),
            (1, "left_joint_a", "left_joint_b", -1.0),
            (2, "right_joint_a", "right_joint_b", 1.0),
            (3, "right_joint_a", "right_joint_b", -1.0),
        )
        with tempfile.TemporaryDirectory() as temporary:
            for motor_index, joint_a, joint_b, expected_rotation_sign in cases:
                tree = ET.ElementTree(ET.fromstring(ET.tostring(self.root)))
                tree.getroot().find("./compiler").set("meshdir", str(ROOT))
                controls = [0.0] * len(self.motors)
                controls[motor_index] = 0.01
                tree.getroot().find('./keyframe/key[@name="CALIB_MID"]').set(
                    "ctrl", " ".join(map(str, controls))
                )
                path = Path(temporary) / f"modal_{motor_index}.xml"
                tree.write(path, encoding="utf-8", xml_declaration=True)
                qvel = _numbers_in_section(
                    self.runtime.formatted_data(path, steps=5, keyframe=0), "QVEL"
                )
                q_a = qvel[self.indices[joint_a]]
                q_b = qvel[self.indices[joint_b]]
                shape_velocity = (q_a + q_b) / 2.0
                rotation_velocity = (q_a - q_b) / 2.0
                self.assertGreater(shape_velocity, 0.0)
                self.assertGreater(expected_rotation_sign * rotation_velocity, 0.0)

    def test_opposite_wheel_torques_drive_world_positive_x(self) -> None:
        source = ET.parse(SCENE_DIR / "free_ground.xml")
        with tempfile.TemporaryDirectory() as temporary:
            displacements = []
            for left_torque, right_torque in ((-0.2, 0.2), (0.2, -0.2)):
                tree = ET.ElementTree(ET.fromstring(ET.tostring(source.getroot())))
                tree.getroot().find("./compiler").set("meshdir", str(ROOT))
                tree.getroot().find('./keyframe/key[@name="CALIB_MID"]').set(
                    "ctrl", f"0 0 0 0 {left_torque} {right_torque}"
                )
                path = Path(temporary) / f"wheel_{left_torque:+g}.xml"
                tree.write(path, encoding="utf-8", xml_declaration=True)
                qpos = _numbers_in_section(
                    self.runtime.formatted_data(path, steps=100, keyframe=0), "QPOS"
                )
                displacements.append(qpos[0])
            self.assertGreater(displacements[0], 0.0, "left-negative/right-positive must drive +X")
            self.assertLess(displacements[1], 0.0, "left-positive/right-negative must drive -X")


if __name__ == "__main__":
    unittest.main()
