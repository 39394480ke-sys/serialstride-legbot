#!/usr/bin/env python3

import csv
import math
import unittest
from pathlib import Path

from forward_kinematics import forward_kinematics, raw_to_joint_angles


class ForwardKinematicsTest(unittest.TestCase):
    def test_raw_angle_conversion(self) -> None:
        self.assertEqual(raw_to_joint_angles(1.008, -1.307), (0.0, -0.0))
        q_a, q_b = raw_to_joint_angles(1.181, -1.484)
        self.assertAlmostEqual(q_a, 0.173)
        self.assertAlmostEqual(q_b, 0.177)

    def test_symmetric_cad_reference_poses(self) -> None:
        path = Path(__file__).resolve().parent / "reference_poses.csv"
        with path.open(newline="", encoding="utf-8") as stream:
            poses = list(csv.DictReader(stream))

        for pose in poses:
            with self.subTest(pose=pose["pose"]):
                result = forward_kinematics(
                    float(pose["qA_rad"]), float(pose["qB_rad"])
                )
                position_error = math.hypot(
                    result["wheel_x"] - float(pose["Hx_m"]),
                    result["wheel_z"] - float(pose["Hz_m"]),
                )
                self.assertLess(position_error, 5e-6)
                self.assertAlmostEqual(result["leg_length"], -result["wheel_z"], 6)
                self.assertAlmostEqual(result["leg_angle"], 0.0, 6)


if __name__ == "__main__":
    unittest.main()
