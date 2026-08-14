#!/usr/bin/env python3
"""Focused tests for the editable mass-property table semantics."""

from __future__ import annotations

import unittest

from sync_inertials import resolve_rows


def row(name: str, **overrides: str) -> dict[str, str]:
    values = {
        "body_name": name,
        "category": "left_links",
        "side": "left",
        "mirror_of": "",
        "mass_kg": "2",
        "inertia_reference_mass_kg": "1",
        "com_x_m": "0.1",
        "com_y_m": "0.2",
        "com_z_m": "0.3",
        "inertia_quat_w": "1",
        "inertia_quat_x": "0",
        "inertia_quat_y": "0",
        "inertia_quat_z": "0",
        "principal_i1_kg_m2": "1",
        "principal_i2_kg_m2": "1.5",
        "principal_i3_kg_m2": "2",
        "property_mode": "uniform_mass_scale",
        "confidence": "C",
        "status": "ESTIMATED",
    }
    values.update(overrides)
    return values


class MassPropertyTableTests(unittest.TestCase):
    def test_uniform_mass_scale_scales_principal_inertia(self) -> None:
        result = resolve_rows(["left"], {"left": row("left")})["left"]
        self.assertEqual(result.mass, 2.0)
        self.assertEqual(result.principal, (2.0, 3.0, 4.0))

    def test_mirror_inherits_mass_and_principal_but_keeps_local_frame(self) -> None:
        left = row("left")
        right = row(
            "right",
            mirror_of="left",
            property_mode="mirror",
            mass_kg="",
            inertia_reference_mass_kg="",
            principal_i1_kg_m2="",
            principal_i2_kg_m2="",
            principal_i3_kg_m2="",
            com_y_m="-0.2",
        )
        result = resolve_rows(["left", "right"], {"left": left, "right": right})
        self.assertEqual(result["right"].mass, result["left"].mass)
        self.assertEqual(result["right"].principal, result["left"].principal)
        self.assertEqual(result["right"].com, (0.1, -0.2, 0.3))

    def test_invalid_inertia_is_rejected(self) -> None:
        invalid = row(
            "invalid",
            mass_kg="1",
            principal_i1_kg_m2="1",
            principal_i2_kg_m2="1",
            principal_i3_kg_m2="3",
        )
        with self.assertRaisesRegex(RuntimeError, "I1 \\+ I2 < I3"):
            resolve_rows(["invalid"], {"invalid": invalid})

    def test_scaled_inertia_cannot_keep_b_confidence(self) -> None:
        overstated = row("overstated", confidence="B")
        with self.assertRaisesRegex(RuntimeError, "confidence C or D"):
            resolve_rows(["overstated"], {"overstated": overstated})


if __name__ == "__main__":
    unittest.main()
