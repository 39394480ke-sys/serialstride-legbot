#!/usr/bin/env python3
"""Audit full-chassis mass properties and emit reproducible reports."""

from __future__ import annotations

import csv
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
RESULTS = ROOT / "results"
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(ROOT))

from sync_inertials import (  # noqa: E402
    DYNAMICS_PATH,
    GEOMETRY_PATH,
    check_frozen_geometry,
    load_rows,
    resolve_rows,
    require,
)
from validate_model import MuJoCo, _numbers_in_section  # noqa: E402


BASELINE_TOTAL_MASS_KG = 3.76954478618
EXPECTED_CATEGORIES = {
    "base_link",
    "left_links",
    "right_links",
    "left_wheel",
    "right_wheel",
    "left_gas_spring",
    "right_gas_spring",
}


def _numbers(node: ET.Element, attribute: str) -> tuple[float, ...]:
    return tuple(float(value) for value in node.get(attribute, "").split())


def _equivalent_box(mass: float, inertia: tuple[float, ...]) -> list[float]:
    ixx, iyy, izz = inertia
    squares = (
        6.0 * (iyy + izz - ixx) / mass,
        6.0 * (ixx + izz - iyy) / mass,
        6.0 * (ixx + iyy - izz) / mass,
    )
    return [math.sqrt(max(value, 0.0)) for value in squares]


def _assert_xml_matches(resolved: dict[str, object]) -> ET.ElementTree:
    tree = ET.parse(DYNAMICS_PATH)
    root = tree.getroot()
    compiler = root.find("compiler")
    require(compiler is not None, "dynamics MJCF has no compiler")
    require(
        compiler.get("balanceinertia") == "false",
        "robot_dynamics.xml must set balanceinertia=false",
    )
    bodies = {node.get("name", ""): node for node in root.findall(".//body")}
    require(set(bodies) == set(resolved), "CSV and dynamics MJCF body sets differ")
    for name, item in resolved.items():
        inertial = bodies[name].find("inertial")
        require(inertial is not None, f"{name}: missing explicit inertial")
        actual = (
            float(inertial.get("mass", "nan")),
            *_numbers(inertial, "pos"),
            *_numbers(inertial, "quat"),
            *_numbers(inertial, "diaginertia"),
        )
        expected = (item.mass, *item.com, *item.quat, *item.principal)
        require(len(actual) == len(expected), f"{name}: incomplete inertial")
        require(
            all(
                math.isclose(left, right, rel_tol=1e-10, abs_tol=1e-14)
                for left, right in zip(actual, expected)
            ),
            f"{name}: MJCF does not match body_mass_map.csv",
        )
    return tree


def _pose_coms(tree: ET.ElementTree) -> tuple[list[dict[str, object]], float, float]:
    keys = tree.getroot().findall("./keyframe/key")
    runtime = MuJoCo()
    reports: list[dict[str, object]] = []
    max_residual = 0.0
    max_velocity = 0.0
    expected_values = (len(tree.getroot().findall(".//body")) + 1) * 3
    for index, key in enumerate(keys):
        text = runtime.formatted_data(DYNAMICS_PATH, keyframe=index)
        subtree_com = _numbers_in_section(text, "SUBTREE_COM")
        require(
            len(subtree_com) == expected_values,
            f"{key.get('name')}: unexpected SUBTREE_COM size",
        )
        velocities = _numbers_in_section(text, "QVEL")
        residuals = _numbers_in_section(text, "EFC_POS")
        velocity = max(map(abs, velocities), default=0.0)
        residual = max(map(abs, residuals), default=0.0)
        max_velocity = max(max_velocity, velocity)
        max_residual = max(max_residual, residual)
        reports.append(
            {
                "pose": key.get("name"),
                "model_com_m": subtree_com[3:6],
                "reference_com_m": None,
                "comparison_status": "UNVERIFIED",
                "max_abs_qvel": velocity,
                "max_abs_equality_residual_m": residual,
            }
        )
    return reports, max_velocity, max_residual


def _write_com_csv(pose_reports: list[dict[str, object]]) -> None:
    path = RESULTS / "com_comparison.csv"
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "pose",
                "model_com_x_m",
                "model_com_y_m",
                "model_com_z_m",
                "reference_com_x_m",
                "reference_com_y_m",
                "reference_com_z_m",
                "comparison_status",
                "notes",
            ]
        )
        for report in pose_reports:
            writer.writerow(
                [
                    report["pose"],
                    *report["model_com_m"],
                    "",
                    "",
                    "",
                    "UNVERIFIED",
                    "No pose-matched SolidWorks or physical COM reference supplied.",
                ]
            )


def main() -> None:
    check_frozen_geometry()
    order, rows = load_rows()
    resolved = resolve_rows(order, rows)
    tree = _assert_xml_matches(resolved)
    categories: dict[str, float] = defaultdict(float)
    bodies = []
    confidence_counts: dict[str, int] = defaultdict(int)
    provisional = []

    for name in order:
        item = resolved[name]
        row = item.row
        category = row["category"]
        require(category in EXPECTED_CATEGORIES, f"{name}: unknown category {category}")
        categories[category] += item.mass
        confidence_counts[row["confidence"]] += 1
        if row["status"] == "PROVISIONAL":
            provisional.append(name)
        bodies.append(
            {
                "body_name": name,
                "category": category,
                "side": row["side"],
                "mirror_of": row["mirror_of"] or None,
                "mass_kg": item.mass,
                "com_m": list(item.com),
                "principal_inertia_kg_m2": list(item.principal),
                "equivalent_inertia_box_m": _equivalent_box(
                    item.mass, item.principal
                ),
                "confidence": row["confidence"],
                "status": row["status"],
            }
        )

    total_mass = sum(item.mass for item in resolved.values())
    xml_mass = sum(
        float(node.get("mass", "0"))
        for node in tree.getroot().findall(".//inertial")
    )
    require(math.isclose(total_mass, xml_mass, abs_tol=1e-12), "mass totals differ")
    pose_reports, max_velocity, max_residual = _pose_coms(tree)

    RESULTS.mkdir(parents=True, exist_ok=True)
    _write_com_csv(pose_reports)
    report = {
        "schema_version": 1,
        "status": "PASS_WITH_PROVISIONAL_INPUTS",
        "model": "simulation/mujoco/full_chassis/robot_dynamics.xml",
        "frozen_geometry": str(GEOMETRY_PATH.relative_to(ROOT)),
        "structure": {
            "body_count": len(resolved),
            "all_bodies_have_explicit_inertials": True,
            "balanceinertia": False,
        },
        "mass": {
            "total_kg": total_mass,
            "frozen_baseline_total_kg": BASELINE_TOTAL_MASS_KG,
            "delta_from_frozen_baseline_kg": total_mass - BASELINE_TOTAL_MASS_KG,
            "categories_kg": dict(sorted(categories.items())),
            "confidence_body_counts": dict(sorted(confidence_counts.items())),
            "provisional_bodies": provisional,
            "physical_total_mass_comparison": {
                "status": "UNVERIFIED",
                "reason": "No precise physical whole-robot mass reference was supplied.",
            },
        },
        "com_by_pose": pose_reports,
        "solidworks_com_comparison": {
            "status": "UNVERIFIED",
            "reason": "No pose-matched SolidWorks assembly COM was supplied.",
        },
        "runtime": {
            "mujoco_version": MuJoCo().version,
            "compile": "PASS",
            "maximum_keyframe_abs_qvel": max_velocity,
            "maximum_keyframe_equality_residual_m": max_residual,
        },
        "bodies": bodies,
        "evidence_classification": {
            "verified": [
                "CSV/MJCF synchronization",
                "positive and physically balanced principal inertias",
                "MuJoCo compilation with balanceinertia=false",
                "model mass totals and keyframe COM values",
                "left/right mass and principal-inertia derivation",
            ],
            "user_observed": [
                "Existing SolidWorks-assigned masses approximately agree with the physical build."
            ],
            "unverified": [
                "Precise physical total mass",
                "Pose-matched SolidWorks or physical whole-robot COM",
                "Gas-spring physical mass",
                "Omitted fastener and moving-bearing mass allocation",
            ],
        },
    }
    output = RESULTS / "mass_audit.json"
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"PASS: {len(resolved)} explicit body inertials, balanceinertia=false")
    print(f"model total mass: {total_mass:.12f} kg")
    for category, mass in sorted(categories.items()):
        print(f"  {category}: {mass:.12f} kg")
    print(f"maximum keyframe equality residual: {max_residual:.3g} m")
    print("physical mass comparison: UNVERIFIED")
    print("SolidWorks COM comparison: UNVERIFIED")


if __name__ == "__main__":
    main()
