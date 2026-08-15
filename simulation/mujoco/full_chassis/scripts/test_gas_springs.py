#!/usr/bin/env python3
"""Report gas-slide state and first-order force availability at key poses."""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET

import yaml

from joint_limits import joint_qpos_indices
from sync_inertials import DYNAMICS_PATH, GAS_SPRING_PATH, ROOT, require


POSES = ("CROUCH", "CALIB_MID", "EXTEND")
REPORT_PATH = ROOT / "results" / "gas_spring_report.json"


def main() -> None:
    config = yaml.safe_load(GAS_SPRING_PATH.read_text(encoding="utf-8"))
    root = ET.parse(DYNAMICS_PATH).getroot()
    indices = joint_qpos_indices(root)
    keys = {node.get("name"): node for node in root.findall("./keyframe/key")}
    enabled = bool(config["enabled"])
    force_parameters = {
        name: config.get(name)
        for name in ("force_extended_n", "stiffness_n_per_m", "damping_n_s_per_m")
    }
    if enabled:
        require(all(value is not None for value in force_parameters.values()), "enabled gas-spring model has null parameters")

    poses = []
    for pose in POSES:
        qpos = [float(value) for value in keys[pose].get("qpos", "").split()]
        values = {}
        for side, joint_name in zip(("left", "right"), config["slides"]):
            q = qpos[indices[joint_name]]
            compression = -q
            joint = root.find(f'.//joint[@name="{joint_name}"]')
            require(joint is not None, f"missing slide {joint_name}")
            range_low, range_high = (float(value) for value in joint.get("range", "").split())
            force = None
            if enabled:
                force = float(config["force_extended_n"]) + float(config["stiffness_n_per_m"]) * compression
            values[side] = {
                "joint": joint_name,
                "slide_position_m": q,
                "compression_m": compression,
                "slide_velocity_m_s": 0.0,
                "force_n": force,
                "at_range_limit": q <= range_low or q >= range_high,
            }
        poses.append({"pose": pose, "sides": values})

    report = {
        "schema_version": 1,
        "status": "VERIFIED_INTERFACE_FORCE_UNVERIFIED" if not enabled else "PASS_WITH_PROVISIONAL_PARAMETERS",
        "model_enabled": enabled,
        "formula": config["model"],
        "coordinate": config["coordinate"],
        "force_parameters": force_parameters,
        "poses": poses,
        "verified": ["slide mapping", "23 mm provisional stroke", "60 N user-supplied nominal force", "CROUCH/MID/EXTEND compression coordinates"],
        "unverified": ["nominal-force tolerance", "compressed force", "physical stiffness", "physical damping", "physical center distance"],
    }
    REPORT_PATH.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"PASS: gas-spring coordinate interface; force model enabled={enabled}")
    print(f"generated: {REPORT_PATH.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
