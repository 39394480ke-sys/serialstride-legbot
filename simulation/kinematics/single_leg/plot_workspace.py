#!/usr/bin/env python3
"""Sample the soft-limit workspace and render geometric summary plots."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt  # noqa: E402

from forward_kinematics import KinematicsError, _CALIBRATION, forward_kinematics


def _linspace(start: float, stop: float, count: int) -> list[float]:
    if count < 2:
        raise ValueError("sample count must be at least 2")
    step = (stop - start) / (count - 1)
    return [start + index * step for index in range(count)]


def sample_workspace(samples_per_joint: int) -> list[dict[str, float]]:
    joints = _CALIBRATION["joints"]
    q_a_values = _linspace(
        *map(float, joints["joint_a"]["soft_limits_q"]), samples_per_joint
    )
    q_b_values = _linspace(
        *map(float, joints["joint_b"]["soft_limits_q"]), samples_per_joint
    )

    samples: list[dict[str, float]] = []
    for q_a in q_a_values:
        for q_b in q_b_values:
            try:
                result = forward_kinematics(q_a, q_b)
            except KinematicsError:
                continue
            samples.append({"q_a": q_a, "q_b": q_b, **result})
    return samples


def render_workspace(samples: list[dict[str, float]], output: Path) -> None:
    figure, axes = plt.subplots(figsize=(7.2, 6.0), constrained_layout=True)
    points = axes.scatter(
        [sample["wheel_x"] for sample in samples],
        [sample["wheel_z"] for sample in samples],
        c=[sample["leg_length"] for sample in samples],
        cmap="viridis",
        s=3,
        linewidths=0,
        rasterized=True,
    )
    axes.scatter([0.0], [0.0], marker="+", color="black", s=70, label="Active axis A")
    axes.set_title("Single-leg wheel-center workspace")
    axes.set_xlabel("Wheel X (m)")
    axes.set_ylabel("Wheel Z (m)")
    axes.set_aspect("equal", adjustable="box")
    axes.grid(True, linewidth=0.5, alpha=0.3)
    axes.legend(loc="best")
    colorbar = figure.colorbar(points, ax=axes)
    colorbar.set_label("Leg length (m)")
    figure.savefig(output, dpi=180)
    plt.close(figure)


def render_leg_lengths(samples: list[dict[str, float]], output: Path) -> None:
    lengths = [sample["leg_length"] for sample in samples]
    minimum = min(lengths)
    maximum = max(lengths)

    figure, axes = plt.subplots(figsize=(7.2, 4.8), constrained_layout=True)
    axes.hist(lengths, bins=60, color="#287271", edgecolor="white", linewidth=0.35)
    axes.axvline(minimum, color="#b23a48", linewidth=1.4, label=f"Min {minimum:.4f} m")
    axes.axvline(maximum, color="#e09f3e", linewidth=1.4, label=f"Max {maximum:.4f} m")
    axes.set_title("Leg-length distribution within joint soft limits")
    axes.set_xlabel("Leg length (m)")
    axes.set_ylabel("Sample count")
    axes.grid(True, axis="y", linewidth=0.5, alpha=0.3)
    axes.legend(loc="best")
    figure.savefig(output, dpi=180)
    plt.close(figure)


def _main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples-per-joint", type=int, default=181)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "results",
    )
    args = parser.parse_args()

    samples = sample_workspace(args.samples_per_joint)
    if not samples:
        raise SystemExit("no valid closed-chain samples within the configured soft limits")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    workspace_path = args.output_dir / "workspace_xz.png"
    lengths_path = args.output_dir / "leg_length_range.png"
    render_workspace(samples, workspace_path)
    render_leg_lengths(samples, lengths_path)

    summary = {
        "samples_per_joint": args.samples_per_joint,
        "candidate_count": args.samples_per_joint**2,
        "valid_count": len(samples),
        "wheel_x_range_m": [
            min(sample["wheel_x"] for sample in samples),
            max(sample["wheel_x"] for sample in samples),
        ],
        "wheel_z_range_m": [
            min(sample["wheel_z"] for sample in samples),
            max(sample["wheel_z"] for sample in samples),
        ],
        "leg_length_range_m": [
            min(sample["leg_length"] for sample in samples),
            max(sample["leg_length"] for sample in samples),
        ],
        "evidence": "model-derived; no independent CAD/raw-pose accuracy validation",
    }
    (args.output_dir / "workspace_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    _main()
