#!/usr/bin/env python3
"""Generate fixed, free-ground, and low-drop scenes from robot_dynamics.xml."""

from __future__ import annotations

import argparse
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from sync_inertials import DYNAMICS_PATH, ROOT, format_number, require


CONFIG_PATH = ROOT / "params" / "contact_params.yaml"
SCENE_DIR = ROOT / "scenes"
SCENES = {
    "fixed_base.xml": (False, False),
    "free_ground.xml": (True, False),
    "drop_test.xml": (True, True),
}


def _load_config() -> dict:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    require(config.get("schema_version") == 1, "unsupported contact schema")
    return config


def _disable_mesh_collisions(root: ET.Element) -> None:
    for geom in root.findall("./worldbody//geom[@type='mesh']"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        geom.set("group", "2")


def _add_collision_geometry(root: ET.Element, config: dict) -> None:
    ground = config["ground"]
    friction = " ".join(
        format_number(float(ground[name]))
        for name in ("sliding_friction", "torsional_friction", "rolling_friction")
    )
    worldbody = root.find("./worldbody")
    require(worldbody is not None, "missing worldbody")
    plane = ET.Element(
        "geom",
        name="ground",
        type="plane",
        size="2 2 0.1",
        pos="0 0 0",
        friction=friction,
        margin=format_number(float(ground["contact_margin_m"])),
        solref=" ".join(map(format_number, ground["solref"])),
        solimp=" ".join(map(format_number, ground["solimp"])),
        rgba="0.25 0.28 0.3 1",
        contype="1",
        conaffinity="2",
    )
    worldbody.insert(0, plane)

    contact = root.find("./contact")
    require(contact is not None, "missing contact section")

    def add_ground_pair(geom_name: str) -> None:
        ET.SubElement(
            contact,
            "pair",
            geom1="ground",
            geom2=geom_name,
            condim="3",
            friction=friction,
            margin=format_number(float(ground["contact_margin_m"])),
            solref=" ".join(map(format_number, ground["solref"])),
            solimp=" ".join(map(format_number, ground["solimp"])),
        )

    base_settings = config["base_collision"]
    base = root.find('./worldbody/body[@name="base_link"]')
    require(base is not None, "missing base_link")
    ET.SubElement(
        base,
        "geom",
        name="base_collision",
        type="box",
        pos=" ".join(map(format_number, base_settings["center_m"])),
        size=" ".join(map(format_number, base_settings["half_size_m"])),
        friction=friction,
        margin=format_number(float(ground["contact_margin_m"])),
        group="3",
        rgba="0.2 0.45 0.8 0.25",
        contype="2",
        conaffinity="1",
    )
    add_ground_pair("base_collision")

    for body_name in config["link_collision_mesh_bodies"]:
        body = root.find(f'.//body[@name="{body_name}"]')
        require(body is not None, f"missing collision body {body_name}")
        visual = body.find("./geom[@type='mesh']")
        require(visual is not None, f"missing visual mesh for {body_name}")
        collision_name = f"{body_name}_collision"
        attributes = {
            name: visual.get(name)
            for name in ("mesh", "pos", "quat", "euler")
            if visual.get(name) is not None
        }
        ET.SubElement(
            body,
            "geom",
            name=collision_name,
            type="mesh",
            friction=friction,
            margin=format_number(float(ground["contact_margin_m"])),
            group="3",
            rgba="0.25 0.65 0.9 0.15",
            contype="2",
            conaffinity="1",
            **attributes,
        )
        add_ground_pair(collision_name)

    wheel = config["wheel_collision"]
    size = f"{format_number(float(wheel['radius_m']))} {format_number(float(wheel['half_width_m']))}"
    for side, body_name in (("left", "link_010"), ("right", "link_025")):
        body = root.find(f'.//body[@name="{body_name}"]')
        require(body is not None, f"missing {side} wheel body")
        ET.SubElement(
            body,
            "geom",
            name=f"{side}_wheel_collision",
            type="cylinder",
            pos="0 0 0",
            size=size,
            friction=friction,
            margin=format_number(float(ground["contact_margin_m"])),
            group="3",
            rgba="0.15 0.15 0.15 0.35",
            contype="2",
            conaffinity="1",
        )
        add_ground_pair(f"{side}_wheel_collision")


def _set_scene_physics(root: ET.Element, config: dict) -> None:
    compiler = root.find("./compiler")
    option = root.find("./option")
    require(compiler is not None and option is not None, "missing compiler or option")
    compiler.set("meshdir", "..")
    option.set("timestep", format_number(float(config["scene"]["timestep_s"])))
    option.set("gravity", " ".join(map(format_number, config["scene"]["gravity_m_s2"])))


def _configure_base(root: ET.Element, config: dict, free: bool, drop: bool) -> None:
    base = root.find('./worldbody/body[@name="base_link"]')
    require(base is not None, "missing base_link")
    contact_z = float(config["scene"]["calib_mid_wheel_contact_base_z_m"])
    height = contact_z + (float(config["scene"]["drop_height_m"]) if drop else 0.0)
    if not free:
        base.set("pos", f"0 0 {format_number(height)}")
        return

    base.attrib.pop("pos", None)
    freejoint = ET.Element("freejoint", name="base_free")
    base.insert(0, freejoint)
    orientation = " ".join(
        format_number(float(value))
        for value in config["scene"]["base_orientation_quat_wxyz"]
    )
    prefix = f"0 0 {format_number(height)} {orientation}"
    for key in root.findall("./keyframe/key"):
        key.set("qpos", f"{prefix} {key.get('qpos', '')}")


def render_scene(
    free: bool,
    drop: bool,
    config: dict | None = None,
    dynamics_xml: bytes | None = None,
) -> bytes:
    config = config or _load_config()
    root = ET.fromstring(dynamics_xml) if dynamics_xml is not None else ET.parse(DYNAMICS_PATH).getroot()
    suffix = "drop" if drop else ("free_ground" if free else "fixed_base")
    root.set("model", f"serial_stride_full_chassis_{suffix}")
    _disable_mesh_collisions(root)
    _add_collision_geometry(root, config)
    _set_scene_physics(root, config)
    _configure_base(root, config, free, drop)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as output:
        output.write(content)
        temporary = Path(output.name)
    os.replace(temporary, path)


def synchronize(check_only: bool) -> None:
    config = _load_config()
    expected = {
        name: render_scene(free, drop, config)
        for name, (free, drop) in SCENES.items()
    }
    if check_only:
        for name, content in expected.items():
            path = SCENE_DIR / name
            require(path.exists(), f"{path.relative_to(ROOT)} does not exist")
            require(path.read_bytes() == content, f"{path.relative_to(ROOT)} is stale")
        print("PASS: three dynamics scenes are synchronized")
        return
    for name, content in expected.items():
        path = SCENE_DIR / name
        _atomic_write(path, content)
        print(f"generated: {path.relative_to(ROOT)}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify without writing")
    args = parser.parse_args()
    synchronize(args.check)


if __name__ == "__main__":
    main()
