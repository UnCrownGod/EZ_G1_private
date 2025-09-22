# File: vision/sim/mujoco/scripts/build_scene.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate a MuJoCo MJCF scene from repo configs."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml

try:
    from mujoco import mjcf
except Exception as exc:  # pragma: no cover
    raise RuntimeError(
        "MuJoCo Python bindings are required. Install via `pip install mujoco` (v3.1+)."
    ) from exc


def load_config(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def build_scene(cfg: dict[str, Any]) -> mjcf.RootElement:
    root = mjcf.RootElement()
    root.compiler.angle = "radian"
    root.option.timestep = cfg.get("sim", {}).get("timestep", 0.002)
    root.option.gravity = cfg.get("sim", {}).get("gravity", [0.0, 0.0, -9.81])

    world = root.worldbody

    # Ground plane
    world.add("geom", type="plane", size=[5, 5, 0.1], rgba=[0.8, 0.8, 0.8, 1.0])

    # Table
    table_cfg = cfg.get("table", {})
    table = world.add(
        "body",
        name="table",
        pos=table_cfg.get("pose", [1.0, 0.0, 0.4]),
    )
    table.add(
        "geom",
        type="box",
        size=[s / 2.0 for s in table_cfg.get("size", [0.6, 0.6, 0.05])],
        rgba=[0.6, 0.4, 0.3, 1.0],
    )

    # Box target
    box_cfg = cfg.get("box", {})
    box = world.add(
        "body",
        name="box_target",
        pos=box_cfg.get("pose", [1.0, 0.0, 0.46]),
    )
    box.add(
        "geom",
        type="box",
        size=[s / 2.0 for s in box_cfg.get("size", [0.05, 0.05, 0.12])],
        rgba=[0.2, 0.7, 0.3, 1.0],
    )

    # Robot (will be merged later once URDF is available)
    robot_cfg = cfg.get("robot", {})
    urdf_path = robot_cfg.get("urdf_path")
    if urdf_path:
        robot_mjcf = mjcf.from_path(str(Path(urdf_path)))
        root.attach(robot_mjcf)

    # Camera sensor
    cam_cfg = cfg.get("camera", {})
    world.add(
        "camera",
        name=cam_cfg.get("name", "front_cam"),
        pos=cam_cfg.get("pose", [0.0, 0.0, 1.2])[:3],
        quat=[1, 0, 0, 0],
        fovy=cam_cfg.get("fov", 60.0),
    )

    return root


def main() -> None:
    parser = argparse.ArgumentParser(description="Build MuJoCo MJCF scene from YAML config")
    parser.add_argument("--config", default="vision/sim/mujoco/configs/demo.yaml")
    parser.add_argument("--output", default="vision/sim/mujoco/scenes/g1_demo.xml")
    args = parser.parse_args()

    cfg_path = Path(args.config)
    scene = build_scene(load_config(cfg_path))
    xml = scene.to_xml_string()
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(xml, encoding="utf-8")
    print(f"Scene written to {out_path}")


if __name__ == "__main__":
    main()
