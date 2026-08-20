"""``generate_mission`` CLI: one YAML -> one self-contained run bundle.

Bundle layout (everything a run needs, reproducible from seed alone)::

    <out>/
        world.sdf, seabed.stl            # for Gazebo
        scene.npz, scene_manifest.yaml   # for the sonar sim + labeler
        trajectory.yaml                  # for sss_path_generation.py
        sonar.yaml                       # acquisition + model params
        mission_snapshot.yaml            # the resolved input config

Modes:
* explicit: the mission YAML pins world config, pattern and sonar profile;
* random:   ``randomize: true`` draws seed, density and pattern parameters
  so batch dataset generation is one shell loop.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ..worldgen.generate import generate_world
from .patterns import build_pattern


def _load(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _randomize(doc: dict[str, Any], rng: np.random.Generator) -> dict[str, Any]:
    m = doc.setdefault("mission", {})
    m["seed"] = int(rng.integers(0, 2 ** 31 - 1))
    wo = doc.setdefault("world_overrides", {})
    obj = wo.setdefault("objects", {})
    obj["density_per_hectare"] = float(rng.uniform(20, 120))
    lm = m.setdefault("lawnmower", {})
    lm["spacing"] = float(rng.uniform(6.0, 12.0))
    lm["heading_deg"] = float(rng.uniform(0.0, 180.0))
    m["pattern"] = str(rng.choice(["lawnmower", "lawnmower", "random"]))
    return doc


def generate_mission(mission_yaml: str | Path, out_dir: str | Path,
                     seed: int | None = None,
                     speed: float | None = None) -> Path:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    doc = _load(mission_yaml)
    m = doc.get("mission", {})

    rng = np.random.default_rng(seed if seed is not None else m.get("seed", 0))
    if m.get("randomize", False):
        doc = _randomize(doc, rng)
        m = doc["mission"]
    if seed is not None:
        m["seed"] = int(seed)
    mission_seed = int(m.get("seed", 0))

    # ---- world: base config + inline overrides ---------------------------
    base_world = Path(m.get("world_config", "config/default_world.yaml"))
    world_doc = _load(base_world)
    for section, patch in (doc.get("world_overrides") or {}).items():
        node = world_doc.setdefault(section, {})
        if isinstance(patch, dict):
            node.update(patch)
        else:
            world_doc[section] = patch
    world_doc.setdefault("world", {})["seed"] = mission_seed
    tmp_world_cfg = out / "_world_config.yaml"
    with open(tmp_world_cfg, "w", encoding="utf-8") as f:
        yaml.safe_dump(world_doc, f, sort_keys=False)
    scene = generate_world(tmp_world_cfg, out,
                           plugin_prefix=str(m.get("gazebo_plugin_prefix",
                                                   "ignition")))

    # ---- trajectory --------------------------------------------------------
    if speed is not None:
        m["speed_override"] = float(speed)
    traj = build_pattern(m, seed=mission_seed)
    if speed is not None and speed > 0.0:
        traj.speed = float(speed)     # duration derives from speed
    traj.save_yaml(out / "trajectory.yaml")

    # ---- sonar profile ------------------------------------------------------
    sonar_src = Path(m.get("sonar_profile", "config/default_sonar.yaml"))
    shutil.copyfile(sonar_src, out / "sonar.yaml")

    # ---- snapshot ------------------------------------------------------------
    with open(out / "mission_snapshot.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False)

    print(f"Mission bundle written to {out.resolve()}")
    print(f"  seed={mission_seed}  objects={len(scene.objects)}  "
          f"pattern={traj.name}")
    print(f"  first waypoint ({traj.waypoints[0][0]:.1f}, "
          f"{traj.waypoints[0][1]:.1f}) -> entry "
          f"({traj.waypoints[1][0]:.1f}, {traj.waypoints[1][1]:.1f})")
    print(f"  MISSION DURATION: {traj.duration:.0f} s "
          f"({traj.duration/60:.1f} min) -- {traj.total_length:.0f} m "
          f"at {traj.speed} m/s")
    print(f"  (full_mission_launch sizes path_publisher total_time to this "
          f"automatically; for manual runs use total_time:={traj.duration*1.1 + 30:.0f})")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True, help="mission YAML")
    ap.add_argument("--out", required=True, help="output bundle directory")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--speed", type=float, default=None,
                    help="survey speed [m/s]; overrides the pattern's "
                         "speed from the mission YAML")
    args = ap.parse_args()
    generate_mission(args.config, args.out, args.seed, args.speed)


if __name__ == "__main__":
    main()
