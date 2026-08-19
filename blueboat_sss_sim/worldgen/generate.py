"""``generate_world`` CLI: world YAML -> scene.npz + scene_manifest.yaml + world.sdf.

Usage:
    generate_world --config config/default_world.yaml --out worlds/run_001 [--seed 7]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from .scene import SceneModel, WorldConfig, generate_scene
from .sdf_writer import write_world_sdf


def generate_world(config_path: str | Path, out_dir: str | Path,
                   seed: int | None = None,
                   plugin_prefix: str = "ignition") -> SceneModel:
    """Programmatic API used by the mission generator and the CLI."""
    with open(config_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    cfg = WorldConfig.from_dict(raw)
    if seed is not None:
        cfg.seed = int(seed)
        raw.setdefault("world", {})["seed"] = int(seed)
    scene = generate_scene(cfg, raw_config=raw)
    scene.save(out_dir)
    write_world_sdf(scene, out_dir, plugin_prefix=plugin_prefix,
                    material_overrides=cfg.material_overrides or None)
    return scene


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True, help="world YAML config")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--seed", type=int, default=None, help="override config seed")
    ap.add_argument("--plugin-prefix", choices=["ignition", "gz"], default="ignition")
    args = ap.parse_args()

    scene = generate_world(args.config, args.out, args.seed, args.plugin_prefix)
    print(f"World written to {Path(args.out).resolve()}")
    print(f"  grid: {scene.grid.nx} x {scene.grid.ny} @ {scene.grid.resolution} m")
    print(f"  objects: {len(scene.objects)}")


if __name__ == "__main__":
    main()
