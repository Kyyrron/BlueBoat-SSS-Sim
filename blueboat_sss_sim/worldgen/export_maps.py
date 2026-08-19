"""``export_scene_maps`` CLI: ground-truth maps from a mission bundle.

Renders the generated scene's **reflectivity**, **depth** and **object**
maps as georeferenced PNGs so mosaics from the visualization app can be
compared against what is actually on the seabed. This is the intended
diagnostic for "what is that dark region?" questions: dark patches in a
mosaic are usually low-reflectivity material patches (mud/seagrass) or
acoustic shadows -- overlaying the mosaic on ``reflectivity.png`` (same
world extent) answers it immediately.

Outputs (in the bundle directory, or --out):
    gt_reflectivity.png   grayscale, white = strong scatterer
    gt_depth.png          grayscale, white = shallow
    gt_objects.png        reflectivity + object markers (crosses + ids)
    gt_extent.yaml        world extent + resolution for georeferencing

Usage:
    ros2 run blueboat_sss_sim export_scene_maps --bundle ~/runs/r2
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

from ..worldgen.scene import SceneModel


def _to_u8(a: np.ndarray, lo: float | None = None,
           hi: float | None = None) -> np.ndarray:
    lo = float(a.min()) if lo is None else lo
    hi = float(a.max()) if hi is None else hi
    return (np.clip((a - lo) / max(hi - lo, 1e-9), 0, 1) * 255).astype(np.uint8)


def export_scene_maps(bundle: str | Path, out_dir: str | Path | None = None) -> Path:
    bundle = Path(bundle)
    out = Path(out_dir) if out_dir else bundle
    out.mkdir(parents=True, exist_ok=True)
    scene = SceneModel.load(bundle)
    g = scene.grid

    # Rasters are stored (ny, nx) with origin at the min corner; flip
    # vertically for image convention (row 0 = max y).
    refl = np.flipud(scene.reflectivity)
    height = np.flipud(scene.height)

    Image.fromarray(_to_u8(refl, 0.0, 1.0), "L").save(out / "gt_reflectivity.png")
    Image.fromarray(_to_u8(height), "L").save(out / "gt_depth.png")

    # Object overlay.
    img = Image.fromarray(_to_u8(refl, 0.0, 1.0), "L").convert("RGB")
    draw = ImageDraw.Draw(img)
    for o in scene.objects:
        px = (o.x - g.origin_x) / g.resolution
        py = img.height - 1 - (o.y - g.origin_y) / g.resolution
        r = max(max(o.length, o.width) * 0.5 / g.resolution, 4)
        draw.ellipse([px - r, py - r, px + r, py + r], outline=(255, 60, 60),
                     width=2)
        draw.text((px + r + 2, py - 6), f"{o.object_id}:{o.type}",
                  fill=(255, 200, 60))
    img.save(out / "gt_objects.png")

    extent = {
        "x_min": float(g.origin_x),
        "y_min": float(g.origin_y),
        "x_max": float(g.origin_x + g.nx * g.resolution),
        "y_max": float(g.origin_y + g.ny * g.resolution),
        "resolution_m": float(g.resolution),
        "image_convention": "row 0 = y_max (north-up)",
        "n_objects": len(scene.objects),
    }
    with open(out / "gt_extent.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(extent, f, sort_keys=False)

    print(f"ground-truth maps written to {out.resolve()}")
    print(f"  extent x [{extent['x_min']}, {extent['x_max']}] "
          f"y [{extent['y_min']}, {extent['y_max']}] @ {g.resolution} m, "
          f"{len(scene.objects)} objects")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bundle", required=True, help="mission bundle directory")
    ap.add_argument("--out", default=None, help="output directory (default: bundle)")
    args = ap.parse_args()
    export_scene_maps(args.bundle, args.out)


if __name__ == "__main__":
    main()
