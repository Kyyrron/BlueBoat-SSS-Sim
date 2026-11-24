"""``export_scene_maps`` CLI: ground-truth maps from a world folder.

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


def _w2px(g, x, y, img_h):
    """World (x, y) -> image pixel (north-up, row 0 = y_max)."""
    return ((np.asarray(x) - g.origin_x) / g.resolution,
            img_h - 1 - (np.asarray(y) - g.origin_y) / g.resolution)


def render_objects_preview(scene: SceneModel, out_path: str | Path,
                           path_xy: np.ndarray | None = None) -> Path:
    """The Screen-5 objects map: reflectivity ground, every object circled
    and labelled, walls as solid lines, the world limit as a border, and
    the source GPS path overlaid. PIL-only, so the smoke test renders it
    headless and the GUI merely displays the same PNG."""
    g = scene.grid
    refl = np.flipud(scene.reflectivity)
    img = Image.fromarray(_to_u8(refl, 0.0, 1.0), "L").convert("RGB")
    draw = ImageDraw.Draw(img)

    # World limits: the image border, drawn explicitly.
    draw.rectangle([0, 0, img.width - 1, img.height - 1],
                   outline=(250, 210, 60), width=2)
    # Walls.
    for w in scene.walls:
        x0, y0 = _w2px(g, w.x0, w.y0, img.height)
        x1, y1 = _w2px(g, w.x1, w.y1, img.height)
        draw.line([float(x0), float(y0), float(x1), float(y1)],
                  fill=(80, 170, 255), width=3)
    # Source path.
    if path_xy is not None and len(path_xy) >= 2:
        px, py = _w2px(g, path_xy[:, 0], path_xy[:, 1], img.height)
        draw.line([c for xy in zip(px.tolist(), py.tolist()) for c in xy],
                  fill=(90, 220, 90), width=2)
    # Objects, circled so they are easy to spot.
    for o in scene.objects:
        px, py = _w2px(g, o.x, o.y, img.height)
        r = max(max(o.length, o.width) * 0.5 / g.resolution, 5)
        draw.ellipse([px - r, py - r, px + r, py + r],
                     outline=(255, 60, 60), width=2)
        draw.text((px + r + 2, py - 6), f"{o.object_id}:{o.type}",
                  fill=(255, 200, 60))
    out_path = Path(out_path)
    img.save(out_path)
    return out_path


def render_depth_image(height: np.ndarray, out_path: str | Path) -> Path:
    """Grayscale depth image (white = shallow) from a height raster --
    the Screen-5 depth panel and the ``gt_depth`` convention."""
    out_path = Path(out_path)
    Image.fromarray(_to_u8(np.flipud(height)), "L").save(out_path)
    return out_path


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
