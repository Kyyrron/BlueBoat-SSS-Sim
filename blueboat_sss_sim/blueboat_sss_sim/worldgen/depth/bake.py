"""Baking a layer stack into the world YAML's own vocabulary.

The builder keeps its layer stack in ``builder_state.yaml`` only. At
build time the stack is composed on the world grid and written as

* ``depth_baked.tif`` -- a float32 north-up geographic-CRS GeoTIFF of the
  depth (positive down) whose pixel lattice *is* the world grid, consumed
  through the ordinary ``depth.mode: file`` path (``load_geotiff``
  recognises the alignment and stamps it losslessly), and
* ``material_map.npz`` -- the material overlay and reflectivity gain,
  consumed through the one additive key ``terrain.materials.map``.

So a layered world is, to ``generate_world`` and everything downstream, a
file-mode world; ``builder_state.yaml`` alone knows it was authored in
layers. The bake is deterministic (seeded streams, no global RNG):
building the same state twice gives the same files.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from ...core.geo import GeoAnchor
from ...core.types import GridSpec, Wall
from ..material_map import write_material_map
from .compose import Composite, compose, stack_from_depth

DEPTH_BAKED_NAME = "depth_baked.tif"
MATERIAL_MAP_NAME = "material_map.npz"


@dataclass
class BakedFiles:
    depth_tif: str
    material_map: str | None
    info: dict[str, Any] = field(default_factory=dict)


def write_depth_geotiff(height: np.ndarray, grid: GridSpec,
                        anchor: GeoAnchor, path: str | Path) -> Path:
    """Write ``height`` (z, negative down) as a depth GeoTIFF whose pixel
    centres are exactly the grid's cell centres (RasterPixelIsPoint,
    EPSG:4326, north-up)."""
    from PIL import Image, TiffImagePlugin

    path = Path(path)
    xs = grid.origin_x + np.arange(grid.nx) * grid.resolution
    ys = grid.origin_y + np.arange(grid.ny) * grid.resolution
    # The equirectangular pair is separable: lon depends on east only,
    # lat on north only, both linearly.
    _, lon = anchor.to_latlon(xs, np.zeros_like(xs))
    lat, _ = anchor.to_latlon(np.zeros_like(ys), ys)
    lon, lat = np.asarray(lon, dtype=np.float64), np.asarray(lat, dtype=np.float64)
    sx = float((lon[-1] - lon[0]) / max(grid.nx - 1, 1)) if grid.nx > 1 else 1e-9
    sy = float((lat[-1] - lat[0]) / max(grid.ny - 1, 1)) if grid.ny > 1 else 1e-9
    depth = (-np.asarray(height, dtype=np.float64))[::-1].astype(np.float32)
    info = TiffImagePlugin.ImageFileDirectory_v2()
    info[33550] = (sx, sy, 0.0)
    info.tagtype[33550] = 12                       # DOUBLE
    info[33922] = (0.0, 0.0, 0.0, float(lon[0]), float(lat[-1]), 0.0)
    info.tagtype[33922] = 12
    # GTModelType geographic, GTRasterType PixelIsPoint, GeographicType 4326.
    info[34735] = (1, 1, 0, 3, 1024, 0, 1, 2, 1025, 0, 1, 2, 2048, 0, 1, 4326)
    info.tagtype[34735] = 3                        # SHORT
    Image.fromarray(np.ascontiguousarray(depth), mode="F").save(
        path, tiffinfo=info)
    return path


def bake_composite(depth_state: Any, grid: GridSpec, walls: Sequence[Wall],
                   anchor: GeoAnchor | None, seed: int,
                   offset: tuple[float, float] = (0.0, 0.0)) -> Composite:
    """Compose a ``DepthState``'s full stack on ``grid`` with the terrain
    stream ``[seed, 0]`` for the source (what an unbaked world would draw)."""
    rng = np.random.default_rng([int(seed), 0])
    return compose(grid, stack_from_depth(depth_state), walls, anchor, rng,
                   int(seed), offset=offset)


def bake_state(state: Any, out_dir: str | Path, grid: GridSpec,
               walls: Sequence[Wall], anchor: GeoAnchor,
               offset: tuple[float, float]) -> BakedFiles:
    """Compose ``state.depth`` on the build grid (world frame) and write
    the baked files into ``out_dir``."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    comp = bake_composite(state.depth, grid, walls, anchor,
                          int(state.objects.seed), offset=offset)
    tif = write_depth_geotiff(comp.height, grid, anchor, out / DEPTH_BAKED_NAME)
    mm = None
    if comp.material_id is not None or comp.gain is not None:
        mm = str(write_material_map(out / MATERIAL_MAP_NAME, grid,
                                    comp.material_id, comp.material_names,
                                    comp.gain))
    return BakedFiles(depth_tif=str(tif), material_map=mm, info=dict(comp.info))
