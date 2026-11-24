"""``material_map.npz`` -- the baked material overlay and reflectivity
gain of a layered world, referenced by the world YAML's single additive
key ``terrain.materials.map`` (numpy only; no other module imported).

Arrays: ``material_id`` int16 ``[ny, nx]`` (-1 = the base layout shows
through), ``material_names`` (index -> name), optional ``gain`` float32
``[ny, nx]`` (multiplies the rendered reflectivity), and the grid
(``origin_x``, ``origin_y``, ``resolution``) it was baked on, checked on
load against the scene grid.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from ..core.types import GridSpec

MATERIAL_MAP_FORMAT = 1


def write_material_map(path: str | Path, grid: GridSpec,
                       ids: np.ndarray | None, names: Sequence[str],
                       gain: np.ndarray | None) -> Path:
    path = Path(path)
    shape = (grid.ny, grid.nx)
    arrays = {
        "format": np.int32(MATERIAL_MAP_FORMAT),
        "material_id": (np.full(shape, -1, dtype=np.int16) if ids is None
                        else np.asarray(ids, dtype=np.int16)),
        "material_names": np.asarray(list(names), dtype=str),
        "origin_x": np.float64(grid.origin_x),
        "origin_y": np.float64(grid.origin_y),
        "resolution": np.float64(grid.resolution),
    }
    if gain is not None:
        arrays["gain"] = np.asarray(gain, dtype=np.float32)
    np.savez_compressed(path, **arrays)
    return path


def load_material_map(path: str | Path, grid: GridSpec
                      ) -> tuple[np.ndarray, list[str], np.ndarray | None]:
    """``(material_id, names, gain)`` for ``grid``; raises by name when the
    file was baked on another grid."""
    path = Path(path)
    with np.load(path, allow_pickle=False) as z:
        ids = np.asarray(z["material_id"], dtype=np.int16)
        names = [str(n) for n in z["material_names"].tolist()]
        gain = (np.asarray(z["gain"], dtype=np.float64) if "gain" in z.files
                else None)
        ox, oy, res = (float(z["origin_x"]), float(z["origin_y"]),
                       float(z["resolution"]))
    if ids.shape != (grid.ny, grid.nx):
        raise ValueError(f"{path.name}: baked on a {ids.shape[1]}x{ids.shape[0]} "
                         f"grid, the world is {grid.nx}x{grid.ny}")
    if (abs(ox - grid.origin_x) > 1e-6 or abs(oy - grid.origin_y) > 1e-6
            or abs(res - grid.resolution) > 1e-9):
        raise ValueError(f"{path.name}: baked at origin ({ox}, {oy}) @ {res} m, "
                         f"the world is ({grid.origin_x}, {grid.origin_y}) @ "
                         f"{grid.resolution} m")
    if len(names) and int(ids.max()) >= len(names):
        raise ValueError(f"{path.name}: material index {int(ids.max())} has no name")
    return ids, names, gain
