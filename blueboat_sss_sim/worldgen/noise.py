"""Lightweight procedural noise (value-noise fBm) for terrain synthesis.

scipy.ndimage.zoom-based value noise: coarse random lattices are smoothly
upsampled and summed over octaves. Deterministic for a given RNG.
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage


def value_noise(shape: tuple[int, int], cells: int, rng: np.random.Generator) -> np.ndarray:
    """One octave of smooth value noise in [-1, 1].

    Args:
        shape: (ny, nx) of the output raster.
        cells: number of noise lattice cells along the longest axis
               (spatial frequency; larger = higher frequency).
        rng:   NumPy random generator.
    """
    ny, nx = shape
    cells = max(2, int(cells))
    cy = max(2, round(cells * ny / max(ny, nx)))
    cx = max(2, round(cells * nx / max(ny, nx)))
    lattice = rng.uniform(-1.0, 1.0, size=(cy, cx))
    zoomed = ndimage.zoom(lattice, (ny / cy, nx / cx), order=3, mode="reflect",
                          grid_mode=True)
    return zoomed[:ny, :nx]


def fbm(shape: tuple[int, int], base_cells: int, octaves: int,
        rng: np.random.Generator, lacunarity: float = 2.0,
        gain: float = 0.5) -> np.ndarray:
    """Fractal Brownian motion field, normalised to roughly [-1, 1]."""
    out = np.zeros(shape, dtype=np.float64)
    amp, freq, norm = 1.0, float(base_cells), 0.0
    for _ in range(max(1, int(octaves))):
        out += amp * value_noise(shape, int(freq), rng)
        norm += amp
        amp *= gain
        freq *= lacunarity
    return out / max(norm, 1e-9)
