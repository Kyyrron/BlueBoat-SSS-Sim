"""An exact, on-disk cache of ``numpy.polynomial.legendre.leggauss`` (2026-11-07).

Why: the ray-cast sensor's GPU beam tables (``sonar/raycast/gpu_render.py``,
``GpuMultipath._beam``) take Gauss-Legendre nodes up to n = 4096. numpy
computes each by a dense ``eigvalsh`` of an n x n matrix: on the system
python's numpy (1.26, the interpreter ``sss_sim_node`` runs on) that is
61 s per renderer, paid by every live start. The nodes are constants, so
this cache stores *numpy's own* result once per numpy version and returns it
bit for bit afterwards: the renderer's output is unchanged.

Only the live node installs it (:func:`install`), before building the
ray-cast renderer; offline code, tests and calibration call numpy directly.
ROS-free, Qt-free.
"""

from __future__ import annotations

import os
import threading
from pathlib import Path

import numpy as np

_LOCK = threading.Lock()
_MEMO: dict[int, tuple[np.ndarray, np.ndarray]] = {}
_ORIGINAL = None


def cache_dir() -> Path:
    """``$XDG_CACHE_HOME/blueboat_sss_sim/gauss_legendre/numpy-<version>``."""
    root = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return root / "blueboat_sss_sim" / "gauss_legendre" / f"numpy-{np.__version__}"


def leggauss(n: int, directory: Path | None = None):
    """``numpy.polynomial.legendre.leggauss(n)``, memoised in memory and on
    disk. A corrupt or unwritable cache falls back to computing."""
    n = int(n)
    with _LOCK:
        hit = _MEMO.get(n)
    if hit is not None:
        return hit[0].copy(), hit[1].copy()
    d = Path(directory) if directory is not None else cache_dir()
    f = d / f"n{n}.npz"
    x = w = None
    try:
        with np.load(f) as z:
            x, w = z["x"], z["w"]
        if x.shape != (n,) or w.shape != (n,):
            x = w = None
    except (OSError, KeyError, ValueError):
        x = w = None
    if x is None:
        compute = _ORIGINAL or np.polynomial.legendre.leggauss
        x, w = compute(n)
        try:
            d.mkdir(parents=True, exist_ok=True)
            tmp = d / f".n{n}.{os.getpid()}.npz"
            np.savez(tmp, x=x, w=w)
            os.replace(tmp, f)
        except OSError:
            pass
    with _LOCK:
        _MEMO[n] = (x, w)
    return x.copy(), w.copy()


def install() -> None:
    """Route ``numpy.polynomial.legendre.leggauss`` through the cache in this
    process (idempotent)."""
    global _ORIGINAL
    leg = np.polynomial.legendre
    if _ORIGINAL is None:
        _ORIGINAL = leg.leggauss
        leg.leggauss = leggauss
