"""Random streams and the few numerical helpers every decision goes through.

Rules of this module (POPULATION_MODEL.md, "Determinism"):

* one stream per ``(placement seed, key, key, ...)``: the keys are strings
  (a feature's stable ID, a class, a process), turned into integers with
  SHA-256, never with Python's ``hash``;
* numpy supplies **uniform doubles only** (``Generator.random``): PCG64 and
  ``SeedSequence`` are stable across numpy versions, the other distributions
  are not promised to be;
* everything that decides something is computed with ``+ - * /``, ``sqrt``
  and comparisons, which IEEE 754 rounds the same way on every platform.
  ``exp`` is :func:`dexp` (a fixed polynomial), sums that decide are
  ``numpy.cumsum`` (sequential by definition), never ``numpy.sum`` (its
  blocking follows the CPU) and never BLAS.

Pure Python and numpy.
"""

from __future__ import annotations

import hashlib
import math

import numpy as np

#: first word of every stream of this package (so that no other user of the
#: same seed shares a stream with it)
STREAM_TAG = 0x4850_4F50          # "HPOP"

_LN2 = 0.6931471805599453
_INV_FACT = tuple(1.0 / math.factorial(k) for k in range(15))


def key_int(text: str) -> int:
    """A stable 64-bit integer of a string (SHA-256)."""
    return int.from_bytes(hashlib.sha256(str(text).encode("utf-8")).digest()[:8], "big")


def stream(seed: int, *keys: str) -> np.random.Generator:
    """The generator of ``(seed, keys...)``. Two different key tuples give
    independent streams; the same tuple gives the same numbers on every
    platform."""
    words = [STREAM_TAG, int(seed) & 0xFFFFFFFFFFFFFFFF] + [key_int(k) for k in keys]
    return np.random.Generator(np.random.PCG64(np.random.SeedSequence(words)))


def dexp(x):
    """``exp(x)`` for ``x <= 0`` (scalar or array), from ``+ - *`` and a
    power of two only: bit-identical everywhere, accurate to about 1e-13
    relative. Values under ``exp(-700)`` are 0."""
    a = np.asarray(x, dtype=np.float64)
    a = np.maximum(a, -700.0)
    n = np.rint(a / _LN2)
    r = a - n * _LN2                              # |r| <= ln2 / 2
    p = np.full(a.shape, _INV_FACT[14])
    for k in range(13, -1, -1):
        p = p * r + _INV_FACT[k]
    out = np.ldexp(p, n.astype(np.int64))
    out = np.where(np.asarray(x, dtype=np.float64) < -700.0, 0.0, out)
    return float(out) if np.ndim(x) == 0 else out


def poisson_from_uniform(u: float, mean: float) -> int:
    """The Poisson(``mean``) quantile of ``u`` in [0, 1), by inversion on
    weights built outwards from the mode with ``*`` and ``/`` only (no
    ``exp``): the same integer on every platform."""
    mean = float(mean)
    if not mean > 0.0:
        return 0
    mode = int(mean)
    # relative weights w(k) / w(mode)
    down, w, k = [], 1.0, mode
    while k > 0:
        w = w * k / mean
        k -= 1
        if w < 1e-18:
            break
        down.append(w)                            # weight of k (after the decrement)
    up, w, k = [], 1.0, mode
    while True:
        k += 1
        w = w * mean / k
        if w < 1e-18:
            break
        up.append(w)
    first = mode - len(down)
    weights = down[::-1] + [1.0] + up
    total = math.fsum(weights)
    target = float(u) * total
    acc = 0.0
    for i, wk in enumerate(weights):
        acc += wk
        if acc > target:
            return first + i
    return first + len(weights) - 1


def pick(u, n: int):
    """Index in ``range(n)`` from a uniform in [0, 1) (scalar or array)."""
    i = np.minimum((np.asarray(u, dtype=np.float64) * n).astype(np.int64), n - 1)
    return int(i) if np.ndim(u) == 0 else i


def uniform(u, lo: float, hi: float):
    """``lo + u (hi - lo)``."""
    return lo + np.asarray(u, dtype=np.float64) * (hi - lo) if np.ndim(u) else lo + float(u) * (hi - lo)


def weighted_index(u: float, weights) -> int:
    """Index drawn with the given weights from one uniform (sequential
    cumulative sum)."""
    acc, total = 0.0, 0.0
    for w in weights:
        total += float(w)
    if total <= 0.0:
        raise ValueError("weighted_index: no positive weight")
    target = float(u) * total
    last = 0
    for i, w in enumerate(weights):
        if float(w) <= 0.0:
            continue
        acc += float(w)
        last = i
        if acc > target:
            return i
    return last


def cos_sin(angle):
    """``(cos, sin)`` of an angle in radians. The one place where the C
    library's rounding can reach a result: every value derived from it is
    quantised before it is written (plan.QUANT)."""
    a = np.asarray(angle, dtype=np.float64)
    c, s = np.cos(a), np.sin(a)
    return (float(c), float(s)) if np.ndim(angle) == 0 else (c, s)
