"""One-way eigenrays from the transducer to contributors (note 02; design §2).

Each path is a list of legs; every leg is tested by the one scene oracle.
Two-way contributions are formed by the caller from ordered (out, back)
pairs, all pairs of one contributor sharing its random factor (Brown17), so
paths interfere -- Lloyd's mirror and the image comb live in the mean
(``receiver.BinGrid.add_group``).

Implemented so far:

* ``direct``: P -> X.
* ``surface``: the image of P in the sea surface z = 0, ``P_s = (Px, Py,
  -Pz)``; the reflection point Q is where P_s -> X crosses z = 0; the path
  length is ``|X - P_s|``. The amplitude factor is the pressure-release
  reflection ``-1`` times the Eckart coherent amplitude ``sqrt(C)``,
  ``C = exp(-(2 k sigma sin psi)^2)`` at the grazing angle psi at Q
  (Eckart 1953; ``waveguide.eckart_c``). The incoherent share ``1 - C`` is
  scattered by the rough surface (an optional term, off). Legs P -> Q and
  Q -> X are each tested for occlusion.

Walls, the floor (local tangent-plane images with Fermat refinement) and
higher-order chains follow behind their gates (``raycast_plan.md``).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..waveguide import eckart_c


@dataclass
class Path:
    """One one-way path to each of n contributors."""

    name: str
    length: np.ndarray        # (n,) total length [m]
    depart: np.ndarray        # (n, 3) unit direction leaving P
    arrive: np.ndarray        # (n, 3) unit propagation direction at X
    factor: np.ndarray        # (n,) complex boundary factor (1 for direct)
    visible: np.ndarray       # (n,) bool, every leg clear


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-300)


def direct(P: np.ndarray, X: np.ndarray, oracle=None,
           eps: float = 1e-6) -> Path:
    d = X - P
    L = np.linalg.norm(d, axis=1)
    u = _unit(d)
    vis = (np.ones(len(X), dtype=bool) if oracle is None
           else ~oracle.occluded(np.broadcast_to(P, X.shape), X, eps))
    return Path("direct", L, u, u, np.ones(len(X), dtype=complex), vis)


def surface(P: np.ndarray, X: np.ndarray, k: float, rms_m: float,
            oracle=None, surface_z: float = 0.0, eps: float = 1e-6) -> Path:
    """The single surface bounce via the image of P in z = surface_z."""
    Ps = P.copy()
    Ps[2] = 2.0 * surface_z - P[2]
    d = X - Ps
    L = np.linalg.norm(d, axis=1)
    # Q on the plane z = surface_z along Ps -> X
    t = (surface_z - Ps[2]) / np.where(np.abs(d[:, 2]) > 1e-300, d[:, 2], 1e-300)
    Q = Ps + t[:, None] * d
    ok = (t > 0.0) & (t < 1.0) & (X[:, 2] < surface_z) & (P[2] < surface_z)
    depart = _unit(Q - P)
    arrive = _unit(X - Q)
    psi = np.arcsin(np.clip(np.abs(arrive[:, 2]), 0.0, 1.0))
    C = eckart_c(psi, rms_m, k) if rms_m > 0.0 else np.ones(len(X))
    factor = -np.sqrt(C).astype(complex)
    vis = ok.copy()
    if oracle is not None and ok.any():
        idx = np.flatnonzero(ok)
        Qd = Q[idx] - np.array([0.0, 0.0, 1e-4])      # just under the surface
        b1 = oracle.bvh.occluded(np.broadcast_to(P, Qd.shape), Qd, eps)
        b2 = oracle.bvh.occluded(Qd, X[idx], eps)
        vis[idx] = ~(b1 | b2)
    return Path("surface", L, depart, arrive, np.where(ok, factor, 0.0), vis)


def bistatic_angles(n: np.ndarray, d_in: np.ndarray, d_out: np.ndarray):
    """Local (sin theta_i, sin theta_s, cos phi) on a facet with unit normal
    ``n``: ``d_in`` the propagation direction arriving at the facet, ``d_out``
    the direction leaving it toward the receiver (J&R 13.1-13.4 geometry;
    backscatter, d_out = -d_in, gives cos phi = -1)."""
    si = -np.einsum("ij,ij->i", n, d_in)
    ss = np.einsum("ij,ij->i", n, d_out)
    ki = d_in + si[:, None] * n           # in-plane parts
    ks = d_out - ss[:, None] * n
    den = np.linalg.norm(ki, axis=1) * np.linalg.norm(ks, axis=1)
    cphi = np.where(den > 1e-12, np.einsum("ij,ij->i", ki, ks)
                    / np.maximum(den, 1e-300), -1.0)
    return si, ss, np.clip(cphi, -1.0, 1.0)
