"""Ground-truth contacts of the ray-cast sensor, read off its own scene.

``raycast_design.md`` §4.9 planned this module; the renderer returned
``contacts=[]`` until the review of 2026-11-11, so a ray-cast run could not
label a dataset (``BlueBoat-Perception`` builds its boxes from
``~/ground_truth/contacts`` only). NC #15: a label must not contradict its
picture -- so nothing here is a formula of the object's nominal size. Every
answer comes from the **same triangle scene** and the **same visibility
oracle** the echoes were computed with:

* **in this ping?** -- surface points of the object (cached once, on the
  triangles the mesh tags with its id) that fall inside the beam band at
  their own range, in front of the baffle;
* **visible** -- any of those points with a clear segment to the transducer
  (``SceneOracle.occluded``: terrain, other objects, quays, the sea surface);
* **extent** -- the span of the visible points in the device's range bins;
* **shadow** -- seabed points behind the object, one bin apart along the
  ping's ground line, whose **first hit** from the transducer is a triangle
  of that object; the shadow ends at the farthest such point. A slope
  lengthens it, a rise cuts it, another object or a quay in front removes
  it, a lattice breaks it (the farthest blocked point is still its end).

Ranges are the ranges the IMAGE shows: echoes are timed with the water's
sound speed and binned with the device's 1500 m/s (``receiver.BinGrid``),
so a point at true range R sits at ``R * 1500 / c``. ``slant_range_m`` keeps
the schema's meaning (true metres to the object's centre on the seabed).

Schema unchanged (``core.types.GroundTruthContact``); direct path only (no
ghost contacts: the labeler skips them). CPU-side numpy over the oracle, no
kernel; nothing here touches ``Ping.power`` or ``Ping.texture``.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

import numpy as np

from ...core.types import GroundTruthContact, Side
from ..config import SPEED_OF_SOUND_MPS


def _surface_points(tv: np.ndarray, n_max: int) -> tuple[np.ndarray, np.ndarray]:
    """Deterministic points on triangles ``tv`` (T, 3, 3), about ``n_max`` in
    all, shared out by area (every triangle gets at least one): the
    triangle index of each point and its position. A fixed low-discrepancy
    sequence in barycentric coordinates, so the set is a pure function of the
    mesh."""
    e1, e2 = tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    total = float(area.sum())
    if total <= 0.0:
        return np.zeros(0, np.int64), np.zeros((0, 3))
    per = np.clip(np.ceil(area / total * n_max).astype(np.int64), 1, 64)
    tri = np.repeat(np.arange(tv.shape[0]), per)
    k = np.concatenate([np.arange(n) for n in per]) + 1
    # additive recurrence on the plastic number: well spread for any count
    g = 1.32471795724474602596
    r1 = (0.5 + k / g) % 1.0
    r2 = (0.5 + k / (g * g)) % 1.0
    # uniform in the triangle: weights (1 - s, s (1 - r2), s r2) on (A, B, C)
    s = np.sqrt(r1)
    pos = (tv[tri, 0] + (s * (1.0 - r2))[:, None] * e1[tri]
           + (s * r2)[:, None] * e2[tri])
    return tri, pos


class LabelOracle:
    """Contacts of one renderer (``RaycastRenderer``), built once per scene."""

    def __init__(self, rr, max_points: int = 512) -> None:
        self.rr = rr
        m = rr.mesh
        self.scale = float(SPEED_OF_SOUND_MPS) / float(rr.c)
        self.offset = float(rr._rc.sampling.surface_offset_m)
        nrm = m.normals()
        self.objects = []
        for o in getattr(rr._scene, "objects", None) or []:
            tris = np.flatnonzero(m.object_id == int(o.object_id))
            if tris.size == 0:
                # nothing of it is in the scene the sonar renders (buried, or
                # smaller than the raster's cell): it cannot be seen
                continue
            tv = m.vertices[m.triangles[tris]]
            ti, pos = _surface_points(tv, int(max_points))
            if pos.shape[0] == 0:
                continue
            centre = pos.mean(axis=0)
            self.objects.append(dict(
                o=o, oid=int(o.object_id), pos=pos, nrm=nrm[tris][ti], tv=tv,
                centre=centre,
                radius=float(np.linalg.norm(pos - centre, axis=1).max())))
        self._c = (np.array([q["centre"] for q in self.objects])
                   if self.objects else np.zeros((0, 3)))
        self._r = (np.array([q["radius"] for q in self.objects])
                   if self.objects else np.zeros(0))

    # ------------------------------------------------------------- one ping
    def contacts(self, side: Side, P: np.ndarray, e_a: np.ndarray,
                 e_n: np.ndarray, bin_m: float) -> list[GroundTruthContact]:
        """The contacts of one ping-side taken from ``P`` with the array axis
        ``e_a`` and the acoustic axis ``e_n`` (``beam.transducer_frame``)."""
        if not self.objects:
            return []
        rr = self.rr
        r_max = float(rr.r_max)
        d = self._c - P
        R = np.linalg.norm(d, axis=1)
        along = np.abs(d @ e_a)
        near = ((R - self._r <= r_max)
                & (along - self._r <= rr._band_width(np.maximum(R, 0.05)))
                & (d @ e_n > -self._r))
        out: list[GroundTruthContact] = []
        for i in np.flatnonzero(near):
            c = self._one(self.objects[int(i)], side, P, e_a, e_n, bin_m, r_max)
            if c is not None:
                out.append(c)
        return out

    def _one(self, q: dict, side: Side, P: np.ndarray, e_a: np.ndarray,
             e_n: np.ndarray, bin_m: float, r_max: float
             ) -> GroundTruthContact | None:
        rr, o = self.rr, q["o"]
        dX = q["pos"] - P
        Rp = np.linalg.norm(dX, axis=1)
        inband = ((Rp <= r_max) & (Rp > 1e-3)
                  & (np.abs(dX @ e_a) <= rr._band_width(np.maximum(Rp, 0.05)))
                  & (dX @ e_n > 0.0))
        if not inband.any():
            return None                       # the object is not in this ping
        idx = np.flatnonzero(inband)
        facing = np.einsum("ij,ij->i", q["nrm"][idx], -dX[idx]) > 0.0
        cand = idx[facing]
        vis = np.zeros(cand.size, dtype=bool)
        if cand.size:
            end = q["pos"][cand] + self.offset * q["nrm"][cand]
            vis = ~rr.oracle.occluded(np.broadcast_to(P, end.shape), end)
        seen = cand[vis]
        # the object's centre on the seabed, in true metres (the schema's field)
        look = e_n[:2] / max(float(np.linalg.norm(e_n[:2])), 1e-9)
        z_obj = float(rr.sheet._locate(np.array([o.x]), np.array([o.y]))[0][0])
        across = float((np.array([o.x, o.y]) - P[:2]) @ look)
        slant = float(np.hypot(across, P[2] - z_obj))
        if seen.size == 0:
            return GroundTruthContact(
                object_id=int(o.object_id), object_type=o.type, side=side,
                slant_range_m=slant, extent_bins=0.0, shadow_bins=0.0, visible=False)
        r_dev = Rp[seen] * self.scale
        extent = max((float(r_dev.max()) - float(r_dev.min())) / bin_m, 1.0)
        shadow = self._shadow_bins(q, P, idx, dX, bin_m, r_max)
        return GroundTruthContact(
            object_id=int(o.object_id), object_type=o.type, side=side,
            slant_range_m=slant, extent_bins=round(extent, 2),
            shadow_bins=round(shadow, 2), visible=True)

    def _shadow_bins(self, q: dict, P: np.ndarray, idx: np.ndarray,
                     dX: np.ndarray, bin_m: float, r_max: float) -> float:
        """Shadow length [device bins] past the object's far foot on the
        seabed: seabed points one bin apart along the ping's ground line
        (the horizontal direction to the in-band part of the object), kept
        while their first hit from ``P`` is a triangle of this object."""
        rr = self.rr
        hxy = dX[idx, :2]
        h = hxy.mean(axis=0)
        hn = float(np.linalg.norm(h))
        if hn < 1e-6:
            return 0.0
        h = h / hn
        # The far foot: where the ping's vertical plane (through P along h)
        # leaves the object's mesh -- exact on the mesh, not a property of
        # the sampled points. An object the plane misses (beside it, inside
        # the band): its farthest in-band point.
        tv = q["tv"]
        dist = (tv[:, :, 0] - P[0]) * (-h[1]) + (tv[:, :, 1] - P[1]) * h[0]
        cuts = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            da, db = dist[:, a], dist[:, b]
            cross = da * db < 0.0
            if cross.any():
                t = da[cross] / (da[cross] - db[cross])
                pt = tv[cross, a, :2] + t[:, None] * (tv[cross, b, :2] - tv[cross, a, :2])
                cuts.append((pt - P[:2]) @ h)
        s_far = (float(np.concatenate(cuts).max()) if cuts
                 else float((hxy @ h).max()))            # ground range of the far foot
        n = int(np.ceil(max(r_max - s_far, 0.0) / bin_m)) + 1
        if n < 2:
            return 0.0
        s = s_far + bin_m * np.arange(n)
        gx, gy = P[0] + s * h[0], P[1] + s * h[1]
        gz = rr.sheet._locate(gx, gy)[0]
        G = np.stack([gx, gy, gz], axis=1)
        v = G - P
        L = np.linalg.norm(v, axis=1)
        ok = (L <= r_max) & (L > 1e-3)
        if not ok.any():
            return 0.0
        G, v, L = G[ok], v[ok], L[ok]
        hits = rr.oracle.closest(np.broadcast_to(P, G.shape), v / L[:, None],
                                 t_max=np.maximum(L - 1e-3, 0.0), t_min=1e-6)
        tri = np.asarray(hits.tri)
        hit = tri >= 0
        mine = np.zeros(L.size, dtype=bool)
        mine[hit] = rr.mesh.object_id[tri[hit]] == q["oid"]
        if not mine.any():
            return 0.0
        r_far = float(L[0]) * self.scale               # the far foot, as imaged
        r_end = float(L[np.flatnonzero(mine)[-1]]) * self.scale
        return max((r_end - r_far) / bin_m, 0.0)
