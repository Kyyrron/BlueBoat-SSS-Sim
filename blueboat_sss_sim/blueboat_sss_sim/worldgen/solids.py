"""Rotated solid primitives for 3-D resting orientations (objects v3).

Until 2026-09-03 every object was a 2.5-D *mask*: a height profile in
[0, 1] evaluated in the yaw-rotated local frame (``objects.py``), so a drum
always stood, a pipe always lay, and nothing could rest on its side, on its
end or tilted on a slope. This module gives each catalog primitive a true
solid -- box, cylinder, cone, capsule, annulus, ellipsoid -- and answers
the only question the 2.5-D raster needs: **the highest point of the
rotated solid on the vertical line through each cell** (a z-buffer from
above), plus the lowest point, so the solid can be *settled* onto the
seabed with its lowest point exactly on the local resting plane.

Physically stable rest poses are tabulated per primitive with weights
(``stable_poses``): a box lands on one of its six faces with probability
proportional to the face area, a drum stands or lies, a cone stands or
lies on a generatrix, a tyre lies flat or (rarely) stands on its tread.
Nothing here is ever airborne or sunk beyond its burial fraction: the
contact rule is geometric, not random.

Conventions: the *object frame* is the canonical resting pose of the class
(the same frame the legacy masks use -- lying cylinders along local x,
upright cylinders along local z); a rest pose is a rotation of that frame
about a horizontal axis; yaw is applied afterwards about world z; the
resting plane is the least-squares plane of the terrain under the
footprint (slope capped), and the object rotation is composed with it so
an object on a bank leans with the bank. No ROS, no Qt, no randomness --
the draw itself lives in ``objects.py`` on a per-object stream.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# ------------------------------------------------------------- rotations
def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body -> world, ZYX (yaw about z, then pitch about y, then roll
    about x) -- the SDF / REP-103 convention."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def rpy_from_rotation(R: np.ndarray) -> tuple[float, float, float]:
    """Inverse of :func:`rotation_rpy` (ZYX), for the SDF pose."""
    pitch = float(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
    if abs(np.cos(pitch)) < 1e-9:                      # gimbal lock
        return float(np.arctan2(-R[1, 2], R[1, 1])), pitch, 0.0
    roll = float(np.arctan2(R[2, 1], R[2, 2]))
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    return roll, pitch, yaw


# ------------------------------------------------------------- primitives
@dataclass(frozen=True)
class Solid:
    """One convex-ish primitive in its canonical object frame.

    kind:  box (sx, sy, sz full sizes) | cylinder (axis, radius, length) |
           cone (radius, height; apex at +z) | capsule (axis, radius, length
           tip to tip) | annulus (r_out, r_in, height; axis z) |
           ellipsoid (semi-axes ax, ay, az).
    axis:  0 = x, 1 = y, 2 = z for cylinder / capsule.
    """

    kind: str
    a: float
    b: float
    c: float = 0.0
    axis: int = 2

    @property
    def extent(self) -> np.ndarray:
        """Half-sizes of the axis-aligned bounding box in the object frame."""
        k = self.kind
        if k == "box":
            return 0.5 * np.array([self.a, self.b, self.c])
        if k in ("cylinder", "capsule"):
            r, L = self.a, self.b
            e = np.full(3, r)
            e[self.axis] = 0.5 * L
            return e
        if k == "cone":
            return np.array([self.a, self.a, 0.5 * self.b])
        if k == "annulus":
            return np.array([self.a, self.a, 0.5 * self.c])
        if k == "ellipsoid":
            return np.array([self.a, self.b, self.c])
        raise ValueError(f"unknown solid kind {k!r}")

    @property
    def centre_offset(self) -> np.ndarray:
        """Object-frame position of the solid's bounding-box centre (the
        cone is defined base-down at z = 0 .. height)."""
        if self.kind == "cone":
            return np.array([0.0, 0.0, 0.5 * self.b])
        return np.zeros(3)


def _slab(p0: np.ndarray, d: np.ndarray, lo: float, hi: float
          ) -> tuple[np.ndarray, np.ndarray]:
    """Parameter interval of a line p0 + t d inside lo <= coordinate <= hi
    (per-sample vectors). Empty intervals come out with t_in > t_out."""
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (lo - p0) / d
        t2 = (hi - p0) / d
    t_in = np.minimum(t1, t2)
    t_out = np.maximum(t1, t2)
    parallel = np.abs(d) < 1e-12
    inside = (p0 >= lo) & (p0 <= hi)
    t_in = np.where(parallel, np.where(inside, -np.inf, np.inf), t_in)
    t_out = np.where(parallel, np.where(inside, np.inf, -np.inf), t_out)
    return t_in, t_out


def _quadratic(A: np.ndarray, B: np.ndarray, C: np.ndarray
               ) -> tuple[np.ndarray, np.ndarray]:
    """Interval where A t^2 + B t + C <= 0 (A >= 0), per sample."""
    disc = B * B - 4.0 * A * C
    ok = disc >= 0.0
    sq = np.sqrt(np.maximum(disc, 0.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-B - sq) / (2.0 * A)
        t2 = (-B + sq) / (2.0 * A)
    lin = np.abs(A) < 1e-14
    # Linear / constant fallback: B t + C <= 0.
    with np.errstate(divide="ignore", invalid="ignore"):
        tl = -C / B
    t_in = np.where(lin, np.where(B > 0, -np.inf, np.where(B < 0, tl, np.where(C <= 0, -np.inf, np.inf))), np.where(ok, t1, np.inf))
    t_out = np.where(lin, np.where(B > 0, tl, np.where(B < 0, np.inf, np.where(C <= 0, np.inf, -np.inf))), np.where(ok, t2, -np.inf))
    return t_in, t_out


def _intersect(a: tuple, b: tuple) -> tuple[np.ndarray, np.ndarray]:
    return np.maximum(a[0], b[0]), np.minimum(a[1], b[1])


def line_interval(solid: Solid, p0: np.ndarray, d: np.ndarray
                  ) -> tuple[np.ndarray, np.ndarray]:
    """Parameter interval [t_in, t_out] of the lines ``p0 + t d`` (object
    frame, shape (3, n)) inside the solid; empty where t_in > t_out."""
    k = solid.kind
    if k == "box":
        e = solid.extent
        iv = _slab(p0[0], d[0], -e[0], e[0])
        iv = _intersect(iv, _slab(p0[1], d[1], -e[1], e[1]))
        return _intersect(iv, _slab(p0[2], d[2], -e[2], e[2]))
    if k == "ellipsoid":
        s = np.array([solid.a, solid.b, solid.c])[:, None]
        q0, qd = p0 / s, d / s
        return _quadratic((qd * qd).sum(0), 2.0 * (q0 * qd).sum(0),
                          (q0 * q0).sum(0) - 1.0)
    if k in ("cylinder", "capsule", "annulus"):
        ax = solid.axis if k != "annulus" else 2
        perp = [i for i in range(3) if i != ax]
        r = solid.a
        L = solid.b if k != "annulus" else solid.c
        A = d[perp[0]] ** 2 + d[perp[1]] ** 2
        B = 2.0 * (p0[perp[0]] * d[perp[0]] + p0[perp[1]] * d[perp[1]])
        C = p0[perp[0]] ** 2 + p0[perp[1]] ** 2 - r * r
        if k == "cylinder":
            return _intersect(_quadratic(A, B, C),
                              _slab(p0[ax], d[ax], -0.5 * L, 0.5 * L))
        if k == "annulus":
            outer = _intersect(_quadratic(A, B, C),
                               _slab(p0[ax], d[ax], -0.5 * L, 0.5 * L))
            C_in = p0[perp[0]] ** 2 + p0[perp[1]] ** 2 - solid.b ** 2
            h_in, h_out = _quadratic(A, B, C_in)
            t_in, t_out = outer
            # Outer minus the hole. Covered entirely by the hole: a miss.
            # Hole open at the top: the solid ends where the hole starts.
            # Hole open at the bottom: the solid starts where it ends. A
            # hole strictly inside: two pieces, whose envelope is the outer
            # interval (top and bottom are what the raster needs).
            has_hole = h_in < h_out
            covered = has_hole & (h_in <= t_in) & (h_out >= t_out)
            top_open = has_hole & (h_in > t_in) & (h_out >= t_out) & (h_in < t_out)
            bot_open = has_hole & (h_in <= t_in) & (h_out < t_out) & (h_out > t_in)
            t_out = np.where(top_open, h_in, t_out)
            t_in = np.where(bot_open, h_out, t_in)
            t_in = np.where(covered, np.inf, t_in)
            t_out = np.where(covered, -np.inf, t_out)
            return t_in, t_out
        # capsule: cylinder of length L - 2r plus two spheres
        half = max(0.5 * L - r, 0.0)
        body = _intersect(_quadratic(A, B, C),
                          _slab(p0[ax], d[ax], -half, half))
        ivs = [body]
        for sgn in (-1.0, 1.0):
            centre = np.zeros(3); centre[ax] = sgn * half
            q0 = p0 - centre[:, None]
            ivs.append(_quadratic((d * d).sum(0), 2.0 * (q0 * d).sum(0),
                                  (q0 * q0).sum(0) - r * r))
        t_in = np.min([iv[0] for iv in ivs], axis=0)
        t_out = np.max([iv[1] for iv in ivs], axis=0)
        # the union of three overlapping intervals along one line is one
        # interval (they share the axis), so min/max is exact
        return t_in, t_out
    if k == "cone":
        r0, h = solid.a, solid.b
        # x^2 + y^2 <= (r0 (1 - z/h))^2, 0 <= z <= h
        kk = r0 / h
        gx, gy = d[0], d[1]
        gz = -kk * d[2]
        cx, cy = p0[0], p0[1]
        cz = kk * (h - p0[2])
        A = gx * gx + gy * gy - gz * gz
        B = 2.0 * (cx * gx + cy * gy - cz * gz)
        C = cx * cx + cy * cy - cz * cz
        slab = _slab(p0[2], d[2], 0.0, h)
        # A >= 0: one interval between the roots. A < 0 (the line is
        # steeper than the cone's generatrix): the inequality holds
        # OUTSIDE the roots, in two pieces; the slab 0 <= z <= h keeps the
        # real cone's piece and discards the mirror cone above the apex.
        pos = A >= 0.0
        quad = _quadratic(np.where(pos, A, 1e-300), B, C)
        # For A < 0 recompute the roots of the flipped inequality.
        disc = B * B - 4.0 * A * C
        sq = np.sqrt(np.maximum(disc, 0.0))
        with np.errstate(divide="ignore", invalid="ignore"):
            r1 = (-B + sq) / (2.0 * A)      # A < 0: r1 <= r2
            r2 = (-B - sq) / (2.0 * A)
        lo1 = _intersect((np.full_like(A, -np.inf), r1), slab)
        lo2 = _intersect((r2, np.full_like(A, np.inf)), slab)
        use1 = lo1[0] <= lo1[1]
        neg_in = np.where(use1, lo1[0], lo2[0])
        neg_out = np.where(use1, lo1[1], lo2[1])
        both = use1 & (lo2[0] <= lo2[1])
        neg_out = np.where(both, np.maximum(lo1[1], lo2[1]), neg_out)
        neg_in = np.where(both, np.minimum(lo1[0], lo2[0]), neg_in)
        nodisc = disc < 0.0
        neg_in = np.where(nodisc, np.inf, neg_in)
        neg_out = np.where(nodisc, -np.inf, neg_out)
        t_in = np.where(pos, np.maximum(quad[0], slab[0]), neg_in)
        t_out = np.where(pos, np.minimum(quad[1], slab[1]), neg_out)
        return t_in, t_out
    raise ValueError(f"unknown solid kind {k!r}")


def zbuffer(solid: Solid, R: np.ndarray, x: np.ndarray, y: np.ndarray
            ) -> tuple[np.ndarray, np.ndarray]:
    """Top and bottom world-z of the rotated solid on the vertical lines
    through (x, y) (world-aligned frame centred on the solid's bounding
    box centre); NaN where the line misses."""
    p_world = np.stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    d_world = np.array([0.0, 0.0, 1.0])
    RT = R.T
    p0 = RT @ p_world + solid.centre_offset[:, None]
    d = np.repeat((RT @ d_world)[:, None], x.size, axis=1)
    t_in, t_out = line_interval(solid, p0, d)
    hit = t_in <= t_out
    top = np.where(hit, t_out, np.nan).reshape(x.shape)
    bottom = np.where(hit, t_in, np.nan).reshape(x.shape)
    return top, bottom


# ------------------------------------------------------------- rest poses
@dataclass(frozen=True)
class RestPose:
    name: str
    roll: float
    pitch: float
    weight: float


def stable_poses(solid: Solid) -> tuple[RestPose, ...]:
    """Physically stable resting poses of the primitive on a flat floor,
    with relative weights; the first entry is the canonical pose."""
    k = solid.kind
    if k == "box":
        sx, sy, sz = solid.a, solid.b, solid.c
        return (RestPose("face:-z", 0.0, 0.0, sx * sy),
                RestPose("face:+z", np.pi, 0.0, sx * sy),
                RestPose("face:-x", 0.0, -np.pi / 2, sy * sz),
                RestPose("face:+x", 0.0, np.pi / 2, sy * sz),
                RestPose("face:-y", np.pi / 2, 0.0, sx * sz),
                RestPose("face:+y", -np.pi / 2, 0.0, sx * sz))
    if k == "cylinder" or k == "capsule":
        r, L = solid.a, solid.b
        slender = L / max(2.0 * r, 1e-6)
        if solid.axis == 2:                      # canonical: standing
            w_lie = float(np.clip(0.4 * slender, 0.3, 0.75))
            return (RestPose("standing", 0.0, 0.0, 1.0 - w_lie),
                    RestPose("inverted", np.pi, 0.0, 0.25 * (1.0 - w_lie)),
                    RestPose("lying", np.pi / 2, 0.0, w_lie))
        # canonical: lying along x; standing on an end only if stubby
        w_stand = 0.0 if (slender > 3.0 or k == "capsule") else float(
            np.clip(0.3 / slender, 0.0, 0.3))
        return (RestPose("lying", 0.0, 0.0, 1.0 - w_stand),
                RestPose("standing", 0.0, np.pi / 2, w_stand))
    if k == "cone":
        half_angle = float(np.arctan2(solid.a, solid.b))
        return (RestPose("upright", 0.0, 0.0, 0.55),
                RestPose("on_side", 0.0, np.pi / 2 - half_angle, 0.45))
    if k == "annulus":
        return (RestPose("flat", 0.0, 0.0, 0.85),
                RestPose("on_tread", np.pi / 2, 0.0, 0.15))
    return (RestPose("canonical", 0.0, 0.0, 1.0),)


def fit_plane(x: np.ndarray, y: np.ndarray, z: np.ndarray,
              max_slope_deg: float = 25.0) -> tuple[float, float, float]:
    """Least-squares plane z = a x + b y + c through the samples, slope
    capped at ``max_slope_deg`` (a rock on a steep bank still rests, it
    does not slide off in this model)."""
    A = np.stack([x.ravel(), y.ravel(), np.ones(x.size)], axis=1)
    coef, *_ = np.linalg.lstsq(A, z.ravel(), rcond=None)
    a, b, c = (float(v) for v in coef)
    g = float(np.hypot(a, b))
    g_max = float(np.tan(np.radians(max_slope_deg)))
    if g > g_max:
        a, b = a * g_max / g, b * g_max / g
        c = float(np.mean(z.ravel() - a * x.ravel() - b * y.ravel()))
    return a, b, c


def plane_rotation(a: float, b: float) -> np.ndarray:
    """Rotation taking world +z onto the normal of z = a x + b y + c."""
    n = np.array([-a, -b, 1.0])
    n /= np.linalg.norm(n)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, n)
    s = np.linalg.norm(v)
    c = float(np.dot(z, n))
    if s < 1e-12:
        return np.eye(3)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def settle(solid: Solid, R: np.ndarray, x: np.ndarray, y: np.ndarray,
           terrain: np.ndarray, burial: float = 0.0,
           max_slope_deg: float = 25.0
           ) -> tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """Stamp height of the rotated solid resting on the terrain.

    Returns ``(bump, profile, centre_z, R_world)``: the height to add to
    the terrain raster (>= 0, zero outside the footprint), the normalised
    profile in [0, 1] for the reflectivity blend, the world z of the
    solid's bounding-box centre, and the final rotation (rest pose composed
    with the local plane tilt). The lowest point of the solid touches the
    resting plane, then the whole solid sinks by ``burial`` times its
    resting height -- never floats, never sinks further than that.
    """
    a, b, c = fit_plane(x, y, terrain, max_slope_deg)
    R_world = plane_rotation(a, b) @ R
    top, bottom = zbuffer(solid, R_world, x, y)
    plane = a * x + b * y + c
    hit = np.isfinite(bottom)
    if not hit.any():
        z0 = np.zeros_like(x)
        return z0, z0, float(c), R_world
    gap = np.where(hit, bottom - plane, np.inf)
    shift = -float(gap.min())                       # lowest point -> plane
    height_rest = float(np.nanmax(top) - np.nanmin(bottom))
    centre_z = shift - burial * height_rest
    top_w = np.where(hit, top + centre_z, -np.inf)
    bump = np.clip(top_w - terrain, 0.0, None)
    bump = np.where(hit, bump, 0.0)
    peak = float(bump.max())
    prof = bump / peak if peak > 0.0 else bump
    return bump, prof, centre_z, R_world
