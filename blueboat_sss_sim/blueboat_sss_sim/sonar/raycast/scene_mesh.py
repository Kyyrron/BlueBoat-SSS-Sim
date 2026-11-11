"""SceneModel -> the ray-cast sensor's triangle scene (Phase 1).

The triangle scene is **derived**, never stored: the ``SceneModel`` stays the
single source of truth (``raycast_design.md`` §5). Built in memory on load:

* **seabed sheet** -- one heightfield of ``height - facet_bump`` (the seabed
  the geometric renderer meets, with every *rough* object stamped into it
  exactly as the raster carries it), two triangles per raster quad, vertices
  at the cell centres. Rough-object cells are tagged with their object id by
  re-stamping each object alone through the same ``rasterize_objects`` the
  world builder used (``owner_raster``);
* **smooth solids** (``is_facet_object``, NC #17) -- one watertight mesh per
  object from its catalog ``Solid``, placed with the transform
  ``sonar.facets.object_patches`` uses (``facet_transform``; Phase-1 gate M2
  holds the two together to 1 mm). Below-seabed parts are hidden by the
  seabed sheet, as ``object_patches`` cuts them;
* **walls** -- the Gazebo box of every wall (``worldgen.sdf_writer._wall_sdf``:
  centred on the authored line, ``thickness`` wide, from the floor -- or the
  pontoon draft -- to ``top_z``). The geometric renderer's reflecting face is
  the zero-thickness line; the 0.5 * thickness offset is reported in the
  comparison, not hidden;
* **apron** -- the scene's ``outside`` rule beyond the grid (``fill``: a flat
  plain at the deepest cell with a vertical skirt; ``edge``: the edge heights
  continued outward), out to ``apron_m``;
* the **sea surface** is not triangulated: it is the analytic plane z = 0,
  intersected exactly by :class:`SceneOracle`.

World limits are not in ``SceneModel`` at all (NC #12). Dry triangles (every
vertex at or above the still waterline) are flagged silent (NC #13's intent:
no return from land); awash terrain below z = 0 is water-covered and may
scatter if a path reaches it -- a deliberate, physical difference from the
geometric renderer's "above the transducer" sampling rule, recorded in
``raycast_phase1.md``.

**objects.mode solid** (2026-11-11, ``raycast.objects``; ``object_mesh.py``):
the sheet is the TERRAIN alone (the world folder's ``depth.npz``) and every
object is its own mesh on it, the parts ``worldgen.compounds`` gives it, each
with its material: ``KIND_SMOOTH_OBJECT`` for a part that also mirrors
(Kirchhoff patches), ``KIND_SOLID`` otherwise; both carry diffuse scatterers.
A wall's material is its own, tagged ``<material>#wall`` (its roughness is
the wall's, never the material table's). The description above is ``raster``,
the default.

ROS-free, Qt-free.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ...core.types import PlacedObject, Wall
from ...worldgen.objects import CATALOG, is_facet_object, rasterize_objects
from ...worldgen.solids import Solid, rot_z, rotation_rpy
from .bvh_cpu import Hits, TriangleBVH

_log = logging.getLogger(__name__)

#: Triangle kinds.
KIND_SEABED = 0
KIND_ROUGH_OBJECT = 1
KIND_SMOOTH_OBJECT = 2
KIND_WALL = 3
KIND_APRON = 4
#: A ROUGH object's own mesh (``raycast.objects.mode: solid``, 2026-11-11):
#: its triangles carry diffuse scatterers under the material law. Not a
#: floor (``multipath``: a floor bounce to it is a real path) and not a wall.
KIND_SOLID = 5
#: What the oracle reports for the analytic sea surface.
TRI_SURFACE = -2

#: Chord sagitta tolerance of the smooth-solid meshes [m]. A PO patch centre
#: lies on the true surface, at most this far outside its chord facets.
CHORD_TOL_M = 2e-4
#: Below this a stamped bump does not own its cell [m].
_BUMP_EPS = 1e-9


@dataclass
class TriangleScene:
    """Triangle soup with per-triangle attributes."""

    vertices: np.ndarray            # (V, 3) float64, world ENU [m]
    triangles: np.ndarray           # (T, 3) int64
    kind: np.ndarray                # (T,) uint8, KIND_*
    object_id: np.ndarray           # (T,) int64, -1 = none
    wall_index: np.ndarray          # (T,) int64, -1 = none
    material: np.ndarray            # (T,) int64 index into material_names
    material_names: list[str]
    dry: np.ndarray                 # (T,) bool, silent land
    surface_z: float = 0.0
    grid_extent: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    #: objects.mode solid: per object id, its placed parts
    #: (``object_mesh.PartSurface``), in the order they were meshed.
    parts: dict = field(default_factory=dict)
    #: How the objects were meshed: "raster" (rough objects are the bumps of
    #: the sheet) or "solid" (the sheet is the terrain, every object its own
    #: mesh; ``object_mesh.py``). The mode in force, after any fallback.
    objects_mode: str = "raster"

    @property
    def n_triangles(self) -> int:
        return int(self.triangles.shape[0])

    def triangle_vertices(self) -> np.ndarray:
        return self.vertices[self.triangles]

    def normals(self) -> np.ndarray:
        """Unit normals (right-hand rule on the vertex order)."""
        tv = self.triangle_vertices()
        n = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0])
        return n / np.maximum(np.linalg.norm(n, axis=1), 1e-300)[:, None]



# =============================================================== primitives
def _ring(n: int, r: float, e1: np.ndarray, e2: np.ndarray,
          centre: np.ndarray) -> np.ndarray:
    th = np.arange(n) * 2.0 * np.pi / n
    return (centre[None, :] + r * np.cos(th)[:, None] * e1[None, :]
            + r * np.sin(th)[:, None] * e2[None, :])


def _segments(r: float, tol: float = CHORD_TOL_M, n_min: int = 12) -> int:
    """Segments around a circle of radius ``r`` keeping the sagitta
    ``r (1 - cos(pi/n))`` <= ``tol``."""
    if r <= tol:
        return n_min
    return max(n_min, int(np.ceil(np.pi / np.arccos(1.0 - tol / r))))


def _band(a: int, b: int, n: int, flip: bool = False) -> list[tuple]:
    """Quads between ring starting at vertex ``a`` and ring at ``b`` (both
    ``n`` long), split into triangles."""
    out = []
    for i in range(n):
        j = (i + 1) % n
        t1, t2 = (a + i, a + j, b + j), (a + i, b + j, b + i)
        if flip:
            t1, t2 = t1[::-1], t2[::-1]
        out += [t1, t2]
    return out


def _fan(c: int, a: int, n: int, flip: bool = False) -> list[tuple]:
    out = []
    for i in range(n):
        t = (c, a + i, a + (i + 1) % n)
        out.append(t[::-1] if flip else t)
    return out


def solid_mesh(sol: Solid, tol: float = CHORD_TOL_M
               ) -> tuple[np.ndarray, np.ndarray]:
    """Watertight, outward-oriented triangle mesh of a catalog ``Solid`` in
    its object frame -- the frame ``solids.tessellate`` and ``zbuffer`` use
    (bounding-box centre at ``centre_offset``; the cone base at z = 0)."""
    k = sol.kind
    e = np.eye(3)
    if k == "box":
        hx, hy, hz = 0.5 * sol.a, 0.5 * sol.b, 0.5 * sol.c
        v = np.array([[sx * hx, sy * hy, sz * hz]
                      for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)])
        f = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6),     # -z, +z
             (0, 1, 4), (1, 5, 4), (2, 6, 3), (3, 6, 7),     # -y, +y
             (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5)]     # -x, +x
        return v.astype(np.float64), np.array(f, dtype=np.int64)
    if k in ("cylinder", "capsule"):
        r, L = sol.a, sol.b
        ea, e1, e2 = e[sol.axis], e[(sol.axis + 1) % 3], e[(sol.axis + 2) % 3]
        n = _segments(r, tol)
        half = 0.5 * L if k == "cylinder" else max(0.5 * L - r, 0.0)
        verts: list[np.ndarray] = []
        faces: list[tuple] = []
        if k == "cylinder":
            verts.append(_ring(n, r, e1, e2, -half * ea))          # 0..n-1
            verts.append(_ring(n, r, e1, e2, half * ea))           # n..2n-1
            verts.append(np.array([-half * ea, half * ea]))        # 2n, 2n+1
            faces += _band(0, n, n)
            faces += _fan(2 * n, 0, n, flip=True)
            faces += _fan(2 * n + 1, n, n)
        else:
            # Capsule: latitude rings over two hemispheres (+ a cylinder).
            # Latitude rings over the two hemispheres; the poles are fan
            # centres, never a ring (a zero-radius ring is degenerate).
            m = max(3, int(np.ceil(0.5 * np.pi / np.arccos(
                1.0 - min(tol / max(r, tol), 0.5)))))
            rings = []
            for i in range(m - 1, 0, -1):          # south cap, pole excluded
                al = 0.5 * np.pi * i / m           # angle from the equator
                rings.append(_ring(n, r * np.cos(al), e1, e2,
                                   (-half - r * np.sin(al)) * ea))
            rings.append(_ring(n, r, e1, e2, -half * ea))
            if half > 0.0:
                rings.append(_ring(n, r, e1, e2, half * ea))
            for i in range(1, m):
                al = 0.5 * np.pi * i / m
                rings.append(_ring(n, r * np.cos(al), e1, e2,
                                   (half + r * np.sin(al)) * ea))
            for g in rings:
                verts.append(g)
            nr = len(rings)
            verts.append(np.array([(-half - r) * ea, (half + r) * ea]))
            for i in range(nr - 1):
                faces += _band(i * n, (i + 1) * n, n)
            faces += _fan(nr * n, 0, n, flip=True)
            faces += _fan(nr * n + 1, (nr - 1) * n, n)
        return np.vstack(verts), np.array(faces, dtype=np.int64)
    if k == "cone":
        r0, H = sol.a, sol.b
        n = _segments(r0, tol)
        base = _ring(n, r0, e[0], e[1], np.zeros(3))
        v = np.vstack([base, [[0.0, 0.0, H], [0.0, 0.0, 0.0]]])
        faces = _fan(n, 0, n) + _fan(n + 1, 0, n, flip=True)
        return v, np.array(faces, dtype=np.int64)
    if k == "annulus":
        ro, ri, c = sol.a, sol.b, sol.c
        n = _segments(ro, tol)
        z0, z1 = -0.5 * c * e[2], 0.5 * c * e[2]
        v = np.vstack([_ring(n, ro, e[0], e[1], z0), _ring(n, ro, e[0], e[1], z1),
                       _ring(n, ri, e[0], e[1], z0), _ring(n, ri, e[0], e[1], z1)])
        faces = (_band(0, n, n)                       # outer wall
                 + _band(2 * n, 3 * n, n, flip=True)  # inner wall (inward)
                 + _band(n, 3 * n, n)                 # top ring
                 + _band(0, 2 * n, n, flip=True))     # bottom ring
        return v, np.array(faces, dtype=np.int64)
    if k == "ellipsoid":                 # the unit sphere, scaled: a lump's
        s = np.array([sol.a, sol.b, sol.c])   # outline, 0.5 % of its size
        v, f = solid_mesh(Solid("capsule", 1.0, 2.0), max(tol / float(s.max()), 5e-3))
        return v * s, f
    raise ValueError(f"no mesh for solid kind {k!r}")


def mesh_volume(v: np.ndarray, f: np.ndarray) -> float:
    """Signed volume (divergence theorem); > 0 for an outward mesh."""
    tv = v[f]
    return float(np.einsum("ij,ij->i", tv[:, 0],
                           np.cross(tv[:, 1], tv[:, 2])).sum() / 6.0)


def solid_volume(sol: Solid) -> float:
    """Closed-form volume of a catalog solid (gate M5)."""
    k = sol.kind
    if k == "box":
        return sol.a * sol.b * sol.c
    if k == "cylinder":
        return np.pi * sol.a ** 2 * sol.b
    if k == "capsule":
        r, L = sol.a, sol.b
        return np.pi * r ** 2 * max(L - 2 * r, 0.0) + 4.0 / 3.0 * np.pi * r ** 3
    if k == "cone":
        return np.pi * sol.a ** 2 * sol.b / 3.0
    if k == "annulus":
        return np.pi * (sol.a ** 2 - sol.b ** 2) * sol.c
    if k == "ellipsoid":
        return 4.0 / 3.0 * np.pi * sol.a * sol.b * sol.c
    raise ValueError(f"no volume for solid kind {k!r}")


def facet_transform(o: PlacedObject, sol: Solid, seabed) -> tuple:
    """Object frame -> world, exactly as ``sonar.facets.object_patches``
    places a facet object (gate M2 holds the two together)."""
    if o.rested:
        R = rotation_rpy(o.roll, o.pitch, o.yaw)
        centre = np.array([o.x, o.y, o.rest_z])
    else:
        R = rot_z(o.yaw)
        z0 = float(seabed(np.array([o.x]), np.array([o.y]))[0])
        e_z = float(np.abs(R @ sol.extent)[2])
        top = z0 + o.effective_height
        centre = np.array([o.x, o.y, min(top - e_z, z0 + e_z)])
    return R, centre - R @ sol.centre_offset


# ================================================================ rasters
def seabed_height(scene) -> np.ndarray:
    """The seabed the beam meets: ``height - facet_bump``."""
    fb = getattr(scene, "facet_bump", None)
    return scene.height - fb if fb is not None else scene.height.copy()


def _solid_pad(o: PlacedObject, res: float) -> float:
    """Half-size of the window ``rasterize_objects`` stamps an object in."""
    if o.rested and CATALOG[o.type].solid(o) is not None:
        return 0.5 * float(np.sqrt(o.length ** 2 + o.width ** 2
                                   + o.proud_height ** 2)) + 2 * res
    return o.footprint_radius + 2 * res


def owner_raster(scene, terrain: np.ndarray | None = None
                 ) -> tuple[np.ndarray, np.ndarray]:
    """(object id, bump) of the rough object owning each cell (-1, 0 where
    none).

    With the world's terrain-only raster (``depth.npz``): the cells the
    stamp raised (``height - facet_bump - terrain > 0``), each owned by the
    nearest rough object whose stamping window covers it. That reads the
    stored raster as it is. Re-stamping a saved object is *not* idempotent
    for a rested object on a slope -- ``rasterize_objects`` composes the
    bank tilt into the stored roll/pitch, so a second stamp tilts it twice --
    which is why the stored raster, never a re-stamp, is the source.

    Without it (a legacy bundle): each rough object stamped alone through
    ``rasterize_objects`` on a flat zero raster, the tallest bump owning."""
    g = scene.grid
    owner = np.full((g.ny, g.nx), -1, dtype=np.int64)
    rough = [o for o in scene.objects if not is_facet_object(o)]
    if terrain is not None:
        bump = seabed_height(scene) - terrain
        raised = bump > _BUMP_EPS
        if not rough or not raised.any():
            return owner, np.where(raised, bump, 0.0)
        jj, ii = np.nonzero(raised)
        x = g.origin_x + ii * g.resolution
        y = g.origin_y + jj * g.resolution
        best_d = np.full(x.size, np.inf)
        best_o = np.full(x.size, -1, dtype=np.int64)
        for o in rough:
            pad = _solid_pad(o, g.resolution)
            dx, dy = x - o.x, y - o.y
            inside = (np.abs(dx) <= pad) & (np.abs(dy) <= pad)
            d2 = np.where(inside, dx * dx + dy * dy, np.inf)
            win = d2 < best_d
            best_d[win] = d2[win]
            best_o[win] = int(o.object_id)
        owner[jj, ii] = best_o
        return owner, np.where(raised, bump, 0.0)
    best = np.zeros((g.ny, g.nx))
    for o in rough:
        h = np.zeros((g.ny, g.nx))
        rasterize_objects([o], g, h, np.zeros((g.ny, g.nx)))
        win = h > np.maximum(best, _BUMP_EPS)
        owner[win] = int(o.object_id)
        best[win] = h[win]
    return owner, best


def load_terrain(world_dir: str | Path | None, scene) -> np.ndarray | None:
    """The world folder's terrain-only raster (``depth.npz``), or None.
    Checked against the scene grid; a mismatch is an error, not a guess."""
    if world_dir is None:
        return None
    p = Path(world_dir) / "depth.npz"
    if not p.exists():
        return None
    d = np.load(p)
    h = np.asarray(d["height"], dtype=np.float64)
    g = scene.grid
    if (h.shape != (g.ny, g.nx)
            or abs(float(d["origin_x"]) - g.origin_x) > 1e-9
            or abs(float(d["origin_y"]) - g.origin_y) > 1e-9
            or abs(float(d["resolution"]) - g.resolution) > 1e-12):
        raise ValueError(f"{p}: depth.npz grid does not match scene.npz")
    return h


# ================================================================ build
def _grid_xy(g) -> tuple[np.ndarray, np.ndarray]:
    xs = g.origin_x + np.arange(g.nx) * g.resolution
    ys = g.origin_y + np.arange(g.ny) * g.resolution
    return np.meshgrid(xs, ys)


def _sheet(z: np.ndarray, g) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Heightfield -> vertices, triangles, and the (row, col) of the cell
    each triangle is assigned to (the quad's lower-left vertex)."""
    xx, yy = _grid_xy(g)
    v = np.stack([xx.ravel(), yy.ravel(), z.ravel()], axis=1)
    ny, nx = z.shape
    j, i = np.meshgrid(np.arange(ny - 1), np.arange(nx - 1), indexing="ij")
    a = (j * nx + i).ravel()
    b, c, d = a + 1, a + nx + 1, a + nx
    tris = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, d], 1)])
    cell = np.concatenate([np.stack([j.ravel(), i.ravel()], 1)] * 2)
    return v, tris.astype(np.int64), cell


def _apron(z: np.ndarray, g, mode: str, apron_m: float
           ) -> tuple[np.ndarray, np.ndarray]:
    """Quads around the grid realising the ``outside`` rule."""
    ny, nx = z.shape
    x0, y0 = g.origin_x, g.origin_y
    x1, y1 = x0 + (nx - 1) * g.resolution, y0 + (ny - 1) * g.resolution
    # Boundary loop, counter-clockwise, as (x, y, z) of the edge vertices.
    xs = x0 + np.arange(nx) * g.resolution
    ys = y0 + np.arange(ny) * g.resolution
    loop = np.concatenate([
        np.stack([xs, np.full(nx, y0), z[0, :]], 1),
        np.stack([np.full(ny - 2, x1), ys[1:-1], z[1:-1, -1]], 1),
        np.stack([xs[::-1], np.full(nx, y1), z[-1, ::-1]], 1),
        np.stack([np.full(ny - 2, x0), ys[-2:0:-1], z[-2:0:-1, 0]], 1)])
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    # Push each boundary vertex outward along the side normal (corners
    # diagonally), so the ring of far vertices has the same order.
    ox = np.where(loop[:, 0] <= x0 + 1e-9, -1.0,
                  np.where(loop[:, 0] >= x1 - 1e-9, 1.0, 0.0))
    oy = np.where(loop[:, 1] <= y0 + 1e-9, -1.0,
                  np.where(loop[:, 1] >= y1 - 1e-9, 1.0, 0.0))
    del cx, cy
    zmin = float(z.min())
    if mode == "edge":
        inner = loop
        far_z = loop[:, 2]
    else:  # "fill": a vertical skirt to the deepest cell, then a flat plain
        inner = np.column_stack([loop[:, :2], np.full(len(loop), zmin)])
        far_z = np.full(len(loop), zmin)
    far = np.column_stack([loop[:, 0] + ox * apron_m,
                           loop[:, 1] + oy * apron_m, far_z])
    n = len(loop)
    verts = [loop, inner, far]
    faces: list[tuple] = []
    if mode != "edge":
        faces += _band(0, n, n, flip=True)              # skirt
    faces += _band(n, 2 * n, n, flip=True)              # plain / continuation
    return np.vstack(verts), np.array(faces, dtype=np.int64)


def _wall_box(w: Wall, z_floor: float) -> tuple[np.ndarray, np.ndarray]:
    """The wall's Gazebo box (``sdf_writer._wall_sdf``) as 12 triangles."""
    ux, uy = w.unit
    nx_, ny_ = -uy, ux
    ht = 0.5 * float(w.thickness)
    z_base = -float(w.draft_m) if w.kind == "pontoon" else z_floor
    z_top = max(float(w.top_z), z_base + 0.05)
    p = [(w.x0, w.y0), (w.x1, w.y1)]
    corners = []
    for z in (z_base, z_top):
        for (px, py), s in ((p[0], -1), (p[1], -1), (p[1], 1), (p[0], 1)):
            corners.append((px + s * ht * nx_, py + s * ht * ny_, z))
    v = np.array(corners, dtype=np.float64)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
         (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
         (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    tri = np.array(f, dtype=np.int64)
    # Orient outward regardless of the wall's direction.
    if mesh_volume(v, tri) < 0:
        tri = tri[:, ::-1]
    return v, tri


def build_scene_mesh(scene, world_dir: str | Path | None = None,
                     apron_m: float = 60.0,
                     chord_tol_m: float = CHORD_TOL_M,
                     terrain: np.ndarray | None = None,
                     objects_mode: str = "raster", lib=None,
                     frequency_hz: float = 450_000.0) -> TriangleScene:
    """Derive the triangle scene from ``scene`` (and the same world folder's
    ``depth.npz``, used only to check the stamping and to know terrain from
    objects). Nothing is written anywhere.

    ``objects_mode`` (``raycast.objects.mode``): ``"raster"`` -- rough
    objects are the bumps of the sheet, as always; ``"solid"`` -- the sheet
    is the terrain alone and every object is its own mesh
    (:func:`_build_solid`, with the world's ``MaterialLibrary`` ``lib``).
    ``"solid"`` needs the terrain (``depth.npz``); a world without one is
    built ``"raster"``, said once."""
    if objects_mode not in ("raster", "solid"):
        raise ValueError(f"objects_mode {objects_mode!r} (raster | solid)")
    if terrain is None:
        terrain = load_terrain(world_dir, scene)
    if objects_mode == "solid":
        if terrain is not None:
            return _build_solid(scene, np.asarray(terrain, dtype=np.float64),
                                apron_m, lib, frequency_hz)
        if getattr(scene, "objects", None):
            _log.warning(
                "ray-cast objects.mode 'solid' needs the terrain before objects "
                "(depth.npz), which %s does not have: its objects stay the bumps "
                "of the height raster ('raster')", world_dir or "this scene")
    g = scene.grid
    zs = seabed_height(scene)
    names = list(scene.material_names)

    def mat_index(name: str) -> int:
        if name not in names:
            names.append(name)
        return names.index(name)

    owner, _ = owner_raster(scene, terrain)
    by_id = {int(o.object_id): o for o in scene.objects}

    v_list, t_list, kind, oid, widx, mat, dry = [], [], [], [], [], [], []
    nv = 0

    def add(v, t, k, o, w, m, d):
        nonlocal nv
        v_list.append(v)
        t_list.append(t + nv)
        nv += v.shape[0]
        kind.append(np.full(t.shape[0], k, dtype=np.uint8))
        oid.append(np.broadcast_to(np.asarray(o, dtype=np.int64),
                                   (t.shape[0],)).copy())
        widx.append(np.full(t.shape[0], w, dtype=np.int64))
        mat.append(np.broadcast_to(np.asarray(m, dtype=np.int64),
                                   (t.shape[0],)).copy())
        dry.append(np.broadcast_to(np.asarray(d, dtype=bool),
                                   (t.shape[0],)).copy())

    # --- seabed sheet (rough objects included, tagged) ---------------------
    v, t, cell = _sheet(zs, g)
    cell_owner = owner[cell[:, 0], cell[:, 1]]
    seabed_mat = np.asarray(scene.material_id, dtype=np.int64)[cell[:, 0],
                                                               cell[:, 1]]
    obj_mat = np.full(t.shape[0], -1, dtype=np.int64)
    for i in np.unique(cell_owner[cell_owner >= 0]):
        obj_mat[cell_owner == i] = mat_index(by_id[int(i)].material)
    m = np.where(cell_owner >= 0, obj_mat, seabed_mat)
    k = np.where(cell_owner >= 0, KIND_ROUGH_OBJECT, KIND_SEABED)
    tz = v[t][:, :, 2]
    is_dry = (tz >= 0.0).all(axis=1)
    add(v, t, 0, cell_owner, -1, m, is_dry)
    kind[-1] = k.astype(np.uint8)

    # --- apron (the scene's outside rule) ----------------------------------
    av, at = _apron(zs, g, getattr(scene, "outside", "fill"), apron_m)
    add(av, at, KIND_APRON, -1, -1, mat_index("outside"), False)

    # --- smooth solids -----------------------------------------------------
    seabed_fn = None
    if any(is_facet_object(o) for o in scene.objects):
        from ...core.geometry import bilinear_sample

        def seabed_fn(x, y, _z=zs, _g=g):
            return bilinear_sample(_z, _g, x, y, fill=float(_z.min()))
    for o in scene.objects:
        if not is_facet_object(o):
            continue
        sol = CATALOG[o.type].solid(o)
        if sol is None:
            continue
        sv, sf = solid_mesh(sol, chord_tol_m)
        R, tr = facet_transform(o, sol, seabed_fn)
        add(sv @ R.T + tr, sf, KIND_SMOOTH_OBJECT, int(o.object_id), -1,
            mat_index(o.material), False)

    # --- walls ---------------------------------------------------------------
    z_floor = float(zs.min()) - 0.5
    for wi, w in enumerate(getattr(scene, "walls", None) or []):
        wv, wt = _wall_box(w, z_floor)
        add(wv, wt, KIND_WALL, -1, wi, mat_index(w.material), False)

    xmin, ymin, xmax, ymax = g.extent
    return TriangleScene(
        vertices=np.vstack(v_list), triangles=np.vstack(t_list),
        kind=np.concatenate(kind), object_id=np.concatenate(oid),
        wall_index=np.concatenate(widx), material=np.concatenate(mat),
        material_names=names, dry=np.concatenate(dry),
        grid_extent=(xmin, ymin, xmax, ymax))


def _build_solid(scene, terrain: np.ndarray, apron_m: float, lib,
                 frequency_hz: float) -> TriangleScene:
    """The triangle scene of ``objects.mode: solid``: the terrain sheet
    (seabed materials only), the apron, every object as the meshes of its
    parts (``object_mesh.object_parts``), the walls. The sheet stays first
    (``SheetSampler`` / ``SheetSpecular`` read it by vertex index)."""
    from ...core.geometry import bilinear_sample
    from ...worldgen.materials import MaterialLibrary
    from .object_mesh import object_parts, part_mesh
    lib = lib if lib is not None else MaterialLibrary()
    g = scene.grid
    if terrain.shape != (g.ny, g.nx):
        raise ValueError("terrain raster does not match the scene grid")
    zs = terrain
    names = list(scene.material_names)

    def mat_index(name: str) -> int:
        if name not in names:
            names.append(name)
        return names.index(name)

    v_list, t_list, kind, oid, widx, mat, dry = [], [], [], [], [], [], []
    nv = 0

    def add(v, t, k, o, w, m, d):
        nonlocal nv
        v_list.append(v)
        t_list.append(t + nv)
        nv += v.shape[0]
        n = t.shape[0]
        kind.append(np.full(n, k, dtype=np.uint8))
        oid.append(np.full(n, o, dtype=np.int64))
        widx.append(np.full(n, w, dtype=np.int64))
        mat.append(np.broadcast_to(np.asarray(m, dtype=np.int64), (n,)).copy())
        dry.append(np.broadcast_to(np.asarray(d, dtype=bool), (n,)).copy())

    v, t, cell = _sheet(zs, g)
    seabed_mat = np.asarray(scene.material_id, dtype=np.int64)[cell[:, 0], cell[:, 1]]
    add(v, t, KIND_SEABED, -1, -1, seabed_mat, (v[t][:, :, 2] >= 0.0).all(axis=1))
    av, at = _apron(zs, g, getattr(scene, "outside", "fill"), apron_m)
    add(av, at, KIND_APRON, -1, -1, mat_index("outside"), False)

    def seabed_fn(x, y, _z=zs, _g=g):
        return bilinear_sample(_z, _g, x, y, fill=float(_z.min()))

    parts = {}
    for o in getattr(scene, "objects", None) or []:
        surfaces = object_parts(o, seabed_fn, lib, frequency_hz)
        if surfaces:
            parts[int(o.object_id)] = surfaces
        for sf in surfaces:
            add(*part_mesh(sf), KIND_SMOOTH_OBJECT if sf.coherent else KIND_SOLID,
                int(o.object_id), -1, mat_index(sf.material), False)

    z_floor = float(zs.min()) - 0.5
    for wi, w in enumerate(getattr(scene, "walls", None) or []):
        wv, wt = _wall_box(w, z_floor)
        add(wv, wt, KIND_WALL, -1, wi, mat_index(f"{w.material}#wall"), False)

    xmin, ymin, xmax, ymax = g.extent
    return TriangleScene(
        vertices=np.vstack(v_list), triangles=np.vstack(t_list),
        kind=np.concatenate(kind), object_id=np.concatenate(oid),
        wall_index=np.concatenate(widx), material=np.concatenate(mat),
        material_names=names, dry=np.concatenate(dry),
        grid_extent=(xmin, ymin, xmax, ymax), parts=parts, objects_mode="solid")


# ================================================================ oracle
@dataclass
class SceneHits:
    """Closest-hit result including the analytic sea surface
    (``tri == TRI_SURFACE``)."""

    t: np.ndarray
    tri: np.ndarray
    u: np.ndarray
    v: np.ndarray


class SceneOracle:
    """The one visibility oracle: triangles (BVH) + the plane z = 0."""

    def __init__(self, mesh: TriangleScene, bvh: TriangleBVH | None = None,
                 backend: str = "cpu"):
        self.mesh = mesh
        if bvh is None:
            if backend == "warp":
                # the GPU oracle (gpu_warp.py), imported only when selected
                from .gpu_warp import WarpBVH
                bvh = WarpBVH(mesh.vertices, mesh.triangles)
            elif backend == "cpu":
                bvh = TriangleBVH(mesh.vertices, mesh.triangles)
            else:
                raise ValueError(f"oracle backend {backend!r} (cpu | warp)")
        self.bvh = bvh

    def closest(self, orig, dirs, t_max=np.inf, t_min: float = 1e-9
                ) -> SceneHits:
        orig = np.atleast_2d(np.asarray(orig, dtype=np.float64))
        dirs = np.atleast_2d(np.asarray(dirs, dtype=np.float64))
        h: Hits = self.bvh.closest(orig, dirs, t_max, t_min)
        tm = np.broadcast_to(np.asarray(t_max, dtype=np.float64),
                             (orig.shape[0],))
        zs = self.mesh.surface_z
        with np.errstate(divide="ignore", invalid="ignore"):
            ts = (zs - orig[:, 2]) / dirs[:, 2]
        up = (dirs[:, 2] > 0) & (orig[:, 2] < zs) & (ts > t_min) & (ts <= tm)
        surf = up & (ts < h.t)
        t = np.where(surf, ts, h.t)
        tri = np.where(surf, TRI_SURFACE, h.tri)
        return SceneHits(t=t, tri=tri, u=np.where(surf, 0.0, h.u),
                         v=np.where(surf, 0.0, h.v))

    #: per-triangle thin-layer properties (``set_layers``); None = all opaque
    layer = None

    def set_layers(self, mask: np.ndarray, rho_ratio: np.ndarray,
                   c_ratio: np.ndarray, thickness: np.ndarray, k1: float) -> None:
        """Triangles that are flooded thin shells: a crossing transmits by
        :func:`layer_transmission` instead of blocking."""
        if not np.any(mask):
            self.layer = None
            return
        self.layer = dict(mask=np.asarray(mask, bool),
                          rho=np.asarray(rho_ratio, float),
                          c=np.asarray(c_ratio, float),
                          d=np.asarray(thickness, float), k1=float(k1))

    def transmit(self, p0, p1, eps: float = 1e-6, max_cross: int = 12
                 ) -> np.ndarray:
        """Complex transmission of the open segment p0 -> p1: 0 where an
        opaque triangle (or the sea surface) is crossed, else the product of
        the thin layers' factors (1 when nothing is crossed)."""
        p0 = np.atleast_2d(np.asarray(p0, dtype=np.float64)).copy()
        p1 = np.atleast_2d(np.asarray(p1, dtype=np.float64))
        n = p0.shape[0]
        zs = self.mesh.surface_z
        crosses = (np.maximum(p0[:, 2], p1[:, 2]) > zs + eps) & \
            (np.minimum(p0[:, 2], p1[:, 2]) < zs - eps)
        T = np.where(crosses, 0.0 + 0j, 1.0 + 0j)
        act = ~crosses
        nrm = None
        for _ in range(max_cross):
            if not act.any():
                break
            i = np.flatnonzero(act)
            d = p1[i] - p0[i]
            L = np.linalg.norm(d, axis=1)
            u = d / np.maximum(L, 1e-300)[:, None]
            h = self.bvh.closest(p0[i], u, t_max=np.maximum(L - eps, 0.0),
                                 t_min=eps)
            hit = h.tri >= 0
            act[i[~hit]] = False
            if not hit.any():
                break
            ih, tr = i[hit], h.tri[hit]
            lay = self.layer
            if lay is None:
                T[ih] = 0.0
                act[ih] = False
                break
            thin = lay["mask"][tr]
            T[ih[~thin]] = 0.0
            act[ih[~thin]] = False
            if thin.any():
                it, tt = ih[thin], tr[thin]
                if nrm is None:
                    nrm = self.mesh.normals()
                cos1 = np.abs(np.einsum("ij,ij->i", nrm[tt], u[hit][thin]))
                T[it] *= layer_transmission(cos1, lay["rho"][tt], lay["c"][tt],
                                            lay["d"][tt], lay["k1"])
                p0[it] = p0[it] + (h.t[hit][thin] + 10 * eps)[:, None] * u[hit][thin]
        return T

    def occluded(self, p0, p1, eps: float = 1e-6) -> np.ndarray:
        """True where the open segment p0 -> p1 crosses the scene or leaves
        the water (crosses z = 0)."""
        p0 = np.atleast_2d(np.asarray(p0, dtype=np.float64))
        p1 = np.atleast_2d(np.asarray(p1, dtype=np.float64))
        zs = self.mesh.surface_z
        out = self.bvh.occluded(p0, p1, eps)
        crosses = (np.maximum(p0[:, 2], p1[:, 2]) > zs + eps) & \
            (np.minimum(p0[:, 2], p1[:, 2]) < zs - eps)
        return out | crosses


# ============================================================ thin layers
def layer_transmission(cos1: np.ndarray, rho_ratio: np.ndarray,
                       c_ratio: np.ndarray, d: np.ndarray, k1: float
                       ) -> np.ndarray:
    """Pressure transmission of a fluid layer (density and sound-speed ratios
    to water, thickness ``d``) in water at incidence cosine ``cos1``
    (Brekhovskikh & Lysanov 2003 ch. 2; Kinsler et al. par. 6.3, oblique):
    ``T = 1 / (cos(k2z d) - i/2 (m + 1/m) sin(k2z d))``, ``m = Z2n / Z1n``,
    ``Z_n = rho c / cos(theta)``, Snell in the layer (evanescent beyond its
    critical angle). Referred to the straight water path through the same
    thickness, in this renderer's exp(-i omega tau) convention (the conjugate
    of the textbook exp(-i omega t) form). Shear in the layer is omitted (a
    fluid layer; Folds & Loggins 1977 is the elastic upgrade), stated."""
    c1 = np.clip(np.abs(np.asarray(cos1, float)), 1e-6, 1.0)
    s1 = np.sqrt(1.0 - c1 * c1)
    s2 = c_ratio * s1
    c2 = np.sqrt((1.0 - s2 * s2).astype(complex))
    c2 = np.where(np.imag(c2) < 0, -c2, c2)
    k2z = (k1 / c_ratio) * c2
    m = rho_ratio * c_ratio * c1 / np.where(np.abs(c2) > 1e-12, c2, 1e-12)
    T = 1.0 / (np.cos(k2z * d) - 0.5j * (m + 1.0 / m) * np.sin(k2z * d))
    return np.conj(T * np.exp(-1j * k1 * c1 * d))
