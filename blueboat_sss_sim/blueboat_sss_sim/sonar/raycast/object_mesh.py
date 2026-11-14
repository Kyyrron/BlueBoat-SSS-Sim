"""Scene objects as their own triangle meshes (``raycast.objects.mode: solid``).

Every object is meshed at load from the manifest alone (nothing is written
into a world, NC #10), as the parts ``worldgen.compounds.parts`` gives it:
one catalog primitive for a class that is one solid, several for a pallet, a
bicycle, a chain. The object is placed by its envelope with the rule the
smooth solids always had (``scene_mesh.facet_transform``: the stored rest
pose, else yaw only with the top at terrain + effective height), so the
stored ground truth stays authoritative and the terrain sheet hides what is
buried.

One surface rule for every part (NC #17): its material's reflection and RMS
roughness split the echo into a coherent share (Kirchhoff patches, a part
whose material is smooth) and a diffuse one (scatterers on the triangles,
every part). Geometry and material names only here; the law is
``scattering.DiffuseLaw`` and ``renderer.deposit_patches``.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ...core.types import PlacedObject
from ...worldgen.compounds import Part, envelope, parts
from ...worldgen.materials import (COHERENT_ROUGHNESS_LIMIT_M, acoustic_reflection,
                                   surface_roughness_m)
from ...worldgen.objects import CATALOG, FACET_SOLID_KINDS, object_roughness_m
from ...worldgen.solids import Patches, tessellate
from .scene_mesh import CHORD_TOL_M, facet_transform, solid_mesh

#: chord tolerance of a part with no mirror patches [m] (a diffuse surface
#: needs its area and its silhouette, not a sub-wavelength figure): 0.5 mm
#: keeps a 0.4 m pile's volume within 0.4 % (raycast_test [O1])
ROUGH_CHORD_TOL_M = 5e-4
#: an instance's construction knobs that change its reflection (``plastic_box``)
_CONSTRUCTION = ("wall_mm", "flooded")


@dataclass(frozen=True)
class PartSurface:
    """One part placed in the world, with the surface it presents."""

    part: Part
    R: np.ndarray            # the part's frame -> world
    t: np.ndarray
    material: str            # the mesh's material name (``scattering.base_material``)
    reflection: float        # |R| at normal incidence
    rms_m: float             # RMS surface height
    coherent: bool           # smooth enough to mirror: it has Kirchhoff patches


def object_parts(o: PlacedObject, seabed, lib, frequency_hz: float
                 ) -> list[PartSurface]:
    """The parts of object ``o`` on the terrain ``seabed(x, y)``; empty when
    nothing of it is proud of the seabed. An instance whose construction
    knobs change its reflection carries its own material name,
    ``<material>#<object id>``."""
    if o.type not in CATALOG or (not o.rested and o.effective_height <= 0.0):
        return []
    from ..facets import object_reflection
    Re, te = facet_transform(o, envelope(o), seabed)
    params = dict(getattr(o, "params", None) or {})
    own = (f"{o.material}#{int(o.object_id)}"
           if any(k in params for k in _CONSTRUCTION) else o.material)
    proud = o.rested or o.effective_height > 0.005
    out = []
    for p in parts(o):
        if p.material is None:
            name, refl, rms = own, object_reflection(o, lib, frequency_hz), object_roughness_m(o)
        else:
            m = lib.get(p.material)
            name, refl, rms = p.material, acoustic_reflection(m, frequency_hz=frequency_hz), \
                surface_roughness_m(m)
        out.append(PartSurface(
            p, Re @ p.R, Re @ p.t + te, name, float(refl), float(rms),
            bool(proud and refl > 0.0 and 0.0 < rms < COHERENT_ROUGHNESS_LIMIT_M
                 and p.solid.kind in FACET_SOLID_KINDS)))
    return out


def part_mesh(s: PartSurface) -> tuple[np.ndarray, np.ndarray]:
    """World vertices (V, 3) and outward triangles (T, 3) of a placed part."""
    v, f = solid_mesh(s.part.solid, CHORD_TOL_M if s.coherent else ROUGH_CHORD_TOL_M)
    return v @ s.R.T + s.t, f


#: base materials of moulded plastic containers: their box parts take the
#: draft angle (``raycast.objects.face_draft_deg``)
MOULDED_MATERIALS = ("pp_shell", "plastic")


@dataclass(frozen=True)
class FaceGeometry:
    """How far an object's mirror surfaces depart from the ideal primitive
    (``raycast.objects.face_*``): geometry, never a level (NC #17)."""

    tilt_rms_deg: float = 0.0     # RMS slope of the waviness along one surface direction
    corr_len_m: float = 0.10      # its correlation length
    draft_deg: float = 0.0        # draft of moulded side walls
    seed: int = 0                 # (world seed, object id): world-fixed

    @property
    def active(self) -> bool:
        return self.tilt_rms_deg > 0.0 or self.draft_deg > 0.0


def wavy_field(X: np.ndarray, seed: int, corr_len_m: float, tilt_rms_deg: float,
               modes: int = 192) -> tuple[np.ndarray, np.ndarray]:
    """A smooth stationary random field ``h(X)`` [m] and its gradient (n, 3):
    a sum of ``modes`` plane waves whose wave vectors are drawn from the
    Gaussian spectrum of the correlation ``exp(-r^2 / l^2)``. Along any
    direction its RMS slope is ``sqrt(2) sigma_h / l``, set to
    ``tilt_rms_deg``. A pure function of ``seed`` and position."""
    rng = np.random.default_rng([int(seed) & 0xFFFFFFFF, 0x5EA1])
    K = rng.normal(0.0, np.sqrt(2.0) / float(corr_len_m), (int(modes), 3))
    ph = rng.uniform(0.0, 2.0 * np.pi, int(modes))
    amp = np.radians(float(tilt_rms_deg)) * float(corr_len_m) / np.sqrt(2.0) * np.sqrt(2.0 / modes)
    arg = np.asarray(X, float) @ K.T + ph
    return amp * np.cos(arg).sum(axis=1), -amp * (np.sin(arg) @ K)


def _rotate(vec: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation of rows ``vec`` about the unit rows ``axis``."""
    c, s = np.cos(angle), np.sin(angle)
    return (vec * c + np.cross(axis, vec) * s
            + axis * np.einsum("ij,ij->i", axis, vec)[:, None] * (1.0 - c))


def perturb_patches(p: Patches, fg: FaceGeometry, draft: np.ndarray | None = None,
                    z0: float | None = None) -> Patches:
    """``p`` with the face geometry ``fg``. The waviness displaces each patch
    along its normal by ``h - min h`` (outward only: a patch never sinks
    under the mesh the visibility rays are cast against) and tilts it by the
    field's slope; the draft leans the near-vertical patches flagged in
    ``draft`` outward about the object's base ``z0``. ``fg`` inactive: ``p``
    itself."""
    if not fg.active or len(p) == 0:
        return p
    c, n, u, v = (np.array(a, dtype=np.float64) for a in (p.centre, p.normal, p.u, p.v))
    if fg.draft_deg > 0.0 and draft is not None and draft.any():
        side = draft & (np.abs(n[:, 2]) < 0.2)
        if side.any():
            d = np.radians(fg.draft_deg)
            nh = n[side].copy()
            nh[:, 2] = 0.0
            nh /= np.linalg.norm(nh, axis=1, keepdims=True)
            zb = float(c[side][:, 2].min() if z0 is None else z0)
            c[side] = c[side] + (np.maximum(c[side][:, 2] - zb, 0.0) * np.tan(d))[:, None] * nh
            ax = np.cross(np.array([0.0, 0.0, 1.0]), nh)        # rotates n_h towards -z
            n[side], u[side], v[side] = (_rotate(q[side], ax, d) for q in (n, u, v))
    if fg.tilt_rms_deg > 0.0:
        h, g = wavy_field(c, fg.seed, fg.corr_len_m, fg.tilt_rms_deg)
        c = c + (h - h.min())[:, None] * n
        u2 = u + np.einsum("ij,ij->i", g, u)[:, None] * n       # tangents of the displaced surface
        v2 = v + np.einsum("ij,ij->i", g, v)[:, None] * n
        u2 /= np.linalg.norm(u2, axis=1, keepdims=True)
        v2 -= np.einsum("ij,ij->i", v2, u2)[:, None] * u2
        v2 /= np.linalg.norm(v2, axis=1, keepdims=True)
        n2 = np.cross(u2, v2)
        n2 *= np.sign(np.einsum("ij,ij->i", n2, n))[:, None]
        n, u, v = n2, u2, v2
    return Patches(c, n, u, v, p.a, p.b)


def part_patches(group: list[PartSurface], seabed, patch_m: float,
                 k: float | None = None, face: FaceGeometry | None = None) -> Patches:
    """The Kirchhoff patches of coherent parts, in the world, above the
    seabed (``facets.object_patches`` for several parts). ``face``: the
    object's face geometry (``perturb_patches``); None or inactive = the
    ideal primitives, as before."""
    each = [tessellate(s.part.solid, patch_m, k).transformed(s.R, s.t) for s in group]
    w = Patches.concat(each)
    if face is not None and face.active:
        draft = np.concatenate([
            np.full(len(q), s.part.solid.kind == "box"
                    and str(s.material).split("#")[0] in MOULDED_MATERIALS)
            for s, q in zip(group, each) if len(q)]) if len(w) else np.zeros(0, bool)
        w = perturb_patches(w, face, draft)
    return w.select(w.centre[:, 2] > seabed(w.centre[:, 0], w.centre[:, 1]) + 1e-4)
