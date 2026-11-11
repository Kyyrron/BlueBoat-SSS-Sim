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


def part_patches(group: list[PartSurface], seabed, patch_m: float,
                 k: float | None = None) -> Patches:
    """The Kirchhoff patches of coherent parts, in the world, above the
    seabed (``facets.object_patches`` for several parts)."""
    w = Patches.concat([tessellate(s.part.solid, patch_m, k).transformed(s.R, s.t)
                        for s in group])
    return w.select(w.centre[:, 2] > seabed(w.centre[:, 0], w.centre[:, 1]) + 1e-4)
