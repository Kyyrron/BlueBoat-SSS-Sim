"""Validation ladder of the ray-cast sensor (``docs/engineering/raycast_plan.md``).

Not part of the smoke test or the Stop hook: run by hand from the package
source root (any interpreter with numpy; GPU checks skip visibly without
their stack)::

    python3 -m test.raycast_test             # every phase implemented so far
    python3 -m test.raycast_test --quick     # one world per section

Codes in brackets are the tolerance table of ``raycast_design.md`` §7.
Phase 1: scene and mesh export [M0-M5], the CPU oracle vs brute force.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blueboat_sss_sim.core.types import GridSpec  # noqa: E402
from blueboat_sss_sim.sonar import acoustics  # noqa: E402
from blueboat_sss_sim.sonar.config import SonarModelConfig  # noqa: E402
from blueboat_sss_sim.sonar.facets import object_patches  # noqa: E402
from blueboat_sss_sim.sonar.raycast import scene_mesh as sm  # noqa: E402
from blueboat_sss_sim.sonar.raycast.bvh_cpu import (TriangleBVH, moller_trumbore,  # noqa: E402
                                                     watertight)
from blueboat_sss_sim.worldgen.materials import MaterialLibrary  # noqa: E402
from blueboat_sss_sim.worldgen.objects import (CATALOG, ObjectFieldConfig,  # noqa: E402
                                               is_facet_object, place_objects,
                                               typical_instance)
from blueboat_sss_sim.worldgen.scene import SceneModel, stamp_objects  # noqa: E402
from blueboat_sss_sim.worldgen.solids import Solid  # noqa: E402
from copy import deepcopy as sm_copy  # noqa: E402

WORLDS_ROOT = Path(os.path.expanduser("~/worlds"))
PASSED = FAILED = SKIPPED = 0


def check(name: str, ok: bool, detail: str = "") -> bool:
    global PASSED, FAILED
    if ok:
        PASSED += 1
    else:
        FAILED += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""))
    return ok


def skip(name: str, why: str) -> None:
    global SKIPPED
    SKIPPED += 1
    print(f"  [SKIP] {name} -- {why}")


# ------------------------------------------------------------------ scenes
def world_dirs() -> list[Path]:
    return sorted(p.parent for p in WORLDS_ROOT.glob("*/*/scene_manifest.yaml"))


def synthetic_scene() -> SceneModel:
    """Every catalog class that has a solid, flat and rested at random, on a
    sloping floor (rested objects bank with it) plus every class without one."""
    g = GridSpec(origin_x=-20.0, origin_y=-15.0, resolution=0.05,
                 nx=801, ny=601)
    xs = g.origin_x + np.arange(g.nx) * g.resolution
    h = np.tile(-3.0 + 0.04 * xs, (g.ny, 1))
    lib = MaterialLibrary()
    classes = {}
    for name, spec in CATALOG.items():
        classes[name] = {"amount": {"kind": "count", "value": 2},
                         "orientation": {"rest": {"enabled": True,
                                                  "randomness": 1.0}}}
    cfg = ObjectFieldConfig.from_dict({"min_separation_m": 1.2,
                                       "margin_m": 2.0, "classes": classes})
    objs = place_objects(g, cfg, lib, np.random.default_rng(7), height=h)
    terrain = h.copy()
    refl = np.full(h.shape, 0.3)
    ras = stamp_objects(objs, g, h, refl)
    sc = SceneModel(grid=g, height=h, reflectivity=refl, **ras.scene_kwargs(),
                    material_id=np.zeros(h.shape, dtype=np.uint8),
                    material_names=["sand"], objects=objs, walls=[], seed=7,
                    config_snapshot={})
    sc._test_terrain = terrain            # noqa: SLF001 (test only)
    return sc


# ------------------------------------------------------------------ checks
def mesh_checks(label: str, scene: SceneModel, world: Path | None,
                terrain: np.ndarray | None, rng: np.random.Generator,
                n_crops: int, rays_per_crop: int) -> None:
    print(f"\n[R1] {label}")
    t0 = time.time()
    mesh = sm.build_scene_mesh(scene, world, terrain=terrain)
    zs = sm.seabed_height(scene)
    print(f"      {mesh.n_triangles} triangles in {time.time() - t0:.1f} s; "
          f"kinds {np.bincount(mesh.kind, minlength=5).tolist()} "
          "(seabed, rough object, smooth object, wall, apron)")

    # [M0] the mesh's source raster is the builder's own stamp of the
    # terrain: re-stamping reproduces height and facet_bump bit for bit.
    if terrain is not None:
        h2, r2 = terrain.copy(), np.zeros_like(terrain)
        ras = stamp_objects([sm_copy(o) for o in scene.objects], scene.grid,
                            h2, r2)
        fb = scene.facet_bump if scene.facet_bump is not None else 0.0
        fb2 = ras.facet_bump if ras.facet_bump is not None else 0.0
        # Re-stamping a rested object on a slope is not idempotent (the
        # stored roll/pitch already carry the bank tilt): compare outside
        # the rested objects' stamping windows only.
        free = np.ones(terrain.shape, dtype=bool)
        g = scene.grid
        n_rest = 0
        for o in scene.objects:
            if o.rested and CATALOG[o.type].solid(o) is not None:
                n_rest += 1
                pad = sm._solid_pad(o, g.resolution) + 2 * g.resolution
                i0, j0 = g.world_to_grid(o.x - pad, o.y - pad)
                i1, j1 = g.world_to_grid(o.x + pad, o.y + pad)
                free[max(int(j0), 0):int(np.ceil(j1)) + 1,
                     max(int(i0), 0):int(np.ceil(i1)) + 1] = False
        dh = float(np.max(np.abs(h2 - scene.height)[free]))
        dfb = float(np.max(np.abs(np.broadcast_to(np.asarray(fb2) - np.asarray(fb),
                                                  terrain.shape))[free]))
        check("[M0] re-stamping the terrain reproduces height and facet_bump",
              dh <= 1e-12 and dfb <= 1e-12,
              f"max |dh| {dh:.1e}, |dfb| {dfb:.1e} outside {n_rest} rested "
              f"objects' windows ({100 * free.mean():.1f} % of cells)")
        owner, _ = sm.owner_raster(scene, terrain)
        raised = (zs - terrain) > 1e-9
        orphan = int(np.count_nonzero(raised & (owner < 0)))
        stray = int(np.count_nonzero(~raised & (owner >= 0)))
        check("[M0] every raised cell has a rough-object owner, no other cell does",
              orphan == 0 and stray == 0,
              f"{int(raised.sum())} raised cells, {orphan} orphans, {stray} strays")
    else:
        skip("[M0] stamping consistency", "no depth.npz (legacy bundle)")

    # [M1] vertices are the raster samples; the triangle split departs from
    # bilinear by the quad twist / 4 (reported).
    seab = mesh.kind <= sm.KIND_ROUGH_OBJECT
    nv = scene.grid.nx * scene.grid.ny
    vz = mesh.vertices[:nv, 2].reshape(zs.shape)
    twist = np.abs(zs[:-1, :-1] + zs[1:, 1:] - zs[1:, :-1] - zs[:-1, 1:]) / 4
    check("[M1] seabed vertices equal height - facet_bump at every cell centre",
          bool(np.array_equal(vz, zs)) and int(seab.sum()) == 2 * (
              scene.grid.nx - 1) * (scene.grid.ny - 1),
          f"max triangle-vs-bilinear departure {twist.max() * 1000:.2f} mm "
          f"(99.9th pct {np.percentile(twist, 99.9) * 1000:.3f} mm)")

    oracle = sm.SceneOracle(mesh)
    bvh = oracle.bvh

    # [M2] smooth solids: every facet-layer patch centre lies on the mesh
    # surface, outside it by at most the chord sagitta.
    facet = [o for o in scene.objects if is_facet_object(o)]
    if facet:
        k = acoustics.wavenumber(SonarModelConfig())
        from blueboat_sss_sim.core.geometry import bilinear_sample

        def seabed(x, y):
            return bilinear_sample(zs, scene.grid, x, y, fill=float(zs.min()))
        worst, n_all, bad = 0.0, 0, []
        for o in facet:
            p = object_patches(o, seabed, 0.01, k)
            if p is None or len(p) == 0:
                continue
            off = 0.01
            mine_t = (mesh.object_id == o.object_id) & (
                mesh.kind == sm.KIND_SMOOTH_OBJECT)
            ob = TriangleBVH(mesh.vertices, mesh.triangles[mine_t])
            h = ob.closest(p.centre + off * p.normal, -p.normal, t_max=0.05)
            d = h.t - off                       # >= 0: centre outside mesh
            ok = (h.tri >= 0) & (d >= -1e-6) & (d <= 1e-3)
            mine = ok
            n_all += len(p)
            if not ok.all():
                bad.append((o.object_id, o.type, int((~ok).sum()), len(p)))
            dd = d[ok & mine]
            if dd.size:
                worst = max(worst, float(dd.max()))
        check("[M2] smooth-solid mesh vs facets.object_patches (<= 1 mm)",
              not bad, f"{len(facet)} objects, {n_all} patches, worst "
              f"{worst * 1000:.3f} mm" + (f"; failing {bad[:4]}" if bad else ""))
        vol_bad = []
        for o in facet:
            sol = CATALOG[o.type].solid(o)
            v, f = sm.solid_mesh(sol)
            err = sm.mesh_volume(v, f) / sm.solid_volume(sol) - 1.0
            if abs(err) > 0.01:
                vol_bad.append((o.type, sol.kind, round(100 * err, 2)))
        check("[M5] smooth-solid mesh volume vs closed form (<= 1 %)",
              not vol_bad, str(vol_bad[:4]) if vol_bad else "all within 1 %")
    else:
        skip("[M2/M5] smooth solids", "no facet object in this world")

    # [M3] walls: the Gazebo box, faces thickness/2 off the authored line.
    walls = getattr(scene, "walls", None) or []
    if walls:
        ok3 = True
        for wi, w in enumerate(walls):
            tv = mesh.vertices[mesh.triangles[mesh.wall_index == wi]]
            ux, uy = w.unit
            nrm = np.array([-uy, ux])
            dist = (tv[..., :2] - np.array([w.x0, w.y0])) @ nrm
            ok3 &= abs(np.abs(dist).max() - 0.5 * w.thickness) < 1e-9
            zb = -w.draft_m if w.kind == "pontoon" else None
            if zb is not None:
                ok3 &= abs(tv[..., 2].min() - zb) < 1e-9
        meta = None
        if world is not None and (world / "metadata.yaml").exists():
            import yaml
            meta = yaml.safe_load((world / "metadata.yaml").read_text())
        same = True
        if meta and meta.get("walls"):
            mw = meta["walls"]
            same = len(mw) == len(walls)
        check("[M3] wall boxes match the manifest (faces, draft) and metadata.yaml",
              bool(ok3) and same and int((mesh.kind == sm.KIND_WALL).sum())
              == 12 * len(walls), f"{len(walls)} walls")
    else:
        skip("[M3] walls", "no wall in this world")

    # [M4] no triangle from anywhere else; dry flags consistent.
    known = np.isin(mesh.kind, [0, 1, 2, 3, 4]).all()
    dz = mesh.vertices[mesh.triangles][:, :, 2]
    dry_ok = bool(((dz >= 0).all(axis=1) == mesh.dry)[seab].all())
    check("[M4] only seabed/objects/walls/apron triangles; dry flags exact",
          bool(known) and dry_ok, f"{int(mesh.dry.sum())} dry triangles")

    # BVH vs brute force on random crops of the scene.
    tri_c = mesh.triangle_vertices().mean(axis=1)
    xmin, ymin, xmax, ymax = mesh.grid_extent
    n_ok = n_tot = 0
    worst_t = 0.0
    special = np.flatnonzero(mesh.kind >= sm.KIND_SMOOTH_OBJECT)
    special = special[mesh.kind[special] != sm.KIND_APRON]
    for c in range(n_crops):
        cx = rng.uniform(xmin + 2, xmax - 2)
        cy = rng.uniform(ymin + 2, ymax - 2)
        if c == 0 and facet:                   # always crop one object
            cx, cy = facet[0].x, facet[0].y
        sel = np.flatnonzero((np.abs(tri_c[:, 0] - cx) < 1.25)
                             & (np.abs(tri_c[:, 1] - cy) < 1.25))
        sel = np.union1d(sel, special)
        sub = TriangleBVH(mesh.vertices, mesh.triangles[sel])
        zc = float(np.max(mesh.vertices[mesh.triangles[sel]][:, :, 2]))
        o = np.column_stack([rng.uniform(cx - 1.5, cx + 1.5, rays_per_crop),
                             rng.uniform(cy - 1.5, cy + 1.5, rays_per_crop),
                             rng.uniform(min(zc, -0.02) - 0.5, -0.01,
                                         rays_per_crop)])
        d = rng.normal(size=(rays_per_crop, 3))
        d /= np.linalg.norm(d, axis=1)[:, None]
        a = sub.closest(o, d, t_max=6.0)
        b = sub.brute_force_closest(o, d, t_max=6.0)
        same_tri = a.tri == b.tri
        both = np.isfinite(a.t) & np.isfinite(b.t)
        dt = np.zeros(rays_per_crop)
        dt[both] = np.abs(a.t[both] - b.t[both])
        n_ok += int((same_tri & (dt <= 1e-9)
                     & (np.isfinite(a.t) == np.isfinite(b.t))).sum())
        n_tot += rays_per_crop
        worst_t = max(worst_t, float(dt.max()))
    check("BVH closest hit == brute force (every ray x every triangle) on random crops",
          n_ok == n_tot, f"{n_ok}/{n_tot} rays over {n_crops} crops, "
          f"max |dt| {worst_t:.1e} m")

    # Full scene: hit points lie on their triangle; nothing closer.
    m = 20000
    o = np.column_stack([rng.uniform(xmin, xmax, m), rng.uniform(ymin, ymax, m),
                         np.full(m, -0.22)])
    d = rng.normal(size=(m, 3))
    d[:, 2] = -np.abs(d[:, 2]) * 0.3
    d /= np.linalg.norm(d, axis=1)[:, None]
    t0 = time.time()
    h = oracle.closest(o, d, t_max=40.0)
    dt_q = time.time() - t0
    hit = h.tri >= 0
    tv = mesh.vertices[mesh.triangles[h.tri[hit]]]
    p_bary = (tv[:, 0] + h.u[hit, None] * (tv[:, 1] - tv[:, 0])
              + h.v[hit, None] * (tv[:, 2] - tv[:, 0]))
    p_ray = o[hit] + h.t[hit, None] * d[hit]
    err = float(np.max(np.linalg.norm(p_bary - p_ray, axis=1))) if hit.any() else 0.0
    closer = oracle.bvh.occluded(o[hit], p_ray - 1e-5 * d[hit])
    check("full-scene closest hits: on their triangle, nothing closer",
          err < 1e-9 and not closer.any(),
          f"{int(hit.sum())}/{m} hit, point error {err:.1e} m, "
          f"{m / max(dt_q, 1e-9):,.0f} rays/s (CPU reference)")
    up = np.column_stack([o[:2000, :2], np.full(2000, -0.5)])
    ud = np.tile([0.0, 0.0, 1.0], (2000, 1))
    hs = oracle.closest(up, ud)
    hb = oracle.bvh.closest(up, ud, t_max=0.5)
    free = hb.tri < 0                     # nothing between z = -0.5 and 0
    ok_free = np.all(hs.tri[free] == sm.TRI_SURFACE) and np.allclose(hs.t[free], 0.5)
    ok_geom = np.all(hs.tri[~free] == hb.tri[~free])
    check("upward rays: the analytic surface at z = 0 unless geometry is closer",
          bool(ok_free and ok_geom),
          f"{int(free.sum())} reach the surface, {int((~free).sum())} meet "
          "land / walls / objects first")


def intersection_checks(rng: np.random.Generator) -> None:
    """The ray-triangle test itself, independently of the BVH."""
    print("\n[R1] ray-triangle test")
    m = 200000
    v = rng.normal(size=(m, 3, 3))
    o = rng.normal(size=(m, 3)) * 2.0
    d = rng.normal(size=(m, 3))
    tw, uw, vw = watertight(o, d, v[:, 0], v[:, 1], v[:, 2], 1e-9)
    tm, um, vm = moller_trumbore(o, d, v[:, 0], v[:, 1] - v[:, 0],
                                 v[:, 2] - v[:, 0], 1e-9)
    # Away from edges the two must agree exactly in hit/miss and to
    # round-off in t, u, v.
    margin = np.minimum.reduce([um, vm, 1 - um - vm])
    clear = np.abs(margin) > 1e-6
    agree = (np.isfinite(tw) == np.isfinite(tm))[clear]
    both = np.isfinite(tw) & np.isfinite(tm)
    err = max(float(np.max(np.abs(tw[both] - tm[both]) / np.maximum(1, np.abs(tm[both])))),
              float(np.max(np.abs(uw[both] - um[both]))),
              float(np.max(np.abs(vw[both] - vm[both]))))
    check("watertight == Moller-Trumbore away from edges (hit/miss, t, u, v)",
          bool(agree.all()) and err < 1e-9,
          f"{int(both.sum())} hits of {m}, max difference {err:.1e}")
    # Leak probe: from 1 mm inside the surface, along the outward normal
    # at every vertex and edge midpoint of closed solids, so the ray crosses
    # the surface exactly at that vertex / edge (a transversal crossing,
    # never a grazing touch). Every such ray must exit through the mesh.
    leaks = probes = mismatch = 0
    for sol in [Solid("cylinder", 0.126, 0.483, axis=0),
                Solid("capsule", 0.05, 0.3, axis=1), Solid("cone", 0.2, 0.5),
                Solid("annulus", 0.35, 0.2, 0.25), Solid("box", 0.4, 0.3, 0.2)]:
        vv, ff = sm.solid_mesh(sol)
        b = TriangleBVH(vv, ff)
        tv = vv[ff]
        fn = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0])
        vn = np.zeros_like(vv)
        for k in range(3):
            np.add.at(vn, ff[:, k], fn)
        e = np.concatenate([ff[:, [0, 1]], ff[:, [1, 2]], ff[:, [2, 0]]])
        en = np.concatenate([fn, fn, fn])
        key = np.sort(e, axis=1)
        uk, inv = np.unique(key, axis=0, return_inverse=True)
        edge_n = np.zeros((uk.shape[0], 3))
        np.add.at(edge_n, inv.ravel(), en)
        targets = np.concatenate([vv, 0.5 * (vv[uk[:, 0]] + vv[uk[:, 1]])])
        normals = np.concatenate([vn, edge_n])
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        for tilt in (0.0, 0.02, 0.2):
            dirs = normals + tilt * rng.normal(size=normals.shape)
            dirs /= np.linalg.norm(dirs, axis=1)[:, None]
            o = targets - 1e-3 * dirs
            # Count only starts verifiably inside (odd number of crossings
            # backwards, by brute force): at a sharp rim the averaged normal
            # can put the start outside, which is no crossing at all.
            back = b.brute_force_closest(o, -dirs, t_max=10.0)
            inside = back.tri >= 0
            h = b.closest(o, dirs, t_max=0.01)
            ref = b.brute_force_closest(o, dirs, t_max=0.01)
            leaks += int((inside & (h.tri < 0)).sum())
            mismatch += int((h.tri != ref.tri).sum())
            probes += int(inside.sum())
    check("no leak: rays crossing every mesh vertex and edge midpoint hit",
          leaks == 0 and mismatch == 0,
          f"{leaks} leaks of {probes} inside-start probes; BVH != brute "
          f"force on {mismatch} edge-case rays")


def direct_path_unit_checks() -> None:
    """Phase 2, direct path: propagation [A1] and the aperture [B1-B3]."""
    from blueboat_sss_sim.sonar.raycast import beam, propagation as pr
    print("\n[R2] direct path: propagation and aperture")
    # [A1] Francois-Garrison vs an independent transcription (the Phase-0
    # numbers script, ~/ros2_ws/sim_runs/raycast/notes/phase0_numbers.txt).
    ref = {(25, 34): 0.1546, (18, 35): 0.1275, (26, 34): 0.1596,
           (24, 0): 0.0397, (18, 0): 0.0476}
    got = {key: pr.absorption_fg(450e3, key[0], key[1], 1.0, 8.0) for key in ref}
    check("[A1] Francois-Garrison absorption at 450 kHz (five site waters)",
          all(abs(got[k] - ref[k]) < 6e-5 for k in ref),
          ", ".join(f"T{k[0]} S{k[1]}: {got[k]:.4f}" for k in ref) + " dB/m")
    r = np.array([0.3, 1.0, 7.3, 20.0, 95.0])
    a = 0.155
    tl = -20 * np.log10(pr.leg_amplitude(r, a))
    check("[A1] leg amplitude = 20 log r + alpha r",
          float(np.max(np.abs(tl - pr.transmission_loss_db(r, a)))) < 1e-9)

    w = pr.Water(temperature_c=20.0, salinity_ppt=35.0)
    k = w.wavenumber
    L = beam.aperture_from_width(0.5, k, two_way=True)
    H = beam.aperture_from_width(50.0, k, two_way=False)
    ap = beam.Aperture(L, H, k)
    e_a, e_n = np.array([1.0, 0, 0]), np.array([0, 1.0, 0])
    e_v = np.cross(e_n, e_a)
    print(f"      aperture L = {L * 100:.2f} cm, H = {H * 1000:.2f} mm, "
          f"far field L^2/lambda = {L * L * k / (2 * np.pi):.1f} m")
    # [B1] near field: Gauss-Legendre vs a dense trapezoid over the exact
    # distances, from 0.5 m to 20 m, on axis to the third sidelobe.
    worst_main = worst_side = 0.0
    for R in (0.5, 1.0, 2.0, 5.0, 10.0, 20.0):
        s = np.linspace(0.0, 3.5 * 2 * np.pi / (k * L), 301)
        d = R * (np.outer(s, e_a) + np.outer(np.sqrt(1 - s * s), e_n))
        b = ap.pattern(d, e_a, e_n, e_v)
        u = np.linspace(-L / 2, L / 2, 40001)
        rr = np.sqrt(R * R - 2 * np.outer(d @ e_a, u) + u[None, :] ** 2)
        f = np.exp(1j * k * (rr - R)) * R / rr
        ref_b = (getattr(np, "trapezoid", None) or np.trapz)(f, u, axis=1) / L
        db = 20 * np.log10(np.maximum(np.abs(b), 1e-12))
        dbr = 20 * np.log10(np.maximum(np.abs(ref_b), 1e-12))
        peak = dbr.max()
        main = dbr > peak - 3.0
        side = (dbr > peak - 40.0) & ~main
        worst_main = max(worst_main, float(np.max(np.abs(db - dbr)[main])))
        if side.any():
            worst_side = max(worst_side, float(np.max(np.abs(db - dbr)[side])))
    check("[B1] near-field aperture integral vs dense exact quadrature",
          worst_main <= 0.05 and worst_side <= 0.5,
          f"main lobe {worst_main:.4f} dB, sidelobes {worst_side:.4f} dB "
          "(0.5-20 m)")
    # [B2] far field: two-way -3 dB full width 0.5 deg.
    s = np.linspace(0, 0.01, 20001)
    d = 1e4 * (np.outer(s, e_a) + np.outer(np.sqrt(1 - s * s), e_n))
    two = np.abs(ap.pattern(d, e_a, e_n, e_v)) ** 4
    s3 = s[np.argmin(np.abs(10 * np.log10(two) + 3.0103))]
    width = 2 * np.degrees(np.arcsin(s3))
    check("[B2] far-field limit: two-way -3 dB width 0.5 deg",
          abs(width - 0.5) <= 0.01, f"{width:.4f} deg at 10 km")
    # [B3] vertical pattern: -3 dB one-way at +-25 deg off the tilted axis.
    sv = np.sin(np.radians(25.0))
    dv = np.array([[0.0, np.sqrt(1 - sv * sv), sv]]) * 5.0
    lvl = 20 * np.log10(abs(ap.pattern(dv, e_a, e_n, e_v, near_field=False)[0]))
    check("[B3] vertical aperture: one-way -3 dB at 25 deg off axis",
          abs(lvl + 3.0103) <= 0.05, f"{lvl:.4f} dB")


def flat_scene(depth: float = -3.0, half: float = 30.0, res: float = 0.1
               ) -> SceneModel:
    n = int(round(2 * half / res)) + 1
    g = GridSpec(origin_x=-half, origin_y=-half, resolution=res, nx=n, ny=n)
    h = np.full((n, n), depth)
    return SceneModel(grid=g, height=h, reflectivity=np.full(h.shape, 0.3),
                      material_id=np.zeros(h.shape, dtype=np.uint8),
                      material_names=["sand"], objects=[], walls=[], seed=11,
                      config_snapshot={}, outside="edge")


def direct_path_render_checks() -> None:
    """Phase 2, direct path: envelope [R1], pose [M6], flat seabed [S1],
    speckle statistics [C1]."""
    from blueboat_sss_sim.core.types import Pose3D, Side
    from blueboat_sss_sim.sonar.config import (SPEED_OF_SOUND_MPS,
                                               AcquisitionParams)
    from blueboat_sss_sim.sonar.renderer import GeometricRenderer
    from blueboat_sss_sim.sonar.raycast import propagation as pr
    from blueboat_sss_sim.sonar.raycast.receiver import BinGrid
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    print("\n[R2] direct path: envelope, pose, flat seabed, speckle")
    acq = AcquisitionParams()
    model = SonarModelConfig()
    scene = flat_scene()
    t0 = time.time()
    from blueboat_sss_sim.sonar.raycast.config import RaycastConfig, SamplingConfig
    # S1 tests the integration, at a sampling fine enough that its own
    # quadrature error (~spacing^2) is far below the gate.
    rr = RaycastRenderer(scene, acq, model, RaycastConfig(
        sampling=SamplingConfig(scatterer_spacing_m=0.0025)))
    print(f"      renderer built in {time.time() - t0:.1f} s; aperture "
          f"{rr.aperture.length_m * 100:.2f} cm; c = {rr.c:.1f} m/s; "
          f"alpha = {rr.alpha:.4f} dB/m; band half-width at 20 m "
          f"{rr._band_width(np.array([20.0]))[0]:.2f} m")
    # [R1] point target: peak at the device range r c_dev / c_true, shape =
    # the envelope.
    r = 10.01234
    g = BinGrid(acq, rr.envelope, rr.omega, SPEED_OF_SOUND_MPS)
    g.add(np.array([1.0]), np.array([2 * r / rr.c]), diffuse=False)
    prof = g.intensity
    b0 = int(np.argmax(prof))
    # parabolic peak interpolation
    y0, y1, y2 = prof[b0 - 1], prof[b0], prof[b0 + 1]
    frac = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
    r_peak = acq.range_start_mm / 1000 + (b0 + 0.5 + frac) * acq.bin_size_m
    r_dev = r * SPEED_OF_SOUND_MPS / rr.c
    shape = prof / prof.max()
    env = rr.envelope(g.t_b - 2 * r / rr.c) ** 2
    env = env / env.max()
    check("[R1] point target: peak at the device range, profile = envelope^2",
          abs(r_peak - r_dev) <= 0.1 * acq.bin_size_m
          and float(np.max(np.abs(shape - env))) <= 0.01,
          f"peak {r_peak:.4f} m vs {r_dev:.4f} m (bin {acq.bin_size_m * 100:.2f} cm); "
          f"envelope half-power width {np.sum(env >= 0.5)} bins")
    # [M6] transmit pose = the geometric renderer's sensor pose
    pose = Pose3D(1.0, -2.0, 0.03, 0.08, -0.05, 0.7)
    gr = GeometricRenderer(scene, acq, model)
    ok6 = True
    for side in (Side.PORT, Side.STARBOARD):
        a, b = rr.sensor_pose(side, pose), gr.sensor_pose(side, pose)
        ok6 &= all(abs(getattr(a, f) - getattr(b, f)) < 1e-12
                   for f in ("x", "y", "z", "roll", "pitch", "yaw"))
    check("[M6] transmit pose == GeometricRenderer.sensor_pose", bool(ok6))

    # [S1] flat seabed, level pose: the rendered mean vs an independent
    # midpoint quadrature of the same sonar equation over the plane.
    pose = Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    t0 = time.time()
    ping = rr.render(Side.PORT, pose, 0.0).ping
    t_render = time.time() - t0
    sensor = rr.sensor_pose(Side.PORT, pose)
    P = np.array([sensor.x, sensor.y, sensor.z])
    from blueboat_sss_sim.sonar.raycast import beam as bm
    e_a, e_n, e_v = bm.transducer_frame(0.0, 0.0, 0.0, 1.0, rr.tilt)
    hq = 0.003
    ref = np.zeros(acq.num_results)
    t_b = 2 * (acq.range_start_mm / 1000 + (np.arange(acq.num_results) + 0.5)
               * acq.bin_size_m) / SPEED_OF_SOUND_MPS
    zs = -3.0
    alt = P[2] - zs
    ys = np.arange(P[1] - alt, P[1] + rr.r_max, hq) + hq / 2
    for y in ys:
        dy = y - P[1]
        gr2 = rr.r_max ** 2 - alt ** 2 - dy ** 2
        Rg = np.sqrt(max(alt ** 2 + dy ** 2, 1e-9))
        wmax = float(rr._band_width(np.array([Rg]))[0]) + 0.05
        if gr2 <= 0:
            continue
        xs = np.arange(-wmax, wmax, hq) + hq / 2 + P[0]
        d = np.column_stack([xs - P[0], np.full(xs.size, dy),
                             np.full(xs.size, zs - P[2])])
        R = np.linalg.norm(d, axis=1)
        bb = rr.aperture.pattern(d, e_a, e_n, e_v)
        two = np.abs(bb) ** 4
        okb = (two >= rr._floor) & (R <= rr.r_max)
        if not okb.any():
            continue
        R, two = R[okb], two[okb]
        cos_i = alt / R
        sig = rr.law.sigma(cos_i, cos_i, np.zeros(cos_i.size, dtype=np.int64))
        pw = two * pr.leg_amplitude(R, rr.alpha) ** 4 * sig * hq * hq
        tau = 2 * R / rr.c
        dt = t_b[None, :] - tau[:, None]
        near = np.abs(dt) <= rr.envelope.half_support_s
        w = np.where(near, rr.envelope(np.where(near, dt, 0.0)), 0.0) ** 2
        ref += pw @ w
    sig_bins = ref > ref.max() * 1e-4
    err = 10 * np.log10(np.maximum(ping.power, 1e-300) / np.maximum(ref, 1e-300))
    e = np.abs(err[sig_bins])
    ranges = acq.range_start_mm / 1000 + (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
    check("[S1] flat seabed: rendered mean vs independent quadrature (<= 0.1 dB)",
          float(e.max()) <= 0.1,
          f"{int(sig_bins.sum())} bins within 40 dB of the peak: median "
          f"{np.median(e):.3f} dB, 99th pct {np.percentile(e, 99):.3f} dB, "
          f"max {e.max():.3f} dB at {ranges[sig_bins][np.argmax(e)]:.2f} m; "
          f"render {t_render:.1f} s per ping-side (CPU reference)")
    # [C1] speckle: texture over a homogeneous patch is Exp(1). Pings 1 m
    # apart along track (independent scatterer sets), bins 6-18 m.
    vals = []
    for i in range(12):
        pg = rr.render(Side.PORT, Pose3D(-6.0 + i * 1.0, 0.0, 0.0), 0.0).ping
        sel = (ranges > 6.0) & (ranges < 18.0)
        vals.append(pg.texture[sel][::4])           # thin out range overlap
    v = np.concatenate(vals)
    from scipy import stats
    D, pval = stats.kstest(v, "expon")
    check("[C1] texture of a homogeneous patch ~ Exp(1) (KS at 5 %)",
          D <= 1.36 / np.sqrt(v.size),
          f"n = {v.size}, KS D = {D:.4f} (critical {1.36 / np.sqrt(v.size):.4f}), "
          f"mean {v.mean():.3f}, CV {v.std() / v.mean():.3f}")


def po_closed_form_checks() -> None:
    """[O1-O3]: plate k^2 A^2 / pi, cylinder k a L^2, sphere pi a^2 through the
    ray caster's PO path (coherent deposit, TL, visibility), unit beam (the
    beam is gated by B1-B3), targets in their own far field at 15 m."""
    from blueboat_sss_sim.core.types import Pose3D, Side
    from blueboat_sss_sim.sonar.config import SPEED_OF_SOUND_MPS, AcquisitionParams
    from blueboat_sss_sim.sonar.raycast import propagation as pr
    from blueboat_sss_sim.sonar.raycast.receiver import BinGrid
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    from blueboat_sss_sim.worldgen.solids import rot_z, tessellate
    print("\n[R2] direct path: physical optics closed forms")
    acq = AcquisitionParams()
    rr = RaycastRenderer(flat_scene(), acq, SonarModelConfig())

    class _Unit:                       # unit beam, gated separately
        length_m = rr.aperture.length_m
        def pattern(self, d, *a, **k):
            return np.ones(np.atleast_2d(d).shape[0], dtype=complex)
    rr.aperture = _Unit()
    P = np.array([0.0, 0.0, -0.22])
    e_a, e_n = np.array([1.0, 0, 0]), np.array([0, 1.0, 0])
    e_v = np.cross(e_n, e_a)
    k = rr.k
    r0 = 15.0
    cases = [
        ("[O1] plate 0.10 x 0.10 m, normal incidence", Solid("box", 0.004, 0.10, 0.10),
         rot_z(np.pi / 2), lambda: k * k * 0.01 ** 2 / np.pi),
        ("[O2] cylinder a 0.05 m, L 0.10 m, broadside", Solid("cylinder", 0.05, 0.10, axis=0),
         np.eye(3), lambda: k * 0.05 * 0.10 ** 2),
        ("[O3] sphere a 0.10 m", Solid("capsule", 0.10, 0.20, axis=0),
         np.eye(3), lambda: np.pi * 0.10 ** 2),
    ]
    for name, sol, Rm, closed in cases:
        patches = tessellate(sol, 0.01, k * rr._rc.po.curvature_refine)
        # place: centre on the look axis at r0 (+ the half-depth facing us)
        centre = P + (r0 + sol.extent[1] if sol.kind == "box" else r0) * e_n
        pw = patches.transformed(Rm, centre - Rm @ sol.centre_offset)
        g = BinGrid(acq, rr.envelope, rr.omega, SPEED_OF_SOUND_MPS)
        n = rr.deposit_patches(P, e_a, e_n, e_v, pw, 1.0, 0.0, g)
        # nearest point of the target (the specular point / the face)
        rs = float(np.min(np.linalg.norm(pw.centre - P, axis=1)))
        s2 = rr.envelope(g.t_b - 2 * rs / rr.c) ** 2
        leg2 = pr.leg_amplitude(rs, rr.alpha) ** 2
        sig = 4 * np.pi * (np.sum(np.abs(g.coherent) ** 2) / np.sum(s2)) / leg2 ** 2
        err = 10 * np.log10(sig / closed())
        check(name, abs(err) <= 0.15,
              f"sigma {10 * np.log10(sig):.2f} dBsm vs {10 * np.log10(closed()):.2f} "
              f"({err:+.3f} dB), {n} lit patches")



def energy_validity_checks() -> None:
    """JR07 eq. 13.18 as a roughness-set validity diagnostic (gates amendment 6a,
    seabed_ssa.normal_incidence_energy): the SSA's normal-incidence scattered
    energy equals the flat-interface |V|^2 in the narrow-lobe limit."""
    import math as _m
    from blueboat_sss_sim.sonar.raycast import seabed_ssa as _ssa
    print("\n[R2] seabed law: normal-incidence energy validity (JR07 13.18)")
    p30 = _ssa.SedimentParams("jr07_fig13_2", 1.845, 1.1782, 0.01624, 3.25, 0.000141, 0.0)
    r = _ssa.normal_incidence_energy(p30, 2 * _m.pi * 30e3 / 1500.0, 30e3, 1500.0)
    check("[S-E] 30 kHz JR07 Fig. 13.2 sand: scattered energy = |V(90)|^2 within 0.1 dB (narrow lobe)",
          r["converged"] and abs(r["ratio_db"]) <= 0.1,
          f"{r['ratio_db']:+.3f} dB, half-energy cone {r['half_energy_cone_deg']:.1f} deg, {r['n']} nodes")
    sax = _ssa.SEDIMENTS["sand"]
    r4 = _ssa.normal_incidence_energy(sax, 2 * _m.pi * 450e3 / 1540.0, 450e3, 1540.0)
    bad = _ssa.SedimentParams("attempt2", sax.a_rho, sax.nu_p, sax.delta_p, 2.6000222831170428,
                              2.355823389776493e-05, 0.0)
    rb = _ssa.normal_incidence_energy(bad, 2 * _m.pi * 450e3 / 1540.0, 450e3, 1540.0)
    check("[S-E] 450 kHz: SAX99 sand valid, attempt 2's fitted set rejected (amendment 6a band)",
          _ssa.energy_valid(r4["ratio_db"]) and not _ssa.energy_valid(rb["ratio_db"]),
          f"SAX99 {r4['ratio_db']:+.2f} dB, attempt 2 {rb['ratio_db']:+.2f} dB "
          f"(band [{_ssa.ENERGY_VALID_LO_DB}, {_ssa.ENERGY_VALID_HI_DB}] dB)")


def correction_layer_checks() -> None:
    """The empirical correction layer (sonar/raycast/correction.py, owner
    2026-11-06): off = bit-identical, mean-only, edge-held, strict artifacts,
    speckle normalisation preserved through the shared noise stage."""
    import tempfile
    import yaml as _yaml
    from blueboat_sss_sim.calibration.seabed import acquisition_for, flat_scene
    from blueboat_sss_sim.core.types import Pose3D as _P, Side as _S
    from blueboat_sss_sim.sonar.config import SonarConfig, find_default_sonar_yaml
    from blueboat_sss_sim.sonar.noise import apply_ping_noise
    from blueboat_sss_sim.sonar.raycast import correction as corr
    from blueboat_sss_sim.sonar.raycast.config import RaycastConfig, shared_stage_model
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    print("\n[R2] empirical correction layer (not physics): switch, mean-only, artifacts")
    cfg = SonarConfig.from_yaml(find_default_sonar_yaml())
    acq = acquisition_for(cfg, 5000, 200, 44.3, 4)
    base = {"seed": 3}

    def render(extra):
        rc = RaycastConfig.from_dict({**base, **extra})
        rr = RaycastRenderer(flat_scene(1.6, 8.0), acq, shared_stage_model(cfg.model, rc.system_gain_db), rc)
        return rr.render(_S.PORT, _P(0.0, 0.0, 0.0, 0.01, 0.0, 0.0), 0.0).ping
    p0 = render({})
    p_off = render({"correction": {"enabled": False, "artifact": "/nonexistent.yaml"}})
    check("[CL1] correction off: power, texture and specular bit-identical to no correction section",
          np.array_equal(p0.power, p_off.power) and np.array_equal(p0.texture, p_off.texture)
          and not hasattr(p_off, "power_uncorrected"))
    art = dict(format=corr.FORMAT, version="test", label="test only", form="u",
               u_knots=[0.0, 1.0, 1.5, 3.0], db=[0.0, 0.0, 0.0, 0.0],
               domain={"altitude_m": [0.5, 3.0]}, fitted_on=["synthetic"], gates="test")
    with tempfile.TemporaryDirectory() as td:
        f0 = Path(td) / "zero.yaml"
        f0.write_text(_yaml.safe_dump(art))
        pz = render({"correction": {"enabled": True, "artifact": str(f0)}})
        check("[CL2] an all-zero table is bit-identical, and keeps the physical mean on the ping",
              np.array_equal(pz.power, p0.power) and np.array_equal(pz.power_uncorrected, p0.power))
        f1 = Path(td) / "crest.yaml"
        f1.write_text(_yaml.safe_dump({**art, "db": [0.0, 6.0, 3.0, 0.0]}))
        pc = render({"correction": {"enabled": True, "artifact": str(f1)}})
        r = (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
        expect = np.interp(np.clip(r / pc.altitude_m, 0.0, 3.0), [0.0, 1.0, 1.5, 3.0], [0.0, 6.0, 3.0, 0.0])
        lit = p0.power > 0
        got = 10 * np.log10(pc.power[lit] / p0.power[lit])
        check("[CL3] mean power x 10^(dB/10) of the table at u = r/altitude; texture untouched; beyond the "
              "last knot the edge value (0 dB) is held",
              np.max(np.abs(got - expect[lit])) < 1e-9 and np.array_equal(pc.texture, p0.texture)
              and np.all(pc.power[r > 3.0 * pc.altitude_m] == p0.power[r > 3.0 * pc.altitude_m]),
              f"max |dB err| {np.max(np.abs(got - expect[lit])):.1e}")
        # speckle normalisation through the shared noise stage, same seed
        m = shared_stage_model(cfg.model, RaycastConfig.from_dict(base).system_gain_db)
        wc = r < pc.altitude_m
        n0 = apply_ping_noise(p0.power, wc, 1.0, m, np.random.default_rng(5), specular=p0.specular,
                              texture=p0.texture)
        n1 = apply_ping_noise(pc.power, wc, 1.0, m, np.random.default_rng(5), specular=pc.specular,
                              texture=pc.texture)
        strong = p0.power > 1e3 * m.noise_floor
        if p0.specular is not None:              # the ringing's own (low-CV) channel must not dominate
            strong &= p0.power > 1e3 * p0.specular
        rel = np.abs(n1[strong] / pc.power[strong] - n0[strong] / p0.power[strong])
        check("[CL4] speckle preserved: noisy / mean identical with and without the correction (same seed) "
              "where the echo dominates the floors",
              strong.sum() > 20 and float(rel.max()) < 1e-2, f"{int(strong.sum())} bins, max |diff| {rel.max():.1e}")
    bad = []
    for mutate in ({"format": "x"}, {"form": "poly"}, {"u_knots": [0.0, 0.0, 1.0, 2.0]}, {"db": [0.0, 1.0]},
                   {"extra": 1}, {"fitted_on": []}, {"db": [0.0, float("nan"), 0.0, 0.0]}):
        try:
            corr.from_dict({**art, **mutate})
            bad.append(mutate)
        except (KeyError, ValueError):
            pass
    check("[CL5] malformed artifacts are refused (format, form, knots, shape, unknown key, no sessions, NaN)",
          not bad, f"accepted: {bad}")
    # gates amendment 6b (2026-11-07): off outside the domain, the physical core, the engine path
    from blueboat_sss_sim.sonar.raycast import calib as _rcal
    rc0 = RaycastConfig.from_dict(base)
    m0 = shared_stage_model(cfg.model, rc0.system_gain_db)
    with tempfile.TemporaryDirectory() as td:
        def art_file(name, **over):
            f = Path(td) / f"{name}.yaml"
            f.write_text(_yaml.safe_dump({**art, "db": [0.0, 6.0, 3.0, 0.0], **over}))
            return str(f)

        def on(path):
            return {"correction": {"enabled": True, "artifact": path}}
        fa = art_file("alt", domain={"altitude_m": [3.0, 5.0]})
        pa = render(on(fa))
        check("[CL6] altitude outside the domain: OFF -- power bit-identical to the physical ping, status "
              "'off:altitude', counted",
              np.array_equal(pa.power, p0.power) and np.array_equal(pa.texture, p0.texture)
              and pa.correction_status == corr.OFF_ALTITUDE
              and corr._CACHE[fa].counts.get(corr.OFF_ALTITUDE, 0) >= 1,
              f"status {getattr(pa, 'correction_status', None)}, altitude {pa.altitude_m:.2f} m")
        other = {"label": "1200", "num_results": 1200, "range_mm": [18000, 52000], "pulse_us": [44.3, 86.5]}
        mine = {"label": "this", "num_results": int(acq.num_results),
                "range_mm": [int(acq.range_length_mm)] * 2, "pulse_us": [44.3, 44.3]}
        p_out = render(on(art_file("acq_out", domain={"altitude_m": [0.5, 3.0], "acquisitions": [other]})))
        p_in = render(on(art_file("acq_in", domain={"altitude_m": [0.5, 3.0], "acquisitions": [other, mine]})))
        check("[CL7] acquisition gate: outside every recorded group -> OFF and bit-identical; inside one -> "
              "applied",
              np.array_equal(p_out.power, p0.power) and p_out.correction_status == corr.OFF_ACQUISITION
              and p_in.correction_status == corr.APPLIED and np.array_equal(p_in.power, pc.power),
              f"{p_out.correction_status} / {p_in.correction_status}")
        p_mud = render(on(art_file("mud", domain={"altitude_m": [0.5, 3.0], "nadir_material": ["mud"]})))
        p_sand = render(on(art_file("sand", domain={"altitude_m": [0.5, 3.0], "nadir_material": ["sand"]})))
        check("[CL8] nadir gate: a seabed of another material -> OFF and bit-identical; sand -> applied",
              np.array_equal(p_mud.power, p0.power) and p_mud.correction_status == corr.OFF_NADIR
              and p_sand.correction_status == corr.APPLIED and np.array_equal(p_sand.power, pc.power),
              f"{p_mud.correction_status} / {p_sand.correction_status}")
        good = corr.core_fingerprint(rc0, m0)
        bad_core = corr.core_fingerprint(RaycastConfig.from_dict({**base, "system_gain_db": 140.0}), m0)
        f_good = art_file("core_ok", domain={"altitude_m": [0.5, 3.0]}, core=good)
        f_bad = art_file("core_bad", domain={"altitude_m": [0.5, 3.0]}, core=bad_core)
        p_core = render(on(f_good))
        try:
            render(on(f_bad))
            refused, why = False, ""
        except ValueError as e:
            refused, why = True, str(e)
        check("[CL9] an artifact fitted on another physical core is refused at construction; the matching "
              "core applies",
              refused and "system_gain_db" in why and p_core.correction_status == corr.APPLIED
              and np.array_equal(p_core.power, pc.power), why[:120])
        # the calibration engine's noisy path: the physical render cached, the correction on a copy
        rc_on = {**base, **on(f_good)}
        rr_e, _, key_e = _rcal.noisy_renderer(cfg, acq, 1.6, 8.0, rc_on, material="sand")
        rr_p, _, key_p = _rcal.noisy_renderer(cfg, acq, 1.6, 8.0, dict(base), material="sand")
        pose = _P(0.0, 0.0, 0.0, 0.01, 0.0, 0.0)
        n_before = len(_rcal._PINGS)
        e_c = _rcal.cached_render(rr_e, key_e, _S.PORT, pose, correction=_rcal.correction_for_rc(rc_on))
        e_p = _rcal.cached_render(rr_p, key_p, _S.PORT, pose)

        def no_build():
            raise AssertionError("a renderer was built for a cached ping")
        e_c2 = _rcal.cached_render(no_build, _rcal.noisy_key(acq, 1.6, 8.0, rc_on, "sand"), _S.PORT, pose,
                                   correction=_rcal.correction_for_rc(rc_on))
        check("[CL10] engine noisy path: the corrected copy equals the renderer-applied correction (bit for "
              "bit), the physical and corrected arms share ONE render (same texture), and a cached ping builds "
              "no renderer",
              key_e == key_p and rr_e is rr_p and len(_rcal._PINGS) == n_before + 1
              and np.array_equal(e_c2.power, e_c.power) and e_c2.texture is e_c.texture
              and np.array_equal(e_c.power, p_core.power)
              and np.allclose(e_c.power, p_core.power, rtol=1e-12, atol=0.0)
              and np.array_equal(e_p.power, p0.power) and np.array_equal(e_c.power_uncorrected, e_p.power)
              and e_c.texture is e_p.texture and e_c.correction_status == corr.APPLIED
              and not hasattr(e_p, "correction_status"),
              f"{len(_rcal._PINGS) - n_before} new render(s)")
    nonlocal_bad = []
    for mutate in ({"db": [1.0, 6.0, 3.0, 0.0]}, {"db": [0.0, 6.0, 3.0, -0.5]},
                   {"form": "u_logalt", "log10_alt_knots": [0.0, 0.5],
                    "db": [[0.0, 6.0, 3.0, 0.0], [0.0, 6.0, 3.0, 2.0]]}):
        try:
            corr.from_dict({**art, **mutate})
            nonlocal_bad.append(mutate)
        except ValueError:
            pass
    check("[CL11] a table that is not localised (non-zero first or last u-knot, either form) is refused",
          not nonlocal_bad, f"accepted: {nonlocal_bad}")

def seabed_law_checks() -> None:
    """[S2-S5]: the small-slope seabed law, the volume term, reflection."""
    import math
    from scipy import special
    from blueboat_sss_sim.sonar.raycast import seabed_ssa as ss
    print("\n[R2] direct path: seabed law (small slope + volume)")
    x = np.linspace(0.0, 300.0, 300001)
    ej = float(np.max(np.abs(ss._j0(x) - special.j0(x))))
    check("[S2] J0 (numpy) vs scipy.special.j0 on [0, 300]", ej < 2e-8,
          f"max |error| {ej:.1e}")
    qs = np.logspace(-4, 3, 57)
    e1 = max(abs(ss.ik(q, 1.0) / (q / (1 + q * q) ** 1.5) - 1) for q in qs)
    e2 = max(abs(ss.ik(q, 2.0) / (math.exp(-1 / (4 * q)) / (2 * q)) - 1)
             # gamma_2 = 4 is the boundary of the physical range (2 < gamma_2
             # < 4): below q = 0.05 its value is exponentially small, under
             # the double-precision floor of any oscillatory quadrature
             # (measured: 6.5e-10 relative above q = 0.05, 3.5e-6 at 0.02).
             for q in qs if q >= 0.05)
    check("[S2] I_K vs closed forms (gamma_2 = 3: q/(1+q^2)^1.5; gamma_2 = 4: "
          "exp(-1/4q)/2q, q >= 0.05), q 1e-4..1e3", e1 < 1e-6 and e2 < 1e-6,
          f"max relative error {e1:.1e} / {e2:.1e}")
    cont = []
    for beta in (1.04, 1.29, 0.6, 1.6):
        a, b = ss.ik_series(0.02, beta), ss.ik_quadrature(0.02, beta)
        cont.append(abs(a / b - 1))
    check("[S2] I_K series and quadrature agree at the switch (q = 0.02)",
          max(cont) < 1e-6, " / ".join(f"{c:.1e}" for c in cont)
          + " for beta 1.04, 1.29, 0.6, 1.6")
    tab = ss.IkTable(1.04)
    qr = np.logspace(-6, 6, 200)
    et = float(np.max(np.abs(tab(qr) / np.array([ss.ik(q, 1.04) for q in qr]) - 1)))
    check("[S2] I_K table interpolation (log-log) vs direct", et < 1e-3,
          f"max relative error {et:.1e}")
    sand = ss.SEDIMENTS["sand"]
    for f_hz, label in ((40e3, "40 kHz"), (451127.0, "450 kHz")):
        k = 2 * math.pi * f_hz / 1500.0
        law = ss.SSALaw(sand, k, f_hz, 1500.0, table=tab)
        g = np.radians(np.array([3.0, 5.0, 8.0]))
        sg = np.sin(g)
        sig = law.roughness(sg, sg, -np.ones(3))
        # SPM (13.9): k^4 |A|^2 W(dK), W = w2 / dK^gamma2
        ci = np.cos(g)
        Vi = ss.reflection(sand, sg)
        stp = ss.sin_theta_p(sand.a_p, ci)
        ap2 = sand.a_p ** 2
        B = stp * stp / (ap2 * sand.a_rho)
        G = (1 - 1 / sand.a_rho) * (-ci * ci - B) - 1 + 1 / (ap2 * sand.a_rho)
        A = 0.5 * (1 + Vi) ** 2 * G
        dK = 2 * k * ci
        spm = k ** 4 * np.abs(A) ** 2 * sand.w2 / dK ** sand.gamma2
        dd = 10 * np.log10(sig / spm)
        check(f"[S2] small slope -> perturbation at low grazing ({label}, 3/5/8 deg)",
              float(np.max(np.abs(dd))) < 0.2,
              " / ".join(f"{v:+.3f}" for v in dd) + " dB")
    k = 2 * math.pi * 451127.0 / 1500.0
    law = ss.SSALaw(sand, k, 451127.0, 1500.0, table=tab)
    rng = np.random.default_rng(3)
    si, sj = rng.uniform(0.05, 1, 200), rng.uniform(0.05, 1, 200)
    cp = rng.uniform(-1, 1, 200)
    rec = float(np.max(np.abs(law.sigma(si, sj, cp) / law.sigma(sj, si, cp) - 1)))
    check("[S2] reciprocity sigma(i, s) = sigma(s, i)", rec < 1e-9,
          f"max relative difference {rec:.1e}")
    gb = np.radians([5, 10, 20, 30, 45, 60, 80, 89])
    bs = law.backscatter(np.sin(gb))
    print("      450 kHz SAX99-sand backscatter strength (dB) at "
          + ", ".join(f"{int(round(np.degrees(a)))} deg {10 * np.log10(b):.1f}"
                      for a, b in zip(gb, bs)))
    vol = law.volume(np.sin(gb), np.sin(gb))
    below = vol[np.degrees(gb) < 25].max()
    above = vol[(np.degrees(gb) > 35) & (np.degrees(gb) < 70)].min()
    check("[S3] sediment volume term suppressed below the critical angle "
          "(sand, 30.5 deg)", below < 0.1 * above,
          f"max below 25 deg {10 * np.log10(below):.1f} dB, min 35-70 deg "
          f"{10 * np.log10(above):.1f} dB")
    lossless = ss.SedimentParams("t", 2.0, 1.16, 0.0, 3.0, 1e-5, 1e-3)
    th = np.radians(np.array([5.0, 15.0, 25.0, 29.0]))
    V = ss.reflection(lossless, np.sin(th))
    Vn = ss.reflection(sand, np.array([1.0]))[0]
    ray = (sand.a_rho * sand.a_p - 1) / (sand.a_rho * sand.a_p + 1)
    check("[S4] flat reflection: total below the critical angle (lossless), "
          "Rayleigh at normal incidence",
          float(np.max(np.abs(np.abs(V) - 1))) < 1e-12 and abs(Vn - ray) < 1e-12,
          f"|V| {np.abs(V).round(12).tolist()}, V(90 deg) {abs(Vn):.4f}")


def grouped_mean_checks() -> None:
    """The coherent-group accumulator the bounce paths rely on: one
    scatterer's paths interfere (mean = |sum|^2), scatterers do not."""
    from blueboat_sss_sim.sonar.config import AcquisitionParams, SPEED_OF_SOUND_MPS
    from blueboat_sss_sim.sonar.raycast.receiver import BinGrid, envelope_for
    print("\n[R2] receiver: coherent groups (path pairs of one scatterer)")
    acq = AcquisitionParams()
    env = envelope_for(acq, 0.0, True, 0.0, 1.4)
    om = 2 * np.pi * 451127
    g = BinGrid(acq, env, om, SPEED_OF_SOUND_MPS)
    det = np.array([1.0, -0.8 + 0.1j])
    tau = np.array([2 * 10 / 1518, 2 * 10.02 / 1518])
    g.add_group(det, np.ones(2), tau, np.array([7, 7]))
    ref = BinGrid(acq, env, om, SPEED_OF_SOUND_MPS)
    ref.add(det, tau, diffuse=False)
    rng = np.random.default_rng(0)
    n = 5000
    d2 = rng.normal(size=n) + 1j * rng.normal(size=n)
    t2 = rng.uniform(2 * 3 / 1518, 2 * 15 / 1518, n)
    g2 = BinGrid(acq, env, om, SPEED_OF_SOUND_MPS)
    g2.add_group(d2, np.ones(n), t2, np.arange(n), chunk=700)
    r2 = BinGrid(acq, env, om, SPEED_OF_SOUND_MPS)
    r2.add(d2, t2, diffuse=True)
    check("grouped mean: one group = |coherent sum|^2; singleton groups = "
          "incoherent mean (chunked)",
          bool(np.allclose(g.mean, np.abs(ref.coherent) ** 2)
               and np.allclose(g.field, ref.field)
               and np.allclose(g2.mean, r2.mean_diffuse)))


def shared_stage_checks() -> None:
    """The shared noise/encoder stages under the ray-cast system gain keep
    the measured reported levels (the NC #4-step units bug, 2026-11-03)."""
    from blueboat_sss_sim.sonar.raycast.config import shared_stage_model
    print("\n[R2] shared stages under the ray-cast system gain")
    base = SonarModelConfig()
    m = shared_stage_model(base, 126.6)
    lv = lambda x, off: 10 * np.log10(x) + off     # noqa: E731
    d1 = abs(lv(m.noise_floor, 126.6) - lv(base.noise_floor, base.calibration_db_offset))
    d2 = abs(lv(m.watercolumn_noise, 126.6)
             - lv(base.watercolumn_noise, base.calibration_db_offset))
    check("noise floor and water column keep their reported dB; reverberation 0",
          d1 < 1e-9 and d2 < 1e-9 and m.reverberation_db == 0.0,
          f"water column {lv(m.watercolumn_noise, 126.6):.1f} dB reported")


def surface_path_checks() -> None:
    """[S6] surface image, [P1] Lloyd's mirror (one-way nulls; two-way
    through the grouped receiver)."""
    from blueboat_sss_sim.sonar.config import AcquisitionParams, SPEED_OF_SOUND_MPS
    from blueboat_sss_sim.sonar.raycast import paths as pa
    from blueboat_sss_sim.sonar.raycast.receiver import BinGrid, envelope_for
    from blueboat_sss_sim.sonar.waveguide import eckart_c
    print("\n[R2] surface path: image, Lloyd's mirror")
    rng = np.random.default_rng(5)
    k = 2 * np.pi * 451127 / 1541.0
    P = np.array([0.0, 0.0, -0.22])
    X = np.column_stack([rng.uniform(-20, 20, 500), rng.uniform(-20, 20, 500),
                         rng.uniform(-8, -0.3, 500)])
    sp = pa.surface(P, X, k, 3e-3)
    Ps = np.array([0.0, 0.0, 0.22])
    psi = np.arcsin(np.abs(sp.arrive[:, 2]))
    ok6 = (np.allclose(sp.length, np.linalg.norm(X - Ps, axis=1))
           and np.allclose(sp.depart[:, 2], -sp.arrive[:, 2])
           and np.allclose(sp.depart[:, :2], sp.arrive[:, :2])
           and np.allclose(sp.factor, -np.sqrt(eckart_c(psi, 3e-3, k))))
    check("[S6] surface image: length |X - P_s|, equal angles, factor -sqrt(C)",
          bool(ok6))
    # [P1] one-way nulls of U = e^{ikLd}/Ld - sqrt(C) e^{ikLs}/Ls (calm)
    zt, zo = 0.22, 0.9
    lam = 2 * np.pi / k
    r = np.linspace(4.0, 60.0, 400001)
    Xr = np.column_stack([r, np.zeros_like(r), np.full_like(r, -zo)])
    d, sf = pa.direct(P, Xr), pa.surface(P, Xr, k, 0.0)
    U = np.exp(1j * k * d.length) / d.length + sf.factor * np.exp(1j * k * sf.length) / sf.length
    a = np.abs(U)
    mins = np.flatnonzero((a[1:-1] < a[:-2]) & (a[1:-1] < a[2:])) + 1
    r_min = r[mins]
    worst_exact = worst_appr = 0.0
    from scipy.optimize import brentq
    for i, rm in zip(mins, r_min):
        # the order from the actual path difference (the approximation is
        # off by one at 4 m, where z/r is not small)
        nn = int(round((sf.length[i] - d.length[i]) / lam))
        f = lambda x: (np.hypot(x, zt + zo) - np.hypot(x, zo - zt)) - nn * lam  # noqa: E731
        rex = brentq(f, 0.5, 500.0)
        worst_exact = max(worst_exact, abs(rm / rex - 1))
        if rm >= 10 * (zt + zo):          # where 2 z_t z_o / r holds
            worst_appr = max(worst_appr, abs(rm / (2 * zt * zo / (nn * lam)) - 1))
    depth = 20 * np.log10(a[mins].max() / a.max())
    check("[P1] Lloyd one-way nulls at L_s - L_d = n lambda (exact); 2 z_t z_o / "
          "(n lambda) within 1 % beyond 10 (z_t + z_o); nulls <= -20 dB",
          worst_exact < 1e-3 and worst_appr < 0.01 and depth <= -20.0,
          f"{len(r_min)} nulls in 4-60 m, exact {worst_exact:.1e}, approx "
          f"{100 * worst_appr:.2f} %, shallowest null {depth:.1f} dB re max")
    # two-way through the grouped receiver, at a null vs the next lobe
    acq = AcquisitionParams(range_length_mm=40000, num_results=1200)
    env = envelope_for(acq, 0.0, True, 0.0, 1.4)
    om = k * 1541.0
    def two_way_peak(rr):
        Xs = np.array([[rr, 0.0, -zo]])
        paths_ = [pa.direct(P, Xs), pa.surface(P, Xs, k, 0.0)]
        dets, taus = [], []
        for pi in paths_:
            for pj in paths_:
                dets.append(pi.factor[0] * pj.factor[0] / (pi.length[0] * pj.length[0]))
                taus.append((pi.length[0] + pj.length[0]) / 1541.0)
        g = BinGrid(acq, env, om, SPEED_OF_SOUND_MPS)
        g.add_group(np.array(dets), np.ones(4), np.array(taus), np.zeros(4, int))
        return g.mean.max()
    def ratio_at(n0):
        f = lambda x, m: (np.hypot(x, zt + zo) - np.hypot(x, zo - zt)) - m * lam  # noqa: E731
        r_null = brentq(lambda x: f(x, n0), 0.5, 500.0)
        r_lobe = brentq(lambda x: f(x, n0 - 0.5), 0.5, 500.0)
        return r_null, 10 * np.log10(two_way_peak(r_null) / two_way_peak(r_lobe))
    r15, q15 = ratio_at(5)       # ~23 m pair spread vs envelope: partly filled
    r29, q29 = ratio_at(4)
    check("[P1] two-way (grouped receiver): a scatterer at the n = 4 null "
          "(~29 m) >= 20 dB under the next lobe; the null fills at shorter range "
          "as the path pairs spread across the envelope", q29 <= -20.0,
          f"n=4 null {r29:.1f} m: {q29:.1f} dB; n=5 null {r15:.1f} m: {q15:.1f} dB")


def bottom_path_checks() -> None:
    """[S4/S5] the bottom path: the seabed sheet's mean field (PO) vs the
    image source, monostatic and oblique; the floor-bounce eigenray vs
    V(psi) sqrt(C); Fermat on a curved floor (P6, <= lambda/20)."""
    from blueboat_sss_sim.core.types import Pose3D, Side
    from blueboat_sss_sim.sonar.config import (SPEED_OF_SOUND_MPS,
                                               AcquisitionParams)
    from blueboat_sss_sim.sonar.raycast import beam as bm
    from blueboat_sss_sim.sonar.raycast import chains as ch
    from blueboat_sss_sim.sonar.raycast import paths as pa
    from blueboat_sss_sim.sonar.raycast import propagation as pr
    from blueboat_sss_sim.sonar.raycast import seabed_ssa as ssa
    from blueboat_sss_sim.sonar.raycast.config import RaycastConfig
    from blueboat_sss_sim.sonar.raycast.receiver import BinGrid
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    print("\n[R2] bottom path: the floor's mean field, floor-bounce eigenrays")
    model = SonarModelConfig()
    worst_db = worst_ph = 0.0
    detail = []
    for alt in (0.6, 1.4, 5.0, 10.0, 20.0):
        acq = AcquisitionParams(range_length_mm=max(20000, int(np.ceil(alt * 1.6)) * 1000))
        rr = RaycastRenderer(flat_scene(-(alt + model.sensor_depth_m)), acq,
                             model, RaycastConfig())
        P = np.array([*(lambda s: (s.x, s.y, s.z))(rr.sensor_pose(Side.PORT, Pose3D(0, 0, 0)))])
        e_a, e_n, e_v = bm.transducer_frame(0, 0, 0, 1.0, rr.tilt)
        for C in (1.0, 0.9, 0.5, 0.1):
            sig = np.sqrt(-np.log(C)) / (2 * rr.k)
            rr.specular.surf.rms_height = (lambda mat, R, lam, dx, s=sig:
                                           np.full(np.shape(R), s))
            c = rr.floor_specular_contributions(P, e_a, e_n, e_v, beam=False,
                                                visibility=False)
            g = BinGrid(acq, rr.envelope, rr.omega, SPEED_OF_SOUND_MPS)
            g.add(c["amp"], c["tau"], diffuse=False)
            V = ssa.reflection(ssa.SEDIMENTS["sand"], np.array([1.0]))[0]
            ref = BinGrid(acq, rr.envelope, rr.omega, SPEED_OF_SOUND_MPS)
            ref.add(np.array([V * np.sqrt(C) * pr.leg_amplitude(2 * alt, rr.alpha)]),
                    np.array([2 * alt / rr.c]), diffuse=False)
            b = int(np.argmax(np.abs(ref.field)))
            e_db = abs(20 * np.log10(abs(g.field[b]) / abs(ref.field[b])))
            e_ph = abs(float(np.angle(g.field[b] / ref.field[b])))
            worst_db, worst_ph = max(worst_db, e_db), max(worst_ph, e_ph)
        detail.append(f"{alt:g} m: {c['patches']} patches")
    check("[S4/S5] monostatic: the floor's mean field = image source V(90) sqrt(C)/(2h) "
          "(0.6-20 m, C = 1/0.9/0.5/0.1; <= 0.05 dB, <= 0.05 rad)",
          worst_db <= 0.05 and worst_ph <= 0.05,
          f"worst {worst_db:.4f} dB, {worst_ph:.4f} rad; " + ", ".join(detail))
    # oblique: bistatic PO between A and B, and the floor-bounce eigenray
    depth = 3.0
    acq = AcquisitionParams(range_length_mm=80000, num_results=1200)
    rr = RaycastRenderer(flat_scene(-depth, half=45.0), acq, model, RaycastConfig())
    rr.r_max = 200.0
    sig = 2e-4
    rr.specular.surf.rms_height = lambda mat, R, lam, dx, s=sig: np.full(np.shape(R), s)
    A = np.array([0.0, 0.0, -0.5])
    e_a, e_n, e_v = bm.transducer_frame(0, 0, 0, 1.0, rr.tilt)
    floor = ch.FloorBoundary(rr.specular)
    wpo = wch = 0.0
    lens = 0.0
    for psi_deg in (80.0, 60.0, 30.0, 15.0, 8.0):
        zB = -1.0
        dx = (depth + A[2] + depth + zB) / np.tan(np.radians(psi_deg))
        B = np.array([0.0, dx, zB])
        rr._band_bbox = lambda P_, ea, en, _d=dx: (-2.0, -3.0, 2.0, _d + 3.0)
        c = rr.floor_specular_contributions(A, e_a, e_n, e_v, B=B, beam=False,
                                            visibility=False)
        g = BinGrid(acq, rr.envelope, rr.omega, SPEED_OF_SOUND_MPS)
        g.add(c["amp"], c["tau"], diffuse=False)
        Af = A.copy(); Af[2] = -2 * depth - A[2]
        Lr = np.linalg.norm(B - Af)
        s_ = np.sin(np.radians(psi_deg))
        V = ssa.reflection(ssa.SEDIMENTS["sand"], np.array([s_]))[0]
        fac = V * np.exp(-0.5 * (2 * rr.k * sig * s_) ** 2)
        ref = BinGrid(acq, rr.envelope, rr.omega, SPEED_OF_SOUND_MPS)
        ref.add(np.array([fac * pr.leg_amplitude(Lr, rr.alpha)]),
                np.array([Lr / rr.c]), diffuse=False)
        b = int(np.argmax(np.abs(ref.field)))
        wpo = max(wpo, abs(20 * np.log10(abs(g.field[b]) / abs(ref.field[b]))))
        cb = ch.build(A, B[None, :], [floor], None, lam=2 * np.pi / rr.k)
        wch = max(wch, abs(20 * np.log10(abs(cb.factor[0]) / abs(fac))))
        lens = max(lens, abs(cb.length[0] - Lr))
    rr.specular.surf.rms_height = type(rr.specular.surf).rms_height.__get__(rr.specular.surf)
    check("[S4/S5] oblique (8-80 deg grazing): bistatic PO mean field and the "
          "floor-bounce eigenray = V(psi) sqrt(C) at the image distance (<= 0.05 dB)",
          wpo <= 0.05 and wch <= 0.05 and lens < 1e-9,
          f"PO worst {wpo:.4f} dB; eigenray worst {wch:.2e} dB, length {lens:.1e} m")
    # a surface chain is paths.surface
    rng = np.random.default_rng(3)
    X = np.column_stack([rng.uniform(-15, 15, 300), rng.uniform(-15, 15, 300),
                         rng.uniform(-2.9, -0.3, 300)])
    P = np.array([0.0, 0.0, -0.22])
    srf = ch.SurfaceBoundary(rr.k, 3e-4)
    a, b2 = ch.build(P, X, [srf]), pa.surface(P, X, rr.k, 3e-4)
    check("surface chain == paths.surface (length, directions, factor)",
          bool(np.allclose(a.length, b2.length) and np.allclose(a.depart, b2.depart)
               and np.allclose(a.arrive, b2.arrive) and np.allclose(a.factor, b2.factor)))
    # [P6] Fermat on a sloping floor vs a brute-force stationary search
    n = 501
    g6 = GridSpec(origin_x=-25.0, origin_y=-25.0, resolution=0.1, nx=n, ny=n)
    xx = g6.origin_x + np.arange(n) * 0.1
    yy = g6.origin_y + np.arange(n) * 0.1
    Xg, Yg = np.meshgrid(xx, yy)
    h6 = -3.0 - 0.04 * Yg - 0.15 * np.sin(2 * np.pi * (Xg + 0.5 * Yg) / 20.0)
    sc6 = SceneModel(grid=g6, height=h6, reflectivity=np.full(h6.shape, 0.3),
                     material_id=np.zeros(h6.shape, dtype=np.uint8),
                     material_names=["sand"], objects=[], walls=[], seed=3,
                     config_snapshot={}, outside="edge")
    rr6 = RaycastRenderer(sc6, AcquisitionParams(), model, RaycastConfig())
    fl6 = ch.FloorBoundary(rr6.specular)
    lam6 = 2 * np.pi / rr6.k
    Xs = np.column_stack([rng.uniform(-6, 6, 40), rng.uniform(4, 18, 40),
                          rng.uniform(-1.5, -0.5, 40)])
    cb6 = ch.build(P, Xs, [fl6], None, lam=lam6)
    worst6 = 0.0
    n_ok = 0
    for i in np.flatnonzero(np.isfinite(cb6.length) & (np.abs(cb6.factor) > 0)):
        Q = cb6.bounces[0][i]
        # local brute force: the path length on a fine sheet grid around Q
        u = np.linspace(-0.3, 0.3, 121)
        qx, qy = np.meshgrid(Q[0] + u, Q[1] + u)
        z, _, _ = rr6.specular._locate(qx.ravel(), qy.ravel())
        Qs = np.column_stack([qx.ravel(), qy.ravel(), z])
        Ls = np.linalg.norm(Qs - P, axis=1) + np.linalg.norm(Xs[i] - Qs, axis=1)
        # the discrete stationary point of the true sheet path length: the
        # grid point of least |grad L| (interior), compared in length
        Lg = Ls.reshape(qx.shape)
        gy, gx = np.gradient(Lg, u[1] - u[0])
        gm = np.hypot(gx, gy)[2:-2, 2:-2]
        jy, jx = np.unravel_index(int(np.argmin(gm)), gm.shape)
        if min(jy, jx) < 2 or max(jy, jx) > gm.shape[0] - 3:
            continue                      # stationary point outside the window
        worst6 = max(worst6, abs(cb6.length[i] - Lg[jy + 2, jx + 2]))
        n_ok += 1
    check("[P6] Fermat on a sloping, gently curved floor (4 % + 0.15 m / 20 m): "
          "the chain's reflection point is stationary (path error <= lambda/20)",
          n_ok > 20 and worst6 <= lam6 / 20,
          f"{n_ok} chains, worst {worst6 * 1e3:.3f} mm vs lambda/20 = "
          f"{lam6 / 20 * 1e3:.3f} mm; iterations {cb6.iterations}")


def comb_checks() -> None:
    """[P2] the image comb: chains in an ideal waveguide (|R| = 1, calm)
    vs the finite image sum at the same bounce cap and the closed form;
    [P4] the pool FBR ramp from the floor's mean field through transducer
    images (isotropic beam): direct, mixed (r_D + r_S)/2, surface-surface."""
    from blueboat_sss_sim.core.types import Pose3D, Side
    from blueboat_sss_sim.sonar.config import (SPEED_OF_SOUND_MPS,
                                               AcquisitionParams)
    from blueboat_sss_sim.sonar import waveguide as wg
    from blueboat_sss_sim.sonar.raycast import beam as bm
    from blueboat_sss_sim.sonar.raycast import chains as ch
    from blueboat_sss_sim.sonar.raycast import seabed_ssa as ssa
    from blueboat_sss_sim.sonar.raycast.config import (PathsConfig,
                                                       RaycastConfig,
                                                       shared_stage_model)
    from blueboat_sss_sim.sonar.raycast.receiver import BinGrid
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    print("\n[R2] image comb and the pool FBR ramp")
    model = shared_stage_model(SonarModelConfig(), 126.6)
    D, zt = 0.9, 0.22
    rr = RaycastRenderer(flat_scene(-D), AcquisitionParams(), model,
                         RaycastConfig(paths=PathsConfig(surface_rms_m=0.0)))
    mp = rr.multipath
    mp.floor.factor = lambda Q, sp, Re: np.ones(Q.shape[0], dtype=complex)
    P = np.array([0.0, 0.0, -zt])
    worst_fin, worst_cl = 0.0, 0.0
    det = []
    for r, M in ((1.5, 6), (1.5, 40), (4.0, 6), (4.0, 12)):
        X = np.array([[0.0, r, -zt]])
        tot = 0.0
        for seq in ch.sequences([mp.surface, mp.floor], M, 0):
            c = ch.build(P, X, seq, None, mp.lam)
            if abs(c.factor[0]) > 0:
                tot += abs(c.factor[0]) ** 2 / c.length[0] ** 2
        im = wg.image_set(zt, D, M + 2)
        sel = im.n_bounce <= M
        ref = float(np.sum(1.0 / (r * r + (im.depth[sel] - zt) ** 2)))
        worst_fin = max(worst_fin, abs(tot / ref - 1.0))
        det.append(f"r {r} m, {M} bounces: {10 * np.log10(tot / wg.ideal_image_sum_closed(r, D, zt)):+.3f} dB re closed form")
        if (r, M) == (1.5, 40):
            worst_cl = abs(10 * np.log10(tot / wg.ideal_image_sum_closed(r, D, zt)))
    check("[P2] ideal-waveguide comb: chains = the image sum at the same bounce cap "
          "(<= 1e-9), -> the closed form (<= 0.2 dB at 40 bounces, r = 1.5 m)",
          worst_fin <= 1e-9 and worst_cl <= 0.2,
          f"finite worst {worst_fin:.1e}; " + "; ".join(det))
    # [P4] pool ramp: nadir 0.68 m in 0.9 m calm water, smooth floor (C = 1)
    rr = RaycastRenderer(flat_scene(-D), AcquisitionParams(), model,
                         RaycastConfig(paths=PathsConfig(surface_rms_m=0.0,
                                                         max_order=3)))
    rr.specular.surf.rms_height = lambda mat, R, lam, dx: np.zeros(np.shape(R))
    s_ = rr.sensor_pose(Side.PORT, Pose3D(0, 0, 0))
    P = np.array([s_.x, s_.y, s_.z])
    e_a, e_n, e_v = bm.transducer_frame(0, 0, 0, 1.0, rr.tilt)
    acq = AcquisitionParams()
    g = BinGrid(acq, rr.envelope, rr.omega, SPEED_OF_SOUND_MPS)
    rep = rr.multipath.floor_coherent(P, e_a, e_n, e_v, g, np.zeros(g.n),
                                      beam=False, visibility=False)
    h = D - zt
    V = ssa.reflection(ssa.SEDIMENTS["sand"], np.array([1.0]))[0]
    a = np.abs(g.coherent)
    worst_pos, amps = 0.0, []
    for L, amp_ref, name in ((2 * h, abs(V) / (2 * h), "direct"),
                             (2 * h + 2 * zt, 2 * abs(V) / (2 * h + 2 * zt), "mixed"),
                             (2 * h + 4 * zt, abs(V) / (2 * h + 4 * zt), "surface-surface")):
        att = 10 ** (-rr.alpha * L / 20)
        b_exp = (0.5 * L * SPEED_OF_SOUND_MPS / rr.c - g.start_m) / g.bin_m - 0.5
        lo, hi = int(b_exp) - 4, int(b_exp) + 5
        k = lo + int(np.argmax(a[lo:hi]))
        y0, y1, y2 = a[k - 1], a[k], a[k + 1]
        frac = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
        worst_pos = max(worst_pos, abs(k + frac - b_exp))
        amps.append(f"{name} {20 * np.log10(a[k] / (amp_ref * att * rr.envelope(np.array([0.0]))[0])):+.2f} dB")
    check("[P4] pool FBR ramp (0.68 m altitude, 0.9 m calm water, smooth floor): "
          "direct / mixed (r_D + r_S)/2 / surface-surface arrivals within 1 bin",
          worst_pos <= 1.0,
          f"worst {worst_pos:.2f} bin; peak amplitude vs image theory: "
          + ", ".join(amps) + f"; floor pairs {rep['pairs']}, order {rep['order_reached']}")


def wall_checks() -> None:
    """[P3] a wall image exists only while its reflection point lies on the
    face; [P5] an opaque sill between the boat and a quay in 0.9 m of calm
    water removes the comb images whose legs it blocks (difference
    reported)."""
    from blueboat_sss_sim.core.types import Pose3D, Side, Wall
    from blueboat_sss_sim.sonar.config import AcquisitionParams
    from blueboat_sss_sim.sonar.raycast import beam as bm
    from blueboat_sss_sim.sonar.raycast import chains as ch
    from blueboat_sss_sim.sonar.raycast.config import (PathsConfig,
                                                       RaycastConfig,
                                                       SamplingConfig,
                                                       shared_stage_model)
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    print("\n[R2] wall paths: image validity, a sill shadowing the comb")
    model = shared_stage_model(SonarModelConfig(), 126.6)
    sc = flat_scene(-3.0)
    sc.walls = [Wall("quay", -2.0, 8.0, 2.0, 8.0, top_z=0.5)]
    rr = RaycastRenderer(sc, AcquisitionParams(), model, RaycastConfig())
    mp = rr.multipath
    face = [f for f in mp.faces if f.n[1] < -0.9][0]
    P = np.array([0.0, 0.0, -0.22])
    xs = np.linspace(-8.0, 8.0, 3201)
    X = np.column_stack([xs, np.full(xs.size, 6.0), np.full(xs.size, -1.0)])
    c = ch.build(P, X, [face], rr.oracle, mp.lam)
    yf = face.c[1]
    Pw = P.copy(); Pw[1] = 2 * yf - P[1]
    t = (Pw[1] - yf) / (Pw[1] - X[:, 1])
    Qx = Pw[0] + t * (X[:, 0] - Pw[0])
    expect = np.abs(Qx - face.c[0]) <= face.hu
    got = np.abs(c.factor) > 0
    check("[P3] wall image: valid exactly while the reflection point lies on the "
          "face (no ghost beyond a wall end)", bool(np.array_equal(expect, got)),
          f"{int(got.sum())} of {xs.size} valid; face half-length {face.hu:.2f} m; "
          f"mismatches {int((expect != got).sum())}")
    # [P5] 0.9 m calm water, quay at y = 8 m, opaque sill at y = 4 m
    acq = AcquisitionParams()
    renders = {}
    for label, walls in (("no sill", [Wall("quay", -10.0, 8.0, 10.0, 8.0, top_z=0.5)]),
                         ("sill", [Wall("quay", -10.0, 8.0, 10.0, 8.0, top_z=0.5),
                                   Wall("sill", -0.6, 4.0, 0.6, 4.0, top_z=-0.35,
                                        thickness=0.3)])):
        s9 = flat_scene(-0.9)
        s9.walls = walls
        r9 = RaycastRenderer(s9, acq, model, RaycastConfig(
            paths=PathsConfig(surface_rms_m=3e-4),
            sampling=SamplingConfig(scatterer_spacing_m=0.01)))
        r9.specular.surf.rms_height = lambda mat, R, lam, dx: np.full(np.shape(R), 1e-5)
        sensor = r9.sensor_pose(Side.PORT, Pose3D(0, 0, 0))
        renders[label] = (r9.render(Side.PORT, Pose3D(0, 0, 0), 0.0).ping.power,
                          dict(r9.multipath.last_report.get("diffuse", {})))
    a, ra = renders["no sill"]
    b, rb = renders["sill"]
    r_dev = acq.range_start_mm / 1000 + (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
    face_r = 7.85 * 1500.0 / rr.c
    bands = []
    for lo, hi in ((face_r - 0.3, face_r + 0.3), (face_r + 0.3, face_r + 1.0),
                   (face_r + 1.0, face_r + 3.0)):
        m = (r_dev >= lo) & (r_dev < hi) & (a > 0)
        bands.append(f"{lo:.1f}-{hi:.1f} m: {10 * np.log10(b[m].sum() / a[m].sum()):+.2f} dB")
    # the images the sill blocks vanish: the wall scatterers straight behind it
    # lose their direct chain; chains over its top (surface) survive
    s9 = flat_scene(-0.9)
    s9.walls = [Wall("quay", -10.0, 8.0, 10.0, 8.0, top_z=0.5),
                Wall("sill", -0.6, 4.0, 0.6, 4.0, top_z=-0.35, thickness=0.3)]
    r9 = RaycastRenderer(s9, acq, model, RaycastConfig(paths=PathsConfig(surface_rms_m=3e-4)))
    Pt = np.array([sensor.x, sensor.y, sensor.z])
    Xw = np.array([[sensor.x, 7.849, -0.8], [sensor.x, 7.849, -0.1]])
    cd = ch.build(Pt, Xw, [], r9.oracle, r9.multipath.lam)
    cs = ch.build(Pt, Xw, [r9.multipath.surface], r9.oracle, r9.multipath.lam)
    ok5 = (not cd.visible[0]) and cd.visible[1] and cs.visible[0]
    check("[P5] a sill between the boat and a quay (0.9 m calm water): the comb "
          "images whose legs it blocks vanish, the others stay (tail reported)",
          bool(ok5) and rb.get("contributors", 0) > 0,
          f"low wall point: direct blocked {not cd.visible[0]}, over-the-top "
          f"surface path clear {bool(cs.visible[0])}; upper point direct clear "
          f"{bool(cd.visible[1])}; sill/no-sill energy: " + ", ".join(bands)
          + f"; chains kept {ra.get('via')} / {rb.get('via')}")


def diffraction_checks() -> None:
    """[D1] ray-traced Fresnel-Kirchhoff on a knife edge (a 2 mm screen's
    top edge) vs the Fresnel integral, v in [-2, 3]; [D2] the shadow edge
    behind a sill scales as sqrt(lambda R)."""
    from scipy.special import fresnel
    from blueboat_sss_sim.core.types import Wall
    from blueboat_sss_sim.sonar.raycast.diffraction import Diffraction
    from blueboat_sss_sim.sonar.raycast import scene_mesh as smx
    print("\n[R2] diffraction: knife edge")
    sc = flat_scene(-20.0)
    sc.walls = [Wall("screen", -25.0, 10.0, 25.0, 10.0, top_z=-2.0,
                     thickness=0.002)]
    mesh = smx.build_scene_mesh(sc)
    orc = smx.SceneOracle(mesh)
    lam = 1541.0 / 451127.0
    dif = Diffraction(orc, lam)
    vs = np.linspace(-2.0, 3.0, 21)
    d1 = d2 = 10.0 - 0.001
    h = vs / np.sqrt(2 * (d1 + d2) / (lam * d1 * d2))
    z = -2.0 - h                       # leg below the edge: v > 0
    A = np.column_stack([np.zeros_like(z), np.full_like(z, 0.0), z])
    B = np.column_stack([np.zeros_like(z), np.full_like(z, 20.0), z])
    U = dif.factor(A, B, np.full(z.size, 10.0))
    S, C = fresnel(vs)                 # scipy returns (S, C)
    F = 0.5 * (1 + 1j) * ((0.5 - C) - 1j * (0.5 - S))
    ref = F
    e_db = np.abs(20 * np.log10(np.abs(U) / np.abs(ref)))
    e_ph = np.abs(np.angle(U / ref))
    check("[D1] knife edge: |U/U0| vs |F(v)| <= 0.5 dB, phase <= 0.1 rad, v in [-2, 3]",
          float(e_db.max()) <= 0.5 and float(e_ph.max()) <= 0.1,
          f"worst {e_db.max():.3f} dB at v = {vs[np.argmax(e_db)]:+.2f}, "
          f"{e_ph.max():.3f} rad at v = {vs[np.argmax(e_ph)]:+.2f}; "
          f"|U| at v=0: {abs(U[np.argmin(np.abs(vs))]):.3f} (0.5)")
    # [D2] the shadow edge behind a sill scales as sqrt(lambda R): the whole
    # geometry scaled with R (transducer height, sill, distances), lambda
    # fixed; ground width between |U|^2 = 0.25 (the geometric boundary) and
    # 0.05 on the shadow side
    Rs = np.array([2.0, 3.0, 4.5, 6.5, 9.5, 14.0, 20.0])
    widths, local = [], []
    for R in Rs:
        zf = -0.16 * R                            # every length scales with R
        sc2 = flat_scene(zf, half=max(30.0, 1.3 * R))
        sc2.walls = [Wall("sill", -0.5 * R, 0.6 * R, 0.5 * R, 0.6 * R,
                          top_z=zf + 0.03 * R, thickness=0.002)]
        m2 = smx.build_scene_mesh(sc2)
        o2 = smx.SceneOracle(m2)
        dif2 = Diffraction(o2, lam, end_margin_m=0.0)
        Pz = np.array([0.0, 0.0, -0.01 * R])
        ys = np.linspace(0.62 * R, 1.0 * R, 1200)
        X2 = np.column_stack([np.zeros_like(ys), ys, np.full_like(ys, zf + 1e-4 * R)])
        L2 = np.linalg.norm(X2 - Pz, axis=1)
        d1v = L2 * (0.6 * R) / ys                 # the plane through the sill
        I = np.abs(dif2.leg_factors(np.broadcast_to(Pz, X2.shape), X2, d1v)) ** 2
        # from the lit far side inward: the geometric boundary (|U|^2 first
        # <= 0.25), then deeper into the shadow (<= 0.05)
        Ir, yr = I[::-1], ys[::-1]

        def cross(level, start=0):
            k = start + int(np.argmax(Ir[start:] <= level))
            # linear interpolation in log intensity between k-1 and k
            a, b = np.log(Ir[k - 1]), np.log(max(Ir[k], 1e-12))
            f = (np.log(level) - a) / (b - a)
            return yr[k - 1] + f * (yr[k] - yr[k - 1]), k
        y25, k25 = cross(0.25)
        y05, _ = cross(0.05, k25)
        widths.append(y25 - y05)
        # local edge width at the geometric boundary: 0.25 / |dI/dy|
        dy = 1e-3 * R
        Xp = np.array([[0.0, y25 - dy, zf + 1e-4 * R], [0.0, y25 + dy, zf + 1e-4 * R]])
        Lp = np.linalg.norm(Xp - Pz, axis=1)
        Ip = np.abs(dif2.leg_factors(np.broadcast_to(Pz, Xp.shape), Xp,
                                     Lp * (0.6 * R) / Xp[:, 1])) ** 2
        local.append(0.25 / abs((Ip[1] - Ip[0]) / (2 * dy)))
    slope = float(np.polyfit(np.log(Rs), np.log(local), 1)[0])
    slope_x = float(np.polyfit(np.log(Rs), np.log(widths), 1)[0])
    check("[D2] shadow-edge width behind a knife-edge sill ~ sqrt(lambda R): local "
          "width 0.25/|dI/dy| at the geometric boundary, log-log slope 0.5 +- 0.05 "
          "over R = 2-20 m (scaled geometry)",
          abs(slope - 0.5) <= 0.05,
          f"slope {slope:.3f}; local widths " + ", ".join(f"{w * 100:.1f} cm" for w in local)
          + f"; (crossing distance 0.25->0.05: slope {slope_x:.3f}, finite-width "
          "nonlinearity of v(y), recorded 2026-11-03)")


def corner_checks() -> None:
    """[O4] dihedral 8 pi a^2 b^2 / lambda^2 (two a x b plates at 90 deg, on
    the bisector), [O5] trihedral 4 pi L^4 / (3 lambda^2) (triangular faces,
    on the symmetry axis): PO at the first bounce in time, GO bounces on the
    other faces after it (Hwang et al. 2015), far field (300 m)."""
    from blueboat_sss_sim.sonar.raycast import chains as ch
    from blueboat_sss_sim.sonar.raycast.multipath import po_pair
    print("\n[R2] corner paths: dihedral, trihedral")
    k = 2 * np.pi * 451127 / 1541.0
    lam = 2 * np.pi / k
    h = 0.002

    def plate(origin, eu, ev, wu, wv, tri):
        eu, ev = np.asarray(eu, float), np.asarray(ev, float)
        uu = (np.arange(int(round(wu / h))) + 0.5) * h
        vv = (np.arange(int(round(wv / h))) + 0.5) * h
        U, V = (a.ravel() for a in np.meshgrid(uu, vv))
        if tri:
            keep = U / wu + V / wv <= 1.0
            U, V = U[keep], V[keep]
        n = np.cross(eu, ev)
        c = np.asarray(origin, float) + np.outer(U, eu) + np.outer(V, ev)
        m = U.size
        poly = (np.array([[0, 0], [wu, 0], [0, wv]]) if tri else
                np.array([[0, 0], [wu, 0], [wu, wv], [0, wv]]))
        face = ch.FaceBoundary("f", origin, n, eu, ev, np.inf, np.inf,
                               lambda s_: np.ones(np.shape(s_), dtype=complex),
                               0.0, k, kind="O", poly=poly)
        return dict(c=c, n=np.tile(n, (m, 1)), u=np.tile(eu, (m, 1)),
                    v=np.tile(ev, (m, 1)), a=np.full(m, h), b=np.full(m, h),
                    face=face)

    def rcs(plates, d, R, max_b):
        P = R * np.asarray(d, float) / np.linalg.norm(d)
        tot = 0.0 + 0.0j
        for pl in plates:
            X = pl["c"]
            dc = ch.build(P, X, [], None, lam)
            others = [q["face"] for q in plates if q is not pl]
            for seq in ch.sequences(others, max_b, 0, max_b):
                cj = ch.build(P, X, seq, None, lam)
                ok = np.abs(cj.factor) > 0
                if not ok.any():
                    continue
                po = po_pair(pl["n"][ok], pl["u"][ok], pl["v"][ok], pl["a"][ok],
                             pl["b"][ok], dc.arrive[ok], -cj.arrive[ok], k)
                L = dc.length[ok] + cj.length[ok]
                tot += np.sum(po * cj.factor[ok] * np.exp(-1j * k * L)
                              / (dc.length[ok] * cj.length[ok]))
        return 4 * np.pi * abs(tot * R * R) ** 2

    a = b = 0.10
    # plate A: x in [0, a], z in [-b/2, b/2], normal +y; plate B: y in
    # [0, a], z in [-b/2, b/2], normal +x (both facing the viewer)
    dih = [plate([0, 0, -b / 2], [0, 0, 1], [1, 0, 0], b, a, False),
           plate([0, 0, -b / 2], [0, 1, 0], [0, 0, 1], a, b, False)]
    s_d = rcs(dih, [1, 1, 0], 300.0, 1)
    ref_d = 8 * np.pi * a * a * b * b / lam ** 2
    e_d = 10 * np.log10(s_d / ref_d)
    L = 0.10
    tri = [plate([0, 0, 0], [1, 0, 0], [0, 1, 0], L, L, True),
           plate([0, 0, 0], [0, 1, 0], [0, 0, 1], L, L, True),
           plate([0, 0, 0], [0, 0, 1], [1, 0, 0], L, L, True)]
    s_t = rcs(tri, [1, 1, 1], 300.0, 2)
    ref_t = 4 * np.pi * L ** 4 / (3 * lam ** 2)
    e_t = 10 * np.log10(s_t / ref_t)
    check("[O4] dihedral 0.1 x 0.1 m plates on the bisector: 8 pi a^2 b^2 / lambda^2 "
          "(<= 0.5 dB)", abs(e_d) <= 0.5,
          f"{10 * np.log10(s_d):.2f} dBsm vs {10 * np.log10(ref_d):.2f} ({e_d:+.3f} dB)")
    check("[O5] trihedral, triangular faces L = 0.1 m, on the axis: 4 pi L^4 / (3 lambda^2) "
          "(<= 0.5 dB)", abs(e_t) <= 0.5,
          f"{10 * np.log10(s_t):.2f} dBsm vs {10 * np.log10(ref_t):.2f} ({e_t:+.3f} dB)")


def transmission_checks() -> None:
    """[O6] a flooded pp_shell box (the 68 L field-box class) on flat sand:
    the shadow behind it is transmitted through its walls by the fluid-layer
    formula at each crossing angle -- REPORTED (the measured 0.11 is a
    hold-out number, never a target; the pool box is the development check
    in Phase 7)."""
    from blueboat_sss_sim.core.types import Pose3D, Side
    from blueboat_sss_sim.sonar.config import AcquisitionParams
    from blueboat_sss_sim.sonar.raycast import scene_mesh as smx
    from blueboat_sss_sim.sonar.raycast.config import (RaycastConfig,
                                                       SamplingConfig,
                                                       shared_stage_model)
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    print("\n[R2] transmission: a flooded pp_shell box")
    lib = MaterialLibrary()
    t_n = smx.layer_transmission(np.array([1.0]), np.array([905 / 1025]),
                                 np.array([2740 / 1500]), np.array([2e-3]),
                                 2 * np.pi * 451127 / 1500)[0]
    model = shared_stage_model(SonarModelConfig(), 126.6)
    acq = AcquisitionParams()
    out = []
    for r0 in (6.1, 10.1):
        res = {}
        for label, with_box in (("box", True), ("none", False)):
            g = GridSpec(origin_x=-15.0, origin_y=-5.0, resolution=0.05, nx=601, ny=601)
            h = np.full((g.ny, g.nx), -1.7)
            objs = []
            if with_box:
                cfg = ObjectFieldConfig.from_dict({"classes": {}, "explicit": [
                    {"type": "plastic_box", "x": -0.345, "y": r0, "yaw": 0.0}]})
                objs = place_objects(g, cfg, lib, np.random.default_rng(1), height=h)
            refl = np.full(h.shape, 0.3)
            ras = stamp_objects(objs, g, h, refl)
            sc = SceneModel(grid=g, height=h, reflectivity=refl, **ras.scene_kwargs(),
                            material_id=np.zeros(h.shape, dtype=np.uint8),
                            material_names=["sand"], objects=objs, walls=[], seed=5,
                            config_snapshot={}, outside="edge")
            rr = RaycastRenderer(sc, acq, model, RaycastConfig(
                sampling=SamplingConfig(scatterer_spacing_m=0.01)))
            res[label] = rr.render(Side.PORT, Pose3D(0, 0, 0), 0.0).ping.power
            if with_box:
                o = objs[0]
                far_edge = np.hypot(r0 + 0.5 * o.width, 1.7 - 0.22) * 1500 / rr.c
        r_dev = (np.arange(acq.num_results) + 0.5) * acq.bin_size_m
        m = (r_dev > far_edge + 0.05) & (r_dev < far_edge + 1.05)
        depth = 10 * np.log10(res["box"][m].mean() / res["none"][m].mean())
        out.append(f"{r0} m: {depth:+.1f} dB")
    check("[O6] flooded pp_shell box: two-way shadow depth over 0-1 m behind the echo "
          "(REPORTED; transmission emerges from the layer formula)", True,
          f"{'; '.join(out)}; one crossing at normal incidence |T|^2 = "
          f"{abs(t_n) ** 2:.3f} ({10 * np.log10(abs(t_n) ** 2):+.2f} dB)")


def fit_basis_checks() -> None:
    """The fit basis (fitbasis.py) reproduces full renders for any fitted
    parameter set (w2, gamma2, f_v, TVG k): <= 0.02 dB within 40 dB of the
    peak, flat sand at two altitudes, level and rolled."""
    import dataclasses
    from blueboat_sss_sim.core.types import Pose3D, Side
    from blueboat_sss_sim.sonar.config import AcquisitionParams
    from blueboat_sss_sim.sonar.raycast.config import (BeamConfig, RaycastConfig,
                                                       ReceiverConfig,
                                                       SeabedLawConfig,
                                                       shared_stage_model)
    from blueboat_sss_sim.sonar.raycast.fitbasis import build_basis, law_and_surface
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    print("\n[R2] fit basis: parametric re-evaluation vs full renders")
    model = shared_stage_model(SonarModelConfig(), 126.6)
    acq = AcquisitionParams()
    sets = [dict(sed={}, f_v=0.0, k=0.0),
            dict(sed={"sand": {"w2": 4.33e-7, "gamma2": 3.3}}, f_v=0.12, k=12.0),
            dict(sed={"sand": {"w2": 2e-6, "gamma2": 2.6}}, f_v=0.3, k=30.0)]
    worst, n_case, t_basis = 0.0, 0, []
    for alt, pose in ((1.4, Pose3D(0, 0, 0)), (6.0, Pose3D(0, 0, 0.05, 0.08, -0.03, 0.0))):
        sc = flat_scene(-(alt + model.sensor_depth_m))
        base_rr = RaycastRenderer(sc, acq, model, RaycastConfig())
        t0 = time.time()
        basis = build_basis(base_rr, Side.PORT, pose)
        t_basis.append(time.time() - t0)
        for st in sets:
            rc = RaycastConfig(seabed=SeabedLawConfig(sediments=st["sed"]),
                               beam=BeamConfig(vertical_floor_amp=st["f_v"]),
                               receiver=ReceiverConfig(tvg_k=st["k"]))
            full = RaycastRenderer(sc, acq, model, rc, mesh=base_rr.mesh,
                                   oracle=base_rr.oracle).render(Side.PORT, pose, 0.0).ping.power
            law, surf = law_and_surface(base_rr, st["sed"])
            pb = basis.power(law, surf, st["f_v"], st["k"])
            sel = full > full.max() * 1e-4
            worst = max(worst, float(np.max(np.abs(10 * np.log10(pb[sel] / full[sel])))))
            n_case += 1
    check("fit basis = full render for any fitted parameter set (<= 0.02 dB)",
          worst <= 0.02,
          f"{n_case} cases, worst {worst:.4f} dB; basis build {np.mean(t_basis):.1f} s per pose")
    # the generated ray-cast profile: shared stage = the comparison profile,
    # raycast section = PROVISIONAL_RAYCAST (regenerate, never hand-edit)
    import yaml
    from blueboat_sss_sim.sonar.config import SonarConfig, find_default_sonar_yaml
    from blueboat_sss_sim.sonar.raycast.config import PROVISIONAL_RAYCAST, load_profile
    prof = ROOT / "config" / "raycast_default_sonar.yaml"
    c, rcp = load_profile(prof)
    g = SonarConfig.from_yaml(find_default_sonar_yaml())
    doc = yaml.safe_load(prof.read_text())
    check("config/raycast_default_sonar.yaml: shared stage = default_sonar.yaml, raycast "
          "section = PROVISIONAL_RAYCAST", c.model == g.model and c.acquisition == g.acquisition
          and doc.get("raycast") == PROVISIONAL_RAYCAST and doc.get("renderer") == "raycast",
          f"system gain {rcp.system_gain_db} dB, TVG k {rcp.receiver.tvg_k}")


def gpu_checks() -> None:
    """[G1] the GPU oracle (gpu_warp.py) = the CPU reference: the Phase-1
    leak probe through it (0 leaks), and renders on both backends (flat sand;
    a quay, a sill and the comb in 0.9 m calm water): mean eps_mu <= 1e-4,
    same-seed intensity |I_gpu - I_cpu| / P <= 1e-3. Skips visibly without Warp."""
    try:
        from blueboat_sss_sim.sonar.raycast.gpu_warp import WarpBVH
    except Exception as exc:                        # noqa: BLE001
        skip("[G1] GPU = CPU", f"no Warp in this interpreter ({type(exc).__name__}); "
             "run with ~/ros2_ws/sim_runs/raycast/venv/bin/python")
        return
    from blueboat_sss_sim.core.types import Pose3D, Side, Wall
    from blueboat_sss_sim.sonar.config import AcquisitionParams
    from blueboat_sss_sim.sonar.raycast.config import RaycastConfig, shared_stage_model
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    print("\n[R4] GPU oracle (Warp) vs the CPU reference")
    rng = np.random.default_rng(4)
    leaks = probes = 0
    for sol in [Solid("cylinder", 0.126, 0.483, axis=0), Solid("capsule", 0.05, 0.3, axis=1),
                Solid("cone", 0.2, 0.5), Solid("annulus", 0.35, 0.2, 0.25),
                Solid("box", 0.4, 0.3, 0.2)]:
        vv, ff = sm.solid_mesh(sol)
        cpu, gpu = TriangleBVH(vv, ff), WarpBVH(vv, ff)
        tv = vv[ff]
        fn = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0])
        vn = np.zeros_like(vv)
        for k in range(3):
            np.add.at(vn, ff[:, k], fn)
        e = np.concatenate([ff[:, [0, 1]], ff[:, [1, 2]], ff[:, [2, 0]]])
        en = np.concatenate([fn, fn, fn])
        uk, inv = np.unique(np.sort(e, axis=1), axis=0, return_inverse=True)
        edge_n = np.zeros((uk.shape[0], 3))
        np.add.at(edge_n, inv.ravel(), en)
        targets = np.concatenate([vv, 0.5 * (vv[uk[:, 0]] + vv[uk[:, 1]])])
        normals = np.concatenate([vn, edge_n])
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        for tilt in (0.0, 0.02, 0.2):
            dirs = normals + tilt * rng.normal(size=normals.shape)
            dirs /= np.linalg.norm(dirs, axis=1)[:, None]
            o = targets - 1e-3 * dirs
            inside = cpu.brute_force_closest(o, -dirs, t_max=10.0).tri >= 0
            h = gpu.closest(o, dirs, t_max=0.01)
            leaks += int((inside & (h.tri < 0)).sum())
            probes += int(inside.sum())
    check("[G1] no leak through the GPU oracle (every vertex and edge midpoint of "
          "closed solids)", leaks == 0, f"{leaks} leaks of {probes} probes")
    model = shared_stage_model(SonarModelConfig(), 126.6)
    acq = AcquisitionParams()
    worst_mu = worst_e = 0.0
    det = []
    for label, kind, extra in (("flat sand 3 m", "flat", {}),
                               ("quay + sill, 0.9 m calm water", "pool",
                                {"paths": {"surface_rms_m": 3e-4},
                                 "sampling": {"scatterer_spacing_m": 0.01}})):
        if kind == "flat":
            sc = flat_scene(-3.0)
        else:
            sc = flat_scene(-0.9)
            sc.walls = [Wall("quay", -10.0, 8.0, 10.0, 8.0, top_z=0.5),
                        Wall("sill", -0.6, 4.0, 0.6, 4.0, top_z=-0.35, thickness=0.3)]
        out = {}
        for be in ("cpu", "warp"):
            rr = RaycastRenderer(sc, acq, model, RaycastConfig.from_dict({"backend": be, **extra}))
            t0 = time.time()
            pg = rr.render(Side.PORT, Pose3D(0, 0, 0), 0.0).ping
            out[be] = (pg.power, pg.power * pg.texture, time.time() - t0)
        Pc, Ic, tc = out["cpu"]
        Pg, Ig, tg = out["warp"]
        sel = Pc > Pc.max() * 1e-4
        mu = float(np.max(np.abs(Pg[sel] - Pc[sel]) / Pc[sel]))
        ef = float(np.max(np.abs(Ig[sel] - Ic[sel]) / Pc[sel]))
        worst_mu, worst_e = max(worst_mu, mu), max(worst_e, ef)
        det.append(f"{label}: eps_mu {mu:.1e}, intensity {ef:.1e}; CPU {tc:.0f} s, GPU {tg:.1f} s")
    check("[G1] renders: GPU oracle = CPU reference (eps_mu <= 1e-4, same-seed "
          "intensity <= 1e-3 of P)", worst_mu <= 1e-4 and worst_e <= 1e-3, "; ".join(det))


# ------------------------------------------------- [F] fast-variant engine paths
def _fast_scene():
    """A shallow harbour corner: sand at -1.5 m, dry land (+0.5 m) behind a
    quay at y = 8 (top 0.5 m, its box buried to the scene floor), a floating
    pontoon at y = 5 (water on both sides) and a submerged sill at y = 3."""
    from blueboat_sss_sim.core.types import Wall
    sc = flat_scene(-1.5, half=20.0)
    ys = sc.grid.origin_y + sc.grid.resolution * np.arange(sc.grid.ny)
    sc.height[ys > 8.6, :] = 0.5
    sc.walls = [Wall("quay", -10.0, 8.15, 10.0, 8.15, top_z=0.5),
                Wall("pontoon", -3.0, 5.0, 3.0, 5.0, top_z=0.3, thickness=2.0,
                     kind="pontoon", draft_m=0.4),
                Wall("sill", -1.0, 3.0, 1.0, 3.0, top_z=-1.1, thickness=0.3)]
    return sc


def _frame(rr, side, pose):
    from blueboat_sss_sim.sonar.raycast import beam as _beam
    s = rr.sensor_pose(side, pose)
    e_a, e_n, e_v = _beam.transducer_frame(s.yaw, s.pitch, s.roll, side.sign, rr.tilt)
    return np.array([s.x, s.y, s.z]), e_a, e_n, e_v


def _sorted_rows(sc_) -> np.ndarray:
    a = np.column_stack([sc_.pos, sc_.area, sc_.tri.astype(np.float64)])
    return a[np.lexsort(a.T[::-1])]


def fast_variant_checks() -> None:
    """[F] the fast variant's engine paths (raycast_fast_optimize_plan.md):
    every knob off reproduces today; on, each does what it says. GPU checks
    skip visibly without Warp."""
    try:
        from blueboat_sss_sim.sonar.raycast import gpu_render as gr
    except Exception as exc:                        # noqa: BLE001
        skip("[F] fast-variant engine paths", f"no Warp in this interpreter ({type(exc).__name__})")
        return
    from blueboat_sss_sim.core.types import Pose3D, Side
    from blueboat_sss_sim.sonar.config import AcquisitionParams
    from blueboat_sss_sim.sonar.raycast.config import RaycastConfig, shared_stage_model
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    from blueboat_sss_sim.sonar.raycast.scene_mesh import KIND_WALL
    print("\n[F] fast-variant engine paths")
    model = shared_stage_model(SonarModelConfig(), 126.6)
    acq = AcquisitionParams()
    sc = _fast_scene()
    pose = Pose3D(0.0, 0.0, 0.0)

    # F1 -- cull_unreachable: GPU set = CPU set; nothing unreachable kept
    sets = {}
    for cull in (False, True):
        rc = RaycastConfig.from_dict({"backend": "warp", "sampling": {"cull_unreachable": cull}})
        rr = RaycastRenderer(sc, acq, model, rc)
        P, e_a, e_n, _ = _frame(rr, Side.PORT, pose)
        g = gr.GpuScene(rr).scatterers(P, e_a, e_n).numpy()
        c = rr._scatterers(P, e_a, e_n)                          # noqa: SLF001
        sets[cull] = (rr, P, g, c)
    rr, P, g, c = sets[True]
    m = rr.mesh
    wall = g.kind == KIND_WALL
    wi = m.wall_index[np.maximum(g.tri, 0)]
    back = np.einsum("ij,ij->i", P - g.pos, g.normal) <= 0.0
    zs = rr.sheet._locate(g.pos[:, 0], g.pos[:, 1])[0]          # noqa: SLF001
    off = rr._rc.sampling.surface_offset_m                       # noqa: SLF001
    names = [w.name for w in sc.walls]
    ok_g, ok_c = _sorted_rows(g), _sorted_rows(c)
    same = ok_g.shape == ok_c.shape and (ok_g.size == 0 or float(np.max(np.abs(ok_g - ok_c))) <= 1e-9)
    _, _, g0, _ = sets[False]
    wall0 = g0.kind == KIND_WALL
    wi0 = m.wall_index[np.maximum(g0.tri, 0)]
    back0 = np.einsum("ij,ij->i", P - g0.pos, g0.normal) <= 0.0
    check("[F1] cull_unreachable: no point above the waterline, no wall point buried, no "
          "quay face turned away; the pontoon keeps its far face; GPU set = CPU set; "
          "off keeps all of them",
          same and len(g) > 0 and not (g.pos[:, 2] > 0).any()
          and not (wall & (g.pos[:, 2] < zs - off)).any()
          and not (wall & back & (wi == names.index("quay"))).any()
          and (wall & back & (wi == names.index("pontoon"))).any()
          and (g0.pos[:, 2] > 0).any() and (wall0 & back0 & (wi0 == names.index("quay"))).any(),
          f"{len(g0)} -> {len(g)} scatterers ({100 * (1 - len(g) / max(len(g0), 1)):.0f} % culled), "
          f"GPU = CPU {same}")

    # F2 -- reselect_every: reuse within the limits, reselect outside them
    rc = RaycastConfig.from_dict({"backend": "warp", "paths": {
        "max_order": 1, "reselect_every": 10, "reselect_move_m": 0.3}})
    rr = RaycastRenderer(sc, acq, model, rc)

    def ping(x, yaw_deg=0.0):
        pg = rr.render(Side.PORT, Pose3D(x, 0.0, 0.0, 0.0, 0.0, np.radians(yaw_deg)), 0.0).ping
        return bool(rr.multipath.last_report["diffuse"].get("reused", False)), pg.power

    seq = [ping(0.01 * k)[0] for k in range(11)]
    _, p_fresh = ping(1.0)
    r_same, p_same = ping(1.0)
    moved = ping(1.31)[0]
    ping(1.0)
    turned = ping(1.0, 2.5)[0]
    # the pontoon's west end face (x = -3) is in front of the transducer, then
    # no longer, after a 0.1 m move: the boundary set changed
    x_face = -3.0 - rr.sensor_pose(Side.PORT, Pose3D(0.0, 0.0, 0.0)).x
    ping(x_face - 0.05)
    crossed = ping(x_face + 0.05)[0]
    rel = float(np.max(np.abs(p_same - p_fresh)) / max(float(np.max(np.abs(p_fresh))), 1e-300))
    rr1 = RaycastRenderer(sc, acq, model, RaycastConfig.from_dict(
        {"backend": "warp", "paths": {"max_order": 1}}))
    for k in range(3):
        rr1.render(Side.PORT, Pose3D(0.01 * k, 0.0, 0.0), 0.0)
    none1 = ("reused" not in rr1.multipath.last_report["diffuse"]
             and not rr1._gpu_mp._sel_cache)                       # noqa: SLF001
    check("[F2] reselect_every 10: pings 2-10 reuse, ping 11 reselects; a 0.31 m move, a "
          "2.5 deg turn and a boundary leaving range reselect; a reused ping = the fresh one; "
          "reselect_every 1 never reuses or stores",
          seq == [False] + [True] * 9 + [False] and r_same and not moved and not turned
          and not crossed and rel <= 1e-9 and none1,
          f"sequence {''.join('R' if q else 'S' for q in seq)}, reused vs fresh {rel:.1e}")

    # F3 -- diffraction.method edge: the closed form on the D1 knife edge,
    # GPU = CPU, continuity, and no factor without a wall box
    from blueboat_sss_sim.core.types import Wall
    from blueboat_sss_sim.sonar.raycast.diffraction import edge_window, fresnel_F
    from blueboat_sss_sim.sonar.raycast.gpu_render import GpuMultipath
    edge_cfg = {"paths": {"max_order": 1}, "diffraction": {"method": "edge"}}
    sd = flat_scene(-20.0)
    sd.walls = [Wall("screen", -25.0, 10.0, 25.0, 10.0, top_z=-2.0, thickness=0.002)]
    rd = RaycastRenderer(sd, acq, model, RaycastConfig.from_dict(edge_cfg))
    dif = rd.multipath.diffraction
    lam = dif.lam
    vs = np.linspace(-2.0, 3.0, 21)
    d1 = d2 = 10.0 - 0.001
    h = vs / np.sqrt(2 * (d1 + d2) / (lam * d1 * d2))
    U = np.ones(vs.size, complex)
    for k, hk in enumerate(h):
        A = np.array([0.0, 0.0, -2.0 - hk])
        B = np.array([[0.0, 20.0, -2.0 - hk]])
        idx, Dk = dif.leg_factors(A, B, np.array([hk > 0]), np.array([hk <= 0]))
        U[k] = Dk[0] if idx.size else np.nan
    ref = fresnel_F(vs)
    e_db = np.abs(20 * np.log10(np.abs(U) / np.abs(ref)))
    e_ph = np.abs(np.angle(U / ref))
    check("[F3] edge method on the D1 knife edge: |F| <= 0.5 dB, phase <= 0.1 rad, v in [-2, 3]",
          bool(np.all(np.isfinite(U))) and e_db.max() <= 0.5 and e_ph.max() <= 0.1,
          f"max {e_db.max():.3f} dB, {e_ph.max():.4f} rad")
    rg = RaycastRenderer(sc, acq, model, RaycastConfig.from_dict({"backend": "warp", **edge_cfg}))
    P, e_a, e_n, _ = _frame(rg, Side.PORT, pose)
    cs = rg._scatterers(P, e_a, e_n)                              # noqa: SLF001
    rng_f = np.random.default_rng(3)
    pick = rng_f.choice(len(cs), size=min(len(cs), 200_000), replace=False)
    off = rg._rc.sampling.surface_offset_m                        # noqa: SLF001
    Xv = cs.pos[pick] + off * cs.normal[pick]
    blk = rg.oracle.occluded(np.broadcast_to(P, Xv.shape), Xv, 1e-6)
    idx_c, D_c = rg.multipath.diffraction.leg_factors(P, Xv, blk, ~blk)
    Dc_full = np.where(blk, 0.0 + 0j, 1.0 + 0j)
    Dc_full[idx_c] = D_c
    import warp as wp
    gm = GpuMultipath(rg.multipath)
    dev = gm.dev

    class _Legs:
        pos = wp.array(cs.pos[pick], dtype=wp.vec3d, device=dev)
        normal = wp.array(cs.normal[pick], dtype=wp.vec3d, device=dev)

        def __len__(self):
            return int(pick.size)
    st = np.where(blk, 1, 3).astype(np.int32)
    Dg = wp.array(np.column_stack([np.where(blk, 0.0, 1.0), np.zeros(pick.size)]), dtype=wp.vec2d,
                  device=dev)
    Tt = wp.array(np.tile([1.0, 0.0], (pick.size, 1)), dtype=wp.vec2d, device=dev)
    gm.last_timing, gm._t = {}, time.perf_counter()               # noqa: SLF001
    gm._edge_diffraction(P, _Legs(), wp.array(st, dtype=wp.int32, device=dev), Dg, Tt)  # noqa: SLF001
    dg = Dg.numpy()
    Dg_full = dg[:, 0] + 1j * dg[:, 1]
    err = float(np.max(np.abs(Dg_full - Dc_full)))
    check("[F3] edge method: GPU leg factors = CPU reference (<= 1e-6) on 2e5 legs of the harbour "
          "corner", err <= 1e-6 and idx_c.size > 0,
          f"{idx_c.size} legs diffracted ({int(blk[idx_c].sum())} shadowed), max |dD| {err:.1e}")
    lone = dif.leg_factors(np.array([0.0, -10.0, -10.0]), np.array([[0.0, -1.0, -10.0]]),
                           np.array([True]), np.array([False]))[0]
    check("[F3] edge method: continuous across the shadow boundary (F(0) = 1/2) and at n2 "
          "(window 0); a shadow cast by no wall box gets no factor",
          abs(fresnel_F(0.0) - 0.5) < 1e-12 and edge_window(dif.n2, dif.n1, dif.n2) == 0.0
          and edge_window(dif.n1, dif.n1, dif.n2) == 1.0 and lone.size == 0)

    # F4 -- precision fp32: the visibility test agrees with fp64, no leak into
    # a closed box, and a render keeps its level and its speckle
    o = gm.oracle
    rng4 = np.random.default_rng(44)
    lo4, hi4 = np.array([-12.0, -4.0, -1.6]), np.array([12.0, 12.0, 0.4])
    a4 = rng4.uniform(lo4, hi4, size=(100_000, 3))
    b4 = rng4.uniform(lo4, hi4, size=(100_000, 3))
    pw = next(w for w in sc.walls if w.kind == "pontoon")
    ins = np.column_stack([rng4.uniform(pw.x0 + 0.01, pw.x1 - 0.01, 20_000),
                           rng4.uniform(pw.y0 - 0.5 * pw.thickness + 0.01,
                                        pw.y0 + 0.5 * pw.thickness - 0.01, 20_000),
                           rng4.uniform(-pw.draft_m + 0.01, pw.top_z - 0.01, 20_000)])
    outp = np.column_stack([rng4.uniform(-12, 12, 20_000), rng4.uniform(-4.0, 3.5, 20_000),
                            rng4.uniform(-1.4, -0.05, 20_000)])

    def blocked(A, B, use32):
        o.use32 = int(use32)
        out = wp.empty(len(A), dtype=wp.int32, device=dev)
        wp.launch(gr._seg_blocked_probe_k, len(A), [o, wp.array(A, dtype=wp.vec3d, device=dev),
                                                    wp.array(B, dtype=wp.vec3d, device=dev),
                                                    wp.float64(1e-6), out], device=dev)
        return out.numpy()
    dis = float(np.mean(blocked(a4, b4, 0) != blocked(a4, b4, 1)))
    leaks = int((blocked(outp, ins, 1) == 0).sum())
    o.use32 = 0
    lv = {}
    for prec in ("fp64", "fp32"):
        rp_ = RaycastRenderer(sc, acq, model, RaycastConfig.from_dict(
            {"backend": "warp", "precision": prec, "paths": {"max_order": 1},
             "sampling": {"cull_unreachable": True}, "diffraction": {"method": "edge"}}))
        pg = rp_.render(Side.PORT, pose, 0.0).ping
        lv[prec] = (pg.power, pg.power * pg.texture)
    lit = (lv["fp64"][0] > 0) & (lv["fp32"][0] > 0)
    dL = np.abs(10 * np.log10(lv["fp32"][0][lit] / lv["fp64"][0][lit]))
    ok_r = (lv["fp64"][1] > 0) & (lv["fp32"][1] > 0)
    dR = np.abs(10 * np.log10(lv["fp32"][1][ok_r] / lv["fp64"][1][ok_r]))
    check("[F4] precision fp32: visibility = fp64 on 1e5 random segments (<= 1e-4 disagree), "
          "0 leaks into a closed box (2e4 probes); a render's mean level |dL| p99 <= 0.05 dB "
          "and realised |dR| median <= 0.05 dB",
          dis <= 1e-4 and leaks == 0 and float(np.percentile(dL, 99)) <= 0.05
          and float(np.median(dR)) <= 0.05,
          f"disagree {dis:.1e}, leaks {leaks}, |dL| p99 {np.percentile(dL, 99):.4f} dB, "
          f"|dR| median {np.median(dR):.4f} dB")

    def rel(a, b):
        return float(np.max(np.abs(a - b)) / max(float(np.max(np.abs(b))), 1e-300))

    # F5 -- band_cull (phase 2): on a wall-free scene no multipath sequence is
    # kept, so dropping the out-of-band sheet points is exact; fewer points;
    # the GPU emits the CPU reference's set
    sf = flat_scene(-6.0, half=25.0)
    out5 = {}
    for band in (False, True):
        rc5 = RaycastConfig.from_dict({"backend": "warp", "paths": {"max_order": 1},
                                       "sampling": {"band_cull": band}})
        r5 = RaycastRenderer(sf, acq, model, rc5)
        P5, e_a5, e_n5, _ = _frame(r5, Side.PORT, pose)
        g5 = gr.GpuScene(r5).scatterers(P5, e_a5, e_n5).numpy()
        c5 = r5._scatterers(P5, e_a5, e_n5)                       # noqa: SLF001
        pg = r5.render(Side.PORT, pose, 0.0).ping
        kept5 = r5.multipath.last_report["diffuse"].get("kept", 0)
        out5[band] = (len(g5), _sorted_rows(g5), _sorted_rows(c5), pg.power,
                      pg.power * pg.texture, kept5)
    n0, _, _, p0, t0, _ = out5[False]
    n1, g1s, c1s, p1, t1, k1 = out5[True]
    same5 = g1s.shape == c1s.shape and float(np.max(np.abs(g1s - c1s))) <= 1e-9
    check("[F5] band_cull: fewer sheet points; with no kept multipath sequence the ping is "
          "unchanged (<= 1e-9); GPU set = CPU set",
          n1 < n0 and k1 == 0 and rel(p1, p0) <= 1e-9 and rel(t1, t0) <= 1e-9 and same5,
          f"{n0} -> {n1} points ({100 * (1 - n1 / max(n0, 1)):.0f} % culled), "
          f"power {rel(p1, p0):.1e}, realised {rel(t1, t0):.1e}, GPU = CPU {same5}")

    # F6 -- the floor channel's image pairs reused with the selection
    # (reselect_every): a reused ping renders as the fresh one
    r6 = RaycastRenderer(sc, acq, model, RaycastConfig.from_dict(
        {"backend": "warp", "paths": {"max_order": 1, "reselect_every": 10,
                                      "reselect_move_m": 0.3}}))
    pf = r6.render(Side.PORT, pose, 0.0).ping
    fresh6 = not r6.multipath.last_report["floor"]["reused"]
    pr_ = r6.render(Side.PORT, pose, 0.0).ping
    reused6 = r6.multipath.last_report["floor"]["reused"]
    r6b = RaycastRenderer(sc, acq, model, RaycastConfig.from_dict(
        {"backend": "warp", "paths": {"max_order": 1}}))
    r6b.render(Side.PORT, pose, 0.0)
    r6b.render(Side.PORT, pose, 0.0)
    never6 = not r6b.multipath.last_report["floor"]["reused"]
    check("[F6] floor image pairs: the second ping reuses the first's kept pairs and renders "
          "the same (<= 1e-9); reselect_every 1 never reuses",
          fresh6 and reused6 and never6 and rel(pr_.power, pf.power) <= 1e-9
          and rel(pr_.power * pr_.texture, pf.power * pf.texture) <= 1e-9,
          f"power {rel(pr_.power, pf.power):.1e}")

    # F7 -- stage timing off (the live node): no value changes
    out7 = {}
    for timing in (True, False):
        r7 = RaycastRenderer(sc, acq, model, RaycastConfig.from_dict(
            {"backend": "warp", "paths": {"max_order": 1}}))
        r7.stage_timing = timing
        pg = r7.render(Side.PORT, pose, 0.0).ping
        out7[timing] = (pg.power, pg.power * pg.texture, dict(r7.last_timing))
    check("[F7] stage_timing off: the same ping (<= 1e-9), and no per-stage GPU timing",
          rel(out7[False][0], out7[True][0]) <= 1e-9 and rel(out7[False][1], out7[True][1]) <= 1e-9
          and "scatterers" in out7[True][2] and "scatterers" not in out7[False][2],
          f"power {rel(out7[False][0], out7[True][0]):.1e}")



# ------------------------------------------------------------- [L] labels
def _label_scene(objects, walls=(), depth: float = -7.0, x_half: float = 5.5,
                 y0: float = -2.0, y1: float = 22.0, res: float = 0.05) -> SceneModel:
    """A flat sand strip with pinned objects (the builder's own stamp)."""
    nx = int(round(2 * x_half / res)) + 1
    ny = int(round((y1 - y0) / res)) + 1
    g = GridSpec(origin_x=-x_half, origin_y=y0, resolution=res, nx=nx, ny=ny)
    h = np.full((ny, nx), depth)
    refl = np.full(h.shape, 0.3)
    objs = [sm_copy(o) for o in objects]
    ras = stamp_objects(objs, g, h, refl)
    return SceneModel(grid=g, height=h, reflectivity=refl, **ras.scene_kwargs(),
                      material_id=np.zeros(h.shape, dtype=np.uint8),
                      material_names=["sand"], objects=objs, walls=list(walls), seed=3,
                      config_snapshot={}, outside="edge")


def _box(oid: int, x: float, y: float, height: float, length: float = 0.5,
         width: float = 0.4, typ: str = "crate_plastic", material: str = "plastic"):
    from blueboat_sss_sim.core.types import PlacedObject
    return PlacedObject(object_id=oid, type=typ, x=x, y=y, yaw=0.0, length=length,
                        width=width, proud_height=height, burial=0.0, reflectivity=0.4,
                        material=material)


def label_checks() -> None:
    """[L] ground-truth contacts of the ray-cast sensor (``labels.py``): read
    off its own mesh and oracle, the schema of the geometric renderer's, the
    shadow against flat-floor geometry, occlusion, and -- with Warp and the
    BlueBoat-Perception checkout -- the labeler's box against the render."""
    import dataclasses
    from blueboat_sss_sim.core.types import GroundTruthContact, Pose3D, Side, Wall
    from blueboat_sss_sim.sonar.config import SPEED_OF_SOUND_MPS, AcquisitionParams
    from blueboat_sss_sim.sonar.raycast.config import RaycastConfig, shared_stage_model
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    print("\n[L] ground-truth contacts (labels.py)")
    model = shared_stage_model(SonarModelConfig(), 126.6)
    acq = AcquisitionParams()
    bin_m = float(acq.bin_size_m)
    rc0 = RaycastConfig.from_dict({"paths": {"max_order": 1}, "labels": {"enabled": True}})
    depth = -7.0
    cases = [(6.0, 0.15), (10.0, 0.15), (15.0, 0.15), (6.0, 0.40), (10.0, 0.40), (15.0, 0.40)]
    xs = [-3.75 + 1.5 * i for i in range(len(cases))]
    mount_x, mount_y = float(model.mount_x_m), float(model.mount_y_abs_m)
    # each box centred `a` metres across from the PORT transducer of a boat at (x - mount_x, 0)
    objs = [_box(i, x, mount_y + a, hgt) for i, (x, (a, hgt)) in enumerate(zip(xs, cases))]
    sc = _label_scene(objs)
    rr = RaycastRenderer(sc, acq, model, rc0)
    scale = float(SPEED_OF_SOUND_MPS) / float(rr.c)

    def contacts(r_, x_boat, side=Side.PORT):
        P, e_a, e_n, _ = _frame(r_, side, Pose3D(x_boat, 0.0, 0.0, 0.0, 0.0, 0.0))
        from blueboat_sss_sim.sonar.raycast.labels import LabelOracle
        if getattr(r_, "_labels", None) is None:
            r_._labels = LabelOracle(r_, int(r_._rc.labels.max_points))   # noqa: SLF001
        return P, r_._labels.contacts(side, P, e_a, e_n, bin_m)           # noqa: SLF001

    # [L1] schema: the dataclass the node serialises, every field typed as the geometric's
    P, cs = contacts(rr, xs[1] - mount_x)
    mine = [c for c in cs if c.object_id == 1]
    ok = (len(mine) == 1 and isinstance(mine[0], GroundTruthContact)
          and isinstance(mine[0].object_id, int) and mine[0].object_type == "crate_plastic"
          and mine[0].side is Side.PORT and isinstance(mine[0].visible, bool)
          and mine[0].ghost is False and mine[0].via == ""
          and all(isinstance(getattr(mine[0], f), float)
                  for f in ("slant_range_m", "extent_bins", "shadow_bins")))
    check("[L1] a contact is the shared GroundTruthContact, typed as the node serialises it", ok,
          f"{mine[0] if mine else cs}")

    # [L2] shadow length against flat-floor geometry, in the device's bins (tolerance: 2 bins)
    worst, lines = 0.0, []
    for i, (x, (a, hgt)) in enumerate(zip(xs, cases)):
        P, cs = contacts(rr, x - mount_x)
        c = next((q for q in cs if q.object_id == i), None)
        dz = float(P[2] - depth)
        a_far = a + 0.5 * 0.4
        expect = (np.hypot(a_far * dz / (dz - hgt), dz) - np.hypot(a_far, dz)) * scale / bin_m
        got = float("nan") if c is None else c.shadow_bins
        worst = max(worst, abs(got - expect) if c is not None and c.visible else np.inf)
        lines.append(f"{a:g} m/H {hgt:g}: {got:.1f} vs {expect:.1f}")
    check("[L2] shadow_bins = flat-floor geometry within 2 bins (6 / 10 / 15 m, H 0.15 / 0.40 m)",
          worst <= 2.0, "; ".join(lines) + f"; worst {worst:.2f} bins")

    # [L2b] slant_range_m keeps the schema's meaning: true metres to the centre on the seabed
    P, cs = contacts(rr, xs[4] - mount_x)
    c = next(q for q in cs if q.object_id == 4)
    want = float(np.hypot(cases[4][0], P[2] - depth))
    check("[L2b] slant_range_m is the true range to the object's centre on the seabed",
          abs(c.slant_range_m - want) < 1e-6, f"{c.slant_range_m:.4f} vs {want:.4f} m")

    # [L4] occlusion: behind a quay, and inside a taller object's shadow -> visible False
    quay = Wall("quay", -3.0, 8.0, 3.0, 8.0, top_z=0.5)
    sc4 = _label_scene([_box(0, -1.5, mount_y + 5.0, 0.40),          # in front of the quay
                        _box(1, -1.5, mount_y + 11.0, 0.40),         # behind it
                        _box(2, 1.5, mount_y + 5.0, 1.00, width=0.6),  # a tall box ...
                        # ... and one wholly inside its shadow: the tall box's far top
                        # edge (5.3 m, 1.0 m high) shades the floor to 6.22 m; at this
                        # box's far edge (5.9 m) the shadow ray is still 0.35 m up
                        _box(3, 1.5, mount_y + 5.7, 0.15)],
                       walls=[quay])
    # the quay spans x in [-3, 0] only, so the second pair is not behind it
    sc4.walls = [Wall("quay", -3.0, 8.0, 0.0, 8.0, top_z=0.5)]
    rr4 = RaycastRenderer(sc4, acq, model, rc0)
    _, ca = contacts(rr4, -1.5 - mount_x)
    _, cb = contacts(rr4, 1.5 - mount_x)
    va = {c.object_id: c for c in ca}
    vb = {c.object_id: c for c in cb}
    ok = (va.get(0) is not None and va[0].visible and va[0].shadow_bins > 0
          and va.get(1) is not None and not va[1].visible and va[1].shadow_bins == 0.0
          and vb.get(2) is not None and vb[2].visible
          and vb.get(3) is not None and not vb[3].visible)
    check("[L4] an object behind a quay or inside another object's shadow is reported "
          "visible: false, shadow 0", ok,
          "; ".join(f"{k}:{'vis' if c.visible else 'hidden'}/{c.shadow_bins:.0f}"
                    for k, c in sorted({**va, **vb}.items())))

    # [L5] no object in the strip -> no contact; labels off -> the renderer reports none
    _, c_none = contacts(rr, 30.0)
    rr_off = RaycastRenderer(sc, acq, model, dataclasses.replace(rc0, labels=type(rc0.labels)()),
                             mesh=rr.mesh, oracle=rr.oracle)
    check("[L5] a ping that sees no object reports no contact; labels off by default",
          c_none == [] and rr_off.labels_on is False and RaycastConfig().labels.enabled is False)

    # [L3] the labeler's box against the RENDER (GPU + the Perception checkout)
    try:
        import warp  # noqa: F401
    except Exception as exc:                        # noqa: BLE001
        skip("[L3] the label box covers the rendered highlight and shadow",
             f"no Warp in this interpreter ({type(exc).__name__})")
        return
    perc = ROOT.parents[1] / "BlueBoat-Perception" / "blueboat_perception"
    if not (perc / "blueboat_perception" / "core" / "sim_labels.py").is_file():
        skip("[L3] the label box covers the rendered highlight and shadow",
             f"no BlueBoat-Perception checkout at {perc}")
        return
    sys.path.insert(0, str(perc))
    from blueboat_perception.core import sim_labels as SL       # noqa: E402
    rcg = dataclasses.replace(rc0, backend="warp")
    rr_g = RaycastRenderer(sc, acq, model, rcg)
    rr_e = RaycastRenderer(_label_scene([]), acq, model, rcg)    # the same floor, no object
    t0 = time.perf_counter()
    n_ping = 0
    lines, ok_all, shift_seen = [], True, True
    for i in (1, 2, 4, 5):                                       # 10 and 15 m, both heights
        x, (a, hgt) = xs[i], cases[i]
        pose = Pose3D(x - mount_x, 0.0, 0.0, 0.0, 0.0, 0.0)
        r1 = rr_g.render(Side.PORT, pose, 0.0)
        r0 = rr_e.render(Side.PORT, pose, 0.0)
        n_ping += 1
        p1, p0 = np.asarray(r1.ping.power), np.asarray(r0.ping.power)
        c = next((q for q in r1.contacts if q.object_id == i), None)
        if c is None:
            ok_all = False
            lines.append(f"{a:g}/{hgt:g}: no contact")
            continue
        sp = r1.ping.pose
        o = SL.WorldObject(object_id=i, type="crate_plastic", x=x, y=mount_y + a, yaw=0.0,
                           length=0.5, width=0.4, height=hgt)
        kw = dict(bin_m=bin_m, range_max_m=float(acq.range_max_m), n_bins=int(acq.num_results))
        cd = {"shadow_bins": c.shadow_bins, "slant_range_m": c.slant_range_m}
        pose6 = [sp.x, sp.y, sp.z, sp.roll, sp.pitch, sp.yaw]
        g = SL.ping_object_bins(pose6, 1, o, cd, depth, range_scale=scale, **kw)
        g1 = SL.ping_object_bins(pose6, 1, o, cd, depth, range_scale=1.0, **kw)
        k = np.arange(p1.size)
        lit0 = p0 > 0
        near = (k >= g["k"][0] - 60) & (k <= g["k"][2] + 60) & lit0
        hot = near & (p1 > 2.0 * p0)
        dark = near & (p1 < 0.5 * p0)
        feat = hot | dark
        inside = (k >= g["k"][0]) & (k <= g["k"][2])
        inside1 = (k >= g1["k"][0]) & (k <= g1["k"][2])
        cov = float((feat & inside).sum()) / max(int(feat.sum()), 1)
        cov1 = float((feat & inside1).sum()) / max(int(feat.sum()), 1)
        over = int(g["k"][2]) - int(np.flatnonzero(feat).max()) if feat.any() else 0
        ok_all &= bool(feat.any() and cov >= 0.98 and over <= SL.DEFAULT_RANGE_MARGIN_BINS + 3)
        shift_seen &= cov1 < 0.98
        lines.append(f"{a:g} m/H {hgt:g}: {int(feat.sum())} feature bins, covered {100 * cov:.1f} % "
                     f"(without the scale {100 * cov1:.1f} %), far end +{over} bins")
    check("[L3] the labeler's box (with range_scale) holds >= 98 % of the rendered highlight "
          "and shadow bins and overshoots by <= margin + 3 bins", ok_all, "; ".join(lines))
    check("[L3b] the same box WITHOUT range_scale misses them (the 1500 m/s bin mapping)",
          shift_seen)
    # labels never change the ping
    rr_n = RaycastRenderer(sc, acq, model, dataclasses.replace(rcg, labels=type(rcg.labels)()),
                           mesh=rr_g.mesh, oracle=rr_g.oracle)
    pose = Pose3D(xs[4] - mount_x, 0.0, 0.0, 0.0, 0.0, 0.0)
    a_ = rr_g.render(Side.PORT, pose, 0.0)
    b_ = rr_n.render(Side.PORT, pose, 0.0)
    rel = float(np.max(np.abs(np.asarray(a_.ping.power) - np.asarray(b_.ping.power)))
                / np.max(np.abs(np.asarray(b_.ping.power))))
    check("[L6] labels on or off: the same ping (<= 1e-9 relative), and off reports no contact",
          rel <= 1e-9 and b_.contacts == [] and len(a_.contacts) >= 1, f"rel {rel:g}")
    t1 = time.perf_counter()
    for _ in range(20):
        contacts(rr_g, xs[4] - mount_x)
    per = (time.perf_counter() - t1) / 20 * 1e3
    t2 = time.perf_counter()
    for _ in range(20):
        contacts(rr_g, 30.0)
    per0 = (time.perf_counter() - t2) / 20 * 1e3
    check("[L7] cost per ping-side: <= 3 ms with an object in the strip, <= 0.2 ms with none",
          per <= 3.0 and per0 <= 0.2, f"{per:.2f} ms / {per0:.3f} ms")
    del t0, n_ping



# ------------------------------------------------- [O] objects.mode solid
def object_checks() -> None:
    """[O] ``raycast.objects.mode: solid`` (``object_mesh.py``): every object
    its own mesh on the terrain sheet. Geometry on the CPU; renders with Warp
    (skipped visibly without it)."""
    import dataclasses
    from blueboat_sss_sim.core.types import PlacedObject, Pose3D, Side
    from blueboat_sss_sim.sonar.config import AcquisitionParams
    from blueboat_sss_sim.sonar.raycast import object_mesh as om
    from blueboat_sss_sim.sonar.raycast.config import RaycastConfig, shared_stage_model
    from blueboat_sss_sim.sonar.raycast.renderer import RaycastRenderer
    from blueboat_sss_sim.worldgen.materials import SAND_REFLECTION, acoustic_reflection
    print("\n[O] objects as their own meshes (objects.mode solid)")
    depth = -7.0

    def flat(x, y):
        return np.full(np.shape(x), depth)

    def obj(oid, typ, material, L, W, H, burial=0.0, x=0.0, y=10.0, yaw=0.0, **kw):
        return PlacedObject(object_id=oid, type=typ, x=x, y=y, yaw=yaw, length=L, width=W,
                            proud_height=H, burial=burial, reflectivity=0.5,
                            material=material, **kw)

    lib = MaterialLibrary()

    def surfaces(o):
        return om.object_parts(o, flat, lib, 451127.0)

    # [O1] a class that is one solid: the catalog primitive, placed on the terrain
    lines, ok = [], True
    for o in (obj(0, "tire_bicycle", "rubber", 0.65, 0.65, 0.20),
              obj(1, "concrete_block", "concrete", 0.6, 0.4, 0.30, burial=0.25),
              obj(2, "mooring_pile", "concrete", 0.4, 0.4, 2.0),
              obj(3, "tent_weight", "concrete", 0.3, 0.14, 0.08)):
        (sf,) = surfaces(o)
        v, f = om.part_mesh(sf)
        sol = CATALOG[o.type].solid(o)
        vol = sm.mesh_volume(v, f)
        top = float(v[:, 2].max()) - depth
        ok &= (not sf.coherent and abs(vol / sm.solid_volume(sol) - 1.0) < 5e-3
               and abs(top - o.effective_height) < 2e-3)
        lines.append(f"{o.type}: {f.shape[0]} tris, volume "
                     f"{100 * (vol / sm.solid_volume(sol) - 1):+.2f} %, top {top:.3f} m")
    check("[O1] a rough class that is one solid is its catalog primitive: volume within 0.5 %, "
          "its top at terrain + effective height (burial sinks it, the sheet hides the rest)",
          ok, "; ".join(lines))

    # [O2] a smooth one keeps the facet objects' own placement (M2's transform) and mirrors
    box = obj(4, "crate_plastic", "plastic", 0.5, 0.4, 0.25, yaw=0.4)
    (sf,) = surfaces(box)
    R, t = sm.facet_transform(box, CATALOG[box.type].solid(box), flat)
    pbox = obj(11, "plastic_box", "pp_shell", 0.716, 0.425, 0.323, params={"wall_mm": 2.0})
    check("[O2] a smooth primitive is placed exactly as before (facet_transform) and mirrors; "
          "an instance with construction knobs carries its own material name",
          sf.coherent and np.array_equal(sf.R, R) and np.array_equal(sf.t, t)
          and sf.material == "plastic" and surfaces(pbox)[0].material == "pp_shell#11")

    # [O3] every catalog class is 3-D parts: outward meshes inside its stored footprint,
    # its top at the stored height (a compound may stop short of it, never stand over it)
    from blueboat_sss_sim.worldgen.objects import typical_instance
    lines, ok, n_tri = [], True, 0
    for name, spec in CATALOG.items():
        o = typical_instance(spec)
        o.x, o.y, o.yaw = 0.0, 10.0, 0.0
        sfs = surfaces(o)
        vs = [om.part_mesh(q) for q in sfs]
        n_tri += sum(f.shape[0] for _, f in vs)
        v = np.vstack([q for q, _ in vs])
        top = float(v[:, 2].max()) - depth - o.effective_height
        half = np.abs(v[:, :2] - [o.x, o.y]).max(axis=0) - 0.5 * np.array([o.length, o.width])
        # a rope or chain wanders 1.5 widths about its line, as its mask does
        slack = 0.015 + (1.5 * o.width if name in ("rope", "chain") else 0.0)
        good = (all(sm.mesh_volume(q, f) > 0.0 for q, f in vs)
                and -0.35 * o.proud_height - 0.01 <= top <= 0.012 and float(half.max()) <= slack)
        ok &= good
        if len(sfs) > 1 or not good:
            lines.append(f"{name} {len(sfs)} parts ({'+'.join(sorted({q.material for q in sfs}))})"
                         + ("" if good else f" BAD top {top:+.3f} half {half.max():+.3f}"))
    pal = surfaces(obj(5, "pallet_wood", "wood", 1.1, 1.1, 0.14, fill_fraction=0.6))
    tyre = surfaces(obj(6, "tire_car", "rubber", 0.65, 0.65, 0.20))
    check("[O3] every catalog class is 3-D parts: outward meshes inside the stored footprint, "
          "top at the stored height; a pallet is boards, a car tyre a tread band on its body",
          ok and len(pal) == 7 + 3 + 3 and all(q.part.solid.kind == "box" for q in pal)
          and [q.material for q in tyre] == ["tyre_tread", "rubber"],
          f"{len(CATALOG)} classes, {n_tri} triangles; " + "; ".join(lines))
    check("[O3b] nothing proud of the seabed: no parts",
          surfaces(obj(10, "concrete_block", "concrete", 0.5, 0.3, 0.2, burial=1.0)) == [])

    # [O3c] one surface rule: the diffuse law of a smooth material is the rough law times
    # its diffuse share 1 - C (Eckart), and fouling replaces a fraction by the sand law
    from blueboat_sss_sim.sonar.raycast.config import SeabedLawConfig
    from blueboat_sss_sim.sonar.raycast.scattering import DiffuseLaw
    k450 = 2.0 * np.pi * 451127.0 / 1500.0
    names = ["sand", "plastic", "concrete", "metal#wall", "gravel"]
    mat = np.arange(len(names))
    ci = np.full(len(names), 0.7)

    def sig(**kw):
        return DiffuseLaw(SeabedLawConfig(), names, k450, **kw).sigma(ci, ci, mat)
    s0, s1, sf_ = sig(), sig(split=True), sig(split=True, fouling=0.4)
    scale = {n: (acoustic_reflection(lib.get(n.split("#")[0]), frequency_hz=451127.0)
                 / SAND_REFLECTION) ** 2 for n in names[1:4]}
    share = -np.expm1(-(2.0 * k450 * 1e-4 * 0.7) ** 2)
    check("[O3c] the diffuse law: a rough material and a wall are the sand law x |R/R_sand|^2; "
          "a smooth one x its diffuse share 1 - C; gravel +3 dB; fouling f mixes in the sand "
          "law; without the split or fouling nothing moves",
          np.allclose(s0[1:4] / s0[0], [scale[n] for n in names[1:4]], rtol=1e-12)
          and np.isclose(s1[1] / s0[1], share, rtol=1e-12)
          and np.array_equal(s1[[0, 2, 3, 4]], s0[[0, 2, 3, 4]])
          and np.isclose(s0[4] / s0[0], 10 ** 0.3, rtol=1e-12)
          and np.allclose(sf_[1:4] / s0[0], 0.6 * s1[1:4] / s0[0] + 0.4, rtol=1e-12)
          and sf_[0] == s0[0] and sf_[4] == s0[4],
          f"plastic diffuse share {10 * np.log10(share):+.1f} dB at cos 0.7")

    # the scenes of the render checks: one block 10 m across, in three materials
    model = shared_stage_model(SonarModelConfig(), 126.6)
    acq = AcquisitionParams()
    mount_x, mount_y = float(model.mount_x_m), float(model.mount_y_abs_m)
    a, hgt = 10.0, 0.40

    def scene_with(material):
        o = [] if material is None else [obj(0, "concrete_block", material, 0.5, 0.4, hgt,
                                             x=mount_x, y=mount_y + a)]
        return _label_scene(o, x_half=2.0, y0=-1.0, y1=21.0, res=0.1)

    def solid_mesh_of(sc):
        terrain = np.full(sc.height.shape, depth)
        return sm.build_scene_mesh(sc, None, terrain=terrain, objects_mode="solid")

    sc_c = scene_with("concrete")
    mesh_c = solid_mesh_of(sc_c)
    from blueboat_sss_sim.sonar.raycast.scene_mesh import KIND_ROUGH_OBJECT, KIND_SOLID
    check("[O4] solid mode: the sheet is the terrain (no object bump in it), the block is 12 "
          "triangles of kind SOLID with its material and id; raster mode tags sheet quads",
          int((mesh_c.kind == KIND_SOLID).sum()) == 12
          and not (mesh_c.kind == KIND_ROUGH_OBJECT).any()
          and mesh_c.objects_mode == "solid"
          and set(mesh_c.object_id[mesh_c.kind == KIND_SOLID].tolist()) == {0}
          and mesh_c.material_names[int(mesh_c.material[mesh_c.kind == KIND_SOLID][0])] == "concrete"
          and bool((sm.build_scene_mesh(sc_c, None).kind == KIND_ROUGH_OBJECT).any()))
    try:
        import warp  # noqa: F401
    except Exception as exc:                        # noqa: BLE001
        skip("[O5]-[O8] solid-mode renders", f"no Warp in this interpreter ({type(exc).__name__})")
        return
    rc = RaycastConfig.from_dict({"paths": {"max_order": 1}, "backend": "warp",
                                  "labels": {"enabled": True},
                                  "sampling": {"scatterer_spacing_m": 0.005}})
    pose = Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def render(sc, mesh=None):
        rr = RaycastRenderer(sc, acq, model, rc, mesh=mesh)
        rr.stage_timing = False
        return rr, rr.render(Side.PORT, pose, 0.0)

    # [O5] no object: solid and raster are the same scene
    sc0 = scene_with(None)
    _, r_ras = render(sc0)
    _, r_sol = render(sc0, solid_mesh_of(sc0))
    p_ras, p_sol = np.asarray(r_ras.ping.power), np.asarray(r_sol.ping.power)
    rel = float(np.max(np.abs(p_ras - p_sol)) / np.max(np.abs(p_ras)))
    relt = float(np.max(np.abs(np.asarray(r_ras.ping.texture) - np.asarray(r_sol.ping.texture))))
    check("[O5] an object-free world renders alike in both modes (<= 1e-9 relative): the "
          "seabed the correction was fitted on is the same sheet", rel <= 1e-9 and relt <= 1e-9,
          f"power {rel:g}, texture {relt:g}")
    p0 = p_sol

    # [O6] the block: a highlight, a shadow as long as geometry says, one visible contact
    rr_c, r_c = render(sc_c, mesh_c)
    p_c = np.asarray(r_c.ping.power)
    bin_m = float(acq.bin_size_m)
    scale = 1500.0 / float(rr_c.c)
    P = np.array([r_c.ping.pose.x, r_c.ping.pose.y, r_c.ping.pose.z])
    dz = float(P[2] - depth)
    a_far = a + 0.2
    k_far = int(np.hypot(a_far, dz) * scale / bin_m)
    k_tip = int(np.hypot(a_far * dz / (dz - hgt), dz) * scale / bin_m)
    k_near = int(np.hypot(a - 0.2, dz - hgt) * scale / bin_m)
    hi = float(10 * np.log10(p_c[k_near - 2:k_far + 3].max() / p0[k_near - 2:k_far + 3].max()))
    sh = p_c[k_far + 6:k_tip - 5] / p0[k_far + 6:k_tip - 5]
    lit = np.flatnonzero((p_c[k_far + 6:] / np.maximum(p0[k_far + 6:], 1e-300)) >= 0.5)
    k_end = k_far + 6 + int(lit[0]) if lit.size else -1
    c = next((q for q in r_c.contacts if q.object_id == 0), None)
    expect = (np.hypot(a_far * dz / (dz - hgt), dz) - np.hypot(a_far, dz)) * scale / bin_m
    check("[O6] a concrete block 10 m across in solid mode: a highlight over the bare seabed, "
          "a shadow under -10 dB that ends where geometry says (+-5 bins with the receive "
          "envelope), and one visible contact whose shadow_bins is geometry within 2 bins",
          hi > 3.0 and float(10 * np.log10(sh.max())) < -10.0 and abs(k_end - k_tip) <= 5
          and c is not None and c.visible and abs(c.shadow_bins - expect) <= 2.0,
          f"highlight {hi:+.1f} dB, shadow max {10 * np.log10(sh.max()):.1f} dB, ends at bin "
          f"{k_end} vs {k_tip}, contact shadow {c.shadow_bins if c else None} vs {expect:.1f}")

    # [O7] material ordering: the same ROUGH block in three materials -> its diffuse echo
    # scales by |R|^2 (a smooth material such as wood takes the Kirchhoff path instead:
    # NC #17's rule, checked by [O2]). "generic" has no acoustic constants: the sand law.
    kR = {m: (acoustic_reflection(lib.get(m), frequency_hz=451127.0) / SAND_REFLECTION) ** 2
          for m in ("concrete", "rubber")}
    kR["generic"] = 1.0
    pw = {"concrete": p_c}
    for m in ("rubber", "generic"):
        scm = scene_with(m)
        pw[m] = np.asarray(render(scm, solid_mesh_of(scm))[1].ping.power)
    win = slice(k_near - 2, k_far + 3)
    num = float((pw["concrete"][win] - pw["rubber"][win]).sum())
    den = float((pw["generic"][win] - pw["rubber"][win]).sum())
    got = 10 * np.log10(num / den)
    want = 10 * np.log10((kR["concrete"] - kR["rubber"]) / (kR["generic"] - kR["rubber"]))
    far_same = float(np.max(np.abs(pw["concrete"][k_tip + 40:] - pw["rubber"][k_tip + 40:]))
                     / np.max(pw["concrete"][k_tip + 40:]))
    check("[O7] the same rough block in concrete, sand-equivalent and rubber: its own echo "
          "scales with the material's |R|^2 (0.1 dB), and the seabed beyond its shadow does "
          "not move", abs(got - want) <= 0.1 and far_same <= 1e-9,
          f"(concrete - rubber) / (generic - rubber) = {got:+.2f} dB vs {want:+.2f} dB; "
          f"scales re sand: " + ", ".join(f"{m} {10 * np.log10(v):+.1f} dB" for m, v in kR.items())
          + f"; far seabed rel {far_same:g}")

    # [O8] repeatability, and the cost of an object in the strip
    r_c2 = rr_c.render(Side.PORT, pose, 0.0)
    rel = float(np.max(np.abs(np.asarray(r_c2.ping.power) - p_c)) / np.max(np.abs(p_c)))
    t0 = time.perf_counter()
    for _ in range(10):
        rr_c.render(Side.PORT, pose, 0.0)
    t_obj = (time.perf_counter() - t0) / 10
    far = Pose3D(30.0, 0.0, 0.0, 0.0, 0.0, 0.0)           # the block out of reach
    rr_c.render(Side.PORT, far, 0.0)
    t0 = time.perf_counter()
    for _ in range(10):
        rr_c.render(Side.PORT, far, 0.0)
    t_far = (time.perf_counter() - t0) / 10
    check("[O8] a repeat render is the same ping (<= 1e-9); cost recorded", rel <= 1e-9,
          f"rel {rel:g}; ping-side {1e3 * t_obj:.1f} ms with the block in the strip, "
          f"{1e3 * t_far:.1f} ms with nothing in reach")

    # [O9] a smooth block: mirror patches plus the diffuse share; fully fouled it is the
    # sand-equivalent block, whatever it is made of
    def with_fouling(f):
        return dataclasses.replace(rc, objects=dataclasses.replace(rc.objects, fouling=f))
    sc_p = scene_with("plastic")
    p_p = np.asarray(render(sc_p, solid_mesh_of(sc_p))[1].ping.power)
    rr_f = RaycastRenderer(sc_p, acq, model, with_fouling(1.0), mesh=solid_mesh_of(sc_p))
    p_f = np.asarray(rr_f.render(Side.PORT, pose, 0.0).ping.power)
    rel = float(np.max(np.abs(p_f - pw["generic"])) / np.max(pw["generic"]))
    body = float(10 * np.log10((p_p[win] - p0[win]).clip(1e-300).sum()
                               / (pw["generic"][win] - p0[win]).clip(1e-300).sum()))
    check("[O9] a smooth (plastic) block mirrors AND scatters diffusely; with fouling 1 it is "
          "the sand-equivalent block to 1e-9, whatever its material",
          rel <= 1e-9 and float((p_p[win] - p0[win]).sum()) > 0.0,
          f"fouled vs sand-equivalent rel {rel:g}; clean plastic echo {body:+.1f} dB re the "
          "sand-equivalent block")

    # [O10] a car tyre: the steel-belted tread band over the same tyre in bare rubber
    def tyre_scene(typ):
        o = obj(0, typ, "rubber", 0.65, 0.65, 0.20, x=mount_x, y=mount_y + a)
        return _label_scene([o], x_half=2.0, y0=-1.0, y1=21.0, res=0.1)
    pt = {}
    for typ in ("tire_car", "tire_bicycle"):
        sct = tyre_scene(typ)
        pt[typ] = np.asarray(render(sct, solid_mesh_of(sct))[1].ping.power)
    kt = slice(int(np.hypot(a - 0.33, dz - 0.2) * scale / bin_m) - 2,
               int(np.hypot(a + 0.33, dz) * scale / bin_m) + 3)
    hi_car = float(10 * np.log10(pt["tire_car"][kt].max() / p0[kt].max()))
    hi_rub = float(10 * np.log10(pt["tire_bicycle"][kt].max() / p0[kt].max()))
    check("[O10] a car tyre's tread band (rubber on its steel belts) returns well over the "
          "same tyre in bare rubber, and over the seabed", hi_car > hi_rub + 6.0 and hi_car > 6.0,
          f"peak over the bare seabed: belted {hi_car:+.1f} dB, bare rubber {hi_rub:+.1f} dB")

    # [O11] a wall is built of its material: steel over concrete = their |R|^2
    from blueboat_sss_sim.core.types import Wall
    pw_w = {}
    for m in ("concrete", "metal"):
        scw = _label_scene([], walls=[Wall("quay", -2.0, mount_y + 12.0, 2.0, mount_y + 12.0,
                                           top_z=0.5, material=m)],
                           x_half=2.0, y0=-1.0, y1=21.0, res=0.1)
        pw_w[m] = np.asarray(render(scw, solid_mesh_of(scw))[1].ping.power)
    kw_ = slice(int(11.7 * scale / bin_m), int(np.hypot(11.85, dz) * scale / bin_m))
    got = float(10 * np.log10((pw_w["metal"][kw_] - p0[kw_]).sum()
                              / (pw_w["concrete"][kw_] - p0[kw_]).sum()))
    want = 20 * np.log10(acoustic_reflection(lib.get("metal"), frequency_hz=451127.0)
                         / acoustic_reflection(lib.get("concrete"), frequency_hz=451127.0))
    check("[O11] a steel wall over a concrete one: the face echo scales with |R|^2 (0.1 dB)",
          abs(got - want) <= 0.1, f"{got:+.2f} dB vs {want:+.2f} dB")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", default="",
                    help="F: only the fast-variant section [F]; L: only the labels [L]; "
                         "O: only the solid-object section [O]")
    args = ap.parse_args()
    rng = np.random.default_rng(20261102)
    n_crops, rays = (2, 5000) if args.quick else (5, 20000)
    t_start = time.time()
    only = {"F": fast_variant_checks, "L": label_checks, "O": object_checks}
    if args.only:
        if args.only not in only:
            ap.error(f"--only {args.only!r}: one of {sorted(only)}")
        only[args.only]()
        print(f"\nraycast_test [{args.only}]: {PASSED} passed, {FAILED} failed, {SKIPPED} "
              f"skipped in {time.time() - t_start:.0f} s")
        return 1 if FAILED else 0

    intersection_checks(rng)
    direct_path_unit_checks()
    direct_path_render_checks()
    grouped_mean_checks()
    shared_stage_checks()
    surface_path_checks()
    bottom_path_checks()
    comb_checks()
    wall_checks()
    diffraction_checks()
    corner_checks()
    transmission_checks()
    fit_basis_checks()
    gpu_checks()
    fast_variant_checks()
    label_checks()
    object_checks()
    po_closed_form_checks()
    seabed_law_checks()
    energy_validity_checks()
    correction_layer_checks()
    sc = synthetic_scene()
    mesh_checks(f"synthetic: {len(sc.objects)} objects, every catalog class "
                "flat and rested on a 4 % slope", sc, None,
                sc._test_terrain, rng, n_crops, rays)   # noqa: SLF001

    worlds = world_dirs()[:1] if args.quick else world_dirs()
    for w in worlds:
        before = {p: p.stat().st_mtime_ns for p in w.rglob("*") if p.is_file()}
        scene = SceneModel.load(w)
        terrain = sm.load_terrain(w, scene)
        mesh_checks(str(w.relative_to(WORLDS_ROOT)), scene, w, terrain, rng,
                    n_crops, rays)
        after = {p: p.stat().st_mtime_ns for p in w.rglob("*") if p.is_file()}
        check("nothing written into the world folder", before == after)

    print(f"\nraycast_test: {PASSED} passed, {FAILED} failed, {SKIPPED} "
          f"skipped in {time.time() - t_start:.0f} s")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
