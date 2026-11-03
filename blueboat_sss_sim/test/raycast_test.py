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
        ref_b = np.trapz(f, u, axis=1) / L
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    rng = np.random.default_rng(20261102)
    n_crops, rays = (2, 5000) if args.quick else (5, 20000)
    t_start = time.time()

    intersection_checks(rng)
    direct_path_unit_checks()
    direct_path_render_checks()
    grouped_mean_checks()
    shared_stage_checks()
    surface_path_checks()
    po_closed_form_checks()
    seabed_law_checks()
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
