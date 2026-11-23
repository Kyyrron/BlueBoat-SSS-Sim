"""The one module that knows the simulator (``blueboat_sss_sim``).

It drives the **frozen** world generator as a library, without changing it:

* :func:`environment_from_state`: the environment of a world-builder state,
  from the generator's own build pipeline (the terrain the world will have),
  reduced to the 1 m planning grid, with the walls and the structures as
  features;
* :func:`apply_plan`: a plan written into a state (pinned instances,
  structures, small-litter layers, the seabed declaration, the placement
  record) and the ``extras`` for ``metadata.yaml``;
* :func:`plan_world`, :func:`build_world`, :func:`run`: a specification
  (``blueboat_campaign/1`` plus ``placement:`` and optionally
  ``environment:``) and seeds built into world folders with
  ``worldgen.world_folder.build_world_folder``.

What makes this possible without touching the generator: it places
``objects.placed + explicit`` verbatim, writes every key of ``extras`` into
``metadata.yaml`` and every unknown ``params`` key of an instance into that
object's ``params`` (``worldgen/groundtruth.py``).

The planning environment is computed with the plan's own seabed layers in
place at zero density, so that the terrain it sees is the terrain of the
built world (a layer changes how a layer-less state is built).

Simulator imports are lazy: the rest of the package works without it.
"""

from __future__ import annotations

import copy
import hashlib
import platform
import shutil
import tempfile
import time
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from . import __version__ as VERSION
from .data import PACKAGE, TARGETS, code_digest, default_parameters, evidence_digest
from .environment import Environment, Feature, block_reduce
from .layers import RESERVED_PREFIX
from .model import populate
from .params import PlacementParams
from .plan import PlacementPlan, plain
from .rng import stream

MANIFEST_FORMAT = "blueboat_population_manifest/1"
#: the planning cell [m]
CELL_M = 1.0
#: id of the zero-density layer the planning environment is computed with
CANONICAL_LAYER = f"{RESERVED_PREFIX}litter_base"
#: keys of a specification that this package reads and the frozen generator
#: does not know
OWN_KEYS = ("placement", "environment")
_ENV_KEYS = {"depth_file", "values", "water_level_m", "smooth_sigma_m", "shift_m", "compress", "limits",
             "walls", "features", "sediment", "clutter", "texture"}


class SimulatorMissing(ImportError):
    """``blueboat_sss_sim`` cannot be imported."""


def _sim():
    """The simulator's modules, imported on first use."""
    try:
        from blueboat_sss_sim.builder import state as bstate
        from blueboat_sss_sim.core import geometry, provenance
        from blueboat_sss_sim.core.types import Wall
        from blueboat_sss_sim.worldgen import campaign, domain, world_folder
        from blueboat_sss_sim.worldgen.materials import MaterialLibrary
        from blueboat_sss_sim.worldgen.scene import WorldConfig
    except ImportError as exc:                                   # pragma: no cover - environment
        raise SimulatorMissing(
            "blueboat_sss_sim cannot be imported: source the workspace, or put the simulator's package source "
            f"root (BlueBoat-SSS-Sim/blueboat_sss_sim) on PYTHONPATH ({exc})") from exc

    class S:
        pass
    s = S()
    s.bstate, s.geometry, s.provenance, s.Wall = bstate, geometry, provenance, Wall
    s.campaign, s.domain, s.world_folder = campaign, domain, world_folder
    s.MaterialLibrary, s.WorldConfig = MaterialLibrary, WorldConfig
    return s


def simulator_identity() -> dict:
    """The simulator in use: its code digest, the generator's digest, and
    whether it is the source tree."""
    s = _sim()
    chk = s.provenance.source_check()
    return {"code_digest": chk["running_digest"][:16], "generator_digest": s.provenance.code_digest()[:16],
            "package_dir": str(s.provenance.package_dir()), "matches_source": chk["matches"]}


def model_identity() -> dict:
    return {"package": PACKAGE, "version": VERSION, "code_digest": code_digest(),
            "evidence_digest": evidence_digest()}


# ----------------------------------------------------------------- environment
def strip_reserved(layers) -> list:
    """The layers without those a plan added."""
    return [copy.deepcopy(dict(L)) for L in (layers or [])
            if not str(dict(L).get("id", "")).startswith(RESERVED_PREFIX)]


def strip_plan_structures(explicit) -> list:
    """The explicit instances without the structures a plan added."""
    out = []
    for e in explicit or []:
        prm = dict(e.get("params") or {})
        src = prm.get("source") if isinstance(prm.get("source"), Mapping) else {}
        if src.get("use") == "structure" and str(prm.get("name", "")).startswith(RESERVED_PREFIX):
            continue
        out.append(copy.deepcopy(dict(e)))
    return out


def canonical_state(state, coarse_class: str = "gravel"):
    """A copy of ``state`` as the planning environment is computed on: no
    objects, the layers a plan added removed, and one zero-density small
    litter layer in their place (it makes the build take the same path as
    the planned world's, and adds nothing)."""
    s = _sim()
    st = copy.deepcopy(state)
    st.objects = s.bstate.ObjectsState(seed=int(state.objects.seed), classes={}, pinned=True)
    st.depth.layers = strip_reserved(state.depth.layers) + [
        {"id": CANONICAL_LAYER, "kind": "small_litter", "seed": 1,
         "params": {"per_m2": 0.0, "material": coarse_class}}]
    if hasattr(st, "placement"):
        st.placement = {}
    return st


def _wall_features(state, records: list) -> list:
    """The walls of a state as features (design frame): one per drawn wall
    (a curved wall is one feature along its chords). A pontoon wall that a
    given record already describes (the campaign's four pontoon walls and
    its one ``pontoon`` feature) is left out."""
    s = _sim()
    by_base: dict[str, list] = {}
    for w in s.geometry.expand_walls(state.walls):
        base = s.geometry.curve_base_name(w["name"]) or w["name"]
        by_base.setdefault(base, []).append(w)
    rec = {str(r["id"]): r for r in records}
    pontoon_recs = [r for r in records if r["kind"] == "pontoon"]
    out = []
    for name in sorted(by_base):
        segs = by_base[name]
        kind = str(segs[0].get("kind", "quay"))
        pts = [[float(segs[0]["x0"]), float(segs[0]["y0"])]] + [[float(w["x1"]), float(w["y1"])] for w in segs]
        if name in rec:
            continue                                   # the record describes this very wall
        if kind == "pontoon" and pontoon_recs:
            mid = np.mean(np.asarray(pts, float), axis=0)
            near = False
            for r in pontoon_recs:
                f = Feature(str(r["id"]), "pontoon", r["points"])
                if float(f.distance(mid[0], mid[1])) < 3.0:
                    near = True
            if near:
                continue
        out.append(Feature(name, "pontoon" if kind == "pontoon" else "quay", pts,
                           clear_m=0.0 if kind == "pontoon" else 0.5 * float(segs[0].get("thickness", 0.3)),
                           attrs={"wall": True}))
    return out


def _object_id(e: Mapping[str, Any]) -> str:
    name = dict(e.get("params") or {}).get("name")
    return str(name) if name else f"{e['type']}@{float(e['x']):.1f},{float(e['y']):.1f}"


def environment_from_state(state, *, features: list | None = None, coarse_class: str | None = None,
                           name: str = "") -> Environment:
    """The planning environment of a builder state (design frame).

    ``features``: feature mappings the state does not hold (a campaign
    basin's berth, outlet and breakwater; a specification's own), in the
    design frame; a record with the id of a wall replaces that wall's
    feature (its kind, e.g. ``breakwater``, and its use)."""
    s = _sim()
    wf = s.world_folder
    records = [dict(r) for r in (features or [])]
    coarse = coarse_class or _coarse_class(state)
    st = canonical_state(state, coarse)
    st.validate_for_build()
    tmp = Path(tempfile.mkdtemp(prefix="harbour_population_env_"))
    try:
        raw, _baked = wf.build_config(st, tmp)
        scene, pre_stamp = wf._generate_with_base(s.WorldConfig.from_dict(raw), raw)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    grid = scene.grid
    res = float(grid.resolution)
    ox, oy = state.spawn_origin()
    water = s.domain.survey_water(pre_stamp, grid, scene.walls)
    factor = max(int(np.floor(CELL_M / res + 0.5)), 1)
    w, d, c = block_reduce(water, -np.asarray(pre_stamp, dtype=np.float64),
                           np.asarray(scene.material_id, dtype=np.int64), factor)
    cell = factor * res
    x0 = float(grid.origin_x) - 0.5 * res + ox
    y0 = float(grid.origin_y) - 0.5 * res + oy
    border = float(default_parameters()("envelope.border_m"))
    blocked = np.zeros(w.shape, dtype=bool)
    b = max(int(np.ceil(border / cell)), 1)
    blocked[:b] = blocked[-b:] = True
    blocked[:, :b] = blocked[:, -b:] = True
    feats = []
    rec_ids = set()
    for r in records:
        f = r if isinstance(r, Feature) else Feature.from_dict(r)
        rec_ids.add(f.id)
        feats.append(f)
    # a record that names a wall takes that wall's solid body
    solid = {}
    for wd in s.geometry.expand_walls(state.walls):
        base = s.geometry.curve_base_name(wd["name"]) or wd["name"]
        if str(wd.get("kind", "quay")) != "pontoon":
            solid[base] = 0.5 * float(wd.get("thickness", 0.3))
    for f in feats:
        if f.id in solid and f.clear_m == 0.0:
            f.clear_m = solid[f.id]
    feats += _wall_features(state, [f.to_dict() for f in feats])
    occupied = []
    ids = {f.id for f in feats}
    for e in strip_plan_structures(state.objects.explicit):
        prm = dict(e.get("params") or {})
        t = str(e["type"])
        el, ew = float(e.get("length", 1.0)), float(e.get("width", 1.0))
        rad = 0.5 * float(np.sqrt(el * el + ew * ew))
        oid = _object_id(e)
        if t == "chain":
            continue                                   # a tether: thin, hanging; no keep-out, no source
        role = str(prm.get("role", ""))
        occupied.append({"x": float(e["x"]), "y": float(e["y"]), "radius": round(rad, 3),
                         "target": bool(role in ("target", "validation") or (not role and t in TARGETS)),
                         "id": oid})
        if t in ("mooring_block", "mooring_pile") and oid not in ids:
            feats.append(Feature(oid, "mooring" if t == "mooring_block" else "pile",
                                 [(float(e["x"]), float(e["y"]))], clear_m=round(rad, 3),
                                 attrs={"object": True}))
            ids.add(oid)
        elif oid in ids:
            # the record of a campaign's block: give it the block's body
            for f in feats:
                if f.id == oid and f.clear_m == 0.0:
                    f.clear_m = round(rad, 3)
    lib = s.MaterialLibrary()
    refl = {str(n): float(lib.get(n).reflectivity) for n in sorted(lib.names)}
    return Environment(
        x0=x0, y0=y0, cell_m=cell, water=w, depth=d, sediment=c,
        sediment_names=[str(n) for n in scene.material_names], features=feats, blocked=blocked,
        occupied=occupied, path_xy=np.asarray(state.path.xy(), dtype=np.float64),
        transducer_depth_m=float(s.domain.sensor_depth_m()), coarse_class=coarse,
        material_reflectivity=refl, name=name or str(state.world_name),
        origin={"kind": "builder_state", "world_seed": int(state.objects.seed),
                "build_resolution_m": res, "block_factor": factor,
                "design_offset": [round(float(ox), 3), round(float(oy), 3)]})


def _coarse_class(state) -> str:
    """The hard class of small litter in this world: the coarser patch class
    its seabed declares, else gravel."""
    sb = dict(getattr(state.depth, "seabed", None) or {})
    for c in sb.get("patch_classes") or []:
        if c in ("gravel", "coarse_sand", "rocks"):
            return str(c)
    return "gravel"


# ------------------------------------------------------------------------ plan
def fouling_of(seed: int) -> float:
    """The world's fouling, for the sonar's ``fouling`` parameter: U(0.2, 1)
    on the placement seed (the validity conditions' draw)."""
    return round(0.2 + 0.8 * float(stream(seed, "world", "fouling").random()), 3)


def apply_plan(state, plan: PlacementPlan, *, origin: Mapping[str, Any] | None = None,
               run: Mapping[str, Any] | None = None) -> dict:
    """Write ``plan`` into ``state`` (in place) and return the ``extras`` for
    ``build_world_folder``. Idempotent: applying again replaces what a plan
    added before.

    * litter and debris -> ``objects.placed`` (a pinned field, as the
      campaign generator's), structures -> ``objects.explicit``;
    * small-litter layers -> ``depth.layers`` (reserved ids);
    * the seabed declaration lists the litter's hard class (the sensor's
      near-nadir correction needs it); only on a ``sand`` reference;
    * ``state.placement`` (when the state has the field: simulator v1.1)
      keeps the record for an Edit re-save and for Screen 5: the block as
      asked, the model's digests, the metadata block, the layers and the
      seabed declaration before and after.
    ``run``: recorded under ``run`` of the metadata (the world's fouling)."""
    s = _sim()
    st = state
    prev = dict(getattr(st, "placement", None) or {})
    seabed_before = prev.get("seabed_before", plain(dict(st.depth.seabed or {})))
    st.objects.placed = [copy.deepcopy(e) for e in plan.instances]
    st.objects.explicit = strip_plan_structures(st.objects.explicit) + [copy.deepcopy(e) for e in plan.structures]
    st.objects.pinned = True
    st.objects.placed_signature = s.bstate.scatter_signature(st.objects.seed, st.objects.classes)
    st.depth.layers = strip_reserved(st.depth.layers) + [copy.deepcopy(L) for L in plan.layers]
    seabed_note = None
    st.depth.seabed = plain(dict(seabed_before or {}))
    if plan.litter_class:
        sb = dict(st.depth.seabed or {})
        ref = sb.get("reference") or st.depth.material
        if ref != "sand":
            seabed_note = (f"the seabed's reference class is {ref!r}: the harbour sensor is calibrated on sand "
                           "only, so no seabed declaration is written for the small litter")
        else:
            patch = [str(c) for c in (sb.get("patch_classes") or [])]
            if plan.litter_class not in patch:
                patch = sorted(patch + [plan.litter_class])
            sb.update(reference="sand", patch_classes=patch)
            st.depth.seabed = sb
    meta = plan.metadata_block()
    if origin:
        meta["origin"] = plain(dict(origin))
    if seabed_note:
        meta["warnings"] = list(meta.get("warnings", [])) + [seabed_note]
    record = plan.state_block()
    record.update(mode="mission_like", metadata=meta, layers=plain(plan.layers),
                  seabed=plain(dict(st.depth.seabed or {})), seabed_before=plain(dict(seabed_before or {})))
    if run:
        record["run"] = plain(dict(run))
    if hasattr(st, "placement"):
        st.placement = record
    out = {"placement": meta}
    if run:
        out["run"] = plain(dict(run))
    return out


# --------------------------------------------------------------- specification
def split_spec(doc: Mapping[str, Any]) -> tuple[dict, dict | None, dict | None]:
    """``(campaign specification, placement block, environment block)`` of a
    specification as read: this package's keys taken out (also out of each
    case, where a case may override ``placement``)."""
    doc = copy.deepcopy(dict(doc))
    placement = doc.pop("placement", None)
    environment = doc.pop("environment", None)
    return doc, placement, environment


def case_placement(doc: Mapping[str, Any], case: str) -> dict | None:
    """The placement block of one case: the specification's, with the case's
    own ``placement`` keys over it (``advanced`` and ``clutter`` merged)."""
    base = copy.deepcopy(dict(doc.get("placement") or {})) if doc.get("placement") is not None else None
    for c in doc.get("cases") or []:
        if str(c.get("name")) == case and c.get("placement") is not None:
            over = dict(c["placement"])
            base = dict(base or {})
            for k, v in over.items():
                if k in ("advanced", "clutter", "feature_tags", "feature_activity") and isinstance(v, Mapping):
                    base[k] = {**dict(base.get(k) or {}), **dict(v)}
                else:
                    base[k] = copy.deepcopy(v)
    return base


def is_mission(placement: Mapping[str, Any] | None) -> bool:
    return bool(placement) and str(placement.get("mode", "mission_like")) != "legacy"


def load_spec(path: str | Path) -> dict:
    """A specification file as written (this package's keys kept); a
    relative trajectory or depth file is resolved beside it."""
    path = Path(path).expanduser().resolve()
    doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    rel = Path(str(doc.get("path", "")))
    if rel.suffix == ".yaml" and not rel.is_absolute() and (path.parent / rel).is_file():
        doc["path"] = str((path.parent / rel).resolve())
    env = doc.get("environment")
    if isinstance(env, Mapping) and env.get("depth_file"):
        f = Path(str(env["depth_file"])).expanduser()
        if not f.is_absolute():
            doc["environment"] = {**dict(env), "depth_file": str((path.parent / f).resolve())}
    return doc


def _campaign_spec(doc: Mapping[str, Any], mission: bool) -> dict:
    """The part of a specification the frozen generator checks. For a
    mission-like world its own litter is switched off (density 0) and every
    case's ``objects`` and ``placement`` are taken out."""
    s = _sim()
    spec, _, _ = split_spec(doc)
    cases = []
    for c in spec.get("cases") or [{"name": "nominal"}]:
        c = {k: copy.deepcopy(v) for k, v in dict(c).items() if k != "placement"}
        if mission:
            c.pop("objects", None)
        cases.append(c)
    spec["cases"] = cases
    if mission:
        d = copy.deepcopy(dict(spec.get("defaults") or {}))
        d["objects"] = {**dict(d.get("objects") or {}), "density_per_ha": 0}
        spec["defaults"] = d
    return s.campaign.check_spec(spec)


def _feature_records(extras: Mapping[str, Any], ox: float, oy: float) -> list:
    """The features a campaign basin recorded (``extras["structures"]``:
    kind, name, points in the world frame), as feature mappings in the
    design frame."""
    kinds = {"quay": "quay", "breakwater": "breakwater", "berth": "berth", "outlet": "outlet",
             "pontoon": "pontoon", "mooring": "mooring"}
    out = []
    for r in extras.get("structures") or []:
        if "points" not in r or r.get("kind") not in kinds:
            continue
        pts = [[float(x) + ox, float(y) + oy] for x, y in r["points"]]
        out.append({"id": str(r["name"]), "kind": kinds[r["kind"]], "points": pts})
    return out


# ------------------------------------------------------------- depth from file
def _adjust(layer_id: str, name: str, **params) -> dict:
    return {"id": layer_id, "kind": "adjust", "name": name, "params": params}


def environment_layers(env_block: Mapping[str, Any]) -> tuple[list, dict]:
    """The ``adjust`` layers that turn a chart-datum depth file into the
    world's floor, and the record of them. In this order: smoothing (a
    Gaussian of ``smooth_sigma_m``), the water level above the file's datum
    (``water_level_m``: the floor gets deeper by it), then either a shift
    (``shift_m``: the floor rises by it) or a compression about ``d_lo``
    (``compress: {k, d_lo, to}``: depth' = to + k (depth - d_lo))."""
    L: list = []
    rec: dict[str, Any] = {}
    sigma = float(env_block.get("smooth_sigma_m", 0.0) or 0.0)
    if sigma > 0.0:
        L.append(_adjust("env_smooth", "Low-pass of the depth file", op="smooth", sigma_m=sigma))
        rec["smooth_sigma_m"] = sigma
    eta = float(env_block.get("water_level_m", 0.0) or 0.0)
    if eta != 0.0:
        L.append(_adjust("env_water_level", "Water level above the file's datum", op="offset", offset_m=eta))
    rec["water_level_m"] = eta
    shift = env_block.get("shift_m")
    comp = env_block.get("compress")
    if shift is not None and comp is not None:
        raise ValueError("environment: give shift_m or compress, not both")
    if shift is not None and float(shift) != 0.0:
        L.append(_adjust("env_shift", "Depth shift into the sensor's band", op="offset", offset_m=-float(shift)))
        rec["shift_m"] = float(shift)
    if comp is not None:
        comp = dict(comp)
        bad = set(comp) - {"k", "d_lo", "to"}
        if bad:
            raise KeyError(f"environment.compress: unknown key(s) {sorted(bad)} (known: k, d_lo, to)")
        k, d_lo, to = float(comp["k"]), float(comp["d_lo"]), float(comp.get("to", comp["d_lo"]))
        if k < 0.5:
            raise ValueError(f"environment.compress.k {k}: at least 0.5 (the owner's rule: prefer a shift)")
        L.append(_adjust("env_compress", "Depth compression into the sensor's band", op="scale", scale=k,
                         datum_m=d_lo))
        if to != d_lo:
            L.append(_adjust("env_compress_to", "Depth compression: its lower end", op="offset",
                             offset_m=to - d_lo))
        rec["compress"] = {"k": k, "d_lo": d_lo, "to": to}
    return L, rec


def state_from_environment(spec: Mapping[str, Any], env_block: Mapping[str, Any], case: str, seed: int):
    """``(BuilderState, extras, feature records)`` of a world on a depth
    file (CSV ``lat,lon,depth|elevation`` or a geographic GeoTIFF), built
    with the generator's own reader (``worldgen/depth/ingest.py``): the
    trajectory anchors the world, the limits are its bounding box plus the
    margin (or ``environment.limits``), the walls and the features come
    from the block."""
    s = _sim()
    camp = s.campaign
    bad = set(env_block) - _ENV_KEYS
    if bad:
        raise KeyError(f"environment: unknown key(s) {sorted(bad)} (known: {sorted(_ENV_KEYS)})")
    if not env_block.get("depth_file"):
        raise ValueError("environment: depth_file is required")
    src = Path(str(env_block["depth_file"])).expanduser()
    if not src.is_file():
        raise FileNotFoundError(f"environment.depth_file {src}")
    path = s.bstate.PathState.from_mcs_file(camp.resolve_path(spec["path"]))
    xy = path.xy()
    m = float(spec["world"]["margin_m"])
    lim = env_block.get("limits")
    if lim:
        rect = {k: float(lim[k]) for k in ("x_min", "y_min", "x_max", "y_max")}
    else:
        rect = {"x_min": float(xy[:, 0].min() - m), "y_min": float(xy[:, 1].min() - m),
                "x_max": float(xy[:, 0].max() + m), "y_max": float(xy[:, 1].max() + m)}
    cond = camp.conditions(spec, case)
    sed = {**dict(cond["sediment"]), **dict(env_block.get("sediment") or {})}
    clutter = str(env_block.get("clutter", cond["clutter"]))
    cond2 = {**cond, "sediment": sed, "clutter": clutter}
    seabed, seabed_rec = camp.seabed_layers(cond2)
    walls, records = [], []
    anchor = path.anchor
    for wdef in env_block.get("walls") or []:
        wdef = dict(wdef)
        name = str(wdef["name"])
        if "points_gps" in wdef:
            pts = [list(anchor.to_en(float(la), float(lo))) for la, lo in wdef["points_gps"]]
            pts = [[float(np.asarray(e).ravel()[0]), float(np.asarray(n).ravel()[0])] for e, n in pts]
        else:
            pts = [[float(x), float(y)] for x, y in wdef["points"]]
        kind = str(wdef.get("kind", "quay"))
        wall_kind = "pontoon" if kind == "pontoon" else "quay"
        for k in range(len(pts) - 1):
            seg = f"{name}_{k + 1}" if len(pts) > 2 else name
            walls.append(camp._wall(seg, pts[k], pts[k + 1], kind=wall_kind,
                                    **{q: wdef[q] for q in ("material", "thickness", "top_z", "draft_m",
                                                            "reflectivity", "roughness_m") if q in wdef}))
        if len(pts) > 2 or kind not in ("quay", "pontoon") or wdef.get("use"):
            rec = {"id": name, "kind": kind if kind in ("quay", "pontoon", "breakwater", "pier") else "quay",
                   "points": pts}
            if wdef.get("use"):
                rec["use"] = str(wdef["use"])
            records.append(rec)
    for f in env_block.get("features") or []:
        f = dict(f)
        if "points_gps" in f:
            pts = []
            for la, lo in f.pop("points_gps"):
                e, n = anchor.to_en(float(la), float(lo))
                pts.append([float(np.asarray(e).ravel()[0]), float(np.asarray(n).ravel()[0])])
            f["points"] = pts
        records.append(f)
    layers, rec = environment_layers(env_block)
    from blueboat_sss_sim.worldgen.terrain import texture_defaults
    tex = texture_defaults()
    tex["roughness"]["amplitude"] = 0.0
    tex["materials"]["uniform"] = sed["reference"]
    patch = sorted({sed["finer"], sed["coarser"]} - {sed["reference"]})
    state = s.bstate.BuilderState(
        path=path, limits_rect=rect, walls=walls,
        depth=s.bstate.DepthState(mode="file", file=str(src), file_values=str(env_block.get("values", "auto")),
                                  texture=tex, material=sed["reference"], layers=layers + seabed,
                                  seabed={"reference": sed["reference"], "patch_classes": patch,
                                          "heterogeneity": sed["heterogeneity"]}),
        objects=s.bstate.ObjectsState(seed=int(seed), classes={}, pinned=True),
        world_name=f"{spec['name']}-{case}-s{int(seed)}")
    state.validate_for_build()
    rec.update(depth_file=src.name, depth_file_sha256=hashlib.sha256(src.read_bytes()).hexdigest()[:16],
               values=str(env_block.get("values", "auto")))
    extras = {
        "campaign": {"name": str(spec["name"]), "case": case, "seed": int(seed),
                     "generator_version": camp.GENERATOR_VERSION},
        "environment": plain(rec),
        "conditions": {"sediment": sed, "clutter": clutter, **seabed_rec},
        "run": dict(spec.get("run") or {}),
        "structures": [{"kind": r["kind"], "name": r["id"]} for r in records],
    }
    return state, extras, records


# ----------------------------------------------------------------------- build
def plan_world(doc: Mapping[str, Any], case: str, seed: int):
    """``(state, extras, plan, environment)`` of one mission-like world of a
    specification: the frozen generator's basin (or a depth file), then the
    population. The world seed seeds the basin and the seabed; the placement
    seed is the block's ``seed`` when given, else the world seed."""
    s = _sim()
    placement = case_placement(doc, case)
    if not is_mission(placement):
        raise ValueError("plan_world: the specification has no mission-like placement for this case")
    _, _, env_block = split_spec(doc)
    spec = _campaign_spec(doc, mission=True)
    seed = int(seed)
    cond = s.campaign.conditions(spec, case)
    if env_block:
        state, extras, records = state_from_environment(spec, env_block, case, seed)
        coarse = extras["conditions"]["sediment"]["coarser"]
    else:
        state, extras = s.campaign.plan_world(spec, case, seed)
        ox, oy = state.spawn_origin()
        records = _feature_records(extras, ox, oy)
        coarse = cond["sediment"]["coarser"]
        extras.pop("population", None)
        extras["conditions"]["objects"] = {"set": "v2", "density_per_ha": 0,
                                           "note": "litter by placement (mission_like)"}
    block = dict(placement)
    block.setdefault("mode", "mission_like")
    adv = dict(block.get("advanced") or {})
    adv.setdefault("uncertain_materials", cond["uncertain_materials"])
    adv.setdefault("face_geometry", cond["face_geometry"])
    block["advanced"] = adv
    block.setdefault("seed", seed)
    params = PlacementParams.from_dict(block)
    if params.mode == "manual":
        raise ValueError("placement.mode manual in a specification: not planned (use plan_manual)")
    if params.small_litter:
        # the level decides the small litter: the campaign's own layer goes
        state.depth.layers = [L for L in state.depth.layers if dict(L).get("id") != "small_litter"]
    env = environment_from_state(state, features=records, coarse_class=coarse, name=state.world_name)
    plan = populate(env, params, params.seed)
    fouling = extras.get("run", {}).get("sonar_fouling")
    ex = apply_plan(state, plan)
    extras.update(ex)
    if fouling is None:
        extras.setdefault("run", {})["sonar_fouling"] = fouling_of(params.seed)
    return state, extras, plan, env


def build_world(doc: Mapping[str, Any], case: str, seed: int, root: str | Path, *, cards: bool = True,
                progress=None) -> dict:
    """Build one (case, seed) of a specification under ``root`` with the
    frozen ``build_world_folder``; the manifest record. An existing folder
    is never touched (``skipped``). A case without a mission-like placement
    goes to the frozen ``campaign.build_world`` as it is."""
    s = _sim()
    root = Path(root).expanduser()
    placement = case_placement(doc, case)
    if not is_mission(placement):
        return s.campaign.build_world(_campaign_spec(doc, mission=False), case, int(seed), root, progress)
    wf = s.world_folder
    spec = _campaign_spec(doc, mission=True)
    t0 = time.time()
    path_name = wf._safe_name(Path(s.campaign.resolve_path(spec["path"])).stem)
    name = wf._safe_name(f"{spec['name']}-{case}-s{int(seed)}")
    final = root / path_name / name
    rec: dict[str, Any] = {"case": case, "seed": int(seed), "world": f"{path_name}/{name}"}
    if final.exists():
        rec["skipped"] = True
        if (final / "metadata.yaml").is_file():
            rec["content_digest"] = (yaml.safe_load((final / "metadata.yaml").read_text()) or {}
                                     ).get("content_digest")
        return rec
    state, extras, plan, env = plan_world(doc, case, seed)
    out = Path(wf.build_world_folder(state, root, progress=progress, extras=extras))
    if out.name != name:
        raise RuntimeError(f"built {out}, expected the folder name {name}")
    meta = yaml.safe_load((out / "metadata.yaml").read_text())
    rec.update(skipped=False, content_digest=meta["content_digest"], plan_digest=plan.digest(),
               environment_digest=plan.environment_digest,
               domain={k: meta["domain"][k] for k in ("share_in_domain", "in_domain", "survey_area_m2")},
               object_counts=meta["object_counts"], detectable_targets=meta["detectable_targets"],
               placement={k: plan.params.get(k) for k in ("profile", "density_level", "seed", "split")},
               realised=plan.report["realised"], warnings=plan.report["warnings"],
               sonar_fouling=extras.get("run", {}).get("sonar_fouling"),
               build_seconds=round(time.time() - t0, 1))
    if cards:
        from .card import write_card
        cdir = root / ".population" / path_name / name
        rec["card"] = str(write_card(out, cdir, plan=plan, environment=env).relative_to(root))
    return rec


def run(spec_path: str | Path, seeds, root: str | Path, cases=None, *, cards: bool = True, progress=None) -> dict:
    """Every (case, seed) of a specification; the manifest (also written to
    ``<root>/.population/<name>/manifest.yaml``, earlier records kept)."""
    _sim()                                     # fails early, naming the cause, without the simulator
    doc = load_spec(spec_path)
    spec = _campaign_spec(doc, mission=False)
    root = Path(root).expanduser()
    mdir = root / ".population" / str(spec["name"])
    mdir.mkdir(parents=True, exist_ok=True)
    mfile = mdir / "manifest.yaml"
    manifest = (yaml.safe_load(mfile.read_text()) or {}) if mfile.is_file() else {}
    worlds = {(w["case"], int(w["seed"])): w for w in manifest.get("worlds", [])}
    for case in (cases or [str(c["name"]) for c in spec["cases"]]):
        for seed in seeds:
            if progress:
                progress(f"{spec['name']}: case {case}, seed {seed}")
            rec = build_world(doc, case, int(seed), root, cards=cards)
            if rec.get("skipped") and (case, int(seed)) in worlds:
                worlds[(case, int(seed))]["skipped_last_run"] = True
            else:
                worlds[(case, int(seed))] = rec
            if progress:
                progress("  skipped (exists)" if rec.get("skipped") else
                         f"  built in {rec['build_seconds']} s, content digest {str(rec['content_digest'])[:16]}")
    manifest = {
        "format": MANIFEST_FORMAT, "name": str(spec["name"]), "specification": plain(doc),
        "specification_file": str(Path(spec_path).resolve()),
        "model": model_identity(), "simulator": simulator_identity(),
        "versions": {"python": platform.python_version(), "numpy": np.__version__},
        "worlds": [plain(worlds[k]) for k in sorted(worlds)],
    }
    mfile.write_text(yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None), encoding="utf-8")
    return manifest


# ------------------------------------------------------------- world folders
def environment_from_world(world_dir: str | Path) -> Environment:
    """The planning environment of an existing world folder, from its
    ``builder_state.yaml`` (objects and earlier plan layers left out; its
    campaign features taken from ``metadata.yaml``)."""
    s = _sim()
    world_dir = Path(world_dir).expanduser()
    state = s.world_folder.load_builder_state(world_dir)
    meta = yaml.safe_load((world_dir / "metadata.yaml").read_text()) or {}
    ox, oy = state.spawn_origin()
    return environment_from_state(state, features=_feature_records(meta, ox, oy), name=world_dir.name)
