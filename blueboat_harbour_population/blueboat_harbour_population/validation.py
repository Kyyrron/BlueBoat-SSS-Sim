"""Validation of the model on many seeds, and the expected-density map.

* :func:`expected_density`: the density the model expects in each planning
  cell (objects per hectare), for the heat map of a world card and of the
  Screen 5 preview.
* :func:`pattern_statistics`: the statistics of one pattern that the
  measured patterns of the evidence also have (nearest-neighbour distances,
  the Clark-Evans ratio, the share of singles when points closer than a
  linking distance are joined, group sizes).
* :func:`run_statistics`: many placement seeds per (environment, profile,
  level): realised density against the level's, share within 30 m of a
  source, distances to the source, clustering, class shares, spacing losses.
  ``python3 -m blueboat_harbour_population stats`` runs it.

Pure Python, numpy, scipy.spatial.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .data import CLASSES, TARGETS, default_parameters
from .environment import Environment
from .model import allowed_mask, populate
from .params import PlacementParams
from .sources import build_sources
from .stats import clark_evans, distance_to_sources


def expected_density(env: Environment, params: PlacementParams | dict, seed: int | None = None) -> np.ndarray:
    """Expected sonar-size objects per hectare in each planning cell (NaN
    where no object may lie), after the spacing rule (the level's density
    over the active band on average)."""
    p = default_parameters()
    if not isinstance(params, PlacementParams):
        params = PlacementParams.from_dict(params, p)
    seed = int(params.seed if seed is None else seed)
    allowed, _ = allowed_mask(env, params, p)
    srcs, rep = build_sources(env, params, p, allowed, seed)
    ret = float(rep["retention"])
    out = np.zeros(allowed.size)
    for s in srcs:
        if not len(s.cells) or s.budget <= 0.0:
            continue
        pdf = np.diff(np.concatenate([[0.0], s.pdf_cum])) / s.pdf_cum[-1]
        share = 1.0 if params.debris else s.target_share
        np.add.at(out, s.cells, s.budget * ret * share * pdf)
    out = out / env.cell_area_ha
    out[~allowed.ravel()] = np.nan
    return out.reshape(allowed.shape)


def link_groups(xy: np.ndarray, link_m: float) -> np.ndarray:
    """Sizes of the groups formed by joining every two points closer than
    ``link_m`` (single linkage)."""
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    n = len(xy)
    if n == 0:
        return np.zeros(0, dtype=np.int64)
    from scipy.spatial import cKDTree
    parent = np.arange(n)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b in cKDTree(xy).query_pairs(float(link_m)):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    roots = np.array([find(i) for i in range(n)])
    return np.bincount(np.unique(roots, return_inverse=True)[1])


def pattern_statistics(xy: np.ndarray, area_m2: float, links: Sequence[float] = (5.0, 10.0, 20.0),
                       perimeter: float | None = None) -> dict:
    """What a mapped pattern of positions is summarised by in the evidence
    (the 292 pot positions of Clallam County, the 936 tyre positions of
    Plymouth Sound): density, nearest-neighbour distances, the Clark-Evans
    ratio, and for each linking distance the share of points that stay
    single and the largest group."""
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    n = len(xy)
    out: dict[str, Any] = {"n": int(n), "per_ha": (n / (area_m2 / 1e4)) if area_m2 > 0 else None}
    if n < 2:
        return out
    from scipy.spatial import cKDTree
    d, _ = cKDTree(xy).query(xy, k=2)
    nn = d[:, 1]
    out.update(nn_median_m=float(np.median(nn)), nn_mean_m=float(nn.mean()),
               clark_evans=clark_evans(xy, area_m2, perimeter), clark_evans_raw=clark_evans(xy, area_m2))
    for L in links:
        sizes = link_groups(xy, L)
        out[f"singles_share_{L:g}m"] = float((sizes == 1).sum()) / n
        out[f"largest_group_{L:g}m"] = int(sizes.max())
    return out


def one_run(env: Environment, block: dict, seed: int) -> dict:
    """The figures of one plan that the many-seed statistics collect."""
    plan = populate(env, {**block, "seed": int(seed), "split": "development", "split_override": "statistics"})
    r, z = plan.report, plan.report["realised"]
    xy = plan.xy()
    feats = [f for f in env.features]
    allowed_m2 = r["allowed_ha"] * 1e4
    rec = {"seed": int(seed), "expected": r["expected"]["total"], "expected_targets": r["expected"]["targets"],
           "n": z["targets"] + z["debris"], "targets": z["targets"], "debris": z["debris"],
           "groups": z["groups"], "grouped": z["grouped_objects"],
           "band_ha": r["active_band_ha"], "allowed_ha": r["allowed_ha"],
           "density_band": z["density_active_per_ha"], "share_30": z["share_within_30m"],
           "removed": z["removed_by_spacing"], "no_room": z["no_room"],
           "per_class": z["per_class"], "expected_per_class": r["expected"]["per_class"],
           "buried": z["effectively_buried"], "on_mud": z["on_mud"]}
    if len(xy) >= 2:
        from .model import allowed_mask
        from .stats import perimeter_m
        allowed, _ = allowed_mask(env, PlacementParams.from_dict({**block, "split_override": "statistics"}),
                                  default_parameters())
        st = pattern_statistics(xy, allowed_m2, perimeter=perimeter_m(allowed, env.cell_m))
        rec.update({k: st[k] for k in st if k not in ("n", "per_ha")})
    if len(xy):
        d = distance_to_sources(xy, [f for f in feats if f.clear_m >= 0.0])
        rec["distances"] = [round(float(v), 2) for v in d[np.isfinite(d)]]
        rec["classes"] = [e["type"] for e in plan.instances]
    return rec


def _job(args) -> dict:
    env_path, block, seeds = args
    env = Environment.load(env_path)
    return {"environment": Path(env_path).stem, "profile": block.get("profile"),
            "level": block.get("density_level"), "runs": [one_run(env, block, s) for s in seeds]}


def run_statistics(env_paths: Sequence[str | Path], profiles: Sequence[str], levels: Sequence[int],
                   n_seeds: int, out_dir: str | Path, jobs: int = 1, first_seed: int = 1,
                   extra_block: dict | None = None, progress=None) -> Path:
    """One JSON-lines file of runs per (profile, level) in ``out_dir``, and
    ``summary.json``. The seeds are spread over the environments."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    env_paths = [str(p) for p in env_paths]
    tasks = []
    for prof in profiles:
        for lvl in levels:
            block = {"profile": prof, "density_level": int(lvl), **(extra_block or {})}
            per_env = [[] for _ in env_paths]
            for k in range(int(n_seeds)):
                per_env[k % len(env_paths)].append(first_seed + k)
            for ep, seeds in zip(env_paths, per_env):
                if seeds:
                    tasks.append((ep, block, seeds))
    t0 = time.time()
    if jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=int(jobs)) as ex:
            results = list(ex.map(_job, tasks, chunksize=1))
    else:
        results = [_job(t) for t in tasks]
    by: dict[tuple, list] = {}
    for res in results:
        by.setdefault((res["profile"], res["level"]), []).extend(
            dict(r, environment=res["environment"]) for r in res["runs"])
    summary = {}
    for (prof, lvl), runs in sorted(by.items()):
        with open(out_dir / f"runs_{prof}_L{lvl}.jsonl", "w", encoding="utf-8") as f:
            for r in runs:
                f.write(json.dumps(r) + "\n")
        summary[f"{prof}/L{lvl}"] = summarise(runs)
        if progress:
            s = summary[f"{prof}/L{lvl}"]
            progress(f"{prof} L{lvl}: {len(runs)} runs, density {s['density_band_mean']:.2f} per ha, "
                     f"retention {s['retention']:.3f}, within 30 m {s['share_30_mean']:.2f}, "
                     f"Clark-Evans {s['clark_evans_median']}")
    (out_dir / "summary.json").write_text(json.dumps(
        {"seconds": round(time.time() - t0, 1), "environments": [Path(p).stem for p in env_paths],
         "n_seeds": int(n_seeds), "summary": summary}, indent=1), encoding="utf-8")
    return out_dir / "summary.json"


def summarise(runs: list) -> dict:
    """Means over runs: realised against expected, the retention of the
    spacing rule, the pattern statistics, the class shares."""
    n = np.array([r["n"] for r in runs], dtype=float)
    exp = np.array([r["expected"] for r in runs], dtype=float)
    removed = np.array([r["removed"] for r in runs], dtype=float)
    noroom = np.array([r["no_room"] for r in runs], dtype=float)
    dens = np.array([r["density_band"] or 0.0 for r in runs], dtype=float)
    ce = [r["clark_evans"] for r in runs if r.get("clark_evans") is not None]
    sh = [r["share_30"] for r in runs if r.get("share_30") is not None]
    dist = np.array([d for r in runs for d in r.get("distances", [])], dtype=float)
    cls: dict[str, float] = {c: 0.0 for c in CLASSES}
    ecls: dict[str, float] = {c: 0.0 for c in CLASSES}
    for r in runs:
        for c, v in (r.get("per_class") or {}).items():
            cls[c] += v
        for c, v in (r.get("expected_per_class") or {}).items():
            ecls[c] += v
    tt = sum(cls[c] for c in TARGETS) or 1.0
    te = sum(ecls[c] for c in TARGETS) or 1.0
    tot = float(n.sum())
    out = {
        "runs": len(runs), "n_mean": float(n.mean()), "expected_mean": float(exp.mean()),
        "n_over_expected": float(n.sum() / exp.sum()) if exp.sum() > 0 else None,
        "retention": float(tot / (tot + removed.sum() + noroom.sum())) if tot + removed.sum() > 0 else 1.0,
        "density_band_mean": float(dens.mean()),
        "dispersion": float(n.var(ddof=1) / n.mean()) if len(n) > 1 and n.mean() > 0 else None,
        "share_30_mean": float(np.mean(sh)) if sh else None,
        "clark_evans_median": round(float(np.median(ce)), 3) if ce else None,
        "clark_evans_mean": round(float(np.mean(ce)), 3) if ce else None,
        "nn_median_m": round(float(np.median([r["nn_median_m"] for r in runs if "nn_median_m" in r])), 2)
        if any("nn_median_m" in r for r in runs) else None,
        "grouped_share": float(sum(r["grouped"] for r in runs) / tot) if tot > 0 else None,
        "target_share": float(sum(r["targets"] for r in runs) / tot) if tot > 0 else None,
        "class_shares": {c: round(cls[c] / tt, 4) for c in TARGETS},
        "expected_class_shares": {c: round(ecls[c] / te, 4) for c in TARGETS},
        "distance_quantiles_m": ({q: round(float(np.quantile(dist, q)), 2) for q in (0.1, 0.25, 0.5, 0.75, 0.9)}
                                 if len(dist) else None),
    }
    for L in ("5", "10", "20"):
        v = [r[f"singles_share_{L}m"] for r in runs if f"singles_share_{L}m" in r]
        out[f"singles_share_{L}m"] = round(float(np.mean(v)), 3) if v else None
    return out


def main(a) -> int:
    envs = sorted(Path(a.envs).expanduser().glob("*.npz"))
    if not envs:
        print(f"no .npz environment in {a.envs}")
        return 2
    p = default_parameters()
    profiles = [s for s in a.profiles.split(",") if s.strip()] or [q for q in p.profiles]
    levels = [int(v) for v in a.levels.split(",") if v.strip()]
    out = run_statistics(envs, profiles, levels, int(a.seeds), a.out, jobs=int(a.jobs),
                         progress=lambda m: print(m, flush=True))
    print(out)
    return 0
