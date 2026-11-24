"""The batch writer: the specifications and the world list of a split, from a
list of environments. Dataset recording starts from its output, with no hand
work:

    python3 -m blueboat_harbour_population batch --envs environments.yaml --split dataset \
        --levels 2,5,8 --worlds 30 --out batches/dataset_a

The list of environments (``blueboat_population_batch/1``)::

    format: blueboat_population_batch/1
    name: dataset_a
    environments:
      - name: basin_fishing                 # a specification file: the frozen campaign's basin generator ...
        spec: specs/basin.yaml
        profile: fishing_harbour
      - name: provincetown_t1               # ... or one with an `environment:` block (a real depth file)
        spec: specs/provincetown_t1.yaml
        harbour: provincetown               # checked against the harbour split
        profile: trap_harbour               # default: the harbour's profile
    placement: {classes: [...], mix: realistic}      # optional: keys for every world

It writes, into the output folder:

* one specification per environment, with one case per density level
  (``l2``, ``l5``, ``l8``) and its ``placement:`` block (split, profile,
  harbour);
* ``batch.yaml``: every world (environment, level, seed) and the ``build``
  command of each specification with its seeds.

World ``k`` takes the seed ``first seed + k`` of the split's range (so the
seeds of two splits never meet), the environment ``k mod E`` and the level
``(k div E) mod L``: every environment sees every level. The world seed and
the placement seed are the same number.

Pure Python (PyYAML).
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Sequence

import yaml

from .params import PlacementParams, harbour_profile
from .splits import SplitError, check_split, seed_range

BATCH_FORMAT = "blueboat_population_batch/1"
_ENV_KEYS = {"name", "spec", "harbour", "profile", "placement"}


def write_batch(envs_file: str | Path, out_dir: str | Path, *, split: str, levels: Sequence[int], worlds: int,
                first_seed: int | None = None) -> dict:
    """Write the batch; ``{manifest, specs, worlds, commands}``. Raises when
    a harbour belongs to another split or the seeds leave the split's range."""
    envs_file = Path(envs_file).expanduser().resolve()
    doc = yaml.safe_load(envs_file.read_text(encoding="utf-8")) or {}
    if doc.get("format") != BATCH_FORMAT:
        raise ValueError(f"{envs_file}: format {doc.get('format')!r} (expected {BATCH_FORMAT})")
    envs = list(doc.get("environments") or [])
    if not envs:
        raise ValueError(f"{envs_file}: no environment")
    names = [str(e.get("name", "")) for e in envs]
    if not all(names) or len(set(names)) != len(names):
        raise ValueError(f"{envs_file}: every environment needs its own name, got {names}")
    levels = [int(v) for v in levels]
    if not levels or any(not 1 <= v <= 10 for v in levels):
        raise ValueError(f"levels {levels}: whole numbers from 1 to 10")
    lo, hi = seed_range(split)
    first = lo if first_seed is None else int(first_seed)
    n = int(worlds)
    if first < lo or first + n - 1 > hi:
        raise SplitError(f"seeds {first}..{first + n - 1} leave the {split} range {lo}..{hi}")
    out_dir = Path(out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    base_placement = dict(doc.get("placement") or {})
    specs: dict[str, dict] = {}
    for e in envs:
        bad = set(e) - _ENV_KEYS
        if bad:
            raise KeyError(f"environment {e.get('name')!r}: unknown key(s) {sorted(bad)} (known: {sorted(_ENV_KEYS)})")
        src = Path(str(e["spec"])).expanduser()
        if not src.is_absolute():
            src = (envs_file.parent / src).resolve()
        sdoc = yaml.safe_load(src.read_text(encoding="utf-8")) or {}
        harbour = str(e.get("harbour", "") or "")
        check_split(first, split, harbour)                     # the harbour belongs to this split
        blk = {**dict(sdoc.get("placement") or {}), **base_placement, **dict(e.get("placement") or {})}
        blk["mode"] = "mission_like"
        blk["split"] = split
        if harbour:
            blk["harbour"] = harbour
        prof = e.get("profile") or (harbour_profile(harbour) if harbour else None) or blk.get("profile")
        if prof:
            blk["profile"] = str(prof)
        blk.pop("seed", None)                                  # the world seed is the placement seed
        blk.pop("density_level", None)
        PlacementParams.from_dict({**blk, "density_level": levels[0], "seed": first})   # validates
        sdoc = copy.deepcopy(sdoc)
        sdoc["name"] = f"{doc.get('name', 'batch')}_{e['name']}"
        sdoc["placement"] = blk
        sdoc["cases"] = [{"name": f"l{v}", "placement": {"density_level": v}} for v in levels]
        # a path or a depth file named beside the source specification stays reachable
        for key in ("path",):
            v = Path(str(sdoc.get(key, "")))
            if v.suffix == ".yaml" and not v.is_absolute() and (src.parent / v).is_file():
                sdoc[key] = str((src.parent / v).resolve())
        env_blk = sdoc.get("environment")
        if isinstance(env_blk, dict) and env_blk.get("depth_file"):
            v = Path(str(env_blk["depth_file"])).expanduser()
            if not v.is_absolute():
                env_blk["depth_file"] = str((src.parent / v).resolve())
        specs[str(e["name"])] = sdoc
    world_list = []
    per: dict[tuple, list] = {}
    for k in range(n):
        ename = names[k % len(names)]
        level = levels[(k // len(names)) % len(levels)]
        seed = first + k
        world_list.append({"index": k, "environment": ename, "specification": f"{ename}.yaml",
                           "case": f"l{level}", "density_level": level, "seed": seed,
                           "world": f"{specs[ename]['name']}-l{level}-s{seed}"})
        per.setdefault((ename, f"l{level}"), []).append(seed)
    files = {}
    for ename, sdoc in specs.items():
        f = out_dir / f"{ename}.yaml"
        f.write_text(yaml.safe_dump(sdoc, sort_keys=False, default_flow_style=None), encoding="utf-8")
        files[ename] = str(f)
    commands = []
    for (ename, case), seeds in sorted(per.items()):
        commands.append(f"python3 -m blueboat_harbour_population build --spec {files[ename]} --cases {case} "
                        f"--seeds {','.join(str(s) for s in seeds)} --out <worlds root>")
    manifest: dict[str, Any] = {
        "format": "blueboat_population_batch_list/1", "name": str(doc.get("name", "batch")), "split": split,
        "seed_range": [lo, hi], "first_seed": first, "levels": levels, "environments_file": str(envs_file),
        "specifications": files, "worlds": world_list, "commands": commands}
    mf = out_dir / "batch.yaml"
    mf.write_text(yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None), encoding="utf-8")
    return {"manifest": str(mf), "specs": files, "worlds": world_list, "commands": commands}
