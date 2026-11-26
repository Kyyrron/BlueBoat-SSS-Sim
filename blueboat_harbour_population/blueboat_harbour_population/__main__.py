"""Command line of the harbour population package.

    python3 -m blueboat_harbour_population <command> ...
    (installed: harbour_population <command> ...)

    check      the model's identity, the parameter file's form, the simulator in use
    populate   plan a population for an environment and print or save the plan (no world is built)
    build      build the worlds of a specification with the frozen world generator
    batch      write the specifications and the world list of a split from a list of environments
    card       write the world card of a built world folder
    stats      many-seed statistics of the model on environments (validation)

A specification is a ``blueboat_campaign/1`` file with a ``placement:`` block
(and optionally ``environment:`` for a depth file). Without ``placement:``,
or with ``placement.mode: legacy``, ``build`` hands it to the frozen
``worldgen_campaign`` unchanged.
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import yaml


def _block_from_args(a) -> dict:
    blk: dict = {"mode": "mission_like"}
    if a.profile:
        blk["profile"] = a.profile
    if a.classes:
        blk["classes"] = [c.strip() for c in a.classes.split(",") if c.strip()]
    if a.mix:
        blk["mix"] = a.mix
    if a.level is not None:
        blk["density_level"] = int(a.level)
    if a.seed is not None:
        blk["seed"] = int(a.seed)
    if a.split:
        blk["split"] = a.split
    if a.harbour:
        blk["harbour"] = a.harbour
    if a.split_override:
        blk["split_override"] = a.split_override
    if a.structures:
        blk["structures"] = a.structures
    clutter = {}
    if a.no_debris:
        clutter["debris"] = False
    if a.no_small_litter:
        clutter["small_litter"] = False
    if clutter:
        blk["clutter"] = clutter
    for item in a.tag or []:
        fid, _, use = item.partition("=")
        blk.setdefault("feature_tags", {})[fid.strip()] = use.strip()
    return blk


def _load_environment(spec: str, case: str | None, world_seed: int | None):
    """An environment from ``spec``: an ``.npz`` environment file, a world
    folder, ``synthetic`` (a test basin), or a specification file (with
    ``--case`` and ``--world-seed``)."""
    from .environment import Environment, synthetic_basin
    if spec == "synthetic":
        return synthetic_basin(100.0, 80.0, quays=("south", "west"), pontoon=True, mooring=1), None
    p = Path(spec).expanduser()
    if p.suffix == ".npz":
        return Environment.load(p), None
    from . import sim_adapter as sa
    if p.is_dir():
        return sa.environment_from_world(p), None
    doc = sa.load_spec(p)
    cases = [str(c.get("name")) for c in (doc.get("cases") or [{"name": "nominal"}])]
    case = case or cases[0]
    blk = sa.case_placement(doc, case) or {"mode": "mission_like"}
    doc2 = copy.deepcopy(doc)
    doc2["placement"] = {**blk, "density_level": 1}
    _, _, _, env = sa.plan_world(doc2, case, int(world_seed if world_seed is not None else 1))
    return env, blk


def cmd_check(a) -> int:
    from . import __version__
    from .data import code_digest, default_parameters, evidence_digest
    p = default_parameters()
    n = sum(1 for _ in p.records())
    tags = {}
    for _, r in p.records():
        tags[r["tag"]] = tags.get(r["tag"], 0) + 1
    print(f"blueboat_harbour_population {__version__}")
    print(f"code digest     : {code_digest()}")
    print(f"evidence digest : {evidence_digest()}")
    print(f"parameters      : {n} records ({', '.join(f'{k} {v}' for k, v in sorted(tags.items()))}); form ok")
    try:
        from . import sim_adapter as sa
        s = sa.simulator_identity()
        print(f"simulator       : code digest {s['code_digest']}, generator digest {s['generator_digest']}, "
              f"{s['package_dir']}")
    except ImportError as exc:
        print(f"simulator       : not importable ({exc})")
    return 0


def cmd_populate(a) -> int:
    from .model import populate
    env, blk0 = _load_environment(a.env, a.case, a.world_seed)
    blk = {**(blk0 or {}), **_block_from_args(a)}
    plan = populate(env, blk)
    r = plan.report
    if a.out:
        plan.save(a.out)
        print(f"plan: {a.out}")
    if a.save_env:
        env.save(a.save_env)
        print(f"environment: {a.save_env}")
    rz = r["realised"]
    print(f"environment {env.name or '-'} ({env.digest()}): {r['allowed_ha']:.3f} ha of placement water, "
          f"{r['active_band_ha']:.3f} ha in the active band, {len(env.features)} features")
    print(f"level {r['level']}: {r['density_per_ha']:g} per ha -> expected {r['expected']['total']:.1f} "
          f"({r['expected']['targets']:.1f} targets, {r['expected']['debris']:.1f} debris)")
    print(f"placed: {rz['targets']} targets, {rz['debris']} debris, {rz['groups']} groups, "
          f"{rz['structures']} structures; density in the band {rz['density_active_per_ha']} per ha; "
          f"within 30 m {rz['share_within_30m']}; Clark-Evans {rz['clark_evans']}")
    print("per class: " + ", ".join(f"{c} {n}" for c, n in rz["per_class"].items()))
    for wmsg in r["warnings"]:
        print("warning: " + wmsg)
    print(f"plan digest {plan.digest()}")
    if a.report:
        print(yaml.safe_dump(plan.to_dict()["report"], sort_keys=False, default_flow_style=None))
    if a.card:
        from .card import card_markdown, top_view
        from .validation import expected_density
        out = Path(a.card)
        out.mkdir(parents=True, exist_ok=True)
        meta = {"world_name": env.name or "plan", "placement": plan.metadata_block(), "seed": None}
        top_view(out / "world_card.png", meta, environment=env, plan=plan,
                 heat=expected_density(env, blk))
        (out / "world_card.md").write_text(card_markdown(meta, model=plan.model), encoding="utf-8")
        print(f"card: {out / 'world_card.md'}")
    return 0


def cmd_build(a) -> int:
    from . import sim_adapter as sa
    seeds = [int(s) for s in a.seeds.split(",") if s.strip()]
    cases = [c for c in a.cases.split(",") if c.strip()] or None
    if a.environment_only:
        # the world of an environment: block without its litter (for Screen 5, or a later plan)
        doc = sa.load_spec(a.spec)
        spec = sa._campaign_spec(doc, mission=False)
        for case in (cases or [str(c["name"]) for c in spec["cases"]][:1]):
            for seed in seeds:
                name = a.name if (a.name and len(seeds) == 1) else None
                out = sa.build_environment_world(doc, case, seed, a.out, world_name=name,
                                                 progress=lambda m: print(m, flush=True))
                print("environment-only world:", out)
        return 0
    man = sa.run(a.spec, seeds, a.out, cases, cards=not a.no_cards, progress=lambda m: print(m, flush=True))
    for w in man["worlds"]:
        for msg in w.get("warnings") or []:
            print(f"{w['world']}: warning: {msg}")
    print("manifest:", Path(a.out).expanduser() / ".population" / man["name"] / "manifest.yaml")
    return 0


def cmd_batch(a) -> int:
    from .batch import write_batch
    out = write_batch(a.envs, a.out, split=a.split, levels=[int(v) for v in a.levels.split(",")],
                      worlds=int(a.worlds), first_seed=a.first_seed)
    print(f"batch: {out['manifest']}  ({len(out['worlds'])} worlds, {len(out['specs'])} specification(s))")
    for cmd in out["commands"]:
        print(cmd)
    return 0


def cmd_card(a) -> int:
    from .card import write_card
    print(write_card(a.world, a.out))
    return 0


def cmd_stats(a) -> int:
    from .validation import main as vmain
    return vmain(a)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="harbour_population", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="identity and parameter check").set_defaults(fn=cmd_check)

    p = sub.add_parser("populate", help="plan a population (no world is built)")
    p.add_argument("--env", required=True, help="an .npz environment, a world folder, a specification, or "
                                                "'synthetic'")
    p.add_argument("--case", default=None, help="case of the specification (default: the first)")
    p.add_argument("--world-seed", type=int, default=None, help="world seed of the specification's basin")
    p.add_argument("--profile", default=None)
    p.add_argument("--classes", default=None, help="comma-separated target classes (default: all seven)")
    p.add_argument("--mix", default=None, help="realistic | balanced")
    p.add_argument("--level", type=int, default=None, help="density level 1..10 (5: a typical harbour)")
    p.add_argument("--seed", type=int, default=None, help="placement seed")
    p.add_argument("--split", default=None, help="development | dataset | tuning | evaluation")
    p.add_argument("--harbour", default=None)
    p.add_argument("--split-override", default=None, help="a reason: lets a foreign seed or harbour through")
    p.add_argument("--structures", default=None, help="auto | keep | none")
    p.add_argument("--no-debris", action="store_true")
    p.add_argument("--no-small-litter", action="store_true")
    p.add_argument("--tag", action="append", help="feature=use (repeat)")
    p.add_argument("--out", default=None, help="write the plan (YAML)")
    p.add_argument("--save-env", default=None, help="write the environment (.npz)")
    p.add_argument("--report", action="store_true", help="print the whole report")
    p.add_argument("--card", default=None, help="write a world card of the plan into this folder")
    p.set_defaults(fn=cmd_populate)

    p = sub.add_parser("build", help="build the worlds of a specification")
    p.add_argument("--spec", required=True)
    p.add_argument("--seeds", required=True, help="comma-separated world seeds")
    p.add_argument("--out", required=True, help="the worlds root")
    p.add_argument("--cases", default="", help="comma-separated case names (default: all)")
    p.add_argument("--no-cards", action="store_true")
    p.add_argument("--environment-only", action="store_true",
                   help="build the world of the environment: block without litter (its first case, or --cases)")
    p.add_argument("--name", default=None, help="with --environment-only and one seed: the world's name")
    p.set_defaults(fn=cmd_build)

    p = sub.add_parser("batch", help="write the specifications and the world list of a split")
    p.add_argument("--envs", required=True, help="the list of environments (YAML)")
    p.add_argument("--split", required=True)
    p.add_argument("--levels", default="5", help="comma-separated density levels to mix")
    p.add_argument("--worlds", required=True, help="number of worlds")
    p.add_argument("--first-seed", type=int, default=None, help="default: the first seed of the split")
    p.add_argument("--out", required=True, help="folder for the specifications and the list")
    p.set_defaults(fn=cmd_batch)

    p = sub.add_parser("card", help="write the world card of a built world")
    p.add_argument("--world", required=True)
    p.add_argument("--out", default=None)
    p.set_defaults(fn=cmd_card)

    p = sub.add_parser("stats", help="many-seed statistics (validation)")
    p.add_argument("--envs", required=True, help="folder of .npz environments")
    p.add_argument("--profiles", default="")
    p.add_argument("--levels", default="1,5,10")
    p.add_argument("--seeds", type=int, default=200)
    p.add_argument("--out", required=True)
    p.add_argument("--jobs", type=int, default=1)
    p.set_defaults(fn=cmd_stats)

    a = ap.parse_args(argv)
    try:
        return int(a.fn(a))
    except (ValueError, KeyError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
