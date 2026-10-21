"""``sss_calibration`` -- registry, manifests, quality, artifacts.

    python3 -m blueboat_sss_sim.calibration registry --check
    python3 -m blueboat_sss_sim.calibration registry --markdown
    python3 -m blueboat_sss_sim.calibration validate-manifest M.yaml \\
        [--root DIR] [--check-files] [--verify-hashes] [--json OUT]
    python3 -m blueboat_sss_sim.calibration quality --out DIR F.svlog ...
    python3 -m blueboat_sss_sim.calibration quality --out DIR \\
        --manifest M.yaml [--root DIR]
    python3 -m blueboat_sss_sim.calibration validate-artifact A.yaml
    python3 -m blueboat_sss_sim.calibration effective-profile \\
        [--profile P.yaml] [--artifact A.yaml] [--session S] \\
        [--set acquisition.range_length_mm=30000] --out F.yaml
    python3 -m blueboat_sss_sim.calibration identifiability \\
        --param model.tvg_compensation --param model.lambert_exponent \\
        --altitude 1.40 --altitude 7.33 [--out DIR]
    python3 -m blueboat_sss_sim.calibration example {artifact,manifest}

Exit status: 0 ok, 1 invalid input / drift / unusable data, 2 usage.
Everything is read-only on its inputs; outputs go only to ``--out`` /
``--json``, which must not lie inside a recording's directory tree.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml


def _refuse_inside(out: Path, inputs: list[Path]) -> None:
    out = out.resolve()
    for p in inputs:
        d = p.resolve().parent
        if out == d or d in out.parents:
            raise SystemExit(f"--out {out} lies inside the recordings' tree "
                             f"({d}); field data are write-once (CM-7)")


def _dump(obj, path: str | None) -> None:
    text = json.dumps(obj, indent=2, default=str)
    if path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(text + "\n", encoding="utf-8")
    else:
        print(text)


def _cmd_registry(a) -> int:
    from .registry import drift_report, load_registry, registry_markdown
    reg = load_registry(a.registry)
    if a.markdown:
        print(registry_markdown(reg), end="")
        return 0
    rep = drift_report(reg, a.profile)
    _dump(rep, a.json)
    print(f"registry: {rep['checked']} parameters, "
          f"{len(rep['drift'])} drifted, {len(rep['implicit'])} implicit "
          "in the profile", file=sys.stderr)
    return 0 if rep["ok"] else 1


def _cmd_validate_manifest(a) -> int:
    from .manifest import load_manifest, validate_manifest
    doc = load_manifest(a.manifest)
    root = Path(a.root) if a.root else Path(a.manifest).resolve().parent
    rep = validate_manifest(doc, root=root, check_files=a.check_files
                            or a.verify_hashes,
                            verify_hashes=a.verify_hashes)
    _dump(rep.to_dict(), a.json)
    return 0 if rep.ok else 1


def _cmd_quality(a) -> int:
    from .manifest import load_manifest
    from .quality import svlog_quality
    files: list[Path] = [Path(f) for f in a.svlog]
    if a.manifest:
        doc = load_manifest(a.manifest)
        root = Path(a.root) if a.root else Path(a.manifest).resolve().parent
        for s in doc.get("sessions") or []:
            for r in s.get("recordings") or []:
                if r.get("kind") == "svlog":
                    files.append(root / str(r["path"]))
    if not files:
        raise SystemExit("no .svlog given (positional or --manifest)")
    out = Path(a.out)
    _refuse_inside(out, files)
    out.mkdir(parents=True, exist_ok=True)
    reports, missing = [], []
    for f in files:
        if not f.exists():
            missing.append(f.name)
            continue
        reports.append(svlog_quality(f, with_hash=a.hash))
    summary = {"format": "blueboat_svlog_quality_summary/1",
               "files": [{"file": r["file"], "usable": r["usable"],
                          "flags": [x["code"] for x in r["flags"]
                                    if x["level"] != "info"]}
                         for r in reports],
               "unavailable": missing}
    for r in reports:
        (out / (Path(r["file"]).stem + ".quality.json")).write_text(
            json.dumps(r, indent=2, default=str) + "\n", encoding="utf-8")
    (out / "quality_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    for r in reports:
        print(f"{r['file']}: {'usable' if r['usable'] else 'NOT USABLE'}; "
              + "; ".join(f"{x['level']}:{x['code']}" for x in r["flags"]))
    if missing:
        print(f"unavailable: {', '.join(missing)}")
    return 0 if reports and all(r["usable"] for r in reports) \
        and not missing else 1


def _cmd_validate_artifact(a) -> int:
    from .artifact import load_artifact, validate_artifact
    rep = validate_artifact(load_artifact(a.artifact))
    _dump({"ok": rep.ok, "errors": rep.errors, "warnings": rep.warnings},
          a.json)
    return 0 if rep.ok else 1


def _parse_set(items: list[str]) -> dict:
    out = {}
    for it in items or []:
        k, sep, v = it.partition("=")
        if not sep:
            raise SystemExit(f"--set {it!r}: expected name=value")
        out[k.strip()] = yaml.safe_load(v)
    return out


def _cmd_effective(a) -> int:
    from .artifact import (effective_config, load_artifact,
                           write_effective_profile)
    art = load_artifact(a.artifact) if a.artifact else None
    eff = effective_config(a.profile, art, a.session, _parse_set(a.set))
    write_effective_profile(eff, a.out, sources={
        "profile": str(a.profile or "default_sonar.yaml"),
        "artifact": (art or {}).get("id"), "session": a.session,
        "run_overrides": _parse_set(a.set)})
    changed = {k: v for k, v in eff.provenance.items()
               if v.startswith("artifact") or v == "run"}
    print(f"wrote {a.out}; {len(changed)} value(s) from the artifact/run, "
          f"{len(eff.not_applied)} artifact value(s) outside the sonar "
          "profile (not applied here)")
    return 0


def _cmd_identifiability(a) -> int:
    from .identifiability import analyse
    res = analyse(a.param, a.altitude, sigma_db=a.sigma_db)
    d = res.to_dict()
    if a.out:
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        _dump(d, str(out / "identifiability.json"))
    else:
        _dump(d, None)
    print(f"condition number {res.condition_number:.3g}; ridges "
          f"{res.ridges or 'none'}", file=sys.stderr)
    return 0


def _cmd_example(a) -> int:
    from .artifact import example_artifact
    from .manifest import example_manifest
    doc = example_artifact() if a.what == "artifact" else example_manifest()
    print(yaml.safe_dump(doc, sort_keys=False), end="")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="sss_calibration", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("registry", help="check the registry against the code")
    r.add_argument("--registry", default=None)
    r.add_argument("--profile", default=None)
    r.add_argument("--check", action="store_true",
                   help="(default) drift report; exit 1 on drift")
    r.add_argument("--markdown", action="store_true")
    r.add_argument("--json", default=None)
    r.set_defaults(fn=_cmd_registry)
    m = sub.add_parser("validate-manifest")
    m.add_argument("manifest")
    m.add_argument("--root", default=None)
    m.add_argument("--check-files", action="store_true")
    m.add_argument("--verify-hashes", action="store_true")
    m.add_argument("--json", default=None)
    m.set_defaults(fn=_cmd_validate_manifest)
    q = sub.add_parser("quality")
    q.add_argument("svlog", nargs="*")
    q.add_argument("--manifest", default=None)
    q.add_argument("--root", default=None)
    q.add_argument("--out", required=True)
    q.add_argument("--hash", action="store_true")
    q.set_defaults(fn=_cmd_quality)
    v = sub.add_parser("validate-artifact")
    v.add_argument("artifact")
    v.add_argument("--json", default=None)
    v.set_defaults(fn=_cmd_validate_artifact)
    e = sub.add_parser("effective-profile")
    e.add_argument("--profile", default=None)
    e.add_argument("--artifact", default=None)
    e.add_argument("--session", default=None)
    e.add_argument("--set", action="append", default=[])
    e.add_argument("--out", required=True)
    e.set_defaults(fn=_cmd_effective)
    i = sub.add_parser("identifiability")
    i.add_argument("--param", action="append", required=True)
    i.add_argument("--altitude", action="append", type=float, required=True)
    i.add_argument("--sigma-db", type=float, default=0.67)
    i.add_argument("--out", default=None)
    i.set_defaults(fn=_cmd_identifiability)
    x = sub.add_parser("example")
    x.add_argument("what", choices=("artifact", "manifest"))
    x.set_defaults(fn=_cmd_example)
    a = ap.parse_args(argv)
    return int(a.fn(a))


if __name__ == "__main__":
    sys.exit(main())
