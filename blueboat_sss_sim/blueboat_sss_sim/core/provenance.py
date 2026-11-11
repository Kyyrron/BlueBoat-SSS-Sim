"""Run provenance: which code, profile and world produced a simulated session.

Why (independent review, 2026-11-11): the package installs by COPY, so a
session can run code older than the source tree, and nothing in a recorded
session said which renderer, profile, world or build produced it. This module
gives the pieces the sonar node publishes on ``~/ground_truth/run_info``:

* a **code digest**: one sha256 over the package's ``*.py`` and the profiles
  ``config/*.yaml``, by logical path, so the source tree and an installed copy
  hash alike when they hold the same code;
* the **build stamp** ``_build_stamp.json`` that ``setup.py`` writes into the
  build (never into the source tree): when it was built, from which source
  root, at which git state and with which digest;
* :func:`source_check`: the installed code against the source root it was
  built from, when that root still exists on this machine -- the files that
  differ are named, which is what tells a stale install from a current one;
* digests of a profile file and of a world folder.

Standard library only (no numpy, no ROS, no Qt), so ``setup.py`` can load it
by path at build time.

CLI::

    python3 -m blueboat_sss_sim.core.provenance --check      # exit 1 when stale
    python3 -m blueboat_sss_sim.core.provenance --json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

STAMP_NAME = "_build_stamp.json"
PACKAGE = "blueboat_sss_sim"
#: the world-folder files that decide what the sonar renders
WORLD_FILES = ("scene_manifest.yaml", "scene.npz", "depth.npz")


# ------------------------------------------------------------------ hashing
def file_sha256(path: str | Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def package_dir() -> Path:
    """The running ``blueboat_sss_sim`` package directory."""
    return Path(__file__).resolve().parents[1]


def config_dir(pkg: Path | None = None) -> Path | None:
    """The profiles beside a package directory: ``<source root>/config`` in a
    checkout, ``<prefix>/share/blueboat_sss_sim/config`` in an install."""
    pkg = package_dir() if pkg is None else Path(pkg)
    src = pkg.parent / "config"
    if src.is_dir():
        return src
    for up in pkg.parents:
        cand = up / "share" / PACKAGE / "config"
        if cand.is_dir():
            return cand
    return None


def tree_files(pkg: Path | None = None, cfg: Path | None = None) -> dict[str, str]:
    """``{logical path: sha256}`` of the code that decides a render: every
    ``*.py`` of the package (``blueboat_sss_sim/...``) and every profile
    (``config/*.yaml``). Byte-compiled files and the build stamp are not
    code."""
    pkg = package_dir() if pkg is None else Path(pkg)
    cfg = config_dir(pkg) if cfg is None else Path(cfg)
    out: dict[str, str] = {}
    for p in sorted(pkg.rglob("*.py")):
        if "__pycache__" in p.parts:
            continue
        out[f"{PACKAGE}/{p.relative_to(pkg).as_posix()}"] = file_sha256(p)
    if cfg is not None:
        for p in sorted(cfg.glob("*.yaml")):
            out[f"config/{p.name}"] = file_sha256(p)
    return out


def digest_of(files: dict[str, str]) -> str:
    h = hashlib.sha256()
    for name in sorted(files):
        h.update(name.encode())
        h.update(b"\0")
        h.update(files[name].encode())
        h.update(b"\n")
    return h.hexdigest()


# ---------------------------------------------------------------------- git
def git_state(root: str | Path) -> dict[str, Any]:
    """``{describe, commit, dirty_files}`` of the repository holding
    ``root``, or ``{}`` (no git, not a repository)."""
    def run(*args: str) -> str | None:
        try:
            r = subprocess.run(["git", "-C", str(root), *args], capture_output=True,
                               text=True, timeout=10)
        except (OSError, subprocess.SubprocessError):
            return None
        return r.stdout.strip() if r.returncode == 0 else None

    commit = run("rev-parse", "HEAD")
    if commit is None:
        return {}
    status = run("status", "--porcelain") or ""
    return {"describe": run("describe", "--always", "--dirty", "--abbrev=10") or commit[:10],
            "commit": commit,
            "dirty_files": len([ln for ln in status.splitlines() if ln.strip()])}


# -------------------------------------------------------------- build stamp
def make_stamp(source_root: str | Path, version: str = "") -> dict[str, Any]:
    """The build stamp of ``source_root`` (the directory holding ``setup.py``,
    the package and ``config/``), as ``setup.py`` writes it."""
    root = Path(source_root).resolve()
    files = tree_files(root / PACKAGE, root / "config")
    return {"schema": "blueboat_sss_sim_build/1",
            "built_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "source_root": str(root),
            "package_version": version,
            "git": git_state(root),
            "source_digest": digest_of(files),
            "files": len(files)}


def build_info() -> dict[str, Any]:
    """What the running code was built from: the build stamp of an installed
    copy, else (a source checkout, a symlink install) the tree itself, read
    now. ``installed`` says which."""
    pkg = package_dir()
    stamp = pkg / STAMP_NAME
    if stamp.is_file():
        try:
            doc = json.loads(stamp.read_text())
            doc["installed"] = True
            return doc
        except (OSError, ValueError):
            pass
    root = pkg.parent
    return {"schema": "blueboat_sss_sim_build/1", "installed": False,
            "built_utc": None, "source_root": str(root),
            "git": git_state(root), "source_digest": digest_of(tree_files(pkg))}


def source_check() -> dict[str, Any]:
    """The running code against the source tree it was built from.

    ``matches``: True (same code), False (they differ: ``differing``,
    ``only_in_source``, ``only_in_install`` name the files), or None when it
    cannot be known (no stamp and not a checkout, or the source root is not
    on this machine). A source checkout run in place matches by definition.
    """
    pkg = package_dir()
    info = build_info()
    here = tree_files(pkg)
    out: dict[str, Any] = {"matches": None, "running_digest": digest_of(here),
                           "source_root": info.get("source_root"),
                           "differing": [], "only_in_source": [], "only_in_install": []}
    if not info.get("installed"):
        out["matches"] = True
        return out
    root = Path(str(info.get("source_root") or ""))
    if not (root / PACKAGE).is_dir():
        return out
    there = tree_files(root / PACKAGE, root / "config")
    out["source_digest_now"] = digest_of(there)
    out["differing"] = sorted(k for k in here if k in there and here[k] != there[k])
    out["only_in_source"] = sorted(k for k in there if k not in here)
    out["only_in_install"] = sorted(k for k in here if k not in there)
    out["matches"] = not (out["differing"] or out["only_in_source"]
                          or out["only_in_install"])
    return out


def stale_message(check: dict[str, Any], limit: int = 8) -> str:
    """One line for a log: what differs and how to rebuild ('' when current
    or unknown)."""
    if check.get("matches") is not False:
        return ""
    names = (check["differing"] + [f"{n} (not installed)" for n in check["only_in_source"]]
             + [f"{n} (deleted from the source)" for n in check["only_in_install"]])
    more = f" and {len(names) - limit} more" if len(names) > limit else ""
    return ("the INSTALLED blueboat_sss_sim is not the source tree "
            f"({check.get('source_root')}): {', '.join(names[:limit])}{more}. "
            "This run executes the old code. Rebuild: cd ~/ros2_ws && rm -rf "
            "build/blueboat_sss_sim install/blueboat_sss_sim && colcon build "
            "--packages-select blueboat_sss_sim")


# -------------------------------------------------------- profile and world
def profile_info(path: str | Path | None) -> dict[str, Any]:
    """``{path, sha256}`` of a sonar profile file ('' = no file)."""
    if not path:
        return {"path": "", "sha256": None}
    p = Path(path).expanduser()
    try:
        return {"path": str(p), "sha256": file_sha256(p)}
    except OSError:
        return {"path": str(p), "sha256": None}


def world_digest(world_dir: str | Path | None) -> dict[str, Any]:
    """``{dir, sha256, files}`` of the world-folder files that decide what the
    sonar renders (those present)."""
    if not world_dir:
        return {"dir": "", "sha256": None, "files": {}}
    d = Path(world_dir).expanduser()
    files = {}
    for name in WORLD_FILES:
        p = d / name
        if p.is_file():
            files[name] = file_sha256(p)
    return {"dir": str(d), "sha256": digest_of(files) if files else None, "files": files}


# ---------------------------------------------------------------------- CLI
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="provenance", description="Build stamp of the blueboat_sss_sim in use, and "
        "whether it is the source tree it was built from.")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 when the installed code differs from its source tree")
    ap.add_argument("--json", action="store_true", help="print everything as JSON")
    args = ap.parse_args(argv)
    info, chk = build_info(), source_check()
    if args.json:
        print(json.dumps({"build": info, "source_check": chk}, indent=1))
    else:
        git = info.get("git") or {}
        print(f"package dir   : {package_dir()}")
        print(f"installed copy: {bool(info.get('installed'))}"
              + (f" (built {info.get('built_utc')})" if info.get("installed") else ""))
        print(f"source root   : {info.get('source_root')}")
        print(f"git           : {git.get('describe', 'n/a')}"
              + (f" ({git.get('dirty_files')} dirty files)" if git else ""))
        print(f"code digest   : {chk['running_digest']}")
        if chk["matches"] is True:
            print("source check  : OK, the running code is the source tree")
        elif chk["matches"] is None:
            print("source check  : unknown (the source root is not on this machine)")
        else:
            print("source check  : STALE -- " + stale_message(chk, limit=40))
    return 1 if (args.check and chk["matches"] is False) else 0


if __name__ == "__main__":
    sys.exit(main())
