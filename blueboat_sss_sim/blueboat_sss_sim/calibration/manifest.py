"""Session manifests: what a field dataset is, and which part may fit.

A manifest (``blueboat_session_manifest/1``, YAML) lists the sessions of a
field campaign -- each a whole outing at one site and regime -- with their
recordings (relative paths, kinds, optional SHA-256), the clock regime, the
regime labels a result will be stratified by, quality flags, and the ROLE
the session plays: ``calibration`` (may be fitted on), ``validation`` or
``test`` (never fitted on), or ``exploratory`` (looked at, not scored).

The rule it enforces: **split by whole session, never by ping.** Adjacent
pings of one pass overlap and share the seabed, the boat's motion and the
device state, so a random ping split leaks the answer into the test set.
Sessions that share a ``group`` (same site and day, say) must share a role.
A recording, or a file hash, that appears under two roles is an error.

The manifest never contains the data, and it never needs to be next to it:
paths resolve against ``root`` (the manifest's directory by default, or an
explicit ``--root``), and a missing file is reported as unavailable, not
invented. ROS-free; stdlib + YAML.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml

MANIFEST_FORMAT = "blueboat_session_manifest/1"
ROLES = ("calibration", "validation", "test", "exploratory")
KINDS = ("svlog", "rosbag", "poslog", "gcs_session", "npy", "other")
_SESSION_KEYS = {"id", "role", "site", "date", "group", "regime", "platform",
                 "clock", "recordings", "notes", "quality_flags"}
_RECORDING_KEYS = {"path", "kind", "sha256", "size_bytes", "purpose",
                   "quality_flags", "exclude", "notes"}
#: How a session's recordings are rendered for scoring (``regime.forward_model``).
#: ``flat_sand`` (the default when absent) is the pass-1/2 seabed engine's flat sand
#: at the stratum altitude; ``scene`` is a session scored against its own world
#: (pass 3, the pool), which the flat-sand engine must never pick up.
FORWARD_MODELS = ("flat_sand", "scene")


def forward_model(session: Mapping[str, Any]) -> str:
    """The session's forward model, ``flat_sand`` when the regime does not say."""
    reg = session.get("regime") or {}
    fm = reg.get("forward_model") if isinstance(reg, Mapping) else None
    return str(fm) if fm else "flat_sand"


@dataclass
class ManifestReport:
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    unavailable: list[str] = field(default_factory=list)
    hashed: dict[str, str] = field(default_factory=dict)
    roles: dict[str, list[str]] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict:
        return {"ok": self.ok, "errors": self.errors,
                "warnings": self.warnings, "unavailable": self.unavailable,
                "roles": self.roles, "hashed": self.hashed}


def load_manifest(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    if not isinstance(doc, Mapping):
        raise ValueError(f"{path}: not a mapping")
    return dict(doc)


def sha256_file(path: str | Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(chunk), b""):
            h.update(block)
    return h.hexdigest()


def validate_manifest(doc: Mapping[str, Any], *, root: str | Path | None = None,
                      check_files: bool = False,
                      verify_hashes: bool = False) -> ManifestReport:
    """Structure, roles and leakage; with ``check_files`` also which
    recordings exist under ``root`` (missing ones are listed in
    ``unavailable``, an error only if ``verify_hashes`` needs them)."""
    rep = ManifestReport()
    e, w = rep.errors, rep.warnings
    if doc.get("format") != MANIFEST_FORMAT:
        e.append(f"format must be {MANIFEST_FORMAT!r}")
    sessions = doc.get("sessions")
    if not isinstance(sessions, list) or not sessions:
        e.append("sessions must be a non-empty list")
        return rep
    root_p = Path(root) if root is not None else None
    ids: set[str] = set()
    group_role: dict[str, tuple[str, str]] = {}
    path_role: dict[str, tuple[str, str]] = {}
    sha_role: dict[str, tuple[str, str]] = {}
    for i, s in enumerate(sessions):
        where = f"sessions[{i}]"
        if not isinstance(s, Mapping):
            e.append(f"{where}: must be a mapping")
            continue
        sid = str(s.get("id", "")).strip()
        where = f"session {sid or i}"
        if not sid:
            e.append(f"{where}: id required")
        elif sid in ids:
            e.append(f"{where}: duplicate id")
        ids.add(sid)
        unknown = sorted(set(s) - _SESSION_KEYS)
        if unknown:
            e.append(f"{where}: unknown keys {unknown}")
        role = s.get("role")
        if role not in ROLES:
            e.append(f"{where}: role must be one of {ROLES}")
            continue
        rep.roles.setdefault(str(role), []).append(sid)
        if s.get("date") is not None:
            try:
                _dt.date.fromisoformat(str(s["date"]))
            except ValueError:
                e.append(f"{where}: date must be YYYY-MM-DD")
        grp = str(s.get("group") or sid)
        if grp in group_role and group_role[grp][0] != role:
            e.append(f"{where}: group {grp!r} is {role} here but "
                     f"{group_role[grp][0]} in session {group_role[grp][1]} "
                     "-- sessions of one group share a role (no leakage "
                     "across a site-day)")
        group_role.setdefault(grp, (str(role), sid))
        if not s.get("regime"):
            w.append(f"{where}: no regime labels (altitude band, bottom, "
                     "gain mode, sea state) -- results cannot be stratified")
        elif forward_model(s) not in FORWARD_MODELS:
            e.append(f"{where}: regime.forward_model must be one of "
                     f"{FORWARD_MODELS}")
        recs = s.get("recordings") or []
        if not recs:
            w.append(f"{where}: no recordings")
        for j, r in enumerate(recs):
            rw = f"{where} recordings[{j}]"
            if not isinstance(r, Mapping) or not r.get("path"):
                e.append(f"{rw}: needs a path")
                continue
            unknown = sorted(set(r) - _RECORDING_KEYS)
            if unknown:
                e.append(f"{rw}: unknown keys {unknown}")
            rel = str(r["path"])
            if Path(rel).is_absolute() or ".." in Path(rel).parts:
                e.append(f"{rw}: path must be relative to the manifest root "
                         "(no absolute paths, no '..')")
            if r.get("kind") not in KINDS:
                e.append(f"{rw}: kind must be one of {KINDS}")
            if rel in path_role and path_role[rel][0] != role:
                e.append(f"{rw}: {rel} is also in {path_role[rel][0]} "
                         f"session {path_role[rel][1]} (leakage)")
            path_role.setdefault(rel, (str(role), sid))
            sha = r.get("sha256")
            if sha:
                if sha in sha_role and sha_role[sha][0] != role:
                    e.append(f"{rw}: same content (sha256) as a "
                             f"{sha_role[sha][0]} recording in session "
                             f"{sha_role[sha][1]} (leakage)")
                sha_role.setdefault(str(sha), (str(role), sid))
            if check_files and root_p is not None:
                f = root_p / rel
                if not f.exists():
                    rep.unavailable.append(rel)
                    if verify_hashes and sha:
                        e.append(f"{rw}: cannot verify sha256, file missing")
                elif verify_hashes:
                    got = sha256_file(f)
                    rep.hashed[rel] = got
                    if sha and got != sha:
                        e.append(f"{rw}: sha256 mismatch -- the recording "
                                 "changed since the manifest was written")
    if "calibration" not in rep.roles:
        w.append("no calibration session")
    if not (set(rep.roles) & {"validation", "test"}):
        w.append("no validation or test session: nothing is held out")
    return rep


def sessions_by_role(doc: Mapping[str, Any]) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for s in doc.get("sessions") or []:
        out.setdefault(str(s.get("role")), []).append(str(s.get("id")))
    return out


def example_manifest() -> dict:
    """The shape of a manifest (documentation and tests); the paths are
    illustrative and need not exist."""
    return {
        "format": MANIFEST_FORMAT,
        "dataset": {"name": "example", "owner": "operator",
                    "description": "two sessions, one held out"},
        "sessions": [
            {"id": "S1", "role": "calibration", "site": "beach_A",
             "date": "2026-11-02", "group": "beach_A/2026-11-02",
             "regime": {"altitude_band_m": [1.0, 2.0], "bottom": "sand",
                        "bottom_label_source": "grab sample",
                        "gain_mode": "fixed_4", "sea_state": "calm",
                        "range_m": 20},
             "clock": {"sonar": "device uptime", "nav": "autopilot boot",
                       "frozen_time_boot": False},
             "recordings": [{"path": "S1/pass01.svlog", "kind": "svlog",
                             "purpose": "seabed_level"}]},
            {"id": "S2", "role": "validation", "site": "beach_A",
             "date": "2026-11-09", "group": "beach_A/2026-11-09",
             "regime": {"altitude_band_m": [1.0, 2.0], "bottom": "sand",
                        "gain_mode": "fixed_4"},
             "recordings": [{"path": "S2/pass01.svlog", "kind": "svlog"}]},
        ],
    }
