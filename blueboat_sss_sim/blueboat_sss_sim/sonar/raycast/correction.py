"""The ray-cast sensor's EMPIRICAL correction layer (owner, 2026-11-06; plan
``bubbly-crafting-pixel.md`` Stage 4; gates amendment 6b, 2026-11-07). NOT
physics.

Physical renderer -> this correction -> the shared noise / encoder stage.

The physical model (``renderer.py``) stays intact and describes what it
models. This layer compensates for documented systematic residuals that the
physics does not represent, inside a declared domain
(``docs/engineering/raycast_assessment.md``). The rules:

* It multiplies the renderer's **mean** ``Ping.power`` by ``10^(dB/10)``
  and nothing else. ``Ping.texture`` (the speckle realisation over the mean)
  is untouched, so the shared noise stage draws the same unit-mean speckle
  around the corrected mean. The uncorrected mean stays on the ping as
  ``power_uncorrected``.
* The dB value comes from a versioned artifact
  (``blueboat_raycast_correction/1``), fitted on calibration sessions only,
  its acceptance pre-registered (gates amendment 6b). It is a table in the
  normalised range ``u = r / altitude`` and, for form ``u_logalt``, in
  ``log10(altitude)``. It is **localised**: 0 dB at the first and last
  u-knot, so nothing outside its u-support changes. No per-recording or
  per-scene term exists.
* **Outside the artifact's domain it is off** (6b, owner 2026-11-07): a ping
  whose nadir altitude, acquisition or nadir surface lies outside the domain
  is returned bit-identical to the physical one, with
  ``correction_status = 'off:<reason>'``; one WARNING per reason and per
  process, counters on the artifact. Nothing is extrapolated -- unless a
  PROFILE declares an altitude edge-hold (2026-11-10, harbour patch,
  ``raycast.correction.altitude_hold_m``): then, inside that band only, the
  table is applied at the altitude clamped to its knots (``applied:held``),
  fading linearly to 0 below it (``applied:taper``), each logged once. The
  artifact itself is never edited.
* A PROFILE may also fade the table in after the first seabed echo
  (2026-11-11, ``raycast.correction.onset_fade_m``): the table is at its
  maximum (+13..15 dB) at the first arrival, which made the simulated nadir
  edge a 2-bin step where the real one climbs over 0.5-0.7 m from the level
  the physical model already gives. Evidence in ``.claude/raycast-live.md``.
* An artifact names the physical core it was fitted on (``core``: the
  ray-cast section without backend / seed / correction, plus the mount and
  tilt keys of the shared model). A renderer with another core refuses to
  apply it (``check_core`` raises): never a silent use.
* Off by default (``raycast.correction.enabled: false``). When off, it is
  not constructed at all, so the output is bit-identical to the physical
  renderer.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

import copy
import dataclasses
import hashlib
import json
import logging
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

_log = logging.getLogger(__name__)

FORMAT = "blueboat_raycast_correction/1"
FORMS = ("u", "u_logalt")
_KEYS = {"format", "version", "label", "form", "u_knots", "log10_alt_knots", "db",
         "domain", "fitted_on", "gates", "provenance", "notes", "core", "status",
         "validity"}
_DOMAIN_KEYS = {"altitude_m", "range_m", "num_results", "range_mm", "acquisitions",
                "nadir_material"}
_ACQ_KEYS = {"label", "num_results", "range_mm", "pulse_us"}
#: the model keys of the physical core (the transducer's placement and tilt)
CORE_MODEL_KEYS = ("sensor_depth_m", "mount_x_m", "mount_y_abs_m", "beam_tilt_deg")
#: the ray-cast keys that are not part of the physical core (``floor``: the
#: noise stage's shadow floor, applied after the mean this layer corrects;
#: ``near_range``: the ringing / water column inside the water column, r < altitude;
#: ``labels``: ground-truth contacts, which never touch the ping;
#: ``seed_per_run``: which speckle realisation, like ``seed``;
#: ``objects``: how OBJECTS are meshed -- the seabed this layer was fitted on
#: is the same sheet either way: an object-free world renders alike in both
#: modes, ``raycast_test [O1]``)
_NOT_CORE = ("backend", "seed", "correction", "floor", "near_range", "labels",
             "seed_per_run", "objects")
#: tolerance on a recorded pulse length [us] (the corpus reports 0.1 us)
PULSE_TOL_US = 0.5

APPLIED = "applied"
#: applied outside the fitted domain by a profile's edge-hold / its taper
APPLIED_HELD = "applied:held"
APPLIED_TAPER = "applied:taper"
APPLIED_STATES = (APPLIED, APPLIED_HELD, APPLIED_TAPER)
OFF_ALTITUDE = "off:altitude"
OFF_ACQUISITION = "off:acquisition"
OFF_NADIR = "off:nadir"


def bin_ranges(acq) -> np.ndarray:
    """Device range of every bin centre [m] -- the renderer's own ``BinGrid``
    expression, shared so every caller computes ``u`` bit for bit alike."""
    return (acq.range_start_mm / 1000.0
            + (np.arange(acq.num_results) + 0.5) * acq.bin_size_m)


def core_fingerprint(rc, model) -> dict:
    """The physical core of a ray-cast configuration: ``{raycast, model,
    sha256}``. ``rc`` is a ``RaycastConfig`` (or its mapping); ``model`` the
    shared ``SonarModelConfig``."""
    sec = dataclasses.asdict(rc) if dataclasses.is_dataclass(rc) else dict(rc)
    sec = {k: v for k, v in sec.items() if k not in _NOT_CORE}
    # The fast variant's engine paths (config.ENGINE_KNOBS, 2026-11-10) are
    # left out at their default, so every artifact fitted before they existed
    # keeps its fingerprint; a non-default value stays in (a fast render is
    # not the core an artifact was fitted on).
    from .config import ENGINE_KNOBS
    sec = copy.deepcopy(sec)
    for key, default in ENGINE_KNOBS.items():
        *path, leaf = key.split(".")
        node = sec
        for part in path:
            node = node.get(part) if isinstance(node, dict) else None
        if isinstance(node, dict) and leaf in node and node[leaf] == default:
            del node[leaf]
    mod = {k: float(getattr(model, k)) for k in CORE_MODEL_KEYS}
    doc = {"raycast": sec, "model": mod}
    canon = json.dumps(doc, sort_keys=True, default=repr)
    return {**json.loads(canon), "sha256": hashlib.sha256(canon.encode()).hexdigest()}


def _flat(d, pre=""):
    out = {}
    for k, v in (d or {}).items():
        if isinstance(v, dict):
            out.update(_flat(v, f"{pre}{k}."))
        else:
            out[f"{pre}{k}"] = v
    return out


@dataclass(frozen=True)
class Correction:
    """A loaded, validated correction artifact."""

    version: str
    form: str
    u_knots: np.ndarray                 # (nu,) strictly increasing, >= 0
    log10_alt_knots: np.ndarray | None  # (na,) strictly increasing, form u_logalt
    db: np.ndarray                      # (nu,) or (na, nu), 0 at both u ends
    domain: dict
    source: str = ""
    core: dict | None = None
    status: str = ""
    #: pings seen per status (applied / off:<reason>)
    counts: dict = field(default_factory=dict, compare=False)
    #: a profile's edge-hold band [m] and taper length [m]
    #: (``CorrectionConfig.altitude_hold_m``); None = off outside the domain
    altitude_hold: tuple | None = None
    altitude_taper_m: float = 0.0
    #: a profile's fade-in length after the first seabed echo [m]; 0 = none
    onset_fade_m: float = 0.0

    # ----------------------------------------------------------- evaluation
    def db_at(self, r_m: np.ndarray, altitude_m: float) -> np.ndarray:
        """The table [dB] at device ranges ``r_m`` for a ping whose nadir
        altitude is ``altitude_m`` -- the table alone, without the domain
        gate (0 outside the u-support by construction; the altitude clamped
        to the knots, which :meth:`correct` never lets a ping reach)."""
        r = np.asarray(r_m, dtype=np.float64)
        alt = float(altitude_m)
        if not (alt > 0.0 and math.isfinite(alt)):
            return np.zeros_like(r)               # no seabed below: nothing to correct
        u = np.clip(r / alt, self.u_knots[0], self.u_knots[-1])
        if self.form == "u":
            return np.interp(u, self.u_knots, self.db)
        la = float(np.clip(math.log10(alt), self.log10_alt_knots[0],
                           self.log10_alt_knots[-1]))
        rows = np.array([np.interp(u, self.u_knots, row) for row in self.db])
        w = np.array([np.interp(la, self.log10_alt_knots,
                                np.eye(len(self.log10_alt_knots))[j])
                      for j in range(len(self.log10_alt_knots))])
        return w @ rows

    def gate(self, altitude_m: float, acq=None, *, nadir_material: str | None = None,
             nadir_is_seabed: bool | None = None, max_ping_rate_hz: float = 0.0) -> str:
        """``APPLIED`` or the ``off:<reason>`` this ping falls under. Every
        input is available in a simulation: the renderer's nadir altitude and
        surface, and its acquisition."""
        d = self.domain
        a = d.get("altitude_m")
        alt = float(altitude_m)
        if not (alt > 0.0 and math.isfinite(alt)):
            return OFF_ALTITUDE
        applied = APPLIED
        if a and not (a[0] <= alt <= a[1]):
            applied = self._held(alt)
            if applied is None:
                return OFF_ALTITUDE
        if any(k in d for k in ("num_results", "range_mm", "acquisitions")):
            if acq is None:
                return OFF_ACQUISITION
            nr = d.get("num_results")
            if nr and int(acq.num_results) not in nr:
                return OFF_ACQUISITION
            rm = d.get("range_mm")
            if rm and not (rm[0] <= int(acq.range_length_mm) <= rm[1]):
                return OFF_ACQUISITION
            groups = d.get("acquisitions")
            if groups:
                pulse = float(acq.pulse_duration_s(max_ping_rate_hz)) * 1e6
                if not any(_acq_in(g, acq, pulse) for g in groups):
                    return OFF_ACQUISITION
        mats = d.get("nadir_material")
        if mats:
            if not nadir_is_seabed or nadir_material not in mats:
                return OFF_NADIR
        return applied

    def _held(self, alt: float) -> str | None:
        """The edge-hold status of an altitude outside the fitted domain, or
        None (off)."""
        h = self.altitude_hold
        if not h:
            return None
        if h[0] <= alt <= h[1]:
            return APPLIED_HELD
        t = float(self.altitude_taper_m)
        if t > 0.0 and h[0] - t < alt < h[0]:
            return APPLIED_TAPER
        return None

    def weight(self, altitude_m: float, status: str) -> float:
        """The fraction of the table applied: 1, or the taper's linear fade
        (``applied:taper``)."""
        if status != APPLIED_TAPER:
            return 1.0
        lo, t = float(self.altitude_hold[0]), float(self.altitude_taper_m)
        return float(min(max((float(altitude_m) - (lo - t)) / t, 0.0), 1.0))

    def correct(self, power: np.ndarray, r_m: np.ndarray, altitude_m: float,
                acq=None, *, nadir_material: str | None = None,
                nadir_is_seabed: bool | None = None,
                max_ping_rate_hz: float = 0.0) -> tuple[np.ndarray, str]:
        """``(power, status)``: the corrected mean power (a new array) when
        the ping is inside the domain, else ``power`` itself, unchanged."""
        status = self.gate(altitude_m, acq, nadir_material=nadir_material,
                           nadir_is_seabed=nadir_is_seabed,
                           max_ping_rate_hz=max_ping_rate_hz)
        self.counts[status] = self.counts.get(status, 0) + 1
        if status not in APPLIED_STATES:
            key = (self.version, status)
            if key not in _WARNED:
                _WARNED.add(key)
                _log.warning("raycast correction %s is OFF for a ping (%s: altitude %.2f m, "
                             "nadir %s); further pings of this reason are counted only",
                             self.version, status, float(altitude_m), nadir_material)
            return power, status
        db = self.db_at(r_m, altitude_m)
        if status != APPLIED:
            key = (self.version, status)
            if key not in _WARNED:
                _WARNED.add(key)
                _log.warning("raycast correction %s applied OUTSIDE its fitted altitudes by the "
                             "profile's edge-hold (%s: altitude %.2f m, held band %s); further "
                             "pings are counted only", self.version, status,
                             float(altitude_m), self.altitude_hold)
            db = db * self.weight(altitude_m, status)
        power = np.asarray(power, dtype=np.float64)
        if self.onset_fade_m > 0.0:
            # the first seabed echo: the first lit bin from 0.9 altitude on (a
            # nearer wall or object does not start the fade)
            r = np.asarray(r_m, dtype=np.float64)
            lit = np.flatnonzero((power > 0.0) & (r >= 0.9 * float(altitude_m)))
            if lit.size:
                # sqrt: it rises at once, as the real first return does (of
                # the shapes tried on the real profile, the closest)
                db = db * np.sqrt(np.clip((r - r[lit[0]]) / self.onset_fade_m, 0.0, 1.0))
        return power * 10.0 ** (db / 10.0), status

    def apply(self, power: np.ndarray, r_m: np.ndarray, altitude_m: float,
              acq=None, **kw) -> np.ndarray:
        """The power :meth:`correct` returns (the status dropped)."""
        return self.correct(power, r_m, altitude_m, acq, **kw)[0]

    def check_core(self, rc, model, accept_engine: bool = False) -> None:
        """Raise unless ``(rc, model)`` is the physical core this artifact
        was fitted on (no check for an artifact without ``core``).
        ``accept_engine``: the performance-only keys (``config.
        PERFORMANCE_KEYS``) take the artifact's own values before the
        comparison, so only the physics must match (2026-11-10)."""
        if not self.core:
            return
        if accept_engine:
            rc = _with_core_engine(rc, self.core.get("raycast") or {})
        got = core_fingerprint(rc, model)
        if got["sha256"] == self.core.get("sha256"):
            return
        want = _flat({k: v for k, v in self.core.items() if k != "sha256"})
        have = _flat({k: v for k, v in got.items() if k != "sha256"})
        diff = sorted(k for k in set(want) | set(have) if want.get(k) != have.get(k))
        raise ValueError(f"raycast correction {self.version}: the renderer's physical core "
                         f"differs from the one the artifact was fitted on ({self.source}): "
                         f"{diff[:12]}{' ...' if len(diff) > 12 else ''}")


def _with_core_engine(rc, core_rc: dict) -> dict:
    """``rc`` as a mapping whose performance-only keys hold the values of
    the artifact's core (``core_rc``), or the engine default where the core
    leaves one out (a fingerprint omits ``ENGINE_KNOBS`` at their default)."""
    from .config import ENGINE_KNOBS, PERFORMANCE_KEYS
    sec = dataclasses.asdict(rc) if dataclasses.is_dataclass(rc) else copy.deepcopy(dict(rc))
    for key in PERFORMANCE_KEYS:
        *path, leaf = key.split(".")
        node, src = sec, core_rc
        for part in path:
            node = node.setdefault(part, {}) if isinstance(node, dict) else None
            src = src.get(part, {}) if isinstance(src, dict) else {}
        if not isinstance(node, dict):
            continue
        if isinstance(src, dict) and leaf in src:
            node[leaf] = src[leaf]
        elif key in ENGINE_KNOBS:
            node[leaf] = ENGINE_KNOBS[key]
    return sec


def _acq_in(g: dict, acq, pulse_us: float) -> bool:
    if "num_results" in g and int(acq.num_results) != int(g["num_results"]):
        return False
    rm = g.get("range_mm")
    if rm and not (rm[0] <= int(acq.range_length_mm) <= rm[1]):
        return False
    pu = g.get("pulse_us")
    if pu and not (pu[0] - PULSE_TOL_US <= pulse_us <= pu[1] + PULSE_TOL_US):
        return False
    return True


_WARNED: set = set()


def _increasing(name: str, v) -> np.ndarray:
    a = np.asarray(v, dtype=np.float64)
    if a.ndim != 1 or a.size < 2 or not np.all(np.diff(a) > 0) or not np.all(np.isfinite(a)):
        raise ValueError(f"correction artifact: {name} must be >= 2 strictly increasing finite values")
    return a


def _pair(name: str, v) -> None:
    if not (isinstance(v, (list, tuple)) and len(v) == 2 and float(v[0]) <= float(v[1])):
        raise ValueError(f"correction artifact: domain {name} must be [low, high]")


def from_dict(doc: dict, source: str = "") -> Correction:
    """Validate an artifact mapping (unknown keys and malformed tables raise)."""
    bad = set(doc) - _KEYS
    if bad:
        raise KeyError(f"correction artifact: unknown keys {sorted(bad)}")
    if doc.get("format") != FORMAT:
        raise ValueError(f"correction artifact: format {doc.get('format')!r} is not {FORMAT}")
    form = doc.get("form")
    if form not in FORMS:
        raise ValueError(f"correction artifact: form {form!r} not in {FORMS}")
    for k in ("version", "fitted_on", "gates", "domain", "db", "u_knots"):
        if k not in doc:
            raise KeyError(f"correction artifact: missing {k!r}")
    u = _increasing("u_knots", doc["u_knots"])
    if u[0] < 0.0:
        raise ValueError("correction artifact: u_knots must be >= 0")
    db = np.asarray(doc["db"], dtype=np.float64)
    la = None
    if form == "u":
        if db.shape != u.shape:
            raise ValueError(f"correction artifact: db shape {db.shape} != u_knots {u.shape}")
        if "log10_alt_knots" in doc:
            raise ValueError("correction artifact: log10_alt_knots given for form 'u'")
    else:
        la = _increasing("log10_alt_knots", doc.get("log10_alt_knots"))
        if db.shape != (la.size, u.size):
            raise ValueError(f"correction artifact: db shape {db.shape} != {(la.size, u.size)}")
    if not np.all(np.isfinite(db)):
        raise ValueError("correction artifact: non-finite dB")
    if np.any(db[..., 0] != 0.0) or np.any(db[..., -1] != 0.0):
        raise ValueError("correction artifact: not localised -- the first and last u-knot "
                         "must hold 0 dB")
    dom = dict(doc["domain"] or {})
    if set(dom) - _DOMAIN_KEYS:
        raise KeyError(f"correction artifact: unknown domain keys {sorted(set(dom) - _DOMAIN_KEYS)}")
    for k in ("altitude_m", "range_mm", "range_m"):
        if dom.get(k) is not None:
            _pair(k, dom[k])
    for g in dom.get("acquisitions") or []:
        if set(g) - _ACQ_KEYS:
            raise KeyError(f"correction artifact: unknown acquisition keys {sorted(set(g) - _ACQ_KEYS)}")
        for k in ("range_mm", "pulse_us"):
            if g.get(k) is not None:
                _pair(f"acquisitions.{k}", g[k])
    if dom.get("nadir_material") is not None and not (
            isinstance(dom["nadir_material"], list) and dom["nadir_material"]):
        raise ValueError("correction artifact: domain nadir_material must be a non-empty list")
    if not doc["fitted_on"]:
        raise ValueError("correction artifact: fitted_on must name its calibration sessions")
    core = doc.get("core")
    if core is not None and not (isinstance(core, dict) and core.get("sha256")):
        raise ValueError("correction artifact: core must carry its sha256")
    return Correction(version=str(doc["version"]), form=form, u_knots=u, log10_alt_knots=la,
                      db=db, domain=dom, source=source, core=core,
                      status=str(doc.get("status", "")))


def resolve_artifact(path: str | Path) -> Path:
    """An artifact path as given, or -- relative and not found from the
    working directory -- the package's ``config/`` file of that name
    (the source tree, else the installed share)."""
    p = Path(path).expanduser()
    if p.is_absolute() or p.exists():
        return p
    from ..config import find_default_sonar_yaml
    return find_default_sonar_yaml().parent / p


def load(path: str | Path) -> Correction:
    import yaml
    p = resolve_artifact(path)
    with open(p, "r", encoding="utf-8") as f:
        return from_dict(yaml.safe_load(f) or {}, source=str(p))


_CACHE: dict = {}


def for_config(cfg) -> Correction | None:
    """The artifact a ``CorrectionConfig`` selects, cached per path; ``None``
    when disabled."""
    if cfg is None or not cfg.enabled:
        return None
    if not cfg.artifact:
        raise ValueError("raycast.correction.enabled without an artifact path")
    hold = getattr(cfg, "altitude_hold_m", None)
    taper = float(getattr(cfg, "altitude_taper_m", 0.0)) if hold else 0.0
    fade = float(getattr(cfg, "onset_fade_m", 0.0))
    key = (cfg.artifact, tuple(hold) if hold else None, taper, fade)
    hit = _CACHE.get(key)
    if hit is None:
        base = load(cfg.artifact)
        hit = _CACHE[key] = (dataclasses.replace(
            base, altitude_hold=tuple(hold) if hold else None, altitude_taper_m=taper,
            onset_fade_m=fade, counts={}) if (hold or fade) else base)
    return hit
