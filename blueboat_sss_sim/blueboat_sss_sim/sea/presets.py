"""Sea-state presets, schedules (timelines) and the command/readback schema.

``config/sea_states.yaml`` is the single source of truth for the presets
(MCS reads that file, it never imports this package). Unknown keys raise
by name, as ``sonar/config.py`` does, so a typo in a preset or a schedule
fails at load rather than silently defaulting.

A :class:`SeaState` is one (current, waves) pair with directions; a
:class:`SeaSchedule` is a list of ``(t_s, SeaState)`` keyframes with
linear or hold interpolation -- the timeline the MCS editor writes and the
node follows. Commands on ``/sim/sea_state/command`` carry either a state
(``{"current": {...}, "waves": {...}, "ramp_s": 20}``) or a whole schedule
(``{"schedule": {...}}``); :func:`parse_command` decodes both.

Plain-language keys (2026-10-08, all additive -- every older preset,
schedule and command reads exactly as before):

* current ``variation: steady | natural | strong`` -> the sigmas, time
  constants and eddy strength of the ``current_variation:`` table, scaled
  by the block's mean speed (explicit sigma fields still win);
* waves ``tp_s: auto`` -> Tp interpolated from the wave presets' own
  (Hs, Tp) table; ``crest: short | long`` -> the directional spreading;
  ``wakes: none | occasional | frequent`` -> the wake-group rate (scaled
  from the preset's own, or from the nearest preset's by Hs);
* keyframe ``ramp_s`` -> how long the change to that keyframe takes under
  ``interpolation: hold`` (the MCS timeline's "gradual over N s").

Optional ``yaw_moment: {std_nm, tau_s}`` (2026-10-21, additive): the
synthetic yaw disturbance of :mod:`.yaw_moment`, in a keyframe or a command.
Absent means off, and a state without it serialises exactly as before.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields, replace
from pathlib import Path
from typing import Any, Mapping

import yaml

import numpy as np

from .events import EventSpec
from .yaw_moment import YawMomentSpec

PRESETS_FORMAT = "blueboat_sea_state_presets/1"
SCHEDULE_FORMAT = "blueboat_sea_schedule/1"

_COMPASS_16 = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
               "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]


def compass_name(deg: float) -> str:
    """16-point compass name of a bearing (0 = N, clockwise)."""
    return _COMPASS_16[int(((float(deg) % 360.0) + 11.25) // 22.5) % 16]


def parse_direction(value: Any) -> float:
    """Degrees from either a number or a compass name (``"NE"`` -> 45)."""
    if isinstance(value, (int, float)):
        return float(value) % 360.0
    s = str(value).strip().upper()
    if s in _COMPASS_16:
        return _COMPASS_16.index(s) * 22.5
    return float(s) % 360.0


def _lerp_angle(a: float, b: float, w: float) -> float:
    d = ((b - a + 180.0) % 360.0) - 180.0
    return (a + w * d) % 360.0


# WMO code table 3700 (the Douglas sea scale) by significant wave height:
# upper bounds of codes 0-8 [m]; code 9 above.
_WMO_3700_HS = (0.0, 0.1, 0.5, 1.25, 2.5, 4.0, 6.0, 9.0, 14.0)


def douglas_from_hs(hs_m: float) -> int:
    """WMO 3700 / Douglas sea-state code of a significant wave height."""
    hs = float(hs_m)
    if hs <= 0.0:
        return 0
    for code, upper in enumerate(_WMO_3700_HS):
        if code and hs <= upper:
            return code
    return 9


# ----------------------------------------------------------------- specs
@dataclass(frozen=True)
class CurrentSpec:
    speed_mps: float = 0.0
    sigma_speed_mps: float = 0.0
    sigma_dir_deg: float = 0.0
    tau_speed_s: float = 60.0
    tau_dir_s: float = 120.0
    from_deg: float = 0.0
    preset: str = "none"
    label: str = "No current"
    description: str = ""
    reference: str = ""
    #: RMS of the weak spatial eddy field (sea/current.py::EddyField).
    spatial_rms_mps: float = 0.0
    #: Plain-language variation level (``current_variation:`` table); ""
    #: means the sigmas are given explicitly.
    variation: str = ""


@dataclass(frozen=True)
class VariationSpec:
    """One row of ``current_variation:``: how much the current wanders
    around its mean, as fractions of the mean speed."""

    sigma_speed_frac: float = 0.0
    sigma_dir_deg: float = 0.0
    tau_speed_s: float = 60.0
    tau_dir_s: float = 120.0
    spatial_frac: float = 0.0
    label: str = ""
    description: str = ""
    reference: str = ""


@dataclass(frozen=True)
class CrestSpec:
    """One row of ``wave_options.crest``: the directional spreading."""

    spread_s: float = 3.0
    label: str = ""
    description: str = ""
    reference: str = ""


@dataclass(frozen=True)
class WakeSpec:
    """One row of ``wave_options.wakes``: scale on the wake-group rate."""

    rate_scale: float = 1.0
    label: str = ""
    description: str = ""


@dataclass(frozen=True)
class WaveSpec:
    hs_m: float = 0.0
    tp_s: float = 0.0
    gamma: float = 1.5
    spread_s: float = 3.0            # short-crested (2026-09-05; was 6)
    from_deg: float = 0.0
    douglas: int = 0
    preset: str = "calm"
    label: str = "Calm (glassy)"
    description: str = ""
    reference: str = ""
    #: Random long-period wave groups (wakes / swell trains), sea/events.py.
    events: EventSpec = field(default_factory=EventSpec)
    #: Plain-language choices this spec was resolved from ("" = explicit).
    crest: str = ""
    wakes: str = ""


def _events_from(raw: Any, where: str) -> EventSpec:
    if raw is None:
        return EventSpec()
    if isinstance(raw, EventSpec):
        return raw
    raw = dict(raw)
    known = {f.name for f in fields(EventSpec)}
    unknown = set(raw) - known
    if unknown:
        raise KeyError(f"Unknown events keys in {where}: {sorted(unknown)}")
    return EventSpec(**{k: (int(v) if k == "max_overlap" else float(v))
                        for k, v in raw.items()})


@dataclass(frozen=True)
class SeaState:
    current: CurrentSpec = field(default_factory=CurrentSpec)
    waves: WaveSpec = field(default_factory=WaveSpec)
    #: Synthetic yaw disturbance (2026-10-21, additive); std 0 = off.
    yaw_moment: YawMomentSpec = field(default_factory=YawMomentSpec)

    @property
    def is_null(self) -> bool:
        return (self.current.speed_mps <= 0.0 and self.current.sigma_speed_mps <= 0.0
                and self.waves.hs_m <= 0.0 and not self.yaw_moment.active)

    def to_dict(self) -> dict:
        d = {"current": asdict(self.current), "waves": asdict(self.waves)}
        if self.yaw_moment.active:
            d["yaw_moment"] = asdict(self.yaw_moment)
        return d

    @property
    def current_active(self) -> bool:
        c = self.current
        return c.speed_mps > 0.0 or c.sigma_speed_mps > 0.0 or c.spatial_rms_mps > 0.0

    @staticmethod
    def interpolate(a: "SeaState", b: "SeaState", w: float) -> "SeaState":
        """Linear blend of every numeric field (angles along the short arc);
        names come from ``b`` once w ≥ 0.5."""
        w = float(min(max(w, 0.0), 1.0))
        cur = {}
        for f in fields(CurrentSpec):
            va, vb = getattr(a.current, f.name), getattr(b.current, f.name)
            if f.name == "from_deg":
                cur[f.name] = _lerp_angle(va, vb, w)
            elif isinstance(va, (int, float)) and not isinstance(va, bool):
                cur[f.name] = (1.0 - w) * va + w * vb
            else:
                cur[f.name] = vb if w >= 0.5 else va
        wav = {}
        for f in fields(WaveSpec):
            va, vb = getattr(a.waves, f.name), getattr(b.waves, f.name)
            if f.name == "from_deg":
                wav[f.name] = _lerp_angle(va, vb, w)
            elif f.name == "douglas":
                wav[f.name] = vb if w >= 0.5 else va
            elif isinstance(va, (int, float)) and not isinstance(va, bool):
                wav[f.name] = (1.0 - w) * va + w * vb
            else:
                # Names and the events block: the target's once past half.
                wav[f.name] = vb if w >= 0.5 else va
        # A wave state with hs == 0 has no meaningful Tp: keep the other's.
        if a.waves.hs_m <= 0.0 < b.waves.hs_m:
            wav["tp_s"] = b.waves.tp_s
        elif b.waves.hs_m <= 0.0 < a.waves.hs_m:
            wav["tp_s"] = a.waves.tp_s
        # Yaw moment: the std blends; the time constant is the active side's
        # (SeaModel crossfades the two processes by w itself, as for the
        # current). An inactive side takes the other's tau, so a ramp from
        # off to on is a pure std ramp.
        ya, yb = a.yaw_moment, b.yaw_moment
        tau = (yb.tau_s if (w >= 0.5 or not ya.active) and yb.active
               else ya.tau_s if ya.active else yb.tau_s)
        yaw = YawMomentSpec((1.0 - w) * ya.std_nm + w * yb.std_nm, tau)
        return SeaState(CurrentSpec(**cur), WaveSpec(**wav), yaw)


# --------------------------------------------------------------- presets
@dataclass
class SeaPresets:
    current: dict[str, CurrentSpec]
    waves: dict[str, WaveSpec]
    hull: dict[str, Any]
    path: str = ""
    variation: dict[str, VariationSpec] = field(default_factory=dict)
    crest: dict[str, CrestSpec] = field(default_factory=dict)
    wakes: dict[str, WakeSpec] = field(default_factory=dict)

    def current_names(self) -> list[str]:
        return list(self.current)

    def wave_names(self) -> list[str]:
        return list(self.waves)

    # ------------------------------------------------- plain-language help
    def auto_tp(self, hs_m: float) -> float:
        """Peak period for a wave height, interpolated from the wave
        presets' own (Hs, Tp) pairs (piecewise linear between presets,
        ``Tp ∝ sqrt(Hs)`` beyond them, the steepness scaling of
        DNV-RP-C205 §3.5.10). 0 for Hs 0."""
        hs = float(hs_m)
        pts = sorted((w.hs_m, w.tp_s) for w in self.waves.values()
                     if w.hs_m > 0.0 and w.tp_s > 0.0)
        if hs <= 0.0 or not pts:
            return 0.0
        if hs <= pts[0][0]:
            return float(pts[0][1] * np.sqrt(hs / pts[0][0]))
        if hs >= pts[-1][0]:
            return float(pts[-1][1] * np.sqrt(hs / pts[-1][0]))
        h, t = zip(*pts)
        return float(np.interp(hs, h, t))

    def nearest_wave_preset(self, hs_m: float) -> WaveSpec | None:
        """The non-calm wave preset closest in Hs (log ratio)."""
        cands = [w for w in self.waves.values() if w.hs_m > 0.0]
        if not cands or hs_m <= 0.0:
            return None
        return min(cands, key=lambda w: abs(np.log(w.hs_m / float(hs_m))))


def find_default_presets_yaml() -> Path:
    """``config/sea_states.yaml``: source tree first, then the installed
    share directory (same resolution as ``find_default_sonar_yaml``)."""
    src = Path(__file__).resolve().parents[2] / "config" / "sea_states.yaml"
    if src.exists():
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = (Path(get_package_share_directory("blueboat_sss_sim"))
                / "config" / "sea_states.yaml")
        if cand.exists():
            return cand
    except Exception:
        pass
    raise FileNotFoundError("config/sea_states.yaml not found in the source "
                            "tree or the installed share directory")


def _spec_from(cls, name: str, raw: Mapping[str, Any], section: str):
    known = {f.name for f in fields(cls)}
    unknown = set(raw) - known
    if unknown:
        raise KeyError(f"Unknown {section} preset keys in '{name}': "
                       f"{sorted(unknown)}")
    kw = {k: v for k, v in raw.items() if k != "preset"}
    if cls is WaveSpec and "events" in kw:
        kw["events"] = _events_from(kw["events"], f"waves preset '{name}'")
    return cls(preset=name, **kw)


def load_presets(path: str | Path | None = None) -> SeaPresets:
    p = Path(path) if path else find_default_presets_yaml()
    with open(p, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    if doc.get("format") != PRESETS_FORMAT:
        raise ValueError(f"{p}: expected format {PRESETS_FORMAT}, got "
                         f"{doc.get('format')!r}")
    extra = set(doc) - {"format", "current", "waves", "hull",
                        "current_variation", "wave_options"}
    if extra:
        raise KeyError(f"{p}: unknown top-level keys {sorted(extra)}")
    variation = {n: _row_from(VariationSpec, n, r or {}, "current_variation")
                 for n, r in (doc.get("current_variation") or {}).items()}
    opts = dict(doc.get("wave_options") or {})
    if set(opts) - {"crest", "wakes"}:
        raise KeyError(f"{p}: unknown wave_options keys "
                       f"{sorted(set(opts) - {'crest', 'wakes'})}")
    crest = {n: _row_from(CrestSpec, n, r or {}, "wave_options.crest")
             for n, r in (opts.get("crest") or {}).items()}
    wakes = {n: _row_from(WakeSpec, n, r or {}, "wave_options.wakes")
             for n, r in (opts.get("wakes") or {}).items()}
    presets = SeaPresets(current={}, waves={}, hull=dict(doc.get("hull") or {}),
                         path=str(p), variation=variation, crest=crest, wakes=wakes)
    for n, r in (doc.get("current") or {}).items():
        r = dict(r or {})
        spec = _spec_from(CurrentSpec, n, r, "current")
        presets.current[n] = apply_variation(presets, spec, set(r))
    wav = {n: _spec_from(WaveSpec, n, r or {}, "waves")
           for n, r in (doc.get("waves") or {}).items()}
    for n, w in wav.items():
        if w.hs_m > 0.0 and w.tp_s <= 0.0:
            raise ValueError(f"waves preset '{n}': hs_m > 0 needs tp_s > 0")
    presets.waves.update(wav)
    return presets


def _row_from(cls, name: str, raw: Mapping[str, Any], section: str):
    known = {f.name for f in fields(cls)}
    unknown = set(raw) - known
    if unknown:
        raise KeyError(f"Unknown {section} keys in '{name}': {sorted(unknown)}")
    return cls(**raw)


def apply_variation(presets: SeaPresets, spec: CurrentSpec,
                    explicit: set[str] | frozenset = frozenset()) -> CurrentSpec:
    """Fill the sigmas, time constants and eddy strength of a current spec
    from its ``variation`` level, scaled by its mean speed; fields named in
    ``explicit`` are kept as given. A spec without a level is returned
    unchanged (legacy explicit sigmas)."""
    if not spec.variation:
        return spec
    if spec.variation not in presets.variation:
        raise KeyError(f"unknown current variation '{spec.variation}'; valid: "
                       f"{list(presets.variation)}")
    v = presets.variation[spec.variation]
    u = max(float(spec.speed_mps), 0.0)
    derived = {"sigma_speed_mps": v.sigma_speed_frac * u,
               "sigma_dir_deg": v.sigma_dir_deg if u > 0.0 else 0.0,
               "tau_speed_s": v.tau_speed_s, "tau_dir_s": v.tau_dir_s,
               "spatial_rms_mps": v.spatial_frac * u}
    return replace(spec, **{k: val for k, val in derived.items() if k not in explicit})


def apply_wave_options(presets: SeaPresets, spec: WaveSpec,
                       explicit: set[str] | frozenset = frozenset(),
                       tp_auto: bool = False) -> WaveSpec:
    """Resolve ``tp_s: auto``, ``crest`` and ``wakes`` on a wave spec, and
    the Douglas code of a custom height; explicit fields win."""
    upd: dict[str, Any] = {}
    if tp_auto:
        upd["tp_s"] = presets.auto_tp(spec.hs_m)
    if spec.crest and "spread_s" not in explicit:
        if spec.crest not in presets.crest:
            raise KeyError(f"unknown wave crest '{spec.crest}'; valid: "
                           f"{list(presets.crest)}")
        upd["spread_s"] = presets.crest[spec.crest].spread_s
    if spec.wakes and "events" not in explicit:
        if spec.wakes not in presets.wakes:
            raise KeyError(f"unknown wakes '{spec.wakes}'; valid: "
                           f"{list(presets.wakes)}")
        scale = presets.wakes[spec.wakes].rate_scale
        base = spec.events
        if not base.active:
            near = presets.nearest_wave_preset(spec.hs_m)
            base = near.events if near is not None else EventSpec()
        upd["events"] = (replace(base, rate_per_hour=base.rate_per_hour * scale)
                         if scale > 0.0 and base.active else EventSpec())
    if spec.preset == "custom" and "douglas" not in explicit:
        upd["douglas"] = douglas_from_hs(spec.hs_m)
    return replace(spec, **upd) if upd else spec


def resolve_state(presets: SeaPresets, current: str = "none",
                  current_from_deg: Any = 0.0, waves: str = "calm",
                  waves_from_deg: Any = 0.0) -> SeaState:
    """Preset names + directions -> :class:`SeaState`; unknown names raise
    listing the valid ones."""
    if current not in presets.current:
        raise KeyError(f"unknown current preset '{current}'; valid: "
                       f"{presets.current_names()}")
    if waves not in presets.waves:
        raise KeyError(f"unknown waves preset '{waves}'; valid: "
                       f"{presets.wave_names()}")
    return SeaState(
        replace(presets.current[current], from_deg=parse_direction(current_from_deg)),
        replace(presets.waves[waves], from_deg=parse_direction(waves_from_deg)))


def _spec_from_entry(presets: SeaPresets, entry: Mapping[str, Any] | None,
                     which: str):
    """One ``current:`` / ``waves:`` block of a keyframe or command: a
    ``preset`` name (with ``from_deg``) and/or explicit numeric fields."""
    entry = dict(entry or {})
    cls, table, default = ((CurrentSpec, presets.current, "none") if which == "current"
                           else (WaveSpec, presets.waves, "calm"))
    name = entry.pop("preset", None)
    if name is None and not entry:
        name = default
    base = table[name] if name is not None else cls()
    if name is not None and name not in table:
        raise KeyError(f"unknown {which} preset '{name}'; valid: {list(table)}")
    if "from_deg" in entry:
        entry["from_deg"] = parse_direction(entry["from_deg"])
    if "from" in entry:
        entry["from_deg"] = parse_direction(entry.pop("from"))
    known = {f.name for f in fields(cls)}
    unknown = set(entry) - known
    if unknown:
        raise KeyError(f"unknown {which} keys {sorted(unknown)}")
    if cls is WaveSpec and "events" in entry:
        entry["events"] = _events_from(entry["events"], f"{which} block")
    tp_auto = False
    if cls is WaveSpec and isinstance(entry.get("tp_s"), str):
        if entry["tp_s"].strip().lower() != "auto":
            raise ValueError(f"waves tp_s must be a number or 'auto', got "
                             f"{entry['tp_s']!r}")
        entry.pop("tp_s")
        tp_auto = True
    explicit = set(entry)
    if name is None:
        entry.setdefault("preset", "custom")
        entry.setdefault("label", "Custom")
    spec = replace(base, **entry)
    if cls is CurrentSpec:
        return apply_variation(presets, spec, explicit)
    return apply_wave_options(presets, spec, explicit, tp_auto)


def yaw_moment_from(raw: Any, where: str = "yaw_moment block") -> YawMomentSpec:
    """A ``yaw_moment: {std_nm, tau_s}`` block; absent/None = off. Unknown
    keys raise by name; std must be >= 0 and tau > 0."""
    if raw is None:
        return YawMomentSpec()
    if isinstance(raw, YawMomentSpec):
        return raw
    raw = dict(raw)
    known = {f.name for f in fields(YawMomentSpec)}
    unknown = set(raw) - known
    if unknown:
        raise KeyError(f"unknown {where} keys {sorted(unknown)}; valid: {sorted(known)}")
    spec = YawMomentSpec(**{k: float(v) for k, v in raw.items()})
    if spec.std_nm < 0.0 or spec.tau_s <= 0.0:
        raise ValueError(f"{where}: std_nm must be >= 0 and tau_s > 0, got {spec}")
    return spec


def state_from_dict(presets: SeaPresets, d: Mapping[str, Any]) -> SeaState:
    return SeaState(_spec_from_entry(presets, d.get("current"), "current"),
                    _spec_from_entry(presets, d.get("waves"), "waves"),
                    yaw_moment_from(d.get("yaw_moment")))


# -------------------------------------------------------------- schedule
@dataclass
class SeaSchedule:
    keyframes: list[tuple[float, SeaState]]
    interpolation: str = "linear"     # linear | hold
    seed: int = 0
    name: str = ""
    #: Optional per-keyframe transition time [s] (keyframe t -> ramp), used
    #: under ``hold``: the change *to* that keyframe takes ramp_s.
    ramps: dict[float, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.keyframes:
            raise ValueError("a sea schedule needs at least one keyframe")
        if self.interpolation not in ("linear", "hold"):
            raise ValueError(f"interpolation must be linear|hold, got "
                             f"{self.interpolation!r}")
        self.keyframes = sorted(self.keyframes, key=lambda kf: kf[0])

    @property
    def duration_s(self) -> float:
        return float(self.keyframes[-1][0])

    def state_at(self, t: float) -> SeaState:
        kf = self.keyframes
        if t <= kf[0][0] or len(kf) == 1:
            return kf[0][1]
        if t >= kf[-1][0]:
            return kf[-1][1]
        for (t0, s0), (t1, s1) in zip(kf[:-1], kf[1:]):
            if t0 <= t < t1:
                if self.interpolation == "hold" or t1 <= t0:
                    return s0
                return SeaState.interpolate(s0, s1, (t - t0) / (t1 - t0))
        return kf[-1][1]

    def ramp_for(self, t: float) -> float | None:
        """Transition time of the keyframe active at ``t`` (None = the
        model's default ramp)."""
        active = None
        for t0, _ in self.keyframes:
            if t0 <= t:
                active = t0
        return None if active is None else self.ramps.get(float(active))

    def next_change(self, t: float) -> tuple[float, SeaState] | None:
        """The next keyframe strictly after t, or None once the schedule
        has played out."""
        for t0, s0 in self.keyframes:
            if t0 > t:
                return float(t0), s0
        return None

    @staticmethod
    def constant(state: SeaState, seed: int = 0) -> "SeaSchedule":
        return SeaSchedule([(0.0, state)], "hold", seed)

    def to_dict(self) -> dict:
        kfs = []
        for t, s in self.keyframes:
            kf = {"t_s": t, **s.to_dict()}
            if float(t) in self.ramps:
                kf["ramp_s"] = self.ramps[float(t)]
            kfs.append(kf)
        return {"format": SCHEDULE_FORMAT, "name": self.name, "seed": self.seed,
                "interpolation": self.interpolation, "keyframes": kfs}

    @staticmethod
    def from_dict(presets: SeaPresets, d: Mapping[str, Any]) -> "SeaSchedule":
        fmt = d.get("format", SCHEDULE_FORMAT)
        if fmt != SCHEDULE_FORMAT:
            raise ValueError(f"expected format {SCHEDULE_FORMAT}, got {fmt!r}")
        extra = set(d) - {"format", "name", "seed", "interpolation", "keyframes"}
        if extra:
            raise KeyError(f"unknown schedule keys {sorted(extra)}")
        kfs, ramps = [], {}
        for i, k in enumerate(d.get("keyframes") or []):
            k = dict(k)
            t = float(k.pop("t_s", k.pop("t", 0.0)))
            if "ramp_s" in k:
                ramps[t] = max(float(k.pop("ramp_s")), 0.0)
            extra = set(k) - {"current", "waves", "yaw_moment"}
            if extra:
                raise KeyError(f"keyframe {i}: unknown keys {sorted(extra)}")
            kfs.append((t, state_from_dict(presets, k)))
        return SeaSchedule(kfs, str(d.get("interpolation", "linear")),
                           int(d.get("seed", 0) or 0), str(d.get("name", "")),
                           ramps)

    @staticmethod
    def from_yaml(presets: SeaPresets, path: str | Path) -> "SeaSchedule":
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        sched = SeaSchedule.from_dict(presets, doc)
        if not sched.name:
            sched.name = Path(path).stem
        return sched

    def to_yaml(self, path: str | Path) -> None:
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False)


# --------------------------------------------------------------- commands
def parse_command(presets: SeaPresets, payload: str | Mapping[str, Any]
                  ) -> tuple[str, Any, float]:
    """Decode a ``/sim/sea_state/command`` payload.

    Returns ``("state", SeaState, ramp_s)`` or ``("schedule", SeaSchedule,
    t0_s)`` where ``t0_s`` is the schedule's origin relative to *now*
    (``"t0": "now"`` or a number of seconds; default now).
    """
    d = json.loads(payload) if isinstance(payload, str) else dict(payload)
    if "schedule" in d:
        sched = SeaSchedule.from_dict(presets, d["schedule"])
        t0 = d.get("t0", "now")
        return "schedule", sched, 0.0 if t0 == "now" else float(t0)
    extra = set(d) - {"current", "waves", "ramp_s", "yaw_moment"}
    if extra:
        raise KeyError(f"unknown command keys {sorted(extra)}")
    return "state", state_from_dict(presets, d), float(d.get("ramp_s", -1.0))
