"""Boat motion helpers for the sea state: pose prediction, hull damping
and the offline closed-loop integrator that gates them (ROS-free).

Why these exist (2026-09-05 review, "waves too powerful, too regular,
too frequent, pitch only"). The published wrench carries the catamaran's
*restoring* stiffness (C33 ≈ 3.5 kN/m, C44 ≈ 580 N·m/rad, C55 ≈ 440
N·m/rad) evaluated at the pose the node last received. Gazebo publishes
odometry at 20 Hz while the node publishes at 100 Hz, so the restoring
spring acted on a pose up to 50 ms stale -- a *delayed* spring on a
heave mode of 0.59 s with ζ ≈ 0.03 (BlueROV2 damping coefficients on a
BlueBoat hull). Delayed restoring feedback pumps energy into the mode:
an offline integration reproduces a 0.6 s heave bounce of 0.27 m
peak-to-peak at 'smooth' (seed 3, 40 s) -- the regular 1.6 Hz "wave" the
user saw --
while the same model with a 100 Hz pose simply contours the sea.

Two fixes, both here:

* :func:`predict_pose` propagates the last odometry (pose + body twist,
  which Gazebo's ``OdometryPublisher`` sends in the child frame) to the
  tick time, so the spring sees the boat where it is, not where it was;
* :func:`damping_wrench` adds the heave / roll / pitch damping a real
  catamaran has and the ROV coefficients lack (radiation + viscous;
  sized for ζ ≈ 0.4, ``config/sea_states.yaml`` ``hull.damping``), so
  whatever lag remains cannot ring.

:func:`simulate_3dof` closes the loop offline -- true state integrated
at the physics step, the "node" fed a pose held for ``pose_hold_s`` --
and is the smoke test's acceptance check: the heave spectrum must sit in
the wave band, roll and pitch in the corpus's measured band (roll std
0.9-1.3°, pitch 0.8-1.5°, ~1 s), and nothing may ring at 1.6 Hz.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from ..core.types import Pose3D
from .hull import HullBoxes, WaveExcitation, rotation_zyx
from .plant import IDENTIFIED, LEGACY, PlantVertical
from .spectrum import G

#: URDF / Hydrodynamics-plugin figures of blueboat_description for the
#: LEGACY plant (read-only mirror for the offline integrator; BlueROV2
#: coefficients, von Benzon 2022). Both plants, the identified one included,
#: live in :mod:`.plant` (2026-11-15); these names stay for the legacy plant.
URDF_MASS_KG = 16.15
#: The ``<inertia>`` attributes (ixx, iyy, izz) of ``blueboat.xacro``'s base
#: link AS WRITTEN -- in the inertial frame, which the xacro rotates by
#: ``rpy="${pi/2} 0 0"``. Do not use these as body-axis moments.
URDF_INERTIA = (2.42369, 3.9235025, 5.6403125)
#: The same tensor in the LINK (body) frame, what Gazebo integrates: the
#: π/2 roll of the inertial frame swaps its y and z axes, so pitch (Iyy)
#: is the attribute ``izz`` and yaw (Izz) the attribute ``iyy``. Until
#: 2026-11-10 the offline mirror took ``iyy`` as pitch (3.92 instead of
#: 5.64 kg·m²): Gazebo's pitch period is 0.72 s, not the 0.60 s the model
#: doc quoted. Smoke ``[10]`` re-derives this from Control's xacro.
URDF_LINK_INERTIA = (URDF_INERTIA[0], URDF_INERTIA[2], URDF_INERTIA[1])
URDF_COG_Z = -0.12913
PLUGIN_ADDED_MASS = (14.57, 0.12, 0.12)          # zDotW, kDotP, mDotQ
PLUGIN_LINEAR = (-17.955, -10.888, -20.761)      # zW, kP, mQ
PLUGIN_QUADRATIC = (-73.371, -40.0, -40.0)       # zWabsW, kPabsP, mQabsQ


@dataclass(frozen=True)
class DampingSpec:
    """Extra hull damping the wave model publishes (N·s/m, N·m·s/rad)."""

    heave_ns_per_m: float = 0.0
    roll_nms_per_rad: float = 0.0
    pitch_nms_per_rad: float = 0.0

    @property
    def active(self) -> bool:
        return (self.heave_ns_per_m > 0.0 or self.roll_nms_per_rad > 0.0
                or self.pitch_nms_per_rad > 0.0)


DAMPING_RATIO_KEYS = ("heave", "roll", "pitch")


def damping_ratios_from_config(cfg: Mapping[str, Any] | None) -> dict | None:
    """``hull.damping_ratio`` (2026-11-15) or None; unknown keys raise."""
    raw = (cfg or {}).get("damping_ratio")
    if raw is None:
        return None
    raw = dict(raw)
    if set(raw) != set(DAMPING_RATIO_KEYS):
        raise KeyError(f"hull.damping_ratio needs exactly {list(DAMPING_RATIO_KEYS)}, got {sorted(raw)}")
    return {k: float(v) for k, v in raw.items()}


def damping_from_ratios(ratios: Mapping[str, float], hull: HullBoxes, plant: PlantVertical) -> tuple:
    """Hull damping giving the damping ratios on the plant Gazebo integrates: b = 2 zeta sqrt(K (I + A)) per
    mode, K from the hull's own restoring (``HullBoxes.restoring_summary``), I + A the plant's rigid inertia plus
    the plugin's added mass, MINUS the plugin's own linear damping (so the share is the complement of whatever the
    plugin carries: all of it after R-8, today's split before). Returns (DampingSpec, details)."""
    r = hull.restoring_summary(inertia=plant.inertia_link, cog_z=plant.cog_z_m,
                               added_mass=plant.added_mass_positive)
    a33, a44, a55 = plant.added_mass_positive
    modes = {"heave": (r["c33_up_n_per_m"], plant.mass_kg + a33, plant.linear[0]),
             "roll": (r["c44_nm_per_rad"], plant.inertia_link[0] + a44, plant.linear[1]),
             "pitch": (r["c55_nm_per_rad"], plant.inertia_link[1] + a55, plant.linear[2])}
    b = {k: damping_for_zeta(ratios[k], K, I, lin) for k, (K, I, lin) in modes.items()}
    det = {k: {"zeta": ratios[k], "stiffness": K, "inertia_with_added": I,
               "b_total": 2.0 * ratios[k] * np.sqrt(K * I), "b_plugin": -lin, "b_sim": b[k],
               "f_n_hz": float(np.sqrt(K / I) / (2.0 * np.pi))}
           for k, (K, I, lin) in modes.items()}
    det["plant"] = plant.name
    det["plant_source"] = plant.source
    return DampingSpec(b["heave"], b["roll"], b["pitch"]), det


def default_plant_for(cfg: Mapping[str, Any] | None) -> PlantVertical:
    """The plant a presets file floats when nothing else says: the CAD hull (a ``catamaran.fin``) is the
    identified plant's, the box catamaran the legacy plant's."""
    cat = dict((cfg or {}).get("catamaran") or {})
    return IDENTIFIED if "fin" in cat else LEGACY


def hull_damping_from_config(cfg: Mapping[str, Any] | None, plant: PlantVertical | None = None,
                             hull: HullBoxes | None = None) -> DampingSpec:
    """``hull.damping`` (fixed coefficients, the legacy file) or ``hull.damping_ratio`` (ratios sized on the
    plant, which needs the hull); unknown keys raise."""
    raw = dict((cfg or {}).get("damping") or {})
    ratios = damping_ratios_from_config(cfg)
    if raw and ratios is not None:
        raise KeyError("hull.damping and hull.damping_ratio are exclusive")
    if ratios is not None:
        if hull is None:
            hull = HullBoxes.from_config(dict(cfg))[1]
        return damping_from_ratios(ratios, hull, plant or default_plant_for(cfg))[0]
    known = {"heave_ns_per_m", "roll_nms_per_rad", "pitch_nms_per_rad"}
    unknown = set(raw) - known
    if unknown:
        raise KeyError(f"Unknown hull.damping keys: {sorted(unknown)}")
    return DampingSpec(**{k: float(v) for k, v in raw.items()})


def damping_for_zeta(zeta: float, stiffness: float, inertia: float,
                     plugin_linear: float = 0.0) -> float:
    """Linear damping coefficient giving damping ratio ``zeta`` on a mode
    of stiffness ``stiffness`` and (added) inertia ``inertia``, minus what
    the Hydrodynamics plugin already supplies (its coefficients are
    negative)."""
    return max(2.0 * zeta * np.sqrt(stiffness * inertia) + plugin_linear, 0.0)


# ---------------------------------------------------------------- pose
def predict_pose(pose: Pose3D, v_body, w_body, dt: float,
                 max_dt: float = 0.15) -> Pose3D:
    """The pose ``dt`` seconds after an odometry sample, propagated with
    its own body-frame twist (constant velocity; ``dt`` clipped to
    ``[0, max_dt]`` so a stale sample is not flung across the world)."""
    dt = float(min(max(dt, 0.0), max_dt))
    if dt <= 0.0:
        return pose
    v = np.asarray(v_body, dtype=np.float64).reshape(3)
    w = np.asarray(w_body, dtype=np.float64).reshape(3)
    r = rotation_zyx(pose.roll, pose.pitch, pose.yaw)
    d = r @ v * dt
    return Pose3D(pose.x + float(d[0]), pose.y + float(d[1]),
                  pose.z + float(d[2]),
                  roll=pose.roll + float(w[0]) * dt,
                  pitch=pose.pitch + float(w[1]) * dt,
                  yaw=pose.yaw + float(w[2]) * dt)


#: Height of Gazebo's ``blueboat/base_link`` origin above the still waterline
#: when the boat floats at rest: the URDF buoyancy box's own equilibrium
#: (about -0.111 m; the catamaran hydrostatics share it by construction,
#: ``hull.py``). Gazebo's poses carry this trim; the renderer -- and every
#: offline and calibration render, at pose z = 0 -- takes z = 0 as the
#: waterline, so ``sensor_depth_m`` is depth below the still water.
GAZEBO_BASE_REST_Z_M = float(HullBoxes.gazebo_box().equilibrium_z())


def waterline_pose(pose: Pose3D, base_rest_z: float) -> Pose3D:
    """A Gazebo base_link pose re-referenced so that z = 0 is the boat
    floating level at rest (the renderer's datum): ``z - base_rest_z``.

    Without it a Gazebo run put the transducers ``sensor_depth_m - 0.111``
    m too deep (0.26 m instead of 0.15) while every calibration render
    assumed 0.15 (2026-10-21). ``base_rest_z = 0`` returns the pose
    unchanged (the pre-fix behaviour, ``base_link_rest_z_m:=0.0``).
    Heave, roll and pitch are untouched: only the constant datum moves.
    """
    if base_rest_z == 0.0:
        return pose
    return Pose3D(pose.x, pose.y, pose.z - float(base_rest_z),
                  roll=pose.roll, pitch=pose.pitch, yaw=pose.yaw)


class HighRatePose:
    """The boat's pose from Gazebo's 100 Hz ``PosePublisher``
    (``/sim/sea_state/boat_pose``, a stamped ``TransformStamped`` bridged
    from gz ``/model/blueboat/pose`` by ``sea_state_launch.pose_bridge``),
    with the body-frame velocity and angular rates differenced
    from consecutive samples -- what the wrench and the sonar need instead
    of the 20 Hz odometry (2026-10-09).

    Why: the hull wrench carries the catamaran's stiff restoring (roll
    C44 ≈ 577 N·m/rad against the Gazebo box's 39). Fed from 20 Hz
    odometry whose twist is itself an average over its 50 ms period, plus
    ~2 physics steps of transport, that spring is delayed enough to pump
    the 0.4 s roll mode: Gazebo showed a self-sustained ±0.5° roll at
    3.2 Hz on calm water, reproduced by a roll-only discrete model (and
    hidden under the wave roll before 2026-10-09). With the 10 ms pose and
    differenced rates the same model is stable up to 4 steps of delay.

    ``update(t, pose)`` per sample; ``state()`` -> ``(pose, v_body,
    w_body, t)`` or None before two samples.
    """

    def __init__(self, max_gap_s: float = 0.1) -> None:
        self.max_gap_s = float(max_gap_s)
        self._prev: tuple[float, Pose3D] | None = None
        self._last: tuple[float, Pose3D, np.ndarray, np.ndarray] | None = None

    def update(self, t: float, pose: Pose3D) -> None:
        prev = self._prev
        if prev is not None and float(t) - prev[0] <= 1e-6:
            return                       # a duplicate of the same physics step
        self._prev = (float(t), pose)
        if prev is None:
            return
        dt = float(t) - prev[0]
        if dt > self.max_gap_s:
            self._last = None
            return
        q = prev[1]
        r = rotation_zyx(pose.roll, pose.pitch, pose.yaw)
        v_world = np.array([pose.x - q.x, pose.y - q.y, pose.z - q.z]) / dt
        dphi = (pose.roll - q.roll + np.pi) % (2 * np.pi) - np.pi
        dth = pose.pitch - q.pitch
        dpsi = (pose.yaw - q.yaw + np.pi) % (2 * np.pi) - np.pi
        phid, thd, psid = dphi / dt, dth / dt, dpsi / dt
        sph, cph = np.sin(pose.roll), np.cos(pose.roll)
        sth, cth = np.sin(pose.pitch), np.cos(pose.pitch)
        # ZYX Euler rates -> body rates
        w_body = np.array([phid - psid * sth,
                           thd * cph + psid * sph * cth,
                           -thd * sph + psid * cph * cth])
        self._last = (float(t), pose, r.T @ v_world, w_body)

    def state(self):
        if self._last is None:
            return None
        t, pose, v_body, w_body = self._last
        return pose, v_body, w_body, t


def damping_wrench(v_world, w_body, water_v_world, spec: DampingSpec,
                   yaw: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Heave damping on the vertical velocity relative to the water, roll
    and pitch damping on the body rates; torque returned in world axes
    (body torque rotated by the yaw -- small roll/pitch assumed)."""
    v = np.asarray(v_world, dtype=np.float64).reshape(3)
    w = np.asarray(w_body, dtype=np.float64).reshape(3)
    wv = np.asarray(water_v_world, dtype=np.float64).reshape(3)
    force = np.array([0.0, 0.0, -spec.heave_ns_per_m * (v[2] - wv[2])])
    tau_body = np.array([-spec.roll_nms_per_rad * w[0],
                         -spec.pitch_nms_per_rad * w[1], 0.0])
    return force, rotation_zyx(0.0, 0.0, yaw) @ tau_body


# ------------------------------------------------------- closed loop
def coarse_hulls(presets_hull: Mapping[str, Any], cells_per_m: float = 10.0
                 ) -> tuple[HullBoxes, HullBoxes]:
    """The two hulls of the presets at a coarser integration lattice (the
    offline loop evaluates them thousands of times)."""
    gz, cat = HullBoxes.from_config(presets_hull)
    return (HullBoxes(gz.mass_kg, gz.boxes, gz.water_density, gz.name,
                      cells_per_m),
            HullBoxes(cat.mass_kg, cat.boxes, cat.water_density, cat.name,
                      cells_per_m, pressure_depth_m=cat.pressure_depth_m,
                      wetted_lever=cat.wetted_lever))


def simulate_3dof(presets, state, *, duration_s: float = 60.0,
                  dt: float = 0.02, pose_hold_s: float = 0.05,
                  predict: bool = True, damping: DampingSpec | None = None,
                  seed: int = 3, xy=(3.0, 2.0), yaw: float = 0.3,
                  depth_m: float = 5.0, cells_per_m: float = 10.0,
                  plant: PlantVertical | None = None) -> dict:
    """Heave / roll / pitch of the boat under ``state``, the wave wrench
    computed from a pose the "node" holds for ``pose_hold_s`` (Gazebo's
    20 Hz odometry) and optionally predicts.

    Forces on the true state: Gazebo's flat-plane box buoyancy, gravity at
    the CoG, the plugin's linear + quadratic damping and added mass of
    ``plant`` (default: the presets hull's plant, :func:`default_plant_for`),
    and the model's wrench (excitation, plus the hull damping when
    ``damping`` is active; None = the presets' own for a ``damping_ratio``
    hull, else none, as before). Returns the time series and the summary the
    smoke test gates on.
    """
    from .model import SeaModel
    from .presets import SeaSchedule

    gz_box, hull = coarse_hulls(presets.hull, cells_per_m)
    exc = WaveExcitation(gz_box, hull, **WaveExcitation.clip_from_config(
        presets.hull))
    pl = plant if plant is not None else default_plant_for(presets.hull)
    # damping None: the box catamaran (legacy file) ran with no hull damping
    # here, the plugin carrying the BlueROV2 terms; a ratio-based hull (the
    # identified plant, R-8) gets its own, because its plugin carries none.
    if damping is None and damping_ratios_from_config(presets.hull) is None:
        damping = DampingSpec()
    model = SeaModel(presets, SeaSchedule.constant(state), excitation=exc,
                     depth_m=depth_m, seed=seed, damping=damping, plant=pl)
    flat = lambda x, y: np.zeros_like(x)  # noqa: E731
    z_eq = gz_box.equilibrium_z()
    a33, a44, a55 = pl.added_mass_positive
    m_eff = pl.mass_kg + a33
    i_eff = (pl.inertia_link[0] + a44, pl.inertia_link[1] + a55)
    weight = pl.mass_kg * G
    lin, quad, cog_z = pl.linear, pl.quadratic, pl.cog_z_m

    n = int(round(duration_s / dt))
    # Torques are produced in world axes (ApplyLinkWrench semantics);
    # the roll / pitch rates are body-axis, so every torque is rotated
    # back by the yaw before it drives p and q (small roll/pitch).
    to_body = rotation_zyx(0.0, 0.0, yaw).T
    z, roll, pitch = z_eq, 0.0, 0.0
    vz, p, q = 0.0, 0.0, 0.0
    held: tuple[float, Pose3D, np.ndarray, np.ndarray] | None = None
    zs = np.zeros(n)
    rolls = np.zeros(n)
    pitches = np.zeros(n)
    fxs = np.zeros(n)
    mzs = np.zeros(n)
    for k in range(n):
        t = k * dt
        true_pose = Pose3D(xy[0], xy[1], z, roll=roll, pitch=pitch, yaw=yaw)
        v_body_true = np.array([0.0, 0.0, vz])
        w_body_true = np.array([p, q, 0.0])
        if held is None or t - held[0] >= pose_hold_s - 1e-9:
            held = (t, true_pose, v_body_true.copy(), w_body_true.copy())
        t_h, pose_h, v_h, w_h = held
        seen = predict_pose(pose_h, v_h, w_h, t - t_h) if predict else pose_h
        v_world_seen = rotation_zyx(seen.roll, seen.pitch, seen.yaw) @ v_h
        sample = model.update(t, seen, twist=(v_world_seen, w_h))
        f_w, t_w = sample.force, sample.torque
        fxs[k], mzs[k] = f_w[0], t_w[2]
        # Gazebo's own terms on the true state.
        f_gz, t_gz = gz_box.submerged_wrench(true_pose, flat)
        r_cog = rotation_zyx(roll, pitch, yaw) @ np.array([0.0, 0.0, cog_z])
        t_grav = np.cross(r_cog, np.array([0.0, 0.0, -weight]))
        fz = (f_gz[2] - weight + f_w[2]
              + lin[0] * vz + quad[0] * vz * abs(vz))
        tau_b = to_body @ (t_gz + t_grav + t_w)
        mx = (tau_b[0]
              + lin[1] * p + quad[1] * p * abs(p))
        my = (tau_b[1]
              + lin[2] * q + quad[2] * q * abs(q))
        # Semi-implicit Euler (velocity first).
        vz += fz / m_eff * dt
        p += mx / i_eff[0] * dt
        q += my / i_eff[1] * dt
        z += vz * dt
        roll += p * dt
        pitch += q * dt
        zs[k], rolls[k], pitches[k] = z, roll, pitch

    t_ax = np.arange(n) * dt
    heave = zs - zs.mean()
    spec = np.abs(np.fft.rfft(heave * np.hanning(n))) ** 2
    freq = np.fft.rfftfreq(n, d=dt)
    spec[freq < 0.05] = 0.0
    f_peak = float(freq[int(np.argmax(spec))]) if spec.any() else 0.0
    hf = float(spec[freq > 1.0].sum() / max(spec.sum(), 1e-12))
    return {
        "t": t_ax, "z": zs, "roll": rolls, "pitch": pitches,
        "z_eq": float(z_eq),
        "heave_std_m": float(heave.std()),
        "heave_p2p_m": float(zs.max() - zs.min()),
        "heave_peak_period_s": float(1.0 / f_peak) if f_peak > 0 else float("inf"),
        "heave_hf_fraction": hf,
        "roll_std_deg": float(np.degrees(rolls.std())),
        "pitch_std_deg": float(np.degrees(pitches.std())),
        "max_offset_m": float(np.abs(zs - z_eq).max()),
        "fx_std_n": float(fxs.std()),
        "mz_std_nm": float(mzs.std()),
        "finite": bool(np.isfinite(zs).all() and np.isfinite(rolls).all()
                       and np.isfinite(pitches).all()),
    }
