"""Hydrostatic (Froude-Krylov) buoyancy of box hulls under a wave surface.

Two hulls are modelled, both as unions of axis-aligned boxes in the body
frame, integrated identically:

* :meth:`HullBoxes.gazebo_box` -- the **Gazebo buoyancy box** of
  ``blueboat_description/urdf/blueboat.xacro`` (``x_size y_size z_size``
  scaled by ``buoyant_correction = 1.01·((mass + 2·prop_mass) /
  (ρ·x·y·z))^(1/3)``, centred at link z = +0.05), on which the world-level
  ``Buoyancy`` system applies graded buoyancy about the flat plane z = 0.
  That box floats with **1 % reserve buoyancy** (it is 99 % submerged at
  rest): under a wave crest it can gain at most 1 % of the weight, under a
  trough it loses most of its lift, so a boat driven by that box's own
  Froude-Krylov force would ride awash and sink by ~Hs/2 on average -- an
  artefact of the URDF, not of the water.
* :meth:`HullBoxes.blueboat_catamaran` -- the **hydrostatic hull** the wave
  excitation uses: two demihulls (BlueBoat 120: 1.2 m long, ~0.15 m wide,
  ~0.76 m apart, 0.376 m high) whose draft at the Gazebo trim reproduces
  the displacement, i.e. a realistic waterplane (0.36 m²) and reserve
  buoyancy, hence realistic heave/roll/pitch stiffness (a catamaran is
  stiff in roll through its hull separation).

The wrench published while waves are active is

    F = H_cat(wave surface) − H_gz(flat plane)

so that, added to the flat-plane term Gazebo itself applies (mirrored by
``H_gz(flat)``), the total force on the hull is the catamaran's buoyancy
under the actual wave surface (Newman 1977, *Marine Hydrodynamics*, ch. 6:
the long-wave Froude-Krylov limit; assumption S4 in the model doc). With
calm water the model publishes nothing and Gazebo's own hydrostatics stand
untouched -- the strict no-op the presets promise. Both hulls share the
trim ``z_eq`` by construction, so the two flat-plane terms are equal at
rest and the replacement introduces no static force.

Torque reference: ``ros_gz_interfaces/EntityWrench`` carries no force
offset across the bridge, so ``ApplyLinkWrench`` applies the force at the
link origin; every moment here is taken about that origin, in world axes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from ..core.types import Pose3D
from .spectrum import G

SurfaceFn = Callable[[np.ndarray, np.ndarray], np.ndarray]
#: (x, y) -> (dη/dx, dη/dy): the wave slope, for the horizontal force.
SlopeFn = Callable[[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]]


def rotation_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body -> world rotation matrix, ZYX (yaw, pitch, roll) convention --
    the inverse of :func:`core.geometry.quat_to_rpy`."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


@dataclass(frozen=True)
class Box:
    """Axis-aligned box in the body frame: size and centre."""
    lx: float
    ly: float
    lz: float
    cx: float = 0.0
    cy: float = 0.0
    cz: float = 0.0


@dataclass
class HullBoxes:
    """A hull as a union of body-frame boxes (see module docstring)."""

    mass_kg: float
    boxes: list[Box]
    water_density: float = 1000.0
    name: str = "hull"
    cells_per_m: float = 40.0
    _fx: np.ndarray = field(init=False, repr=False)
    _fy: np.ndarray = field(init=False, repr=False)
    _ztop: np.ndarray = field(init=False, repr=False)
    _zbot: np.ndarray = field(init=False, repr=False)
    _da: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        fx, fy, zt, zb, da = [], [], [], [], []
        for b in self.boxes:
            nx = max(int(round(b.lx * self.cells_per_m)), 4)
            ny = max(int(round(b.ly * self.cells_per_m)), 4)
            xs = (np.arange(nx) + 0.5) / nx * b.lx - b.lx / 2.0 + b.cx
            ys = (np.arange(ny) + 0.5) / ny * b.ly - b.ly / 2.0 + b.cy
            gx, gy = np.meshgrid(xs, ys, indexing="ij")
            n = gx.size
            fx.append(gx.ravel())
            fy.append(gy.ravel())
            zt.append(np.full(n, b.cz + b.lz / 2.0))
            zb.append(np.full(n, b.cz - b.lz / 2.0))
            da.append(np.full(n, (b.lx / nx) * (b.ly / ny)))
        self._fx = np.concatenate(fx)
        self._fy = np.concatenate(fy)
        self._ztop = np.concatenate(zt)
        self._zbot = np.concatenate(zb)
        self._da = np.concatenate(da)

    # ---------------------------------------------------------- factories
    @staticmethod
    def gazebo_box(mass: float = 16.01, prop_mass: float = 0.07,
                   x_size: float = 1.2, y_size: float = 0.91,
                   z_size: float = 0.376, density: float = 1000.0,
                   offset_z: float = 0.05) -> "HullBoxes":
        """The URDF collision box, from the xacro's own formula."""
        total = mass + 2.0 * prop_mass
        bc = 1.01 * (total / (density * x_size * y_size * z_size)) ** (1.0 / 3.0)
        return HullBoxes(total, [Box(x_size * bc, y_size * bc, z_size * bc,
                                     0.0, 0.0, offset_z)],
                         density, name="gazebo_box")

    @staticmethod
    def blueboat_catamaran(mass: float = 16.15, length: float = 1.2,
                           demihull_beam: float = 0.15,
                           separation: float = 0.76, height: float = 0.376,
                           trim_z: float | None = None,
                           density: float = 1000.0) -> "HullBoxes":
        """Two demihulls placed so that, with the link origin at the Gazebo
        trim ``trim_z`` (default: the Gazebo box's own equilibrium), the
        submerged volume equals the displacement ``mass / ρ``."""
        if trim_z is None:
            trim_z = HullBoxes.gazebo_box().equilibrium_z()
        area = 2.0 * length * demihull_beam
        draft = mass / (density * area)
        # World waterline z = 0 sits at link z = -trim_z; the bottoms are
        # `draft` below it.
        bottom = -trim_z - draft
        cz = bottom + height / 2.0
        boxes = [Box(length, demihull_beam, height, 0.0, +separation / 2.0, cz),
                 Box(length, demihull_beam, height, 0.0, -separation / 2.0, cz)]
        return HullBoxes(mass, boxes, density, name="blueboat_catamaran")

    @staticmethod
    def from_config(cfg: dict) -> tuple["HullBoxes", "HullBoxes"]:
        """(gazebo box, hydrostatic hull) from the ``hull:`` block of
        ``config/sea_states.yaml``; unknown keys raise by name."""
        known = {"mass_kg", "gazebo_box", "catamaran", "water_density",
                 "target_entity", "target_entity_type", "damping",
                 "wave_force_clip"}
        unknown = set(cfg) - known
        if unknown:
            raise KeyError(f"Unknown hull keys: {sorted(unknown)}")
        rho = float(cfg.get("water_density", 1000.0))
        gz = dict(cfg.get("gazebo_box") or {})
        gz_known = {"mass", "prop_mass", "x_size", "y_size", "z_size", "offset_z"}
        if set(gz) - gz_known:
            raise KeyError(f"Unknown hull.gazebo_box keys: "
                           f"{sorted(set(gz) - gz_known)}")
        box = HullBoxes.gazebo_box(density=rho, **gz)
        cat = dict(cfg.get("catamaran") or {})
        cat_known = {"length", "demihull_beam", "separation", "height"}
        if set(cat) - cat_known:
            raise KeyError(f"Unknown hull.catamaran keys: "
                           f"{sorted(set(cat) - cat_known)}")
        hull = HullBoxes.blueboat_catamaran(
            mass=float(cfg.get("mass_kg", box.mass_kg)), density=rho,
            trim_z=box.equilibrium_z(), **cat)
        return box, hull

    # ---------------------------------------------------------- geometry
    @property
    def volume_m3(self) -> float:
        return float(sum(b.lx * b.ly * b.lz for b in self.boxes))

    @property
    def waterplane_area_m2(self) -> float:
        return float(sum(b.lx * b.ly for b in self.boxes))

    # -------------------------------------------------------- integration
    def submerged_wrench(self, pose: Pose3D, surface: SurfaceFn,
                         slope: SlopeFn | None = None
                         ) -> tuple[np.ndarray, np.ndarray]:
        """Buoyancy force (world, N) and moment about the link origin
        (world, N·m) under the surface ``surface(x, y) -> η``: vertical
        columns over the body-frame footprint, each submerged by
        clip(η − z_bottom, 0, z_top − z_bottom).

        With ``slope`` the linear Froude-Krylov **horizontal** force is
        added too: the pressure gradient under a sloping surface pushes
        each submerged column downhill, ``f_h = −ρ g V ∇η`` (Newman 1977
        ch. 6, long-wave limit), applied at the column's mid-depth so it
        also yaws the hull when the slope differs across the demihulls.
        Without ``slope`` the result is bit-identical to the vertical-only
        model (flat water gives exactly zero either way)."""
        r = rotation_zyx(pose.roll, pose.pitch, pose.yaw)
        pos = np.array([pose.x, pose.y, pose.z])
        top_b = np.stack([self._fx, self._fy, self._ztop], 1)
        bot_b = np.stack([self._fx, self._fy, self._zbot], 1)
        top_w = pos + top_b @ r.T
        bot_w = pos + bot_b @ r.T
        mid_xy = 0.5 * (top_w[:, :2] + bot_w[:, :2])
        eta = np.asarray(surface(mid_xy[:, 0], mid_xy[:, 1]), dtype=np.float64)
        height = np.clip(top_w[:, 2] - bot_w[:, 2], 1e-6, None)
        h = np.clip(eta - bot_w[:, 2], 0.0, height)
        f = self.water_density * G * h * self._da
        force = np.array([0.0, 0.0, float(f.sum())])
        lever_x = mid_xy[:, 0] - pos[0]
        lever_y = mid_xy[:, 1] - pos[1]
        # tau = sum(lever x (0,0,f)) = sum([ly*f, -lx*f, 0])
        torque = np.array([float((lever_y * f).sum()),
                           float((-lever_x * f).sum()), 0.0])
        if slope is not None:
            sx, sy = slope(mid_xy[:, 0], mid_xy[:, 1])
            fx = -f * np.asarray(sx, dtype=np.float64)
            fy = -f * np.asarray(sy, dtype=np.float64)
            lever_z = bot_w[:, 2] + 0.5 * h - pos[2]
            force[0] += float(fx.sum())
            force[1] += float(fy.sum())
            torque[0] += float((-lever_z * fy).sum())
            torque[1] += float((lever_z * fx).sum())
            torque[2] += float((lever_x * fy - lever_y * fx).sum())
        return force, torque

    # ----------------------------------------------------------- statics
    def equilibrium_z(self) -> float:
        """Link-origin height at which the flat-plane buoyancy equals the
        weight (bisection), i.e. the still-water trim."""
        target = self.mass_kg * G
        flat = lambda x, y: np.zeros_like(x)  # noqa: E731
        lo, hi = -2.0, 2.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            f, _ = self.submerged_wrench(Pose3D(0.0, 0.0, mid), flat)
            if f[2] > target:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def restoring_summary(self, inertia=(2.42369, 3.9235025, 5.6403125),
                          cog_z: float = -0.12913,
                          added_mass=(14.57, 0.12, 0.12)) -> dict:
        """Hydrostatic stiffness (finite-difference about the trim) and
        natural periods, from the URDF inertia and the Hydrodynamics
        plugin's added-mass diagonal (heave zDotW, roll kDotP, pitch
        mDotQ) -- reported in the model doc."""
        flat = lambda x, y: np.zeros_like(x)  # noqa: E731
        z_eq = self.equilibrium_z()
        d = 0.002
        fz_up = self.submerged_wrench(Pose3D(0, 0, z_eq + d), flat)[0][2]
        fz_dn = self.submerged_wrench(Pose3D(0, 0, z_eq - d), flat)[0][2]
        c33_up = (self.mass_kg * G - fz_up) / d
        c33_dn = (fz_dn - self.mass_kg * G) / d
        ang = 0.01
        # Restoring about the link origin plus the weight's own moment
        # (gravity acts at the CoG, at link z = cog_z below the origin).
        mx = self.submerged_wrench(Pose3D(0, 0, z_eq, roll=ang), flat)[1][0]
        my = self.submerged_wrench(Pose3D(0, 0, z_eq, pitch=ang), flat)[1][1]
        w_arm = self.mass_kg * G * (-cog_z)        # CoG below origin
        c44 = -(mx - w_arm * np.sin(ang)) / ang
        c55 = -(my + w_arm * np.sin(ang)) / ang
        ixx, iyy, _ = inertia
        return {"name": self.name, "equilibrium_z_m": float(z_eq),
                "waterplane_m2": self.waterplane_area_m2,
                "volume_m3": self.volume_m3,
                "reserve_buoyancy": float(self.volume_m3 * self.water_density
                                          / self.mass_kg - 1.0),
                "c33_up_n_per_m": float(c33_up), "c33_down_n_per_m": float(c33_dn),
                "c44_nm_per_rad": float(c44), "c55_nm_per_rad": float(c55),
                "t_heave_s": float(2 * np.pi * np.sqrt(
                    (self.mass_kg + added_mass[0]) / max(c33_up, 1e-9))),
                "t_roll_s": float(2 * np.pi * np.sqrt(
                    (ixx + added_mass[1]) / max(c44, 1e-9))),
                "t_pitch_s": float(2 * np.pi * np.sqrt(
                    (iyy + added_mass[2]) / max(c55, 1e-9)))}


class WaveExcitation:
    """F = H_hull(wave) − H_gz(flat): the wrench to publish while waves are
    active (see module docstring).

    The catamaran's *excess* buoyancy ``H_hull,z − m g`` is soft-clipped
    above ``clip_knee_g`` times the weight (a C1 tanh knee of width
    ``clip_width_g`` weights): a shallow-draft hull buried by a crest it
    did not ride would otherwise publish several weights of slam. Below
    the knee nothing changes; a contouring hull never reaches it.
    """

    def __init__(self, gazebo_box: HullBoxes, hull: HullBoxes,
                 clip_knee_g: float = 2.5, clip_width_g: float = 1.0) -> None:
        self.gazebo_box = gazebo_box
        self.hull = hull
        self.clip_knee_g = float(clip_knee_g)
        self.clip_width_g = float(clip_width_g)

    @staticmethod
    def clip_from_config(cfg) -> dict:
        """``hull.wave_force_clip`` of the presets (unknown keys raise)."""
        raw = dict((cfg or {}).get("wave_force_clip") or {})
        unknown = set(raw) - {"knee_g", "width_g"}
        if unknown:
            raise KeyError(f"Unknown hull.wave_force_clip keys: {sorted(unknown)}")
        return {"clip_knee_g": float(raw.get("knee_g", 2.5)),
                "clip_width_g": float(raw.get("width_g", 1.0))}

    def __call__(self, pose: Pose3D, surface: SurfaceFn,
                 slope: SlopeFn | None = None
                 ) -> tuple[np.ndarray, np.ndarray]:
        fw, tw = self.hull.submerged_wrench(pose, surface, slope)
        ff, tf = self.gazebo_box.submerged_wrench(pose, lambda x, y: np.zeros_like(x))
        mg = self.hull.mass_kg * G
        e = float(fw[2]) - mg
        knee, width = self.clip_knee_g * mg, self.clip_width_g * mg
        if e > knee and width > 0.0:
            e_c = knee + width * np.tanh((e - knee) / width)
            ratio = (mg + e_c) / max(float(fw[2]), 1e-9)
            fw = fw * ratio
            tw = tw * ratio
        return fw - ff, tw - tf


# Backwards-compatible name used by the package __init__ / smoke test.
HullBox = HullBoxes
