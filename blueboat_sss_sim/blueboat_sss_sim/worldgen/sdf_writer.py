"""SceneModel -> Gazebo world SDF.

Design choices:

* The seabed is exported as a **visual-only triangle mesh** (STL, decimated
  from the heightmap). The BlueBoat floats at the surface and never touches
  the seabed, so a seabed collision body would only cost physics time;
  it can be enabled (``seabed_collision: true``) for future AUV work.
* Objects are exported as simple SDF primitives (box / cylinder) with the
  material colour -- their *acoustic* appearance is owned entirely by the
  sonar renderer, Gazebo only needs them visible (they carry no
  collision body; the walls do).
* The world-level plugin block is copied from the existing BlueBoat
  ``world.sdf`` so the vehicle spawns and behaves identically
  (physics, buoyancy, sensors, scene broadcaster). The plugin filename
  prefix is configurable: ``gz-sim`` (Garden/Harmonic, the default) or
  ``ignition-gazebo`` (Fortress).
"""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

from ..core.types import PlacedObject, Wall
from .materials import MaterialLibrary
from .objects import CATALOG
from .scene import SceneModel

_WORLD_PLUGINS = """\
    <physics name="10ms" type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="{p}-physics-system" name="{n}::Physics"></plugin>
    <plugin filename="{p}-user-commands-system" name="{n}::UserCommands"></plugin>
    <plugin filename="{p}-scene-broadcaster-system" name="{n}::SceneBroadcaster"></plugin>
    <plugin filename="{p}-imu-system" name="{n}::Imu"></plugin>
    <plugin filename="{p}-sensors-system" name="{n}::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="{p}-buoyancy-system" name="{n}::Buoyancy">
      <graded_buoyancy>
        <default_density>1000</default_density>
        <density_change>
          <above_depth>0</above_depth>
          <density>1</density>
        </density_change>
      </graded_buoyancy>
    </plugin>
    <plugin filename="{p}-apply-link-wrench-system" name="{n}::ApplyLinkWrench"></plugin>
"""


def _write_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    """Minimal binary STL writer (normals left zero; Gazebo recomputes)."""
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(faces)))
        tri = vertices[faces]  # (n, 3, 3)
        zero_n = struct.pack("<3f", 0.0, 0.0, 0.0)
        for a, b, c in tri:
            f.write(zero_n)
            f.write(struct.pack("<9f", *a, *b, *c))
            f.write(b"\0\0")


def _seabed_mesh(scene: SceneModel, max_verts_axis: int = 200
                 ) -> tuple[np.ndarray, np.ndarray]:
    """Decimate the heightmap into a triangle grid mesh (world coords)."""
    g = scene.grid
    step = max(1, int(np.ceil(max(g.nx, g.ny) / max_verts_axis)))
    hz = scene.height[::step, ::step]
    ny, nx = hz.shape
    xs = g.origin_x + np.arange(0, g.nx, step)[:nx] * g.resolution
    ys = g.origin_y + np.arange(0, g.ny, step)[:ny] * g.resolution
    xx, yy = np.meshgrid(xs, ys)
    verts = np.stack([xx, yy, hz], axis=-1).reshape(-1, 3).astype(np.float32)

    idx = np.arange(ny * nx).reshape(ny, nx)
    a = idx[:-1, :-1].ravel()
    b = idx[:-1, 1:].ravel()
    c = idx[1:, :-1].ravel()
    d = idx[1:, 1:].ravel()
    faces = np.concatenate([np.stack([a, b, c], 1), np.stack([b, d, c], 1)])
    return verts, faces.astype(np.int64)


def _object_sdf(o: PlacedObject, z_base: float, lib: MaterialLibrary) -> str:
    spec = CATALOG[o.type]
    color = lib.get(o.material).color
    h = max(o.effective_height, 0.01)
    z = z_base + h / 2 - o.proud_height * o.burial * 0.5
    if o.rested and spec.solid(o) is not None:
        return _rested_object_sdf(o, spec, z_base, color)
    if spec.sdf_shape == "cylinder_upright":
        geom = (f"<cylinder><radius>{o.length / 2:.3f}</radius>"
                f"<length>{h:.3f}</length></cylinder>")
        pose = f"{o.x:.3f} {o.y:.3f} {z:.3f} 0 0 {o.yaw:.4f}"
    elif spec.sdf_shape == "cone":
        # SDF <cone> only exists from SDFormat 15 (Ionic); Harmonic renders
        # nothing for it. An upright cylinder at half the base radius is the
        # visual stand-in -- the acoustic shape is entirely mask-driven.
        geom = (f"<cylinder><radius>{o.length / 4:.3f}</radius>"
                f"<length>{h:.3f}</length></cylinder>")
        pose = f"{o.x:.3f} {o.y:.3f} {z:.3f} 0 0 {o.yaw:.4f}"
    elif spec.sdf_shape == "cylinder":
        geom = (f"<cylinder><radius>{o.width / 2:.3f}</radius>"
                f"<length>{o.length:.3f}</length></cylinder>")
        pose = f"{o.x:.3f} {o.y:.3f} {z:.3f} 0 1.5708 {o.yaw:.4f}"
    else:
        geom = (f"<box><size>{o.length:.3f} {o.width:.3f} {h:.3f}</size></box>")
        pose = f"{o.x:.3f} {o.y:.3f} {z:.3f} 0 0 {o.yaw:.4f}"
    r, gg, b = color
    return f"""
    <model name="obj_{o.object_id:04d}_{o.type}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <visual name="v">
          <geometry>{geom}</geometry>
          <material>
            <ambient>{r} {gg} {b} 1</ambient>
            <diffuse>{r} {gg} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _wall_sdf(w: Wall, z_floor: float) -> str:
    """One basin wall as a static box, visible and *solid*.

    Walls carry a collision mirroring the visual: like a real quay or
    pontoon, the boat cannot drive through one. They are also emitted so
    that the Gazebo scene and the acoustic scene cannot disagree about a
    structure this large -- the renderer bounces ghosts off these same
    walls, read from the same manifest.
    """
    cx, cy = 0.5 * (w.x0 + w.x1), 0.5 * (w.y0 + w.y1)
    # A pontoon floats: its box spans the draft to the top and leaves the
    # water beneath it free, exactly as the acoustic model treats it.
    z_base = -float(w.draft_m) if w.kind == "pontoon" else z_floor
    h = max(w.top_z - z_base, 0.05)
    cz = z_base + h / 2.0
    yaw = float(np.arctan2(w.y1 - w.y0, w.x1 - w.x0))
    box = f"<box><size>{w.length:.3f} {w.thickness:.3f} {h:.3f}</size></box>"
    return f"""
    <model name="wall_{w.name}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <visual name="v">
          <geometry>{box}</geometry>
          <material>
            <ambient>0.55 0.55 0.52 1</ambient>
            <diffuse>0.55 0.55 0.52 1</diffuse>
          </material>
        </visual>
        <collision name="c">
          <geometry>{box}</geometry>
        </collision>
      </link>
    </model>"""


def _rested_object_sdf(o: PlacedObject, spec, z_base: float, color) -> str:
    """A 3-D rested object: the same primitive the acoustic stamp used,
    at the settled pose (rest orientation composed with the bank tilt,
    centre ``rest_z`` above the pre-stamp terrain)."""
    from .solids import rotation_rpy, rpy_from_rotation, rot_y
    sol = spec.solid(o)
    R = rotation_rpy(o.roll, o.pitch, o.yaw)
    if sol.kind in ("cylinder", "capsule") and sol.axis == 0:
        # The SDF cylinder's axis is z; the class frame has it along x.
        R = R @ rot_y(np.pi / 2)
        geom = (f"<cylinder><radius>{sol.a:.3f}</radius>"
                f"<length>{sol.b:.3f}</length></cylinder>")
    elif sol.kind in ("cylinder", "annulus"):
        geom = (f"<cylinder><radius>{sol.a:.3f}</radius>"
                f"<length>{(sol.b if sol.kind == 'cylinder' else sol.c):.3f}"
                f"</length></cylinder>")
    elif sol.kind == "cone":
        geom = (f"<cylinder><radius>{sol.a / 2:.3f}</radius>"
                f"<length>{sol.b:.3f}</length></cylinder>")
    else:
        geom = (f"<box><size>{sol.a:.3f} {sol.b:.3f} {sol.c:.3f}</size></box>")
    roll, pitch, yaw = rpy_from_rotation(R)
    z = z_base + o.rest_z
    r, gg, b = color
    return f"""
    <model name="obj_{o.object_id:04d}_{o.type}">
      <static>true</static>
      <pose>{o.x:.3f} {o.y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
      <link name="link">
        <visual name="v">
          <geometry>{geom}</geometry>
          <material>
            <ambient>{r} {gg} {b} 1</ambient>
            <diffuse>{r} {gg} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _limits_sdf(scene: SceneModel) -> str:
    """Four invisible, collision-only boxes just outside the world extent.

    The world-builder's "limits": the boat physically cannot leave the
    drawn rectangle, but nothing is visible and -- crucially -- nothing is
    added to ``scene.walls``, so the sonar never sees them (limits are
    non-reflective by design; only authored walls reflect).
    """
    xmin, ymin, xmax, ymax = scene.grid.extent
    z_lo = float(scene.height.min()) - 0.5
    # Reach over any dry land (a beach above z = 0) as well as the water.
    z_hi = max(1.0, float(scene.height.max()) + 0.5)
    h = z_hi - z_lo
    cz = z_lo + h / 2.0
    t = 0.5
    sx = (xmax - xmin) + 2.0 * t
    sy = (ymax - ymin) + 2.0 * t
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    sides = (
        ("north", cx, ymax + t / 2, sx, t),
        ("south", cx, ymin - t / 2, sx, t),
        ("east", xmax + t / 2, cy, t, sy),
        ("west", xmin - t / 2, cy, t, sy),
    )
    out = []
    for name, px, py, bx, by in sides:
        out.append(f"""
    <model name="limit_{name}">
      <static>true</static>
      <pose>{px:.3f} {py:.3f} {cz:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{bx:.3f} {by:.3f} {h:.3f}</size></box></geometry>
        </collision>
      </link>
    </model>""")
    return "".join(out)


def write_world_sdf(scene: SceneModel, out_dir: str | Path,
                    world_name: str = "generated_ocean",
                    plugin_prefix: str = "gz",
                    water_alpha: float = 0.4,
                    seabed_collision: bool = False,
                    material_overrides: dict | None = None,
                    limits_collision: bool = False) -> Path:
    """Write ``world.sdf`` + ``seabed.stl`` into ``out_dir``.

    Args:
        plugin_prefix: ``"gz"`` (Garden/Harmonic plugin naming) or
            ``"ignition"`` (Fortress).
        limits_collision: emit four invisible collision-only boxes along
            the world extent (the builder's impenetrable, non-reflective
            limits).
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    lib = MaterialLibrary(material_overrides)

    verts, faces = _seabed_mesh(scene)
    _write_stl(out / "seabed.stl", verts, faces)

    if plugin_prefix == "gz":
        p, n = "gz-sim", "gz::sim::systems"
    else:
        p, n = "ignition-gazebo", "ignition::gazebo::systems"

    xmin, ymin, xmax, ymax = scene.grid.extent
    sx, sy = xmax - xmin, ymax - ymin
    # The water plane must cover the grid extent, not sit at the Gazebo
    # origin: a builder world's rectangle is drawn around the GPS path and
    # is generally offset from (0, 0) (the anchor). The legacy configs'
    # origin = (-size/2, -size/2) made the two coincide and hid this.
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)

    # GPS anchor: Gazebo's standard mechanism for georeferencing a world.
    # World (0, 0) is the anchor; the world frame is its local ENU frame,
    # matching the manifest's geo block and the MCS path design frame.
    geo_block = ""
    if scene.geo_anchor is not None:
        a = scene.geo_anchor
        geo_block = f"""
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{a.lat0:.9f}</latitude_deg>
      <longitude_deg>{a.lon0:.9f}</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""

    collision_block = ""
    if seabed_collision:
        collision_block = """
        <collision name="c">
          <geometry><mesh><uri>seabed.stl</uri></mesh></geometry>
        </collision>"""

    objects_sdf = "".join(
        _object_sdf(o, float(scene.sample_height(np.array([o.x]), np.array([o.y]))[0]), lib)
        for o in scene.objects)
    # Walls sit on the deepest point they span, so a sloping basin never
    # leaves one floating above the seabed.
    walls_sdf = "".join(
        _wall_sdf(w, float(np.min(scene.sample_height(
            np.linspace(w.x0, w.x1, 16), np.linspace(w.y0, w.y1, 16)))))
        for w in scene.walls)
    limits_sdf = _limits_sdf(scene) if limits_collision else ""

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{world_name}">
    <scene><grid>false</grid></scene>{geo_block}
{_WORLD_PLUGINS.format(p=p, n=n)}
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="water_plane">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>
      <link name="link">
        <visual name="water_plane">
          <geometry>
            <plane><size>{sx:.1f} {sy:.1f}</size><normal>0 0 1</normal></plane>
          </geometry>
          <material>
            <ambient>0 0 1 {water_alpha}</ambient>
            <diffuse>0 0 1 {water_alpha}</diffuse>
            <specular>0 0 1 {water_alpha}</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="seabed">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <geometry><mesh><uri>seabed.stl</uri></mesh></geometry>
          <material>
            <ambient>0.65 0.58 0.42 1</ambient>
            <diffuse>0.65 0.58 0.42 1</diffuse>
          </material>
        </visual>{collision_block}
      </link>
    </model>
{objects_sdf}{walls_sdf}{limits_sdf}
  </world>
</sdf>
"""
    path = out / "world.sdf"
    path.write_text(sdf, encoding="utf-8")
    return path
