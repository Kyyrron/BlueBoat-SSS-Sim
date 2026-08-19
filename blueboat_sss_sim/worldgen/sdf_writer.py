"""SceneModel -> Gazebo world SDF.

Design choices:

* The seabed is exported as a **visual-only triangle mesh** (STL, decimated
  from the heightmap). The BlueBoat floats at the surface and never touches
  the seabed, so a seabed collision body would only cost physics time;
  it can be enabled (``seabed_collision: true``) for future AUV work.
* Objects are exported as simple SDF primitives (box / cylinder) with the
  material colour -- their *acoustic* appearance is owned entirely by the
  sonar renderer, Gazebo only needs them visible and physically present.
* The world-level plugin block is copied from the existing BlueBoat
  ``world.sdf`` so the vehicle spawns and behaves identically
  (physics, buoyancy, sensors, scene broadcaster). The plugin filename
  prefix is configurable: ``ignition-gazebo`` (Fortress, current project
  default) or ``gz-sim`` (Garden/Harmonic).
"""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

from ..core.types import PlacedObject
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
    if spec.sdf_shape == "cylinder_upright":
        geom = (f"<cylinder><radius>{o.length / 2:.3f}</radius>"
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


def write_world_sdf(scene: SceneModel, out_dir: str | Path,
                    world_name: str = "generated_ocean",
                    plugin_prefix: str = "ignition",
                    water_alpha: float = 0.4,
                    seabed_collision: bool = False,
                    material_overrides: dict | None = None) -> Path:
    """Write ``world.sdf`` + ``seabed.stl`` into ``out_dir``.

    Args:
        plugin_prefix: ``"ignition"`` (Fortress; matches the current BlueBoat
            world) or ``"gz"`` (Garden/Harmonic plugin naming).
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

    collision_block = ""
    if seabed_collision:
        collision_block = """
        <collision name="c">
          <geometry><mesh><uri>seabed.stl</uri></mesh></geometry>
        </collision>"""

    objects_sdf = "".join(
        _object_sdf(o, float(scene.sample_height(np.array([o.x]), np.array([o.y]))[0]), lib)
        for o in scene.objects)

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{world_name}">
    <scene><grid>false</grid></scene>
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
{objects_sdf}
  </world>
</sdf>
"""
    path = out / "world.sdf"
    path.write_text(sdf, encoding="utf-8")
    return path
