# BlueBoat-SSS-Sim — architecture, commands and working notes (for a future simulator version)

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 1. What this module is for

The thesis tests *aspect-aware adaptive replanning* for SSS survey from a
surface vehicle; its headline result is a model-conditional **simulated**
comparison against a seriously tuned two-pass orthogonal survey (the Replanner is
built but has not yet run in Gazebo; Evaluation is planned, not built; superproject
`.claude/CLAUDE.md` §1), so
output must be defensible as evidence, not merely plausible imagery. (2026-11-18) A
built-world run hosts a replanner-owned run with `full_mission_launch.py path_source:=replanner`
(Control `docs/REQUESTS.md` R-3, owner order; launch file only, outside the code digest). **Worlds in, SSS data
out:** World Builder = MCS anchored path + operator intent → georeferenced
Gazebo world with ground truth; sonar node = world + vehicle odom →
profiles, raw frames, ground-truth contacts. Pictures are the visualization
app's job, missions MCS/Control's, training data BlueBoat-Perception's dataset
mode's (the in-package YOLO recorder was deleted in the 2026-09 rework; the
dataset mode labels the seabed images from this node's
`/side_scan_sonar/ground_truth/*` topics).

---

## 2. Architecture in one page

**The sonar is not a Gazebo plugin.** Gazebo owns hull dynamics and pose; a
ROS 2 Python node owns acoustics, rendering against the same generated scene
→ the acoustic scene is static during a run (fine for seabed litter); the
renderer is swappable without touching Gazebo. Module map and data flow:
`docs/architecture.md`.

**Sea state:** the ROS-free `sea/` package drives `ros/sea_state_node.py`
(current + a wave wrench into Gazebo; since 2026-11-15 also wind on the windage, the
ported yaw-moment stand-in and a kappa bracket, the CAD hull for the identified plant
with damping sized on the plant read from the robot description, and the frozen
`sea_states_legacy.yaml` for the legacy plant -- R-8, `docs/sea_state_model.md`
§5.9-5.11); its model notes are in
`blueboat_sss_sim/blueboat_sss_sim/sea/CLAUDE.md` (loaded when working there),
the physics and DOIs in `docs/sea_state_model.md`.

**Single source of truth:** one `SceneModel` emits both `world.sdf` and
`scene.npz`, so they cannot diverge; never generate one without the other.

**GPS anchoring:** world ENU = the source path's design frame **translated so
world (0, 0) = the spawn origin** (the path's first waypoint if inside the
limits, else the rectangle centre; `BuilderState.spawn_origin`). The boat
always spawns at (0, 0) (hard-coded by `upload_rov_launch.py` / MCS
`Sim_launch.py`), so it starts inside the area; `geo_anchor` = that point's
GPS. A path whose waypoints sit far from its designer origin still yields a
world the boat spawns inside — every conversion goes through lat/lon.
`core/geo.py` = the project-wide equirectangular pair (byte-for-byte
MCS/GCS). `sss_sim_launch.py` feeds the MAVROS shim the world's anchor.

**Layering:** `core/`, `worldgen/`, `sonar/`, `analysis/` import no ROS and
no Qt (standalone-testable); `ros/` is a thin ROS shell; `builder/` is the
only Qt code and imports no ROS (`builder/state.py`, `builder/paths.py` are
Qt-free so the smoke test drives the whole build headless). Keep it so — the
offline smoke test depends on it. `builder/layout.py` = the Qt screen-size
contract every page obeys.

**Field calibration and multipath:** `blueboat_sss_sim/calibration/` (ROS-free;
`sss_calibration`): parameter registry (smoke `[16]`), session manifests, seabed engine
(`score | fit | scan`). A calibration never edits `default_sonar.yaml`. `sonar/waveguide.py`
adds the shallow-water image comb; each term is **off / neutral by default** (bit-identical),
and the pool values live only in the generated site overlay
`config/pool_environment_pass4_sonar.yaml`, never a field profile. Tooling map, pass
history and the NC #6 constant history: `blueboat_sss_sim/blueboat_sss_sim/calibration/CLAUDE.md`
(loads when working there; read it before any calibration or level-constant work).

**Extension seams:** `SonarRenderer` (ABC, `sonar/renderer.py`; everything
downstream is renderer-agnostic); `DEPTH_PRESETS`
(`worldgen/depth/presets.py`), `CATALOG` (`worldgen/objects.py`),
`register(LayerSpec(...))` (`worldgen/depth/layers/`) — one registry entry
adds a bathymetry pattern / object class / seabed layer to the GUI with zero
GUI edits (§8); sonar profiles `config/*_sonar.yaml` — one file per idea,
each the whole model, so a frozen copy renders with its frozen constants.

**Crash safety:** a world builds into a hidden `.<name>.partial` sibling,
**renamed into place** at the end — a `~/worlds` directory is a whole world
or absent; a dead build's partial is swept by the next build. The wizard
autosaves every page turn (`~/.cache/blueboat_sss_sim/builder_autosave.yaml`).

---

## 7. Commands

All package-relative commands run from the package source root,
`BlueBoat-SSS-Sim/blueboat_sss_sim/`. Offline tools are `python3 -m
blueboat_sss_sim.<module>` (no ROS); ROS entry points need `colcon build
--packages-select blueboat_sss_sim` from `~/ros2_ws`. Pinging starts with
`ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'`.
The dev end-to-end (`full_mission_launch.py with_control:=true`) runs from
the workspace root with `source env.sh` (venv + workspace) —
`blueboat_control`'s nodes import casadi/acados at module scope regardless
of `controller_type`.

### Offline smoke test

```bash
python3 -m test.smoke_test        # from the package source root
```

The only automated gate of the Stop hook (no lint, no type-check). ~540
checks (sections to `[20]`: object set version 2, population, campaign), ~260 s, on the system python; the venv runs one visible `SKIP`
fewer checks (the bridge checks and the `ApplyLinkWrench` detection need
ROS; one more SKIP when the `BlueBoat-SSS` checkout beside it is missing). The venv
interpreter is `~/ros2_ws/.venv/bin/python3` (`env.sh` activates it only
when sourced from `~/ros2_ws`). Which checks were added when: `git log -p test/smoke_test.py`.
The slow calibration checks are `python3 -m test.calibration_fit_test` (manual, ~2 min).
`[12]` imports PySide6's QtGui, which needs `libEGL.so.1`: a container
without it crashes there (`apt-get install libegl1`).
Section `[10]` (sea state) is unit-level only. Boat motion is never simulated offline (the
user's rule, 2026-10-08): the existing `simulate_3dof` stays the only
offline loop. `grep -n 'print("\['
test/smoke_test.py` lists the sections. Two rules the code does not
explain: the **render fixture** `test/fixtures/render_baseline_v1.npz` is
rebaselined only for a deliberate model change, by deleting the file, and only after
the old model has re-rendered the old fixture bit for bit; it is
compared to **float precision** (1e-12 of each array's scale) — the venv's
numpy 2.4 and the system numpy 1.26 differ by a few ULP. It renders with the default
profile's mount, so a geometry change in `config/default_sonar.yaml` is a model change
(and checks with hard-coded geometry must be made mount-aware, never widened). `[2f]` reads the
field corpus at `$BLUEBOAT_SSS_CORPUS` (default `~/ros2_ws/data/SSS_data`) and
skips without it. Calibration gates: `[2c]`, `[2e]`, `[2f]`; object physics:
`[14]`. Runs serialize on `/tmp/blueboat_sss_smoke.lock` — give a manual run
a generous timeout. GUI checks, outside the gate:
`QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke`.

### Gazebo gate — waves and current end to end

Real time, ~5–6 min per scenario, **not** in the Stop hook. Command, checks, where the validation inputs live and how they are
rebuilt: `.claude/gazebo-gate.md` — read it before running or changing
`test/gazebo_sea_test.py` or `analysis/sea_validation.py`.

### Session hooks

`.claude/settings.json` (gitignored) runs `.claude/tools/smoke_gate.py`
(stdlib only; its docstring is the spec): **PostToolUse** marks the session
dirty on an in-scope edit and reminds to regenerate worlds after
`config/`/`sonar/`/`worldgen/` edits; **Stop** runs the suite when an
in-scope file changed and **exits 2 to block the stop** until it passes.
The hook's `timeout` (450 s) must stay above the script's
`SUITE_TIMEOUT_S` (420 s, raised from 300 s on 2026-10-20 when the suite
reached ~205 s): a harness-cancelled Stop hook does not block, so
a too-short timeout makes the gate pass silently.

---

## 8. Working on this module

- **Read `docs/` first** — `architecture.md`, `sonar_model.md` (physics,
  assumptions A1–A13), `topics.md` (raw-frame byte map),
  `integration_guide.md`, `configuration_guide.md`, `developer_guide.md`,
  `roadmap.md`.
- Frames: world ENU, z = 0 at the surface, depths negative; world (0, 0) is
  the GPS anchor. Internal math uses ENU yaw; hardware- or user-facing
  values use compass degrees. Angle conversions in `core/geometry.py`;
  **geodesy only in `core/geo.py`** (never a second formula, never a
  neighbour's copy).
- Sides: `Side.PORT.sign = +1` (+y body), `STARBOARD = -1`; look direction =
  vehicle heading ∓ 90°. On the wire `transducer_heading_deg` is the
  **mounting angle**, constant −90 (port) / +90 (starboard), as the device
  reports it (`sonar/encoder.py`, since 2026-09-15; older sim recordings
  carry the absolute bearing); `vehicle_heading_deg` carries the compass
  heading. `Side.channel` (0 port, 1 starboard) goes to byte 34 of the raw
  frame, the authoritative side identity downstream.
- Randomness: every stochastic component takes a `numpy.random.Generator`;
  nothing touches the global RNG. World content derives entirely from the
  seed (split streams under `depth:` — §5).
- Wizard controls: tool rows are `builder.layout.ToolBar` (wrapping),
  previews `ScaledImage`, side-by-side panels `split(...)`; never a bare
  `QHBoxLayout` tool row, whose minimum width becomes the window's.
- Adding a litter class / depth pattern / seabed layer: one registry entry
  (`CATALOG`, `DEPTH_PRESETS`, `register(LayerSpec)`) —
  `docs/developer_guide.md`. A class reproducing a *known* object gives its
  dimensions as `params_schema` rows overriding the drawn sizes **after** the
  draw, so the RNG stream is untouched. How it SOUNDS is never a class
  setting (NC #17). Construction knobs the renderer reads (`wall_mm`,
  `flooded`) go in `_CONSTRUCTION_PARAMS` so a pinned entry carries them.
  A class that is not one solid gets a builder in `worldgen/compounds.py`
  (parts inside the stored footprint, from the instance's own seeded
  values); the ray-cast and the Gazebo visuals both read it.
  Geomorphology + citation go into `docs/seabed_model.md`.
- Cost budget of the **geometric** renderer (20 Hz at the default 50 ms `msec_per_ping`, 25 ms real-time
  budget per ping; the ray-cast runs in a slowed simulation instead, §3.1): renderer ~6–7 ms
  per ping-side; facet layer ~1.2 ms per smooth object in the beam per
  ping-side; wall multipath ×1.0–1.5 (only with walls).
- Walls a boat can cross are **pontoons**; a quay under the hull is a
  modelling error, not a renderer bug. Objects meant to lie, lean or stand at
  random get `orientation.rest`; the legacy flat stamp stays the default so
  no existing world moves.
- The corpus carries **GPS and attitude** in every file (id-150, 25 Hz);
  three processor-written files freeze `time_boot_ms` (`read_svlog` →
  `clock_frozen`, file-order pairing). `worldgen.depth` must not import
  `analysis` at module level (`analysis/__init__` reaches `worldgen.scene`):
  `depth/svlog.py` imports `analysis.bathymetry` inside the function.
- "X looks wrong" on the sea state: reproduce offline first with
  `sea.simulate_3dof`; a depth report with
  `python3 -m blueboat_sss_sim.analysis.bathymetry <svlog> --lat0 --lon0`.
- "The level / the shadows look wrong": reproduce against **both** NC #6
  anchors (`beach_sept/bath3.svlog`, `reflectionEvidenceFullharbour.svlog`),
  never one — 18 dB hot at 1.2 m *and* correct at 7 m is a slope, which a
  single-altitude comparison reads as an offset. Compare the seabed's
  absolute dB-vs-slant tail past 1.4 × altitude and the reported peak
  against `66.60 − 11.28·log₁₀ alt` separately — different statistics,
  fitted by different constants (`docs/sonar_model.md` §6).
- `~/ros2_ws/sim_review_work/` (outside the repos) holds the 2026-09-18
  review's measurement scripts — **provenance for `sim_review.md`, never a
  gate**: frozen at the pre-2026-09-18 constants, so re-running them reports
  large deltas that are *the fix*.
- Running a script by path imports the **installed** package
  (`~/ros2_ws/install/...`), not the source tree, so such measurements
  silently describe the last `colcon build`. Prefix
  `PYTHONPATH=<package source root>` (or pipe the code through
  `python3 - <<'PY'` from that root).
- Superproject rules hold here (root `CLAUDE.md` §2.1, §3): **never
  `git commit` or `git push`** — the user stages and commits;
  **`BlueBoat-Control` and `BlueBoat-MCS` are updated by additions only**
  (owner, 2026-10-20; they were frozen at tag `v1.0`) — without an explicit
  user order they are read-only neighbours, and a needed change becomes an
  item in this module's `TODO.md`.
