# BlueBoat-SSS-Sim

Synthetic side-scan sonar platform for the BlueBoat USV — a ROS 2
(ament_python) package with two separated deliverables:

1. **World Builder** — PySide6 wizard producing **GPS-anchored Gazebo
   worlds** around the MCS Pattern Designer's GPS-anchored survey paths
   (limits + reflective walls on a satellite map, authored bathymetry, a
   litter catalog scattered statistically **or placed by hand**), saved as
   immutable folders `~/worlds/<path_name>/<world_name>/`;
2. **simulated sonar** — a **drop-in replacement** for the real dual
   Cerulean Omniscan 450 SS interface; the downstream stack runs unmodified.

**Purely additive**: modifies no control stack, robot description,
interfaces package, MCS or GCS. Missions are MCS/Control's; this package
**no longer serves `/path_request`** (Control's `path_generation` is the only
server; its `from_yaml` branch reads MCS deployed trajectories directly).
"Package source root" = `BlueBoat-SSS-Sim/blueboat_sss_sim/`, one level
below the git root.

---

## 1. What this module is for

The thesis tests *aspect-aware adaptive replanning* for SSS survey from a
surface vehicle; its headline result is a model-conditional **simulated**
comparison against a seriously tuned two-pass orthogonal survey (Replanner and
Evaluation are planned, not built; superproject `.claude/CLAUDE.md` §1), so
output must be defensible as evidence, not merely plausible imagery. A
built-world run cannot yet host a replanner-owned run (Control
`docs/REQUESTS.md` R-3). **Worlds in, SSS data
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
(current + a wave wrench into Gazebo); its model notes are in
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

## 3. Interface (the contract — this is where bugs come from)

Node name `side_scan_sonar`, so private topics resolve under
`/side_scan_sonar/`.

### 3.1 `sss_sim_node` — the drop-in sonar

| Topic | Type | Dir | QoS | Other side |
|---|---|---|---|---|
| `/side_scan_sonar/port/profile` | `blueboat_interfaces/OmniscanProfile` | pub | BEST_EFFORT, KEEP_LAST, 10 | `sss_processor_node` |
| `/side_scan_sonar/starboard/profile` | 〃 | pub | 〃 | 〃 |
| `/side_scan_sonar/port/raw` | `std_msgs/UInt8MultiArray` | pub | 〃 | `sss_processor_node` (rebuilds `.svlog`) |
| `/side_scan_sonar/starboard/raw` | 〃 | pub | 〃 | 〃 |
| `/side_scan_sonar/ground_truth/contacts` | `std_msgs/String` (JSON) | pub | default 10 | offline analysis; only on a cycle with contacts; **simulation-only, additive** |
| `/side_scan_sonar/ground_truth/sensor` | `std_msgs/String` (JSON) | pub | default 10 | every ping cycle: the boat pose at the ping time and each transducer's rendered pose; **simulation-only, additive** (2026-10-08), read by `analysis/sea_validation.py` |
| `/side_scan_sonar/ping/enable` | `std_msgs/Bool` | sub | default 10 | operator / launch one-shot |
| `/sim/sea_state/boat_pose` | `geometry_msgs/TransformStamped` | sub | default 50 | `sea_state_launch.py` bridge (param `pose_topic`); the primary pose |
| `/blueboat/odom` | `nav_msgs/Odometry` | sub | default 10 | Gazebo bridge (param `odom_topic`); the fallback pose (optional while `boat_pose` flows) |

Pinging is **off at startup**, like hardware; run-dependent parameters are
re-read on every enable.

Parameters — names, semantics **and defaults** = the real `sss_node.py`
since 2026-09-28; `[4b]` asserts each and reads the real driver's source to
assert the cross-module equality, skipping visibly without it:

| Parameter | Default (sim = real) |
|---|---|
| `range_start_mm` | 0 |
| `range_length_mm` | 20000 |
| `msec_per_ping` | 0 (free run) |
| `gain_index` | -1 (device auto; the simulator resolves it to its calibrated 4) |
| `num_results` | 1200 (the device maximum, 16.7 mm bins at 20 m) |
| `pulse_len_percent` | 0.00125 |

= the device's **best range resolution**: pulse =
`max(44.3 µs, pulse_len_percent × 2R/c)` (**MEASURED**, every corpus
recording, 13 settings, within 1 µs — `AcquisitionParams.pulse_duration_s`),
so 0.00125 stays on the 44.3 µs floor up to ~27 m. The renderer's range
envelope follows the acquisition (`pulse/Ts + filter_duration_percent × N +
1` bins), as does the along-range lag-1 texture correlation (`[2g]`).
Seabed level is bin-count invariant (0.1 dB, 600 vs 1200), so the
calibration carries over; **smoke gates still measure at the calibration
acquisition** (`_cal_config()`: 600 bins, 0.002, gain 4 — the anchors').

Launches pass all six from the resolved profile (explicit `sonar_config:=` >
a legacy bundle's frozen `sonar.yaml` > `config/default_sonar.yaml`), each
overridable per run, so acquisition always traces to a named file. Profiles
are each the default with **one idea** changed; `default_sonar.yaml` is the
sim-vs-real comparison profile (`shallow_beach_sonar.yaml` lightens the
field box's shadow past the device's), and `fault_injection_sonar.yaml` is a
**test** profile, never calibration or comparison.

`gain_index: -1` = the device's command-only sentinel; the sim resolves it
to the calibrated index 4 and reports 4. Modelled ladder **0–7** (4–7
measured on the corpus); any other index raises in `AcquisitionParams`.

Sim-only params: `scene_dir` (**required**; world folder or legacy bundle),
`sonar_config`, `odom_topic`, `publish_ground_truth`, `seed`,
`predict_pose` (default true), `pose_topic` (default
`/sim/sea_state/boat_pose`).

**The sonar follows the hull (2026-10-08; 100 Hz pose 2026-10-09).** Each
ping is rendered at the boat's pose propagated to the ping time with its
own rates (`sea.motion.predict_pose`, the function the hull wrench uses):
Gazebo's per-step model pose, stamped (`/sim/sea_state/boat_pose`, bridged
by `sea_state_launch.py` from blueboat_description's own `PosePublisher`;
`sea.motion.HighRatePose` differences the rates), the 20 Hz odometry only
as a fallback. The sea-state node uses the same pose for the hull wrench:
on the 20 Hz odometry the catamaran's stiff roll restoring rang at 3.2 Hz
(`docs/sea_state_model.md` §5.6). `renderer._sensor_pose` rotates the lever arm
`[mount_x, ±mount_y, −sensor_depth]` by the full attitude (yaw only
before). Level poses are bit-identical to before, so the render fixture
and every calm-water comparison stand; any wave-run recording made before
2026-10-08 is not comparable bit for bit.

Ground-truth JSON per ping cycle:
`{"t_sim": float, "contacts": [{"side","object_id","type","slant_range_m","extent_bins","shadow_bins","visible","ping_number","ghost","via"}]}`
— `ghost`/`via` mark a multipath image (`via: "wall:<name>"` / `"surface"`,
`""` direct); consumers aggregate on `(object_id, via)`; both keys default
to direct, so pre-wall streams parse unchanged. A wall's own face echo is a
contact with `type: "wall"` and a **string** `object_id: "wall:<name>"`
(objects keep integer ids); published only on cycles that have contacts.

### 3.2 `mavros_shim_node` — on by default in the launches

Subscribes `/blueboat/odom`; publishes `/mavros/global_position/compass_hdg`
(`Float64`), `/mavros/imu/data` (`Imu`), `/mavros/local_position/pose`
(`PoseStamped`), `/mavros/local_position/velocity_local` (`TwistStamped`,
world frame — the `.svlog` vx/vy), `/mavros/global_position/global` (`NavSatFix`, BEST_EFFORT
sensor-data QoS, ~5 Hz) from the ENU position about `sim_origin_lat` /
`sim_origin_lon` — **filled by `sss_sim_launch.py` from the world's geo
anchor** when empty; the legacy 43.6961 / 7.3080 default applies only to
unanchored worlds, with a warning. The odom subscription is the node's
entire drive train (f489a21 shipped without it: four topics, zero messages).

### 3.3 World Builder — `world_builder` (GUI, PySide6)

A six-screen wizard, in Create or Edit mode. **Its full description is in
`blueboat_sss_sim/blueboat_sss_sim/builder/CLAUDE.md`**, which loads when
working on files there. Two rules hold everywhere: **never rebuild the scene
or open a modal from a slot that a canvas item event emitted** (the
2026-09-16 use-after-free segfault that lost a world), and a saved world is
immutable — every save, Edit re-saves included, takes a new `_i` folder
(NC #10).

### 3.4 CLI entry points

Console scripts are listed in `setup.py`; each takes `--help`. The `.svlog`
tools (`sss_svlog_compare`, `analysis.svlog_digest`, `analysis.object_signature`,
`analysis.wall_contrast`, `sss_calibration_report`) — which answers which
question, and their caveats — are in the superproject skill
`.claude/skills/svlog/SKILL.md`. Run `sss_svlog_compare` on every model
change; run `svlog_digest` first on any `.svlog`.

### 3.5 Launch files

`full_mission_launch.py` (Gazebo on the world folder + bridges + spawn +
sonar/shim; `headless:=true` runs the Gazebo server only (`gz sim -s`, for
scripted runs); `with_control:=true trajectory_file:=<yaml>` adds the unmodified
control stack with **`blueboat_control`'s own `path_generation`** on
`trajectory:=from_yaml:<file>` — dev convenience, operational missions are
MCS's; additive 2026-10-21 for scripted campaigns (BlueBoat-Experiments):
`gain_profile` and `control_params_file` reach `master_control` only when set
(`master_control_parameters`, smoke-checked to return the old dict
otherwise) and `with_sonar:=false` (forwarded to `sss_sim_launch.py`) drops
the sonar node and its auto ping, keeping the shim), `sss_sim_launch.py`, `sim_world_launch.py`, `sea_state_launch.py`
(also **standalone next to BlueBoat-Control's `Sim_launch.py`**,
`world_name:=ocean` — MCS starts it there for every empty-world run, a calm
sea included, since 2026-10-09). In such a world the node injects
`ApplyLinkWrench` at run time; it must detect the system by
`/world/<w>/wrench/persistent`, **never** by `/wrench`, which the launch's own
bridge advertises (that test skipped the injection until 2026-10-09, so the
stock world's wrenches went nowhere; `sea_state_node.wrench_system_loaded`).

The smoke test calls launch-free module-scope resolvers directly
(`resolve_acquisition`, `resolve_sonar_config`, `resolve_world_anchor`,
`trajectory_total_time`, `world_name_from_sdf`, `wrench_bridges`,
`gz_args_for`). **The three launch modules the smoke test imports
(`sss_sim_launch`, `full_mission_launch`, `sea_state_launch`) import their
ROS symbols optionally** (`sim_world_launch.py`, never imported offline,
does not) (`try: from
simple_launch import … except ImportError`): module-scope `simple_launch` →
`launch` → `lark` (absent from the workspace venv, the Stop hook's usual
`python3`) would kill the offline gate. Keep it so.

---

## 4. NON-NEGOTIABLE CONSTRAINTS

Violating any of these breaks another module, the hardware swap, or the
scientific argument.

1. **Interface fidelity is absolute.** Topic names, message types, QoS
   (BEST_EFFORT/KEEP_LAST/10), parameter names, ping-enable semantics = the
   real `sss_node.py`. **Never define a new message type** — reuse
   `blueboat_interfaces`; extra simulation data goes on new additive JSON
   topics (as `ground_truth/contacts`).

2. **`raw` frames must be byte-valid Ping Protocol, checksum included** —
   downstream rebuilds `.svlog` files from them for SonarView. Layout:
   `'B''R' | u16 payload_len | u16 msg_id=2198 | u8 src | u8 dst | 52-byte
   fixed payload | u16[num_results] | u16 checksum(sum of all prior bytes)`;
   frame length `8 + 52 + 2·num_results + 2` (1262 B at 600 bins).
   `parse_frame()` is the round-trip oracle; keep it working.

3. **Both sides must be published from the same timer tick** — the
   simulator's own rule (downstream no longer forces it): the two channels'
   ping counters stay in lockstep, every simulated row two-sided by
   construction.

4. **The first bottom return must survive every change to the acoustic
   model.** Six coupled pieces make downstream bottom tracking lock; remove
   any and the tracker fails silently: (1) the vertical beam weight at nadir
   (at tilt 20°/FWHM 50° the Gaussian tail alone gives 0.00436 there, so the
   FBR rides on it and `beam_sidelobe_floor` (0.004) is the guard when the
   fan is rolled up >0.55°, the tilt reduced or the aperture narrowed; keep
   w(nadir) ≥ ~0.004); (2) the specular near-nadir lobe
   (`specular_strength`); (3) the specular component rendered and speckled
   **separately** from the diffuse field, low CV (`specular_looks`); (4)
   pulse-length range smearing (`pulse_smearing`); (5) (2026-09-18) that
   component's **own range law** (`specular_spreading_exponent` 2.0: a
   coherent reflection is an image source, spreading once over the folded
   path, not twice like the diffuse field) — it makes the FBR follow the
   device's measured altitude law instead of falling two to three times too
   fast; set equal to `spreading_exponent`, the pre-2026-09-18 behaviour is
   back exactly; (6) (2026-09-19) the coherent channel keeps the per-sample
   **mean** normalisation under `image_formation: energy` (`_Acc.den_s`) —
   an image source's aperture is a Fresnel zone, not a resolution cell;
   area-weighting it puts the nadir `r/y` growth on the FBR (measured: the
   reported peak **8.6 dB** off the corpus anchor). The smoke test
   re-verifies FBR lock with every object family (open-frame lattice classes
   included) in the scene; `[2c]` gates the altitude law itself.

5. **Speckle statistics are load-bearing, not decoration.** Diffuse:
   multiplicative `Exp(1)`; coherent specular: `Gamma(L, 1/L)`, low CV.
   Since 2026-09-02 `Exp(1)` comes from the renderer's coherent
   ground-scatterer field (`Ping.texture`, `coherent_scatterers: true`), so
   it is also *correlated* along range (pulse) and track (footprint overlap)
   as the corpus measures; `Ping.power` stays the noiseless mean,
   `speckle: false` is untouched. Smoke `[2g]` gates mean, CV and both lag-1
   correlations.

6. **Power calibration is anchored to the field corpus, and moves once** —
   it moved on 2026-09-18, on TWO altitudes; the 2026-09-19 image-formation
   rework did NOT move it again (its acceptance test: the two-anchor mean
   offset shifted 0.20 dB at `bath3` and 0.38 dB at the harbour, inside the
   0.67 dB repeatability floor). `calibration_db_offset` (**81.1 dB**, was
   97.2) is still the **only** level constant, fit against the reported
   level, never a count histogram. A single-altitude fit cannot separate
   level from *slope*: the 97.2 dB anchor was right at the harbour corpus's
   6–9 m and **18 dB hot** at the beach corpus's 1.2 m. The 2026-09-18 fit
   uses `bath3` (1.40 m, 12 371 pings) and `reflectionEvidenceFullharbour`
   (7.33 m, 1 445 pings) **together**, moving four constants as one package:
   `tvg_compensation` 0 → **0.30** (the device compensates part of its range
   loss), `specular_spreading_exponent` → **2.0** (NC #4),
   `specular_strength` 30 → **12**, `calibration_db_offset` → 81.1. Result:
   seabed dB-vs-slant within **0.6 dB at both altitudes**; the reported
   peak's altitude law within **0.2 dB per decade** of the device's
   (66.60 − 11.28·log₁₀ alt, measured over 19 gain-4 recordings). Noise
   floor (**3.39e-09**) and water column (**4.68e-06**) encode measured
   *levels* and moved with the offset without changing them;
   `reverberation_db` (**18**, the floor of a shadow) is new, measured on the
   corpus quay. Changing any of them silently invalidates every comparison
   against real data. `docs/sonar_model.md` §6 records which constants are
   measured, jointly determined, undetermined — keep it current whenever one
   moves. Residual: the reported *peak* is still ~5 dB hot with the right
   slope (the near-nadir return the tail fit deliberately excludes).

7. **Never raise launch's global log level to suppress noise.** Filter by
   message content on the screen handler instead.

8. **Never modify the control stack or other packages.** Integrate by
   serving identical interfaces, not by patching neighbours (MCS trajectory
   discovery included: `builder/paths.py` reimplements the predicate rather
   than importing `mcs.*`).

9. **The renderer's ground-sample step must stay coupled to the slant-bin
   size.** A fixed step leaves range bins unpopulated.

10. **World folders are immutable snapshots** — never overwritten by Create
    (name dedupe), Edit (re-saves take a new `_i` folder) or tooling
    (`mission_metrics` etc. write only into `--out`). Regenerate rather than
    edit; `builder_state.yaml` makes regeneration a load-away. Legacy mission
    bundles keep loading (their frozen `sonar.yaml`/`trajectory.yaml` are the
    resolver fallbacks) and are equally write-once.

11. **Project-level (owner's decision, 2026-10-20; superproject CM-11):** the
    headline detector is a YOLO model in BlueBoat-Perception, trained on
    *simulated* datasets that Perception's dataset mode labels from this
    module's ground truth, with its confidence calibrated on held-out
    simulated worlds. Real water validates this simulator (boat motion and
    sensor), runs the heading-response experiment and serves the final
    demonstration; a labelled real dataset blocks nothing. The consequence
    stays stated: a simulation-trained detector inherits its aspect
    sensitivity from this module's model, so the model is what the real
    heading-response experiment bounds, and the module's data are also for
    the policy comparison and development.

12. **Limits never reflect.** World-limit borders are Gazebo collision-only
    geometry, in neither `scene.walls`, the rasters, nor the manifest. Only
    authored walls are acoustic.

13. **No return from land, and no objects on it.** Terrain at or above the
    transducer (a dry beach, an awash bank) scatters nothing back into the
    water (`renderer._ping_geometry`, `dz > 0` mask; the horizon cull hides
    what lies beyond the waterline); random placement never lands above
    `DRY_HEIGHT_M` (−0.05 m). A fully wet world is bit-identical either way.

14. **Nothing ticked means nothing placed.** `objects.classes: {}` (no class
    enabled in the builder) yields zero objects; only a config *without* the
    key falls to the legacy random composition (2026-09-05 fix — an empty
    selection used to scatter 29 objects).

15. **A shadow is a measurement, not a hole** (2026-09-18; extended
    2026-09-19: the floor is referenced to the **ambient** seabed level at
    that range, not the culled bin's own would-be response, which collapses
    to zero on a back-facing flank or over land, put the worst bin 40 dB
    under the device's and drew a hard black band between a simulated object
    and its shadow; `reverberation_reference: unoccluded` restores it).
    Three corpus-anchored rules, each reversible by one constant: a culled
    bin holds the **reverberation floor** (`reverberation_db` 18 below the
    lit seabed at that range, with the seabed's own speckle — the quay
    measures 17–25; `0` restores the white receiver floor that made every
    shadow 30 dB too deep); a **transmitting** material attenuates its own
    shadow rather than zeroing it (`pp_shell` 0.11, measured on the field
    box's 9.6 dB shadow; every other material opaque, as every object was);
    **ground truth is read off the rendered mask**, never from
    `h·R/altitude`, so a label cannot contradict its picture. A detector
    learns a shadow's statistics; a merely *empty* shadow teaches nothing the
    real device will confirm.

16. **A world can have dry land in it, and it is authored, not carved.** The
    `land` layer lifts a drawn area over the waterline and carries the
    seabed up to it, the outline **rounded** by an opening-closing disc
    (`round_m`) so a handful of clicked vertices reads as a shoreline, not a
    polygon. A dredged basin with a negative floor is the hard-edged
    alternative and composites normally rather than being silently swallowed
    by its "deepen" blend. NC #13 does the rest: dry cells return nothing,
    the horizon cull hides what lies beyond.

17. **Objects are rendered from material physics, never tuned per object**
    (2026-09-28, the user's rule: "apply it to any kind of object/material
    ... we don't have to tune this ever again"). Only **material** and
    **geometry** decide how an object returns:
    * **rough** materials (RMS height >= `COHERENT_ROUGHNESS_LIMIT_M`,
      1.6 mm: concrete, rope, tyre tread, generic) and **aggregates** of many
      members (lattice classes, rope, chain, anchor, debris, coils) stay on
      the raster's Lambert path — the right law for a wavelength-rough
      scatterer, bit-identical to before;
      (fired brick, 1 mm, is on the smooth side: C = 6e-7 at normal
      incidence, 0.5 at 77° off it — a facet object, smoke `[14]`);
    * **smooth solid primitives** (`objects.is_facet_object`: a smooth
      material on a box / cylinder / capsule / cone / annulus — any primitive
      `solids.tessellate` knows — proud of the seabed) scatter in the **facet
      layer** (`sonar/facets.py`): coherent Kirchhoff patches scaled by the
      material's reflection coefficient and the Eckart coherent fraction,
      plus the incoherent residual; glints, their aspect dependence (a wall
      within ~1 deg of broadside), edges and layover EMERGE from the
      geometry. In the raster they only occlude (`facet_bump`; the seabed
      beneath dimmed by their two-way transmission, 0 when opaque);
    * a material's acoustics are **physical constants with citations**
      (`_ACOUSTIC` in `worldgen/materials.py`: density, sound speed,
      backing, wall), plus MEASURED properties labelled as such (a shell's
      transmission from a field shadow). The one constant tying an object's
      cross-section to the calibrated seabed is literature
      (`sand_lambert_mu_db`, J&R), never fitted to an object.

    **Adding a class** = its `Solid` primitive (or aggregate mask); **adding
    a material** = its physical constants, with a citation, never a
    brightness. A brightness knob on an object or a class is a violation.
    New object physics is proved first against textbook closed forms (plate,
    cylinder, sphere — smoke `[14]`), then against the corpus at the field's
    own geometry; `docs/sonar_model.md` §12 carries the derivation, the
    evidence and the limits.

---

## 5. Data: world folders, formats, what not to overwrite

`~/worlds/<path_name>/<world_name>/`, built by
`worldgen/world_folder.build_world_folder`. Every file and its format:
`docs/configuration_guide.md` §3. What the code does not tell you:

- `scene.npz` + `scene_manifest.yaml` are the **ground truth**.
  `SceneModel.stale_rasters()` names the object rasters a world built before
  the current model lacks — logged by every loader; the cure is a rebuild
  (Edit), never a patch.
- `metadata.yaml` is **the cross-module contract** (`blueboat_world_meta/1`:
  anchor, spawn, limits, walls and every object in local + GPS). Downstream
  reads it, **never** `builder_state.yaml` (GUI-internal, may churn).
- `depth.npz` is the **terrain-only** height raster (pre-object-stamp).
- `scene_manifest.yaml` stays **version 1**: the `geo:` block is additive;
  `SceneModel.load` rejects any other version.
- RNG determinism: a config **with** a `depth:` section splits the seed into
  two streams (`[seed, 0]` terrain, `[seed, 1]` objects), so editing the
  bathymetry never moves the objects. A legacy config without `depth:` keeps
  the single historical stream, bit-identical to every existing bundle.
- Rasters are `(ny, nx)`, origin at the min corner; `export_scene_maps` flips
  vertically for north-up images.

---

## 6. Configuration

World YAML schema: `docs/configuration_guide.md` §1; sonar config (every
`acquisition:` / `model:` knob): `docs/configuration_guide.md` §2, the
physics behind them `docs/sonar_model.md`; sea state:
`docs/sea_state_model.md`. **Unknown keys raise by name** in the sonar
sections, `terrain:`, `walls:`, `depth:` (presets, layers, masks, svlog,
paint) and `orientation:`; `world:`, `objects:` (top level, classes,
explicit entries), `materials:` and extra `geo:` keys ignore them —
`configuration_guide.md` lists both.
The traps:

- `geo:` `theta_deg != 0` is rejected (the world frame is the anchor's ENU
  frame by definition).
- With `depth:`, `terrain:` contributes only texture; without it, legacy
  behaviour is bit-identical.
- **An `objects.explicit` entry pins only what it states**: every other field
  is drawn from the shared RNG stream, so an entry of just `type,x,y` moves
  when anything before it changes. Screen 5 therefore writes all of
  `worldgen.objects.PINNED_FIELDS` + `material`, and
  `roll_deg`/`pitch_deg`/`rest` **only** for an object that actually rested
  (any tilt sends the instance down the 3-D solid path). `transmission` and
  `roughness_m` follow a pinned `material` (since 2026-10-21); `object_id`
  is positional and seeds rope/chain/debris/coil/lattice shapes, so deleting
  or reordering a pinned object re-draws those shapes of later objects.
- `BuilderState.objects.placed` (never in a world YAML) = Screen 4's random
  field pinned into verbatim instances; while non-empty,
  `state_to_world_config` emits `classes: {}` — keeping the classes would
  place the field twice. `placed_signature` detects a later Screen-4 change.
- Every material carries physical acoustic constants (`_ACOUSTIC` in
  `worldgen/materials.py`), never a brightness (NC #17); `transmission` is 0
  for all but `pp_shell`. `config/materials_literature.yaml` is an
  **override**, not the default, so existing worlds do not move.
- `DEFAULT_COMPOSITION` is deliberately unchanged — new classes enter via
  `objects.classes`, never by reshuffling legacy seeds.
- `wall_multipath_enabled: true` by default is a strict no-op without walls.

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

The only automated gate of the Stop hook (no lint, no type-check). **444
checks** since 2026-10-21 (439 + four synthetic-yaw-moment checks in `[10]` + the
`master_control_parameters` launch check; 444 / 444 on the system python that day) (441 + one visible `SKIP`, covering the two bridge checks and the
`ApplyLinkWrench` detection, when the interpreter has no ROS; one
more when the `BlueBoat-SSS` checkout beside it is missing), ~205 s (2026-10-20).
Section `[10]` (sea state) is unit-level only — the current's statistics,
bound, smoothness, tick-independence and no-reset; the plain-language
preset keys; the sensor lever arm; the image's roll/pitch/yaw response
under prescribed poses. Boat motion is never simulated offline (the
user's rule, 2026-10-08): the existing `simulate_3dof` stays the only
offline loop. `grep -n 'print("\['
test/smoke_test.py` lists the sections. Two rules the code does not
explain: the **render fixture** `test/fixtures/render_baseline_v1.npz` is
rebaselined only for a deliberate model change, by deleting the file, and is
compared to **float precision** (1e-12 of each array's scale) — the venv's
numpy 2.4 and the system numpy 1.26 differ by a few ULP. It renders with the default
profile's mount, so a geometry change in `config/default_sonar.yaml` is a model change.
The 2026-10-12 move of `mount_y_abs_m` from 0.20 to 0.285 m left the gate red, and the
first failure hid a second: the pontoon-crossing check assumed the old mount. Both were
repaired on 2026-10-19. The fixture was rebaselined, after a re-render at 0.20 had
reproduced the old one bit for bit, and the crossing pose is now placed from the
profile's mount. The suite ran 434 / 434 again on the system python, and 431 + 1 visible
`SKIP` on the venv python. `[2f]` reads the
field corpus at `$BLUEBOAT_SSS_CORPUS` (default `~/ros2_ws/data/SSS_data`) and
skips without it. Calibration gates: `[2c]`, `[2e]`, `[2f]`; object physics:
`[14]`. Runs serialize on `/tmp/blueboat_sss_smoke.lock` — give a manual run
a generous timeout. GUI checks, outside the gate:
`QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke`.

### Gazebo gate — waves and current end to end

```bash
source ~/ros2_ws/env.sh            # + colcon build --packages-select blueboat_sss_sim
python3 -m test.gazebo_sea_test --world-dir ~/worlds/sea_validation/flat_5p0m \
    --out ~/ros2_ws/sim_runs/sea_validation/<date>_5m      # --quick: line 1 only
```

Not in the Stop hook: real time, ~5–6 min per scenario (calm / current /
waves / both), headless (`headless:=true`) with the unmodified control
stack on `~/.config/blueboat_mcs/trajectories/sea_validation.yaml`. It
records a bag, analyses it (`analysis/sea_validation.py`, also the
`sss_sea_validation` CLI) and checks: calm is a null test; the realised
current matches its mean within the run's statistics and its bound; the
hull rides the waves (heave std = Hs/4, period ≈ Tp, roll/pitch in the
corpus band, impulse conserved, no 1.6 Hz line); and the sonar shows it
(back-scanned / skipped footprints, uneven pings per target, targets
moved or distorted against calm). `report.md` + figures in `--out`. The
validation worlds and trajectory are data built with the World Builder
and the Pattern Designer — how, and the results, in
`docs/sea_state_validation.md`. They live outside every repository and were
lost once (empty from 2026-10-12). They were rebuilt on 2026-10-20 by
`~/ros2_ws/sim_runs/sea_validation/build_inputs.py` (deterministic, refuses
to overwrite). On 2026-10-20 the gate ran 24/25 (5 m) and 23/24 (1.5 m). The
only failure each time was an intermittent "fresh pose" stall in *waves*,
which passed when re-run (`TODO.md`). On 2026-10-21, after the synthetic
yaw moment and the launch pass-throughs were added (none configured), it ran
24/24 + 1 INFO on 1.5 m, every scenario first time
(`~/ros2_ws/sim_runs/sea_validation/2026-10-21_1p5m_yawmoment_setup`).

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
  assumptions A1–A12), `topics.md` (raw-frame byte map),
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
  Geomorphology + citation go into `docs/seabed_model.md`.
- Cost budget (20 Hz cap, 25 ms real-time budget per ping): renderer ~6–7 ms
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
