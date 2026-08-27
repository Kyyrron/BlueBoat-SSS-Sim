# TODO — BlueBoat-SSS-Sim

Open items only. Settled facts live in `CLAUDE.md`.

Priority marks: **[P0]** blocks use of the package · **[P1]** correctness or
cross-module risk · **[P2]** realism / project value.

---

## Blocking

### [P0] `test/smoke_test.py` still imports the pre-rename module path
Line 297 does `from blueboat_sss.dataset.labeler import YoloBox`. The Python
package is `blueboat_sss_sim`, so the import raises `ModuleNotFoundError` and the
run aborts partway through section `[3]` — after the tiles and labels are written
but before the normalized-coordinate check, the visual artifact, and the final
`ALL CHECKS PASSED`. Everything before that point passes, so the failure is easy
to mistake for a late-stage regression.

This is the only automated gate the package has. One-line fix:
`blueboat_sss.dataset.labeler` → `blueboat_sss_sim.dataset.labeler`. The symbol
itself is fine — `YoloBox` is defined at `blueboat_sss_sim/dataset/labeler.py:36`
and `dataset/exporter.py:24` already imports it correctly via a relative import.

It is also the last `blueboat_sss`-without-suffix reference anywhere in the tree;
fixing it closes the rename out completely.

---

## Cross-module correctness

### [P1] `channel_number` is hardcoded to 1 on both channels
`PingEncoder.__init__` takes `channel_number: int = 1`
(`blueboat_sss_sim/sonar/encoder.py:73`) and `sss_sim_node` constructs both
encoders without passing it (`blueboat_sss_sim/ros/sss_sim_node.py:161`), so port
and starboard raw frames both carry `channel_number = 1` at byte 34.

The real convention is **0 = port, 1 = starboard**. The GCS `.svlog` reader
treats the packet's own `channel_number` as the authoritative side — never the
device/`src` tag — and unpacks the per-ping group as
`p, s = g.get(0), g.get(1)`
(`BlueBoat-SSS/blueboat_sss/blueboat_gcs/core/svlog.py:391-399, 439`). A `.svlog`
rebuilt from simulated raw frames therefore has every ping tagged starboard, and
because profiles are grouped by `ping_number`, the port frame and the starboard
frame of the same ping overwrite each other in the group dict — one side is lost
outright.

This violates the project-level rule that side identity comes from the packet
(root `CLAUDE.md` CM-5). It does **not** affect `sss_processor_node`, which routes
by topic rather than by `channel_number`, so nothing in the live graph complains —
it fails silently and only shows up in SonarView / replay.

Fix is to pass the side's channel number when constructing each encoder. Decide
deliberately whether `Side` should carry the mapping (alongside `sign`) or whether
the node should pass it explicitly.

### [P1] `full_mission_launch.py` passes a `world` argument that does not exist
`full_mission_launch.py:123-125` includes `blueboat_description`'s
`world_launch.py` with `launch_arguments={"sliders": False, "world": world_file}`.
That launch file's active `generate_launch_description` declares only `gui`,
`spawn`, `thr` and `spawn_pose`
(`BlueBoat-Control/blueboat_description/launch/world_launch.py:7-10`) — neither
`world` nor `sliders`. It hard-codes
`sl.gz_launch(sl.find('blueboat_description', 'world.sdf'), "-r")`.

So on the default `use_existing_world_launch:=true` path the bundle's generated
`world.sdf` is not what Gazebo loads — the stock description world is. Every
sonar ping is still rendered against the bundle's `scene.npz`, so the acoustic
image and the Gazebo scene would disagree about what is on the seabed.

A `world` argument exists only inside the commented-out block at
`world_launch.py:30-65`, and even there it resolves through
`sl.find('blueboat_description', world)`, so an absolute bundle path would not
work unmodified. Two options: add a proper `world` argument to `world_launch.py`
that accepts an absolute path (a `blueboat_control` change — see CM-3 before
doing it), or default `use_existing_world_launch` to `false` here and keep the
`ExecuteProcess` branch, which already launches the bundle world directly.

### [P1] Gazebo generation vs plugin prefix
The simulator and the stack it spawns into currently declare **different Gazebo
generations**:

- `config/default_mission.yaml:7` sets `gazebo_plugin_prefix: gz`, so a generated
  `world.sdf` carries Garden/Harmonic `gz-sim-*-system` plugin names. (The
  code-level fallback when the key is absent is `ignition` — see
  `mission/generate.py` and `worldgen/sdf_writer.py`.)
- `BlueBoat-Control` is uniformly **Fortress**-named: `ignition-gazebo-*-system` /
  `ignition::gazebo::systems::*` in `blueboat_description/urdf/world.sdf`,
  `blueboat.xacro`, `hydrodynamics.xacro` and `thrusters_ur.xacro`.

The robot model spawned into a generated world therefore asks for plugins from a
different generation than the world does; whichever set is not installed fails to
load, silently. `BlueBoat-Control/TODO.md` tracks the same mismatch from its side.

Confirm which Gazebo generation is actually installed on the development machine,
then align both — this package's default and the description package's plugin
names — to that. Do not change one without the other.

### [P1] Decide on the diverging acquisition-parameter defaults
The simulator defaults to `range_length_mm: 15000` and `gain_index: 4`; the real
`sss_node.py` defaults to `30000` and `-1` (device auto) — both confirmed in the
respective sources. The sim values were chosen to match the 15 m field capture
used for power calibration, so this is defensible — but it means "launch with
defaults" produces a different swath and gain in sim than on hardware, which will
silently distort any sim-vs-real comparison. Either align the defaults, or set
both explicitly in every launch file and mission profile. Note that
`gain_index: -1` has no implementation in the simulator's gain ladder
(`ANALOG_GAIN_TABLE` covers 0–5 and falls back to the gain-4 value); if defaults
are aligned, that case needs defining.

### [P1] Transducer lateral offset disagrees across modules
The simulator mounts transducers at `mount_y_abs_m: 0.20` (`sonar/config.py:80`,
`config/default_sonar.yaml:20`); `sss_processor_node.py:102-103` still has
`TRANSDUCER_Y_OFFSET_PORT_M / _STBD_M = 0.0` with its "TODO: measure on the
physical BlueBoat" comment intact. Until both use the same number,
slant-range-corrected geometry from simulated data carries a systematic
across-track bias. Measure on the physical boat, then set both.

---

## Housekeeping

### [P2] `sim_world_launch.py` docstring contradicts its own default
The docstring says `gz_cmd` defaults to `'ign gazebo'` (Fortress); the code at
line 17 defaults to `'gz sim'`. Fix the docstring, or the default — whichever the
answer to the Gazebo-generation item above turns out to be.

### [P2] License declared two ways
`package.xml` says `<license>MIT</license>`; `setup.py` (`license="Apache-2.0"`)
and `README.md` ("License: Apache-2.0") say Apache-2.0. Pick one and make the
three agree.

---

## Open questions from the data

### [P1] Explain the dark rectangle in the mosaic
Working hypothesis: it is ground truth — low-reflectivity material patches (mud
ρ≈0.3, seagrass) image 5–10 dB darker than sand. Secondary candidate: the
water-column cut using a stale tracked altitude. Settle it by overlaying the
mosaic on `export_scene_maps` output (`gt_reflectivity.png` + `gt_extent.yaml`
share the world extent). If the dark region does **not** coincide with a
low-reflectivity patch, it is a bug in the mosaic path, not the renderer.
Needs a completed mission run and the GCS — not attempted here.

---

## Realism and project value

### [P2] Sim-to-real calibration — highest leverage remaining item
`base_scale`, `calibration_db_offset`, `lambert_exponent`, `specular_strength`
and the noise floor are anchored to a *single* decoded frame. Capture a real
flat-seabed run of a known bottom type, then fit these by matching per-range
intensity histograms between real and synthetic waterfalls. This converts the
model from "plausible defaults" to measured values and is the precondition for
trusting synthetic training data.

### [P2] Add a shallow world config for detection work
Default world is 4 m deep (`config/default_world.yaml:11`); the thesis regime
(and CORAL-class litter survey) runs 1–3 m. Shadow length scales as h·R/altitude,
so a `base_depth: 2.5` world config markedly improves object contrast and shadow
visibility. Ship it as a second config rather than changing the default. No such
config exists yet — `config/` holds only the three defaults plus `materials.yaml`.

### [P2] Wall multipath (mirror-source model)
The top realism item for the enclosed-basin regime the thesis characterises. Add
`walls:` to the world config, reflect the transducer across those planes and
across z=0, render each virtual source with a reflection-loss factor, and sum
into the same range bins. Produces the ghost returns and hard negatives that
regime study needs, with per-ghost ground truth. The second-bottom-echo ghost
already shipped (`multipath_enabled`) is the first rung. Currently `walls`
appears nowhere in the code or config — only in `docs/roadmap.md`.

### [P2] Use `ground_truth/contacts` for detector metrics before field work
Per-class P(detection) vs range curves, and the mission-level metrics the thesis
reports, can be computed entirely in simulation from the existing ground-truth
stream. Nothing new is needed in this module — this is an analysis script that
does not exist yet.

### [P2] Object visibility in map-projected mosaics
~29 objects of 20–40 cm in the default world (60/hectare over 80×60 m) are a few
pixels at 4 m altitude after mosaic decimation. Evaluate detectors in the
**waterfall** domain (where the thesis detector runs); if the mosaic must show
them, the fix is a shallower world and/or a finer mosaic resolution, not a
renderer change.

---

## Automation (Skills / subagents / hooks)

Two workflows have recurred in every session and are worth automating; the rest
is not yet justified. Neither is wired — this package has no
`.claude/settings.json`.

- [ ] **Post-change smoke-test hook.** `python3 -m test.smoke_test` has been run
      after every substantive change across three sessions, and has caught real
      regressions each time (empty range bins, azimuth check measuring the wrong
      component, FBR contrast). Wire it as a hook that runs before any change is
      considered complete, and treat a failing check as blocking. Blocked on the
      [P0] item above — the suite cannot currently reach the end.
- [ ] **"Regenerate the bundle" reminder.** Mission bundles freeze `sonar.yaml`
      and `trajectory.yaml`, and stale bundles have already caused confusion
      about whether a fix took effect. Any change under `config/`, `sonar/` or
      `mission/` should surface a reminder to re-run `generate_mission` before
      testing in ROS.

Not yet justified: a dedicated release/packaging subagent (packaging only
modified files was a chat-delivery constraint, not a repo workflow), or a
docs-sync agent (doc updates have so far been small and colocated with the code
change).

---

## Blocked — not verifiable on this machine

Attempted and deliberately not answered. Each needs the Linux development
machine, the device, or a field session; none should be guessed at.

- **Clean rebuild and executable resolution.** `rm -rf build install log &&
  colcon build --packages-select blueboat_sss_sim`, then `ros2 pkg executables
  blueboat_sss_sim` and `ros2 run blueboat_sss_sim generate_mission --help`.
  Static inspection says all seven `console_scripts`, `package.xml` `<name>`,
  `setup.cfg` `script_dir` and `resource/blueboat_sss_sim` agree, so this is
  expected to pass — but it has not been observed.
  *NOT VERIFIABLE ON THIS MACHINE (Windows, no ROS2/colcon).*
- **Launch screen-noise filter on Jazzy.** `full_mission_launch.py` attaches a
  `logging.Filter` via `launch.logging.launch_config.get_screen_handler()` — a
  semi-private API, wrapped in try/except so a failure degrades to "noisy" rather
  than "broken". Confirm that with `quiet:=true` the gz-bridge and
  process-bookkeeping lines are gone **and** `master_control`, `path_publisher`
  and the SSS nodes still print INFO. If the API moved, filter at a different
  point rather than reverting to a global log level.
  *NOT VERIFIABLE ON THIS MACHINE (Windows, no ROS2/colcon).*
- **`num_results: 1200` in a live graph.** Verified offline only (frame length
  2462 B, no empty bins — the smoke test asserts both). Check throughput,
  `sss_processor_node` behaviour, and visualization-app handling at 12.5 mm bins
  before relying on high-resolution runs.
  *NOT VERIFIABLE ON THIS MACHINE (Windows, no ROS2/colcon).*
- **FBR fix against the real processor, not an emulation.** The lock fix was
  validated by re-implementing the processor's detector and bootstrap criteria
  offline (100% of 10-ping windows lockable while moving, first lock at the
  10-pair minimum). It has not been observed with the actual `sss_processor_node`
  in a live graph. Run a full mission and confirm "FBR bootstrapped" appears
  within a few seconds of ping enable, then that `~/processed` publishes
  continuously for the whole pattern.
  *NOT VERIFIABLE ON THIS MACHINE (Windows, no ROS2/colcon).*
- **20 Hz vs 22 ms ping rate.** The Omniscan 450 spec sheet says ≤20 Hz; the
  field capture's device `timestamp_ms` deltas say 22 ms (45 Hz) per channel at
  15 m. Shipped as `max_ping_rate_hz` (default 20, `0` disables), so either is
  reproducible. Resolve against the device — firmware version, whether the spec
  figure is a max-range guarantee, or a longer capture at several range
  settings — then set the default from evidence and record which is right.
  *NOT VERIFIABLE ON THIS MACHINE (needs the Omniscan 450 hardware).*
- **Sim-to-real calibration fit.** See the [P2] item above.
  *NOT VERIFIABLE ON THIS MACHINE (needs a field session).*

---

## Known limitations (accepted, not bugs)

Recorded so they are not "rediscovered" as defects. These match the explicit
assumption list A1–A7 in `docs/sonar_model.md` §7:

- Static acoustic scene during a run; no intra-ping motion (stop-and-hop pings).
- Straight rays, no refraction; single-bounce direct path except the optional
  second-bottom-echo ghost.
- 2.5-D heightfield scene: no overhangs or cavities; objects are height +
  reflectivity stamps. Wrecks with internal structure would need a mesh path.
- Uncorrelated inter-ping speckle.
- No biofouling, sediment interaction, or vegetation dynamics.
