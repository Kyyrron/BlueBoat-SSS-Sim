# blueboat_sss_sim — GPS-anchored worlds + simulated Side Scan Sonar

Two things, cleanly separated:

1. **World Builder** — a PySide6 wizard that creates **GPS-anchored Gazebo
   worlds** around the MCS Pattern Designer's GPS-anchored survey paths:
   draw the world limits and reflective harbour walls on a satellite map,
   author the bathymetry (realistic harbour / river / beach / canal
   presets, or real soundings from CSV / GeoTIFF), scatter litter objects
   from a 29-class catalog (clustered, sized and buried by simple
   distributions), preview, save. Worlds land in
   `~/worlds/<path_name>/<world_name>/` and are **immutable** — every
   save, edited or not, creates a new folder.
2. **Simulated SSS** — a physically motivated Omniscan 450 simulation
   publishing a **byte-identical drop-in replacement** of the real
   `sss_node.py` interface (profiles, raw Ping-Protocol frames,
   ground-truth contacts), calibrated against the field corpus.

Purely additive — no neighbour package is modified. Missions belong to
MCS/Control: this package serves worlds and sonar data, nothing else.

## Start tutorial: test the whole chain, from a new world to analysing a run

This walk-through takes you once through everything the simulator offers. You draw a survey path, build a world
around it, fly the mission in Gazebo from the **MCS App**, record the sonar with the **GCS App**, then look at
what was recorded. First you do it with the default (geometric) sonar, which runs in real time. Then you repeat it
with the ray-cast sensor.

```
 MCS Pattern Designer      World Builder           MCS App              GCS App              your terminal
 (a GPS-anchored path) --> (world around it) --> (launch in Gazebo) --> (START + Record) --> (look at the results)
       step 1                  step 2                step 3              steps 4-5              step 6
```

Plan on about 30 minutes for the geometric run. A ray-cast run takes much longer, because the whole simulation
is slowed down (step 7).

### Before you start (once)

Open a terminal and build the workspace. The `env.sh` line is needed in **every** terminal you open for this
tutorial: it loads both ROS and the Python environment.

```bash
cd ~/ros2_ws && colcon build
source env.sh
```

Optional checks. Run them if something looks wrong later:

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-SSS-Sim/blueboat_sss_sim
python3 -m test.smoke_test                       # simulator self-test, ~4 min, must end "ALL CHECKS PASSED"
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-MCS
QT_QPA_PLATFORM=offscreen python3 smoke_test.py  # MCS self-test, must end "SMOKE TEST PASSED"
/usr/bin/python3 -c "import warp; print(warp.__version__)"   # only for the ray-cast (step 7): 1.19.0
```

### Step 1: draw a GPS-anchored survey path (MCS App)

A world is always built around a survey path that is tied to real GPS coordinates. So the path comes first.

```bash
cd ~/ros2_ws && source env.sh
cd src/BlueBoat-SideScanSonar/BlueBoat-MCS && python3 run.py
```

1. The MCS window opens. The status bar should read **ROS: connected**.
2. Click **Create Survey Pattern** in the bottom toolbar. The Survey Pattern Designer opens.
3. Click **Set GPS Origin… → Enter Coordinates…** and paste your site's coordinates as Google Maps shows them,
   for example `34.408249, 133.524355`. The satellite map of the site appears.
4. Draw or generate a pattern, such as a lawnmower over the area you want to survey.
5. **Save** it with a name, for example `my_harbour`. It is stored in
   `~/.config/blueboat_mcs/trajectories/my_harbour.yaml`, and the launch dialog will label it **(GPS)**.

Leave the MCS App open; you need it again in step 3.

### Step 2: build the world (World Builder)

In a second terminal:

```bash
cd ~/ros2_ws && source env.sh
ros2 run blueboat_sss_sim world_builder
```

Click **Create Gazebo World** and go through the six screens:

| Screen | What you do |
|---|---|
| 1. GPS path | Pick the path from step 1. Only GPS-anchored paths are listed. |
| 2. World limits & walls | Drag one rectangle on the satellite map: that is the world. Click segments to add **quays** (solid walls from the seabed) or **pontoons** (floating; the sonar sees under them). The path must lie inside the rectangle. |
| 3. Depth & seabed | Choose a depth preset (harbour basin, channel, beach, …), a soundings file, or a real `.svlog` recording. Optionally add layers: channels, dunes, materials, … The previews update live. |
| 4. Objects | Choose which litter classes to scatter and how many (or none). |
| 5. Place objects by hand | Click to drop objects exactly where you want them; drag to move, double-click to edit. Optional. |
| 6. Preview & save | Check the depth and object maps, give the world a name, and **Generate**. |

The world is saved to `~/worlds/my_harbour/<world name>/`. Worlds are never overwritten: saving again, even from
**Edit World**, makes a new folder with a `_1`, `_2`, … suffix. Close the World Builder when you are done.

### Step 3: launch the mission in Gazebo (MCS App)

Back in the MCS App, click **Launch Mission** and fill the dialog:

| Field | Set it to |
|---|---|
| Mission type | **Gazebo simulation**, Source **Controller** |
| Trajectory | your path from step 1, marked **(GPS)** |
| Controller | **PID** (or LoS / MPC) |
| Log note | a short tag, for example `tutorial` |
| Session | **Normal session**. *Dataset creation* also saves every seabed image as a labelled YOLO training sample under `~/ros2_ws/data/Dataset_data/`. |
| Sonar model | **Geometric** for this first run |

Click **OK**, then answer the two dialogs that follow:
1. **World:** choose the world from step 2 and click **Launch in World**. The world is listed only if its
   limits contain your path.
2. **Sea state:** leave both cards off for calm water, or add current and waves.

Gazebo (and rviz) open. The boat spawns at the world's origin, waits a few seconds for its GPS fix, then drives the
path. In the MCS window you see the boat, its trail and the path on the satellite map.

### Step 4: record the sonar (GCS App)

Start the GCS **after** the simulation is up. Its sonar processor then picks up the simulation clock. In a third
terminal:

```bash
cd ~/ros2_ws && source env.sh
cd src/BlueBoat-SideScanSonar/BlueBoat-SSS/blueboat_sss && python3 -m blueboat_gcs.main
```

1. Check that the toolbar's **Sonar:** selector says **Simulation**. Never choose *Real sonars* in a
   simulation.
2. Wait for the status bar: *Processing pipeline up — press START for live acquisition.*
3. Press **▶ START**. The waterfall and the sonar mosaic on the map start filling, and detections (the dummy
   detector, for now) appear on the map.
4. Press **⏺ Record: OFF** so that it reads **Record: ON**. From now on, everything is saved in a session.
5. Let it run for a few minutes. A new seabed image is written about every 130 pings, a few seconds of survey.

### Step 5: stop cleanly

1. **GCS:** press **Record: ON** to stop recording, which saves the session, then **■ STOP**.
2. **MCS:** press **Stop Mission**. It shuts the launch down safely, closing Gazebo, the sonar and the
   controller. Wait until **Launch Mission** is available again before the next run.

### Step 6: look at the results

**The GCS session.** It is in `~/ros2_ws/data/SSS_data/sessions/<YYYY_MM_DD-HH_MM_SS>/`:

| In the session | What it is |
|---|---|
| `<date>.svlog` | the raw sonar recording, the same format as the real boat writes (SonarView can open it too) |
| `seabed_images/seabed_00000.png` … | the seabed pictures the detector sees: 256 rows, both sides |
| `seabed_images/metadata/*.json`, `*_world.npz` | per row: position, altitude and time; per pixel: world x/y and the raw dB |
| `waterfall/`, `mosaic/` | the live waterfall and the georeferenced mosaic, as pictures and data |
| `detections/detections.csv` | every detection shown on the map |

**Replay it in the GCS.** Click **Open SVLOG** and pick the session's `.svlog`. The replay window lets you:
- render any time range, or replay at x1–x8;
- **Run AI** (Perception's detector) on it;
- **Save pictures from the log**.

**The mission report (control side).** It is in `~/ros2_ws/data/Robot_data/<date>-sim-tutorial-poslog/`: the
`…-poslog.png` shows the GPS track, distance to target, speed and thrust.

**From a terminal** (`cd ~/ros2_ws && source env.sh` first; `S` is your session folder):

```bash
S=~/ros2_ws/data/SSS_data/sessions/<YYYY_MM_DD-HH_MM_SS>

# What is in the recording: pings, sides, acquisition, ping rate, bottom range, GPS / attitude
python3 -m blueboat_sss_sim.analysis.svlog_digest $S/*.svlog

# Detection ground truth: which objects of the world the run saw, and how well (metrics.md + metrics.json)
ros2 run blueboat_sss_sim mission_metrics --world ~/worlds/my_harbour/<world name> \
    --source svlog --input $S/*.svlog --out ~/metrics/tutorial

# Run Perception again over the whole session, offline (the same result as live)
python3 -m blueboat_perception.cli.offline $S

# Compare the simulated recording with a real one: timing, levels, texture statistics
ros2 run blueboat_sss_sim sss_svlog_compare --out ~/compare/tutorial $S/*.svlog <a real .svlog>
```

### Step 7: the same run with the ray-cast sensor

The ray-cast is the second sonar model: 3-D ray casting on the GPU. One ping takes longer than the real
50 ms (0.1 s in open water, 1–16 s with a quay in range), so the **whole simulation runs slower** (physics,
control, sonar and the GCS clock together) and, since 2026-11-09, **pauses while a slow ping renders**, so no
ping is ever lost. The recording is identical to a real-speed run; only the wall-clock time is longer.

Repeat step 3 with these changes in the launch dialog:

| Field | Set it to |
|---|---|
| Sonar model | **Ray-cast — live (first-order multipath)**: the full model, bit-identical images, faster near quays. **Ray-cast — fast**: a harbour mode for speed (10 mm scatterers, reused echo-path choices, closed-form diffraction); judge its look against plain **Ray-cast**, the full default profile. |
| Slow-down | live: **x0.5–x1**; fast: **x2** (up to x4). The pauses take care of slow pings, so a high setting only speeds up the open water. |
| Extra args | Nothing needed: a ray-cast run starts the Gazebo server and its window as a separate client (`headless:=auto`; a window crash at start-up no longer stops the run, the window comes back), add `headless:=true` to run without the window (frees its share of the GPU). For the harbour hybrid: `sonar_config:=/home/<you>/calib/raycast/correction/raycast_harbour_hybrid_v2_sonar.yaml`. |

Then do steps 4–6 exactly as before. What to expect:
- **Steady.** The 530 m harbour lawnmower takes about 1.5 h with live; fast aims at real time or faster in open
  water (measure it: the 2026-11-10 phase-2 numbers are offline estimates). The MCS toolbar's **SIM**
  badge shows the measured factor; it drops while the boat passes a quay (the simulation is paused there).
- **Late pings?** If the sonar node's console says it is behind the sim clock *although the simulation is
  held*, relaunch with Extra args `sonar_hold_sim:=always`.
- **Keep the range at 20 m**, the default, and never above 30 m. That includes the GCS's range control. Above
  40 m the GPU runs out of memory (see *Range advice* below).
- **Known limitations of today's ray-cast:**
  - the GCS tracks the bottom 1.4–1.6 m too deep, so the georeferencing near nadir is off;
  - there are no live ground-truth contacts, so *Dataset creation* gives empty labels. `mission_metrics
    --source svlog` still works: it replays the recorded track over the world;
  - its levels are a testable harbour version, not yet a validated model.

### If something goes wrong

| Symptom | Likely cause and fix |
|---|---|
| MCS status bar says ROS not connected | The terminal was not sourced: `cd ~/ros2_ws && source env.sh`, then restart the app. |
| The World Builder does not list your path | The path is not GPS-anchored. In the Pattern Designer: **Set GPS Origin…**, then save again. |
| No world dialog after **OK** in MCS | No world's limits contain the path, or the path is not GPS-anchored. |
| The GCS waterfall stays empty | You did not press **▶ START**, or **Sonar:** is not *Simulation*. |
| GCS times look wrong in a slowed run | The GCS was started before the simulation. Close it and start it again with Gazebo running. |
| Ray-cast: `ray-cast render failed … cycle skipped` | The range is too long for the GPU. Go back to 20 m. |
| Ray-cast: very few pings | The Slow-down factor is too high for this world. Use the factor the sonar node prints. |

## Quick start

```bash
colcon build --packages-select blueboat_sss_sim && source install/setup.bash

# Build a world (GUI; needs `pip install PySide6`, everything else doesn't)
ros2 run blueboat_sss_sim world_builder

# Inspect it in Gazebo alone
ros2 launch blueboat_sss_sim sim_world_launch.py world_dir:=$HOME/worlds/<path>/<world>

# Sonar + MAVROS shim over a running simulation
ros2 launch blueboat_sss_sim sss_sim_launch.py world_dir:=$HOME/worlds/<path>/<world>

# Dev end-to-end (Gazebo + robot + sonar; optional control stack driving
# the world's own ready-made trajectory — the source path, spawn-aligned)
ros2 launch blueboat_sss_sim full_mission_launch.py \
    world_dir:=$HOME/worlds/<path>/<world> \
    with_control:=true trajectory_file:=$HOME/worlds/<path>/<world>/source_path_world.yaml

# Headless world generation from a world YAML (no GUI; run from the
# package source root, blueboat_sss_sim/, where config/ lives)
ros2 run blueboat_sss_sim generate_world --config config/default_world.yaml --out ~/worlds/dev/w1
```

Offline verification without ROS (also wired as the session gate), from
`blueboat_sss_sim/`: `python3 -m test.smoke_test` (496 checks on the system python with the field
corpus and the BlueBoat-SSS checkout, 493 + one visible SKIP on the venv; visible SKIPs without them). Calibration tooling:
`python3 -m blueboat_sss_sim.calibration --help` (`docs/engineering/`). GUI checks:
`QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke`.

## The ray-cast sensor (second sonar model)

Besides the default **geometric** renderer, which is the calibrated comparison model, the simulator has a 3-D
**ray-cast** sensor (`sonar/raycast/`, on the GPU through NVIDIA Warp). It renders a ping far more slowly than the
50 ms ping period, so a ray-cast run slows the **whole simulation** onto one clock. It still pings at 20 Hz in
sim time, and the `.svlog`, the GCS sessions and the seabed images are those of a real-speed run.

- **Prerequisites.** An NVIDIA GPU, and `warp-lang==1.19.0` importable both by `/usr/bin/python3` (the
  interpreter `sss_sim_node` runs on) and in the ROS venv.
- **From the MCS App:**
  1. Launch Mission → Gazebo simulation, and pick a personalized world.
  2. Set **Sonar model** to *Ray-cast — live* (or *— fast*, or plain *Ray-cast*, the full default profile)
     and **Slow-down x0.5**. The sonar node pauses Gazebo while a slow ping renders (a quay in range), so
     every ping keeps its 50 ms sim-time period (`sonar_hold_sim`, default `auto`).
  3. For the harbour hybrid, add to **Extra args**:
     `sonar_config:=/home/<you>/calib/raycast/correction/raycast_harbour_hybrid_v2_sonar.yaml`.
  4. Run the GCS App as usual: START, then Record. The seabed images land in the session as at real speed.
  - The live variants (`config/raycast_live_sonar.yaml`, `raycast_fast_sonar.yaml`) switch off only parts that
    cost GPU time without changing how the image looks; the evidence is in `.claude/raycast-live.md`.
- **From the command line:**
  ```bash
  ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=$HOME/worlds/<path>/<world> \
      with_control:=true trajectory_file:=<deployed path yaml> \
      sonar_renderer:=raycast real_time_factor:=0.5 \
      sonar_config:=raycast_live_sonar.yaml        # or raycast_fast_sonar.yaml; [sonar_hold_sim:=auto]
  ```
- **Verified end to end (2026-11-07)**, Shiraishijima harbour, 20 m / 1200 bins, x0.03:
  - MCS launch → ray-cast hybrid → GCS START / Record;
  - 20.3 Hz in sim time;
  - a session with the `.svlog`, the seabed images (schema 5), the waterfall, the mosaic and Perception's
    detections, as at real speed.
- **Known limitation in the GCS.** The processor's bottom tracker reads the ray-cast's first return 1.4–1.6 m
  late and unsteadily, against 2 cm with the geometric renderer. The images are fine, but their georeference
  (ground range near nadir) is off by that much.
- **What it is today.** The ray-cast's absolute levels are not yet validated. The harbour hybrid is a testable
  version with known limitations, and its correction is active only at 5.55–11.3 m nadir altitude
  (`blueboat_sss_sim/docs/engineering/raycast_assessment.md` §13). The ray-cast publishes no ground-truth
  contacts yet, so it produces no Perception dataset labels.

### Range advice for the ray-cast: keep `range_length_mm` ≤ 30000 (20 m is the survey setting)

The ray-cast's GPU working set grows with the range. Past the device memory it fails with **CUDA error 101**,
which is Warp's report of a device out-of-memory error. The node then logs `ray-cast render failed (…); cycle
skipped` and pings nothing for that cycle.

Peak use per ping-side, measured 2026-11-07 on an RTX 4060 (8 GB): harbour world, the worst case with the quay
inside the swath, one renderer as in the live node.

| range | 20 m | 25 m | 30 m | 35 m | 40 m | 50 m |
|---|---|---|---|---|---|---|
| peak GPU memory | 2.3 GB | 3.1 GB | 3.6 GB | 5.7 GB | 6.3 GB | **out of memory** |

- **20 m** (the project's survey range) and up to **30 m** leave ≥ 4.6 GB free.
- **35–40 m** still runs, but leaves under 3.5 GB. A second GPU process can then push it over. An offline
  replay holding several renderers did fail at 40 m.
- **Never use more than 40 m** with the ray-cast on an 8 GB GPU. That includes the GCS's range control,
  which re-ranges the simulated sonar too.
- The geometric renderer is unaffected, since it runs on the CPU.

Details: `blueboat_sss_sim/docs/engineering/raycast_assessment.md` §13.7 and `docs/integration_guide.md` §2b.

## GPS anchoring in one paragraph

A world's local ENU frame is its source path's design frame **re-based so
that world (0, 0) — where the boat always spawns — is the path's first
waypoint** (or the area centre when that lies outside the drawn limits):
the boat is guaranteed to start inside the world, at the survey start.
The world's `geo_anchor` is that point's GPS; `world.sdf` carries
`<spherical_coordinates>`, the manifest a `geo:` block, and
`metadata.yaml` lists the spawn, every object, wall and limit corner in
GPS (globally identical whatever the local frame — the GPS layer absorbs
the shift). `source_path_world.yaml` in each folder holds the path in
world coordinates, ready for the dev launch. The conversion is the
project-wide equirectangular pair (`blueboat_sss_sim.core.geo`, same math
as MCS `core/geo.py` and the GCS); the MAVROS shim publishes
`/mavros/global_position/global` about the world's own anchor
automatically.

## Documentation

| Doc | Contents |
|---|---|
| [docs/architecture.md](blueboat_sss_sim/docs/architecture.md) | design principles, layout, dataflow, decision records |
| [docs/integration_guide.md](blueboat_sss_sim/docs/integration_guide.md) | install, launch composition, sim↔real swap, GPS contract |
| [docs/configuration_guide.md](blueboat_sss_sim/docs/configuration_guide.md) | every YAML knob (world / depth / sonar) + world-folder formats |
| [docs/topics.md](blueboat_sss_sim/docs/topics.md) | ROS interface + raw-frame byte map |
| [docs/sonar_model.md](blueboat_sss_sim/docs/sonar_model.md) | acoustic model, noise stack, walls/pontoons, explicit assumptions |
| [docs/seabed_model.md](blueboat_sss_sim/docs/seabed_model.md) | bathymetry presets, seabed layers, materials, measured seabed from a recording; geomorphology references |
| [docs/svlog_depth_survey.md](blueboat_sss_sim/docs/svlog_depth_survey.md) | how to survey a site so a `.svlog` can become a world's depth and reflectivity |
| [docs/sea_state_model.md](blueboat_sss_sim/docs/sea_state_model.md) | current + waves: physics, presets, Gazebo coupling, references (DOI), research log |
| [docs/sea_state_validation.md](blueboat_sss_sim/docs/sea_state_validation.md) | the same survey in calm water / current / waves / both, in Gazebo: how to run it, what it shows, results |
| [docs/developer_guide.md](blueboat_sss_sim/docs/developer_guide.md) | code tour, conventions, extension recipes, testing |
| [docs/roadmap.md](blueboat_sss_sim/docs/roadmap.md) | prioritized realism upgrades |
| [docs/engineering/raycast_design.md](blueboat_sss_sim/docs/engineering/raycast_design.md) | the ray-cast sensor: physics, paths, integration contract (§11) |
| [docs/engineering/raycast_assessment.md](blueboat_sss_sim/docs/engineering/raycast_assessment.md) | the ray-cast's calibration status, the harbour-first hybrid (§13), GPU memory (§13.7) |
| [docs/engineering/](blueboat_sss_sim/docs/engineering/) | field calibration: design, runbook, results (`calibration_results.md`) |

## What's inside

| Package | Role |
|---|---|
| `core/` | shared types, geometry, **geodesy** (`geo.py`), trajectories |
| `worldgen/` | scene model, terrain texture, **authored bathymetry** (`depth/`), object catalog + placement, SDF writer, **world folders** (`world_folder.py`), map exports |
| `sonar/` | calibrated acoustic renderer, noise, Ping-Protocol encoder, walls (quays/pontoons) + multipath; `sonar/raycast/` the ray-cast sensor (GPU) and its empirical correction layer |
| `calibration/` | field calibration against the `.svlog` corpus (`sss_calibration`): registry, manifests, the seabed engine, artifacts |
| `sea/` | ROS-free sea state: JONSWAP wave field, a seeded smooth Gauss-Markov current with weak eddies, hydrostatic wave force on the hull, presets, plain-language options and timelines |
| `builder/` | the world-builder GUI (the only PySide6 code; zero ROS) |
| `analysis/` | ground-truth detection metrics, `.svlog` reader, sim-to-real calibration |
| `ros/` | `sss_sim_node` (the drop-in sonar), `mavros_shim_node`, `sea_state_node` (current + waves into Gazebo) |
