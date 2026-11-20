# Start tutorial: from a new world to analysing a run

*Moved verbatim from `README.md` on 2026-11-18 (session 11). **Since the 2026-11-18 freeze**, a run whose
results are used must be the frozen simulator (`python3 -m blueboat_sss_sim.core.provenance --check` → code
digest `cc29e08780aac4cd`) inside the envelope of `../SIM_VALIDITY_CONDITIONS.md` (superproject root):
the production ray-cast `raycast_harbour_fast` at 20 m / 1200 bins, altitude 4.5–7.0 m, campaign worlds.
The tutorial below still shows the legacy geometric renderer first; Step 7 is the ray-cast. Labels come
from the rendered picture and the world metadata, never from a contact.*

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
