# BlueBoat-SSS-Sim — development guide

How to work on the simulator: the AI coding assistant setup, the tests, the freeze rules and what a
future version could improve. For using the simulator, see the [README](../README.md) and the
[tutorial](TUTORIAL.md). For a tour of the code, see the package's
[developer guide](../blueboat_sss_sim/docs/developer_guide.md); this page does not repeat it.

## 1. Install Claude Code

The project is developed with [Claude Code](https://docs.claude.com/en/docs/claude-code/overview),
an AI coding assistant that runs in the terminal.

```bash
curl -fsSL https://claude.ai/install.sh | bash        # native installer
# or, with Node.js 18 or newer:
npm install -g @anthropic-ai/claude-code
```

Then start it and log in with your Anthropic account:

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar               # the whole project: cross-module rules included
claude
# or only the simulator (needed for its test hook, below):
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-SSS-Sim && claude
```

From the superproject, Claude also reads the project rules, the simulator's validity conditions and
the shared skills (among them `sim-run-recording`, which runs and records a simulated survey from the
command line, and `svlog`, for reading sonar logs).

## 2. What Claude reads in this module

| File | Role |
|---|---|
| `.claude/CLAUDE.md` | Loaded at start: the freeze, the architecture in brief, the interface and the non-negotiable rules |
| `.claude/constraints.md` | The 17 non-negotiable rules in full, with their reasons |
| `.claude/interface.md` | The sonar node, the autopilot stand-in node, the World Builder, the command-line tools, the launch files |
| `.claude/raycast-live.md`, `sensor-pose.md` | The ray-cast sensor in live runs; the sensor's mounting and pose path |
| `.claude/worldgen-campaign.md`, `data-config.md` | Generated worlds, world folders and settings |
| `.claude/development.md`, `gazebo-gate.md` | Architecture, commands, working notes; the Gazebo sea-state test |
| `.claude/rules/*.md` | Load the right topic file automatically, including a reminder of the freeze on every package file |
| `.claude/TODO.md`, `TODO2.md` | Open items, for a future version only |
| `blueboat_sss_sim/launch/CLAUDE.md`, `.../builder/CLAUDE.md`, `.../calibration/CLAUDE.md`, `.../sea/CLAUDE.md` | Folder notes, loaded when Claude works there |

## 3. What to set up after a fresh clone

Two Claude files are not tracked by git (`.gitignore`):

- **`.claude/tools/smoke_gate.py`**, the test gate script. After an edit of a package Python file
  or a `config/*.yaml`, it marks the session, and when Claude wants to stop it runs the offline
  smoke test and blocks the stop until it passes. Ask the owner for the file.
- **`.claude/settings.json`**, which wires that script as two hooks: one after every edit
  (`smoke_gate.py --post-tool-use`, timeout 10 s) and one when Claude stops
  (`smoke_gate.py --stop`, timeout 450 s, which must stay above the script's own 420 s limit).
  Without the script, the stop hook refuses to stop with "smoke-test gate missing", so create both
  files together or neither.

Also:

- **Warp for the system Python** (`warp-lang==1.19.0`), and an NVIDIA card, for the ray-cast sensor.
- **The field corpus** under `~/ros2_ws/data/SSS_data` (or `$BLUEBOAT_SSS_CORPUS`). Calibration checks
  skip without it.
- **The BlueBoat-SSS checkout beside this repository**: one smoke check reads the real driver's
  source to prove both declare the same parameters.

## 4. Tests

From the package folder `BlueBoat-SSS-Sim/blueboat_sss_sim/`:

```bash
python3 -m blueboat_sss_sim.core.provenance --check     # which code is installed (exit 1 = stale install)
python3 -m test.smoke_test                              # offline gate: about 540 checks, about 4 minutes
QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke     # World Builder screens
QT_QPA_PLATFORM=offscreen python3 -m test.placement_test   # the Mission-like placement screen (about 3 minutes)
```

The population model's own tests, from `BlueBoat-SSS-Sim/blueboat_harbour_population/`:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest      # about 1 minute
```

The Gazebo sea-state test runs in real time (5 to 6 minutes per scenario) and is described in
`.claude/gazebo-gate.md`.

Two traps:

- **Running a script by its path imports the installed package**, not your source tree. Prefix
  `PYTHONPATH=<package folder>` when you measure source changes.
- **The package installs by copy.** After any change, rebuild before a live run, or the run uses the
  old code (the provenance check says so).

## 5. Rules for changes

- **The simulator code is frozen.** No change to any Python file of the package or to
  `config/*.yaml`. A change means a new simulator version, decided by the owner only. The launch files,
  the repository-root `docs/` and the population model are outside the frozen code.
- **Do not add or remove files in `blueboat_sss_sim/docs/`**: they are installed with the package.
  New notes go to the repository-root `docs/`.
- **The simulated sonar is a drop-in replacement**: topic names, message types, delivery settings,
  parameter names and defaults equal the real driver's. Never define a new message type; extra
  simulation data goes on additional JSON topics.
- **Never modify another module**; serve identical interfaces instead.
- **Worlds are never overwritten**; regenerate instead.
- **Objects are rendered from material physics, never tuned one by one.** A brightness setting on an
  object is not allowed.
- **Power calibration moves once, on both field anchors**, never to fit a single recording.
- AI sessions never commit, push or tag: the owner does.

## 6. Next steps (for a future simulator version)

From `.claude/TODO.md`; none of them is worked on in the frozen version.

1. **Measure the beam and the sonar's mounting on the real boat.** The vertical beam pattern could
   not be measured from the recordings, and the mounting position is adopted from a drawing.
2. **Record known objects in the harbour at 5 to 7 m depth.** This would test how a flat face's
   bright "flash" depends on the viewing angle (today a bracket of two settings), and the plastic box,
   whose shadow is not validated at harbour depth.
3. **Test the corner-reflection rule on sand.** An upright face on sand returns most of its echo
   through a seabed bounce, and no recording tests that rule yet.
4. **Make the simulated boat yaw like the real one.** In simulation it yaws less than the real boat
   on straight lines (with the model predictive controller), which makes simulated pictures steadier
   than real ones.
5. **Finish a live Gazebo pass of a World Builder world with the user interface**, and mark
   hand-placed objects in the world's `metadata.yaml`.
