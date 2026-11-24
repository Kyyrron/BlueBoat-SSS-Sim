# Gazebo gate — waves and current end to end

Moved from `.claude/CLAUDE.md` §7 on 2026-11-10. Read before running or changing
`test/gazebo_sea_test.py`, `analysis/sea_validation.py`, or the sea-validation inputs.

```bash
source ~/ros2_ws/env.sh            # + colcon build --packages-select blueboat_sss_sim
python3 -m test.gazebo_sea_test --world-dir ~/worlds/sea_validation/flat_5p0m_v2 \
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
`~/ros2_ws/sim_runs/sea_validation/build_inputs.py` ((2026-11-18) not on this machine any more: `~/ros2_ws/sim_runs/sea_validation/` was removed before session 11; the `_v2` worlds' builder is `~/ros2_ws/sim_runs/sim_acceptance/02_world_builder/worlds_src/build_sea_validation_v2.py`) (deterministic, refuses
to overwrite). On 2026-10-20 the gate ran 24/25 (5 m) and 23/24 (1.5 m). The
only failure each time was an intermittent "fresh pose" stall in *waves*,
which passed when re-run (`TODO.md`).

**Since 2026-11-15 the gate's worlds are `flat_5p0m_v2` and `flat_1p5m_v2`**: the
object set version 2 deleted `pipe_pvc`, so the two worlds were rebuilt from
their own `builder_state.yaml` with `pipe_steel` (steel shell, 5 mm wall) in
place of the PVC pipes and `spec: 2` on every instance; positions, yaws, sizes,
burials, depth, seed and path unchanged (script and record:
`~/ros2_ws/sim_runs/sim_acceptance/02_world_builder/worlds_src/build_sea_validation_v2.py`,
`sea_validation_v2_built.yaml`; the 2026-10-20 script no longer exists).
`flat_1p5m` was deleted; `flat_5p0m` is kept only because the Belief
follow-up of session 06 reads its `metadata.yaml` -- it no longer loads as a
scene. **First run on the v2 worlds: 2026-11-16 (session 03)**, default `--waves harbour_chop` (the
physical fetch-limited sea since 2026-11-15; its July-attitude comparisons are REPORTS since then, the
motion check is "the waves move the hull, bounded"), the start hold on, geometric sonar:
`flat_1p5m_v2` **26 / 26**; `flat_5p0m_v2` **24 / 26** -- the two failures are the fresh-pose check in
*waves* and *both* (0.16 / 0.19 % of pings over 60 ms, worst 90 / 140 ms; rule <= 0.1 %), run while the
other world's gate shared the CPU; the same intermittent check as on 2026-10-20. Evidence and reports:
`~/ros2_ws/sim_runs/sim_acceptance/03_sea_state/gate/`. Run the gate from a shell WITHOUT `set -u`
(`/opt/ros/jazzy/setup.bash` fails under it). At harbour_chop the hull moves 0.23-0.34° (roll) and the sonar
consequences (back-scan, image correlation 0.4-0.6 against calm) still pass.
