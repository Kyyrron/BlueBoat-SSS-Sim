# Gazebo gate — waves and current end to end

Moved from `.claude/CLAUDE.md` §7 on 2026-11-10. Read before running or changing
`test/gazebo_sea_test.py`, `analysis/sea_validation.py`, or the sea-validation inputs.

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
which passed when re-run (`TODO.md`).
