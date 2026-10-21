# Sea state (`sea/`)

Moved verbatim from `BlueBoat-SSS-Sim/.claude/CLAUDE.md` §2 on 2026-09-28 so it
loads only when working in this folder (§6 and CM-3 point into that module file
and the root `CLAUDE.md`).

**Sea state (2026-09-03):** ROS-free `sea/` (JONSWAP waves with finite-depth
dispersion, Fossen Gauss-Markov current, hydrostatic Froude-Krylov force on
the catamaran hull minus Gazebo's own box term, presets + timelines) drives
`ros/sea_state_node.py`: the water velocity the boat's `Hydrodynamics`
plugin drags against + a one-shot wave wrench per physics step via gz-sim
`ApplyLinkWrench` (injected at run time if the world lacks it, so the stock
`Sim_launch.py` world works). Boat and sonar unchanged: roll/heave/yaw were
already coupled, pitch now is (`renderer._ping_geometry`, `dz·tan(pitch)`
along-track). **2026-09-05:** the node *predicts* the 20 Hz odometry to the
tick from its twist and adds hull damping (`sea/motion.py`) — the restoring
spring on a 50 ms-stale pose rang the 0.6 s heave mode at 1.6 Hz ("too
powerful, too regular, too frequent"); the field is short-crested (12
bearings, s 3, seeded random detune); the hull also pushes surge/sway/yaw
(first-order horizontal Froude–Krylov); wake groups are half as frequent and
spectrally broad; `simulate_3dof` closes the loop offline as the gate
(`docs/sea_state_model.md` §5.5–5.8, §9). Presets `config/sea_states.yaml`
(§6); physics + DOIs `docs/sea_state_model.md`; MCS picks the situation
after the world (its own code, CM-3).

**2026-10-08 — current, sensor, Gazebo gate.** The current is a pure
function of `(seed, t, x, y)` and the schedule (`current.py`): the
configured speed/direction are the **mean**; Fossen's Gauss-Markov
variation is sampled *exactly* on a fixed sim-time grid (Gillespie 1996),
joined C1 (Catmull-Rom), soft-bounded (`bounded`, ≈2.8 σ), never restarted
by a ramp (ramps blend means and sigmas, `CurrentField.components`), plus
a weak divergence-free eddy field (`EddyField`, Kraichnan modes 25-100 m,
judgement scale). The old Euler-Maruyama process stepped on the jittered
ROS-timer `dt` and shared one RNG between blend endpoints: same seed ≠
same current. Plain-language keys resolve **here, never in MCS**
(`presets.apply_variation` / `apply_wave_options`: `variation`,
`tp_s: auto`, `crest`, `wakes`, keyframe `ramp_s`); explicit fields win,
old files read unchanged. Rules that hold:

* Never step the current on a tick `dt` again, and never reset a unit
  process at a retarget — tick-independence and continuity are smoke
  checks in `[10]`.
* Nothing here simulates the boat offline beyond the existing
  `simulate_3dof` (the user's rule): end-to-end motion is tested in
  Gazebo by `test/gazebo_sea_test.py`.
* **One hull in every run (2026-10-09, S11 resolved by the user's call
  "make the choices that maximize the simulation's justification").**
  `hull.hydrostatics: catamaran` (default): the catamaran hydrostatics
  replace Gazebo's soft URDF box in every run, calm water included
  (`SeaModel.update`'s flat-water fast path, 0.14 ms a tick), so a change
  of sea never switches the hull and calm-vs-waves isolates the sea. The
  box's 2.2° under-way trim was a thrust couple on unphysically soft
  pitch stiffness; the corpus's trim is mostly a static per-session
  offset (at rest +1.2..+2.8°) with a small dynamic part of inconsistent
  sign — no trim term is fitted (`docs/sea_state_model.md` §5).
  `gazebo_box_when_calm` restores the old behaviour for old runs / A/B.
* **The wrench and the sonar run on the 100 Hz pose** (2026-10-09,
  `motion.HighRatePose` fed by `/sim/sea_state/boat_pose`): on the 20 Hz
  odometry the catamaran's roll restoring rang at 3.2 Hz (±0.5° on calm
  water; reproduced by a roll-only discrete model, stable on the 100 Hz
  pose up to 4 steps of delay). Never feed the stiff hull spring from the
  odometry again; it is only the fallback.

**2026-10-21 — synthetic yaw disturbance (`yaw_moment.py`, ordered by the
user for the AROB 2027 study in BlueBoat-Experiments).** An optional
`yaw_moment: {std_nm, tau_s}` block on a `SeaState` (keyframes and
commands) adds a seeded first-order Gauss-Markov moment to the wrench's
`torque.z` -- the current's `UnitGaussMarkov` with stream id 13 and knots
at `tau/12`. It is a fitted surrogate, not physics
(`docs/sea_state_model.md` §5.9). Rules that hold:

* **Absent means off and bit-identical**: `std_nm = 0` evaluates nothing;
  a state without the block serialises (`to_dict`) exactly as before.
  `[10]` checks both; never let the term touch force or the x/y torque.
* Ramps change its std, never its realisation; like the current, it is a
  pure function of `(seed, t)`.
* Campaign conditions using it are scenario files, never new keys in
  `config/sea_states.yaml` (MCS parses that file with its own reader).

