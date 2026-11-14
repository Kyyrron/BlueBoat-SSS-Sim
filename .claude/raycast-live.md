# Ray-cast renderer: selection, slow-down, live runs (SSS-Sim CLAUDE.md §3.1)

<!-- Moved verbatim from .claude/CLAUDE.md §3.1 on 2026-11-07 (/doctor) so it loads on demand. -->

**Renderer selection and the slow-down (2026-11-07, owner order;
`raycast_design.md` §11).** `renderer` `""` = the profile's top-level
`renderer:` key, else `geometric`; `raycast` builds the ray-cast sensor
(`sonar/raycast`, imported only then) from the profile's `raycast:` section,
the encoder/noise stage under its system gain (`shared_stage_model`),
re-read on every enable (mesh + oracle built once per node). A profile
without a `raycast:` section leaves pinging OFF with an error.
`raycast_backend` `auto` (default) = `warp` when the `warp` module imports in
the node's Python, else the profile's (the CPU reference, very slow). Launch
`sonar_renderer:=` (both launches; `raycast` without `sonar_config:=`
resolves `config/raycast_default_sonar.yaml`, over a legacy frozen profile).
The geometric path is byte-identical. The ray-cast stamps both sides at
the tick's sim time and renders both before publishing either (NC #3); a
failed render skips the cycle. **It is far slower than the 50 ms period, so
the run is slowed as a whole**: `full_mission_launch.py real_time_factor:=`
(0.01-4.0 since 2026-11-10, default 1) appends `gz sim -z <rtf / max_step_size>` (verified:
`-z 20` on the 10 ms world measures RTF 0.1999), and every node runs on
`/clock` (the control nodes with `time_source: ros`, the GCS processor
under `use_sim_time`), so the slowed run pings at 20 Hz in sim time and
records what a real-speed one does. A tick more than 1.5 periods of sim time
after the last one logs (throttled) the achieved rate and the factor that
would hold the period (the first 5 ticks after an enable are exempt). MCS offers both
(launch dialog: Sonar model, Slow-down x0.010-x4.000 since 2026-11-10).
**Live ray-cast prerequisites (2026-11-07).** `warp-lang==1.19.0` must import in the
**node's** interpreter: `sss_sim_node` runs on `/usr/bin/python3` (numpy 1.26), so it is
installed there (`pip install --user --break-system-packages`) as well as in the ROS venv
(superproject `requirements.txt`). A node launched with `renderer` resolving to `raycast`
builds and warms the renderer at start-up (one discarded cycle), so the first enable pings
at once; `core/gauss_legendre_cache.py` caches numpy's own Gauss-Legendre nodes (the GPU
beam tables, n up to 4096) under `~/.cache/blueboat_sss_sim/gauss_legendre/numpy-<v>/`,
bit for bit (a smoke check): without it the first render cost 61 s per start on the system
numpy, with it ~1 s. **Measured live** on `sea_validation/flat_5p0m` (RTX 4060, the CPU
shared with a 14-core job): one two-side cycle 0.5-0.8 s along the survey line (renders are
mostly numpy on the CPU; 0.13-0.35 s per side offline), so x0.15 pinged 8-12.5 Hz and
**x0.08 held 20 Hz** (no late tick in 2.5 min).
**Harbour, end to end (2026-11-07).** The run went through the MCS App's own `LaunchManager`: Sonar model
Ray-cast, Slow-down x0.03, Extra args `sonar_config:=~/calib/raycast/correction/raycast_harbour_hybrid_v2_sonar.yaml`.
- Setup: `Shiraishijima-wl3p00`, 20 m / 1200 bins, PID on the lawnmower path.
- The GCS App ran START → Record → STOP and produced 3 seabed images (schema 5), the adopted `.svlog`, the
  waterfall, the mosaic and Perception's detections, as at real speed.
- Rate: 20.3 Hz in sim time. GPU: 1.5 GB in use.
- Harness and evidence: `~/ros2_ws/sim_runs/raycast/e2e/`.

**Known limitation seen there.** The processor's bottom tracker does not lock on the ray-cast's first return:
- tracked +1.4 to +1.6 m late, jumping by metres;
- the geometric renderer at the same poses tracks within 2 cm.

The images themselves are fine, but their georeference uses that altitude (`raycast_assessment.md` §13.6).

**Range limit (GPU memory).** The harbour worst case, one renderer, peaks at 2.3 GB at 20 m, 3.6 GB at 30 m and
6.3 GB at 40 m. At 50 m it runs out of memory: CUDA error 101, logged as `ray-cast render failed … cycle skipped`.
Keep the ray-cast ≤ 30 m and never above 40 m on the 8 GB GPU. The GCS's range control re-ranges the node too.
Sources: README, `raycast_assessment.md` §13.7.

**Ground truth under the ray-cast (2026-11-11).** With `raycast.labels.enabled` (the two harbour
profiles; off elsewhere) `sonar/raycast/labels.py` fills `RenderedPing.contacts` from the renderer's
own mesh and visibility oracle: which objects a ping insonifies, `visible`, and `shadow_bins` =
seabed points behind the object whose first hit is that object (terrain, quays and other objects
included). Same schema as the geometric sonar, direct path only, about 1 ms per ping-side with an
object in the strip; the ping is unchanged. `ground_truth/sensor` also carries `range_scale`
(1500 / c): the ray-cast images a point at true range R at 0.973 R, as the real device does, and
Perception's label geometry multiplies its slant ranges by it (without it a box sits ~0.4 m past
its echo at 15 m). Gates: `python3 -m test.raycast_test --only L` ([L1]-[L7]).
**Bottom line (NC #4 for the ray-cast).** `analysis/raycast_bottomline.py` replays a recorded
session's track through the current source and the processor's OWN detector and `FBRTracker`.
On the 2026-11-11 track (3 305 pings, harbour fast): detection 100 %, first return − truth median
−0.02 m, 99.1–99.6 % within 0.30 m, tracker locked 99.7 %, jumps p90 0.08–0.10 m (the session's own
run, a STALE install, read +1.35 m late). Offline, heave not replayed: the owner's live run after a
clean rebuild is the confirmation.
`main()` also swallows rclpy's teardown race (`RCLError` "context is not valid" once `rclpy.ok()` is False). The
long ray-cast callback had made it print a traceback and "process has died" at every MCS stop.

**Holding the simulation, and the live variants (2026-11-09; plan
`~/.claude/plans/find-the-lastest-session-prancy-torvalds.md`, evidence
`~/ros2_ws/sim_runs/raycast/live_opt/`).** The first full MCS + GCS ray-cast run
(`SSS_data/sessions/2026_11_09-18_13_06`, harbour `Shiraishijima-wl3p00`, default profile, x0.10)
held 20 Hz for 78 s of open water and then lost **501 of 2 212 pings**. The cause was the quay
in the port beam (`wall_3`, 17 m away): a port ping-side cost 1–7 s and **16 s at the pier corner**,
against 0.06–0.08 s in open water. Where the time goes there:
- the quay boxes are sampled whole at 5 mm (buried part, back face, top above water), so 23–38 % of
  all contributors receive no sound but pay every bound and pair;
- the multipath selection bounds every candidate sequence (71 at the corner, ~0.13 s each);
- diffraction on the wall-face legs costs 1–5 s.

Over the whole planned path the median cycle is 0.12 s: **the cost is a rare quay spike**, so a
uniform slow-down is the wrong lever (x0.003 would be needed at the corner).

- **Hold** (`core/sim_hold.py`; node `hold_sim_while_rendering` `auto` = `when_needed` for the
  ray-cast; `full_mission_launch.py sonar_hold_sim:=`): Gazebo runs at `real_time_factor`, and the
  node pauses the world (`/world/<gz_world_name>/control`, gz-transport) when a cycle outlasts
  0.6 × its wall budget (`period / real_time_factor`), resuming as soon as it ends. Every ping keeps
  its 50 ms sim-time period whatever it costs. `always` pauses around every render (fallback);
  without a world name (a standalone sonar, the GCS bag-replay sonar) nothing is held.
  `~/ground_truth/sensor` says `sim_held` per cycle. With hold, **Slow-down x0.5 is the fastest
  setting**; the full lawnmower mission is estimated at ~1.3 h (live) / ~0.9 h (fast), 0 pings lost
  (fixed x0.10 today: 2.2 h, ~4 % lost, all at the quays).
- **Live variants** (generated by `sonar/raycast/config.write_variant_profiles`, never hand-edited;
  listed by MCS's Sonar model menu through their `variant:` block):
  - `raycast_live_sonar.yaml` = default + `paths.max_order: 1`: **bit-identical** on 176 / 176 route
    side-pings, the 12 session poses and an object pass (tyre, box, pipe on flat 5 m); pier corner
    15.9 → 7.4 s.
  - `raycast_fast_sonar.yaml` (2026-11-10) = live + the fast engine paths (`config.ENGINE_KNOBS`):
    `sampling.cull_unreachable`, `paths.reselect_every: 10` / `reselect_move_m: 0.3`,
    `diffraction.method: edge` (diffraction back on, closed form), `precision: fp32`. Offline, one
    harbour ping-side: pier corner 2.04 → 0.29 s (0.17 s on a reused-selection ping), quay
    0.91 → 0.43 / 0.23 s, open water 0.08 → 0.07 s (`~/ros2_ws/sim_runs/raycast/fast_opt/timing_*.json`).
    Its look vs "Ray-cast" is the owner's live test; no look claim is made offline. Before
    2026-11-10 it was live + `diffraction.enabled: false` (identical by eye, −35 % mission time).
    **Phase 2 (2026-11-10, owner: "real time, faster if possible"):** + `sampling.scatterer_spacing_m:
    0.010` (owner's choice, look-changing: speckle redrawn, mean level ≲ 0.15 dB by the flat-sand sweep)
    and `sampling.band_cull` (sheet points only inside the beam band at their range: exact without a kept
    multipath sequence, near-exact with one); the floor channel's kept image pairs reused with the
    selection. Every profile also lost exact overhead (vectorised wall plan, the envelope table uploaded
    once, no per-stage sync in the node). Run loop: the hold pauses `max(5 ms, 0.1 × budget)` before the
    next tick, and `real_time_factor` / the MCS Slow-down go up to **x4**. Timings and the as-built notes:
    `docs/engineering/raycast_fast_optimize_plan.md` §4b.
  - `raycast_default_sonar.yaml` stays the full model (the pool's comb needs orders ≥ 2; the fit
    used it).
  - Rejected, visible: wall ghosts off, `beam.floor_db −30`, 7 mm scatterers, coarser diffraction.
- **Recommended MCS settings:** Sonar model "Ray-cast — live" (exact, Slow-down x0.5–x1) or "— fast"
  (Slow-down **x2**, try x3–x4; quays are still held, so a high setting only speeds the open water),
  no extra args (the Gazebo GUI runs as a separate client since 2026-11-10); `headless:=true` hides it
  and frees its share of the GPU.
- **GCS processor:** it now follows a `/clock` that appears after its start-up (it starts before
  Gazebo, so its rate warnings were wall-paced: "2.0 pings/s" was 20 Hz of sim time).
- **Verified live (2026-11-09, headless, harbour, live variant, x0.5; `live_opt/hold_live_test/`).**
  - 1 548 pings over 77 s of sim time, **every gap exactly 50.0 ms, 0 late ticks, 0 "behind the sim
    clock" warnings**, including about 125 pier-corner pings at 4–7 s of wall time each (the offline
    7.4 s); every cycle was held (0.15 s open-water renders exceed the 60 ms arm point).
  - Open water ran at an effective x0.32 (better than the x0.25 estimate).
  - `/clock` **keeps publishing while Gazebo is paused** (largest wall gap 0.12 s), so the MCS station
    clock never reports a stall.
  - Teardown: a resume that fails because Gazebo is already gone is silent (fixed after this run).
    Control's end-of-run reports can print a KeyboardInterrupt traceback when a CLI launch is killed
    with SIGINT; MCS's Stop runs the safe shutdown first. RViz still starts in a `headless:=true`
    launch (Control's `upload_rov_launch.py` has no `rviz` pass-through; frozen module).

**Gazebo GUI crash at start-up (2026-11-09, 23:23 run).** An MCS ray-cast launch with the Gazebo GUI
on froze: the **GUI process** segfaulted ~5 s after start in gz-transport's discovery thread
(`Discovery::RecvMessages` → `Version()` → `env("GZ_TRANSPORT_TOPIC_STATISTICS")` → `getenv`, called for
every discovery message, racing the GUI's own environment writes at start-up: an upstream,
intermittent race inside Gazebo's GUI process). The `gz sim` wrapper then stopped the server with SIGINT
(its `server_console.log` shows a clean shutdown) and exited 0, so the launch kept running with no
Gazebo: no `/clock`, the boat never moved (empty poslog), the sonar logged `/world/generated_ocean/control is
not advertised`, and the sea-state injection timed out — consequences, not separate bugs. Not caused by
the hold: the sonar opens its gz-transport node only at ping enable, ~5 s after Gazebo had gone.
Fixes in `full_mission_launch.py`:
- `headless` defaults to `auto`: **server only for the ray-cast** (the GUI can crash and competes for the
  GPU), the GUI as before for the geometric sonar; `headless:=false` shows it. MCS needs no change.
  Verified live: a ray-cast launch without the argument runs `gz sim … -r -s -z 50`, hold engaged.
- Gazebo exiting by itself makes the launch log the reason and **shut down** instead of freezing
  (`gazebo_exit_actions`, never during an intended shutdown). Smoke-checked; not yet seen live.
- The sonar retries finding `/world/<name>/control` every 2 s when Gazebo was not up at enable (a slow
  world load would otherwise have left the hold off for the whole run).
- **2026-11-10: the GUI is back, crash-isolated.** Server-only hid Gazebo from every MCS ray-cast run
  ("only RViz starts"), though the server was running. `auto` with the ray-cast now runs the server with
  `-s` and the GUI as a **separate, respawning client** (`gz sim -g --force-version 8`, process `gz_gui`,
  `gazebo_gui_mode` / `gz_gui_client_cmd`). A GUI segfault then kills only the window, which comes back
  ~3 s later. The server, `/clock` and the run go on, and `gazebo_exit_actions` ignores `gz_gui`.
  `headless:=false` = one process as before; `headless:=true` = no GUI. Smoke-checked; not yet seen live.

**The `fast` variant's engine paths (done 2026-11-10).** Spec and status log:
`blueboat_sss_sim/docs/engineering/raycast_fast_optimize_plan.md`. Rule: **only the fast option changes**;
default and live stay identical (guard `~/ros2_ws/sim_runs/raycast/fast_opt/guard.py --check`: six harbour /
object / pool poses, both sides, within 1e-9 relative; a repeat render differs by ~1e-15 from GPU atomic
order, so "bit-identical" means that). Code tests: `python3 -m test.raycast_test --only F`. Read the plan
before touching `sonar/raycast` for speed. Where the time now goes at a quay: the fp64 echo-path geometry in
the pair and bound kernels (chain building); fp32 visibility gained only 2–5 %.

**Harbour profiles (2026-11-10; plan `~/.claude/plans/pasted-content-id-09ed-see-in-sleepy-hartmanis.md`,
evidence `~/ros2_ws/sim_runs/raycast/harbour_patch/`).** The 2026-11-10 harbour comparison
(`~/ros2_ws/sim_runs/waves_review/REPORT.md`) measured, on the fast profile at 20 m / 1200 / 44.3 µs:
seabed **+11 dB** hot (71–78 % of the detector input saturated), anchor shadows **empty** (−41…−58 dB
re the seabed in front; real −3…−34, geometric −20…−27), the first return read +0.5–0.6 m late.
Why, and the patch (`config/raycast_harbour_live_sonar.yaml`, `raycast_harbour_fast_sonar.yaml`,
generated by `write_variant_profiles`; MCS lists them; default / live / fast unchanged):
- **Level.** Every other ray-cast profile carries the provisional core (`PROVISIONAL_RAYCAST`, gain
  143.7, TVG 12), anchored at bath3 alone (1.4 m altitude) -- the single-altitude trap of NC #6. The
  harbour fit of 2026-11-07 (hybrid v2, `~/calib/raycast/correction/`) never reached a profile. The
  harbour profiles carry its core (`config.HARBOUR_V2_CORE`: gain 126.91, TVG 3.76, SAX99 sand,
  f_v 0.217, beam height 47.1°, surface 1 mm): on the validation sessions the far-field seabed is at
  real − sim = −0.3…+1.6 dB at this acquisition.
- **First return.** That core leaves the near-nadir crest 5–10 dB cold, which the tracker reads late.
  Its empirical table (`config/raycast_correction_harbour_v2.yaml`, a byte copy of the v2 artifact,
  sha256 9557a8a1…) restores it, but is fitted down to 5.55 m only. The profiles **edge-hold** it to
  4.5 m (`correction.altitude_hold_m`, status `applied:held`; validation strata at 4.8–5.4 m) and fade it
  to 0 at 3.5 m (`applied:taper`); above 11.3 m it stays off. `accept_engine_knobs` lets the artifact
  apply under the live/fast engine keys (`config.PERFORMANCE_KEYS`); a physics difference still raises.
  No new crest physics (earlier attempts failed): at this acquisition the real harbour has no sharp crest
  and its tracker jumps 0.16 / 0.88 m (p50 / p90), so the geometric's 2 cm lock is not the target.
- **Shadows.** The design chose "shadow floor from physics only" (`reverberation_db` 0), but its test D3
  (the corpus quay) never ran (CUDA 101) and the fill sources are absent in fast mode (no volume
  reverberation, coherent surface, no object diffraction). The profiles turn on the geometric's measured
  NC #15 floor: `raycast.floor.reverberation_db` 18 under the ping's own ambient reference
  (`sonar/raycast/floor.py`: block medians of its mean over u ≥ 1.2, a power-law fit; CPU, O(bins)),
  noise stage only (the renderer's model, hence multipath selection, is untouched).
- **Measured once** (`harbour_patch/level_check.py`, flat sand vs the toOptimize strata at 44.3 µs,
  4.8 / 6.0 / 7.5 m): far seabed real − sim −10.7…−12.2 → **−0.8…−1.5 dB**, crest −3.3…−7.0 →
  **−2.0…+1.8 dB**, fast vs full engine ≤ 0.19 dB (`raycast_assessment.md` §14). The bottom line and
  the shadows are the owner's live look test.
- **v2, the same day, after the owner's run** (`SSS_data/sessions/2026_11_10-08_37_26`, "the nadir
  zone looked better before; horizontal lines"; evidence `~/ros2_ws/sim_runs/raycast/harbour_patch/line_diag/`,
  `near_range/`). Two causes, neither the correction table, whose crest was right (45–46 dB mean vs
  real 46–52):
  - **Shadow lines = the floor's reference (v1).** A free per-ping power-law fit jumped 6–8 dB between
    pings behind the anchor. Now (`floor.py`): fitted on the physical mean (before the table), Theil–Sen
    slope and median level, both smoothed across pings per side, the table's factor carried across the
    shadow. On the run's track: ping-to-ping p90 0.09–0.25 dB, max 1.4–2.3 dB (was 7.6).
  - **Bottom line = the shared model's near range, measured at 600 bins.** At 1200 bins / 44.3 µs the
    real device rings 3× shorter (the processor's rule finds its end at 0.138 m, sim 0.47 m) with a
    water column ~12 dB quieter (14.5 dB mean of dB, sim 26). The rule's noise estimate (real 28.5 dB,
    sim 41) + 8 dB put its threshold (49 dB) above a correctly levelled seabed (40–46 dB): it fired on
    speckle (tracker − true p90 +3.2 m, jumps p90 1.25 m). The +11 dB-hot seabed had hidden this; the
    geometric hides it with its sharp crest. Now `raycast.near_range` (`config.HARBOUR_NEAR_RANGE`):
    the ringing table and water column (17.04 dB, Exp(1)) MEASURED on AM toOptimize (8 372 pings,
    1200 / 44.3 µs / gain 4; generator `near_range/make_table.py`), applied only at that acquisition
    (`num_results` 1200, pulse 44.3–50 µs), noise stage + coherent channel only. On the run's track
    (1 052 pings a side): ringing end 0.133 m, noise estimate 29.4 dB, FBR − true nadir median −0.03 /
    −0.06 m, 99.7–100 % within 0.3 m, jumps p50 / p90 0.02 / 0.07 m (real 0.16 / 0.88: the sim is now
    steadier than the real boat, which also heaves and sees fish).
- **Status, for the thesis:** a provisional *site* calibration with an empirical layer that failed its
  strict gates (P2 at 9–10 m, P3, S6) while improving every band; harbour worlds only; never the
  sim-to-real comparison profile; the floor is a measured stand-in, not physics (D3 open).
- **2026-11-11 (independent review, option A; `docs/engineering/independent_review_2026-11-11.md`):**
  the harbour profiles also set `floor.addition: field` (the floor added as a field: the plain
  seabed's dark speckle tail stays at −20 dB re the mean, it was −16.7 with the power floor; real
  −20.5), `labels.enabled` (above) and `seed_per_run` (each run's speckle from its noise seed,
  replayable with `seed:=`), and **`objects.mode: solid`**: the sheet is the terrain (`depth.npz`)
  and every object is its own mesh, built at load from the manifest (`sonar/raycast/object_mesh.py`:
  the catalog primitive, or the class's mask at ≤ 1 cm; existing worlds need no rebuild). Measured on
  the harbour world, one ping abeam of each object, against the same seabed without it: the 7 cm
  tent weight's shadow −4 → −16 dB (highlight +7 dB), pallets −14…−18 → −20…−33 dB, bicycle tyres
  (2 cm proud, rubber) a dark ring without highlight in both modes. Whole 11-11 track: 35 vs 34 ms
  per cycle (+3 %), tracker unchanged. Gates `raycast_test --only O` ([O1]–[O8]). Default / live /
  fast are unchanged, and the harbour profiles change only where rough objects are in view
  (`~/ros2_ws/sim_runs/raycast/review_a/guard_a.py --check`, `guard_detail.py`).
- **2026-11-11, second pass (one surface rule; `docs/simulator_justification.md`):**
  - **Objects are parts with materials** (`worldgen/compounds.py`, `object_mesh.py`): a pallet is 13
    boards (156 triangles, was a 15 000-triangle mask skin), a bicycle 14 parts in three materials, a
    car tyre a steel-belted tread band (`tyre_tread`, +8.0 dB re sand, an upper bound: no absorption)
    on its rubber body; the mask skins are gone. Every part scatters diffusely; a smooth one also
    mirrors, and the diffuse law carries its share `1 − C` (`scattering.DiffuseLaw`, the same in the
    GPU kernels). `raycast_test --only O`: fully fouled, a plastic block is the sand-equivalent block
    to 1e-9; a belted tyre peaks +9.8 dB over the seabed (bare rubber +0.4 dB).
  - **Fouling** `raycast.objects.fouling` (0 in every profile; node `fouling`, launch
    `sonar_fouling:=`): the fraction of object and wall surface that scatters like the seabed and
    does not mirror. Not calibrated: sweep it.
  - **World:** a wall is built of `Wall.material` (concrete default, or metal = steel sheet pile;
    World Builder wall dialog; steel − concrete = +2.46 dB); gravel and rocks take the sand law at
    their published level (+3 / +6.5 dB, J&R §12.1; seagrass stays a gap); the diffuse law and the
    seabed mean field read the world's `materials:` overrides.
  - **Nadir onset** `correction.onset_fade_m: 0.8`: the table fades in as √(distance) after the
    first seabed echo. Real (first 1 500 pings of `toOptimize-harbourScan`, 1200 bins): the first
    return starts 9.5 dB under the far seabed and climbs over 0.5–0.7 m; before, the sim stepped up
    in 2 bins with a +4 dB crest. Now within 2.6 dB of the real profile at every offset (1.4 dB rms);
    processor tracker on the 11:54 track: locked 99.7 %, altitude − truth median +0.28 m (p10 +0.04,
    p90 +0.63), jumps p50 / p90 0.13 / 0.42 m (real 0.16 / 0.88). Tried and rejected: smooth step
    0.8 m (4.5 dB, +0.49 m), √ 1.2 m (2.3 dB, +0.42 m), √ 0.6 m (3.0 dB, +0.20 m)
    (`~/ros2_ws/sim_runs/raycast/review_a/bottomline_2026_11_11b_*.json`).
  - Default / live / fast: unchanged to 1e-15 (`guard_a.py --check`); harbour: identical beyond the
    fade at object-free poses. Cost: a pallet in reach adds its ~28 000 mirror patches (40 vs 35 ms
    per cycle on the 07:57 track with stage timing on).
  - Not modelled: hollow bodies, sheet-pile corrugation, rubber absorption, a mirror return from a
    wall set smoother than 1.6 mm; a compound's faces are not bounce mirrors.

**Completion round, session 01 (2026-11-14): the harbour profiles after verification.** Evidence and commands:
`~/ros2_ws/sim_runs/sim_acceptance/01_sonar_physics/handoff_01.md` (this machine only); model text:
`docs/sonar_model.md` section 15. What a session must know before touching `sonar/raycast`:

- **Roles.** `raycast_harbour_fast` = the production sensor (harbour worlds, altitude 4.5–11.3 m, range ≤ 30 m:
  `sampling.max_range_m` raises beyond it). `raycast_harbour_live` = the same physics on slow settings, the
  reference for convergence. `…_flash_narrow`, `…_flash_wide`, `…_shell_opaque` = bracket ends for the sensitivity
  study, each production with one documented physical input at an end. `raycast_harbour_{fast,live}_20261112` =
  the profiles as they rendered for the 2026-11-12 review (both legacy phase switches on; not listed in MCS;
  regression only). The generic `raycast_default / live / fast` warn when loaded on a scene with objects.
- **Two phase conventions were wrong and are fixed** (carrier `exp(−i ω τ)`): the near-field pattern
  (`beam.legacy_phase_sign`) and the seabed / wall reflection coefficient (`paths.legacy_reflection_phase`).
  Any new complex factor in a coherent sum must be written in the carrier's convention; smoke `[9]` and `[19]`
  hold the two.
- **New inputs, all off by default** (a profile without them renders as before): `objects.layer_model: elastic`
  (`layers.py`, shell walls as solid plates; constants `worldgen/materials.py:_ELASTIC`),
  `objects.face_tilt_rms_deg / face_corr_len_m / face_draft_deg` (`object_mesh.py:FaceGeometry`),
  `beam.taper_pedestal` (`beam.py:taper`), `sampling.max_range_m`. The harbour production patch is
  `config.py:HARBOUR_PRODUCTION_2026_11_14`; the harbour core's 1 mm sea surface is dropped (engine default
  0.02 m), which removes the zig-zag fringes of the beach replays.
- **The correction artifact's core check** accepts `config.py:ACCEPTED_CORE_KEYS` at another value and ignores
  `correction.py:_NOT_CORE_KEYS`: each was shown not to move the seabed mean. A key that does move it must not
  be added there; refit the level instead (NC #6: once).
- **Regenerate, never hand-edit**: `python3 -c "from blueboat_sss_sim.sonar.raycast.config import
  write_variant_profiles; import pathlib; write_variant_profiles(pathlib.Path('config'))"` from the package
  source root; smoke `[9]` compares each packaged file with its generator.
- **Cost of an engine edit**: any change to a kernel of `gpu_render.py` rebuilds the kernels (about 3.5 min on
  first use). The whole-engine verification (V1–V9) is on the GPU and outside the smoke gate; rerun it from the
  evidence folder after an engine change.
- **Many objects in reach** (`gpu_render.py:GpuMultipath.facets`, `_bounce_candidates`, `_faces_in_reach`): each
  smooth object's echo-path selection used to test every face of every other object as a bounce mirror, one
  device launch and read-back each (11 400 per ping-side with 30 objects, 3 s, none kept). Since 2026-11-14 a
  face goes to the device test only when a bounce on it can geometrically reach the object (the bound is in the
  docstring; it is conservative, so the kept set and the render are unchanged: 7e-16, the device's run-to-run
  difference). It applies with `paths.max_order: 1` only (the harbour profiles); a profile with a higher order
  keeps the full test, and its cost. Check after any change there:
  `throughput/facet_cost.py TAG` then `facet_cost.py compare A B` in the evidence folder. Still true: in a
  cluttered scene the selection is redone on most pings, because the set of faces within range changes with
  every step, so `paths.reselect_every` rarely applies there.
- **What a flash looks like in the harbour profiles** (atlas of 2026-11-14, evidence `atlas/`, handoff 2.18):
  an upright face returns through the corner path with the seabed, so its highlight grows with range (grazing
  40° at 6 m, 17° at 16 m in 5 m of water) and is absent at 6 m in 10 m of water. PP container's long face,
  5 m of water, half-width with the highlight ≥ 3 dB over the seabed, narrow / production / wide: ±0.5° / ±0.25° /
  none at 6 m; ±2.5° / ±1.0° / ±5.0° at 16 m. A wavy face removes a weak flash rather than widening it. The
  opposite face of the same object (another seeded waviness) differs by 0.7 dB in the median, up to 5.6 dB: do
  not read a single pass as the object's level. A study that depends on flashes runs on both bracket ends.
- **Live cost and ping fidelity, measured once** (2026-11-14, headless, real-harbour world, harbour-fast at
  20 m, requested factor 2, hold `when_needed`; evidence `throughput/live/…/c2_live_summary.json`, handoff 2.17):
  real-time factor 1.19 (1.08 near the quays to 1.42 in open water), about 84 missions of 20 simulated minutes
  per 24 h; 29 per 24 h at the offline cost of a 30-object scene. Ping numbers consecutive on both sides, but
  99.90 % of the nominal 20 Hz: one 470 ms gap 1.4 s after the first ping (a 239 ms start-up cycle) and 44
  intervals of 40 or 60 ms among 9 612. Before reading a live recording as "every ping", check its stamp
  intervals (`throughput/live_summary.py <run folder>`); `hold_sim_while_rendering:=always` is untested here.
