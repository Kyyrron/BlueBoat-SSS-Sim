# Ray-cast "fast" variant — optimisation plan (items 1–4)

Status: **implemented 2026-11-10** (code tests pass; the owner's live look test is pending). Owner order
2026-11-10, restructured the same day; the "as built" notes below record where the build departs from the
plan. Update the status log
at the end in the session that does the work.

**Shape of the work (owner, 2026-11-10): one session, three steps, no realism campaign.**
1. Make all the updates (items 1–4 below).
2. Make the code consistent and lean, and make every **code** test pass.
3. Hand over to the owner, who runs the simulation and judges speed against look.

The owner does the realism and look comparisons, so the session runs **none** of the previous draft's
look gates: no baseline campaign over 88 route poses, no dense quay set, no waterfalls, no difference images,
no mission estimate, no Gazebo launch. The evidence base stays `~/ros2_ws/sim_runs/raycast/live_opt/`.

## 0. Rules that hold for every change

- **Only `fast` changes.** `config/raycast_default_sonar.yaml` ("Ray-cast") and `raycast_live_sonar.yaml`
  ("Ray-cast — live") must render **bit-identically**. Every new code path sits behind a new knob whose
  default is today's behaviour; only the `raycast_fast_sonar.yaml` variant turns knobs on.
- **Modular by construction, so a later ray-cast update reaches every mode.**
  - Variants stay "default profile + deltas": `RAYCAST_VARIANTS` in `sonar/raycast/config.py`, written by
    `write_variant_profiles`. Change only the fast entry's deltas, then regenerate; never hand-edit a
    profile. The smoke check (`_diff == _want`) fails if a packaged variant is stale, so a later change to
    the default reaches fast as soon as it is regenerated.
  - **One kernel source per computation.** fp32 is the same kernel built at a second precision, through a
    dtype-parameterised factory (the `pair_kernel(name, n)` pattern) or a Warp generic kernel. Never a
    copied kernel: a later fix to a copy would not reach the other mode.
  - The edge diffraction (item 3) shares with the sampled method everything except the inner integral:
    leg construction, the `n1` / `n2` window, the two-way product and the application to the field. A later
    change to that shared code reaches both methods. The `[F]` test comparing the two methods fails if they
    drift apart.
  - The selection cache (item 2b) wraps the existing selection function and holds no second copy of its
    logic.
  - **No silent knob.** A fast knob the CPU reference does not implement raises a clear error on
    `backend: cpu`; it is never quietly ignored. The exception is `precision`, which by definition is
    GPU-only, so the CPU stays fp64.
- MCS is not touched: it re-reads the variant's label and description from the profile.
- No fitting; no change to acquisition, calibration constants or scatterer spacing; no `git commit` /
  `git push`; no Gazebo launch or process kill (memory `ask-before-live-sim-tests`).
- The fast variant targets the **harbour** (quays, first-order multipath). It is not the pool's mode,
  because the pool's comb needs higher orders. Owner, 2026-11-10: no second "full multipath" fast variant.

**Today's `fast`** = default + `paths.max_order: 1` + `diffraction.enabled: false`.
**Target `fast`** = today's + items 1–4, with diffraction back **on** through the closed form (owner,
2026-11-10).

## 1. Where the time goes (measured 2026-11-09, `RaycastRenderer.last_timing`, per port ping-side)

| pose | total | diffraction + classify | `select` | `pairs` | contributors | candidate / kept seqs |
|---|---|---|---|---|---|---|
| open water | 0.08 s | — | 0.01 | 0.01 | 2.1 M | 2 / 0 |
| pier corner x = 51.75 m (default) | 15.7 s | 5.1 | 9.4 | 1.0 | 6.1 M | 71 / 3 |
| pier corner (fast) | ~2.0 s | — | 0.96 | 0.96 | 6.1 M | 7 / 2 |
| quay, t = 90.5 s (fast) | 2.8 s | — | 1.33 | 1.34 | 6.7 M | 7 / 3 |

The cost drivers:
- Quay boxes are sampled whole at 5 mm, including the buried part, the land side and the top
  (`scene_mesh.py`, `_wall_emit`). 58–66 % of wall points cannot receive sound.
- `GpuMultipath._select_set` makes one full-n `_bound_k` launch plus one host sync per candidate sequence,
  in fp64.
- Every kernel is fp64, and the RTX 4060 runs fp64 at 1/64 of the fp32 rate.

## 2. Step 1 — the updates

### Before the first code change: a small exactness guard (minutes, not hours)

Write `~/ros2_ws/sim_runs/raycast/fast_opt/guard.py`, outside the repository and modelled on
`live_opt/stage_probe.py`. Run it with `/usr/bin/python3` and `PYTHONPATH=<package source root>`.

It renders `power` and `texture` (float64 `.npz`) for the **default** and **live** profiles at **six fixed
poses**, both sides:
- open water;
- one quay pose;
- the pier corner (x = 51.75 m) of `Shiraishijima-wl3p00`;
- one pose with objects in the beam, over `sea_validation/flat_5p0m`;
- one pool pose (`pool_line_and_back/pool_2w_1`);
- one turn pose.

It stores them once as the baseline. `guard.py --check` re-renders and asserts the largest |Δ| relative to
each array's scale is ≤ **1e-9**. *As built:* a repeat render is **not** exactly 0 (1e-15 to 1e-14 relative,
the GPU's fp64 atomic summation order), so "bit-identical" means this tolerance, far below any model
change. This is a code test, not a look test: it catches a shared-code edit that leaks into
the evidence profiles. The cost is about one minute per profile, dominated by the default's pier corner.

### Item 1 — do not create points that cannot receive sound

Knob `sampling.cull_unreachable: bool = False`; fast sets it `true`. The emit step culls, relative to the
transducer position P:
- (a) any scatterer above the still waterline (`z > 0`), which includes the box tops;
- (b) wall points buried below the local seabed by more than `surface_offset_m`, using the `Sheet` raster
  `_sheet_emit` already uses;
- (c) whole triangles of a `kind: quay` wall whose outward normal faces away from P (`n · (P − A) ≤ 0`).
  Pontoons keep every face.

Where:
- `GpuScene._wall_plan` skips the (c) triangles before the lattice launch;
- `_wall_emit` / `_sheet_emit` drop the (a) and (b) points;
- if `scene_mesh.py` lacks a per-triangle wall kind / owner array, add one.

CPU reference: implement it if it is a few lines in `renderer._scatterers`, otherwise raise on
`backend: cpu`.

*As built:* CPU and GPU both cull (`scatterers.cull_wall_triangle` / `wall_is_quay` are shared; the sheet's
count and emit passes share `gpu_render.sheet_point`); the wall kind is looked up per triangle from
`mesh.wall_index`, no new mesh array. [F1]: the GPU set equals the CPU set; 54 % of the points of a
harbour-corner test scene are culled.

### Item 2 — cheaper echo-path selection

- **2a, `paths.batched_bound: bool = False`** (fast: `true`), **exact**:
  - one 2-D launch over (contributor × the order's candidate sequences) into an
    `(n_seq, NCOPY, n_bins)` buffer, then one reduction and **one** host sync per order;
  - the refined `bound(1)` of a kept floor sequence stays a separate launch;
  - upload the `_seq_arrays` per order;
  - on the fast path only, read single elements with `arr[-1:]` instead of full `.numpy()` copies
    (`slot`, `runid`, `escan`, `_fc_patches`, `_sheet_plan`).
- **2b, `paths.reselect_every: int = 1`** (fast: `10`) **and `paths.reselect_move_m: float = 0.0`**
  (fast: `0.3`):
  - cache per side: the kept list, its boundary set `mp.boundaries(P)`, the sensor pose and the ping count;
  - reuse the cache while fewer than N pings have passed, the sensor has moved less than the distance and
    turned less than 2°, **and the boundary set is unchanged**;
  - kept sequences are still evaluated exactly every ping. Only the choice of which ones to evaluate is
    reused.

*As built:* **2a is dropped.** Code reading showed the cost of a selection is the per-contributor compute
(~0.19 s per sequence over 6.7 M contributors), not the per-sequence launch or host sync, so batching would
save almost nothing. It also cannot be bit-identical: `best` is raised after each kept sequence *within* an
order (`_select_set`), and fp64 atomic order changes on a 2-D grid. Its exact part was kept, everywhere
since it changes no value: one-element reads (`gpu_render._last`) instead of full `.numpy()` copies. 2b is
`GpuMultipath._cached_selection` (keyed by side, and by side and object for facets; a reused ping also skips
the `level` stage), with `renderer.render` passing `key=side`; `release_device` drops it, and the CPU
reference raises when `reselect_every > 1`. [F2] checks the reuse and every invalidation rule.

### Item 3 — closed-form straight-edge diffraction (diffraction back on in fast)

Knob `diffraction.method: str = "sampled"`; fast sets `"edge"` and `diffraction.enabled: true`.

The quantity is the same as D1 (the `diffraction.py` docstring): the per-leg Kirchhoff ratio U/U0. For a
half-plane it is `F(v) = ((1+i)/2) ∫_v^∞ exp(−iπt²/2) dt`, with:
- v = the edge's signed clearance × `sqrt(2/(λ d_eff))`, `d_eff = d1 d2/(d1+d2)`;
- this renderer's `exp(−iωτ)` sign;
- the same `n1` / `n2` window;
- the out-leg factor × the back-leg factor.

**Edge set**, built once per scene and cached:
- each wall face's waterline (face ∩ z = 0);
- each wall face's toe (face ∩ the seabed sheet, as a polyline over the sheet cells);
- the vertical box ends.

Legs whose nearest blocker is not in the edge set (objects, relief) get **no** factor, as in today's fast.
Two edges in range (the pier corner) multiply.

*As built:* the edge set is each wall box's **free** edges, those with water on both sides
(`diffraction.free_edges`):
- the vertical corners, clipped to the water column;
- the top edges of a submerged box (a sill);
- the bottom edges where a box floats above the seabed (a pontoon).

The waterline of a wall rising out of the water and its toe on the seabed are closed corners: nothing can
wrap around them.

Per leg, the **single nearest** edge (smallest zone index) is used, never a product. A thin wall's two
parallel top edges, 2 mm apart, would otherwise double-count, and a pier's two corners lie metres apart,
where only one matters per leg.

A shadowed leg takes a factor only when it crosses exactly one wall box (slab test), from that box's edges.
A lit leg takes one only when it crosses none.

F(v) comes from `scipy.special.fresnel` on the CPU and from a 65 537-point table on the GPU; the
two agree within 5e-8. [F3] checks the D1 knife edge (0.001 dB, 0.0014 rad), GPU = CPU, continuity, and
that a shadow cast by something that is not a wall box gets no factor.

Where:
- `diffraction.py`: the CPU closed form (`scipy.special.fresnel`) and the edge extraction;
- `gpu_render.py` `_diffraction`: the method switch plus `_edge_factor_k`, with a device Fresnel
  approximation and the edges looked up by a uniform grid or a BVH over segments.

### Item 4 — fp32 kernels

Knob `precision: str = "fp64"` (top level of `RaycastConfig`, next to `backend`); fast sets `"fp32"`.

- **Convert:** `_direct_k`, `_bound_k`, the pair kernels, `_edge_factor_k` and the visibility casts, each
  through its single source (§0).
- **Positions:** relative to P before the cast (≤ ~30 m → ~2 µm).
- **Stays fp64:** the bin accumulation (`x.field`, `x.mean`, `x.coh`, reductions) and the delay-to-bin
  mapping.
- **Visibility:**
  - a fp32 instantiation of the watertight test, or `wp.Mesh` + `wp.mesh_query_ray` with the same
    `surface_offset_m`;
  - always `fuse_fp: False` (the 2026-11-04 leak lesson);
  - `bvh_query_next` with an explicit `max_dist`.
- **Keep only what pays:** a quick timing of the fast profile on the guard's quay and open-water poses (a
  few seconds) keeps only the kernels that got faster. A converted kernel that is not faster is reverted to
  fp64, and the reason is noted in the status log.

*As built (owner, 2026-11-10: visibility only):* `precision: fp32` switches the per-triangle visibility test
at its single choke point, `seg_blocked` → `seg_blocked32`. That is the float32 watertight loop the default
diffraction already used, now shared with `seg_occluded32`. The flag lives on the `Oracle` struct.

The echo-path geometry (chain building, Newton solves) stays fp64. Converting it means making ~1500 lines
of struct-typed device code generic, which the default and live profiles also run.

Measured on the fast profile: `direct` −35 %, the whole quay or pier ping-side only −2 to −5 %. It is kept,
since it is faster. [F4]: 0 disagreements on 1e5 random segments, 0 leaks into a closed box, |ΔL| p99 0.000
dB. **The remaining cost at a quay is the fp64 chain geometry in the pair and bound kernels.** That is the
next candidate if more speed is wanted.

## 3. Step 2 — consistency, optimisation, code tests

**While working:** after each item, run only that item's new `[F]` checks and `guard.py --check`. Never the
full suites after a small edit.

**New section `[F] fast-variant engine paths` in `test/raycast_test.py`**, small synthetic scenes only:

| item | check |
|---|---|
| 1 | culling rules (a)–(c) on a synthetic quay + pontoon: the expected point counts, and nothing culled with the knob off |
| 2a | `kept` and the rendered ping bit-identical with `batched_bound` on vs off |
| 2b | the cache reuses within the limits and reselects on ping count, on move or yaw, and when a boundary enters or leaves range |
| 3 | the closed form vs the sampled disc on the D1 knife edge: \|F\| ≤ 0.5 dB, phase ≤ 0.1 rad, v ∈ [−2, 3]; GPU edge kernel = CPU closed form |
| 4 | fp32 vs fp64 on a small scene (mean level \|ΔL\| p99 ≤ 0.05 dB, realised \|ΔR\| median ≤ 0.05 dB); the existing watertight / intersection checks run in fp32; zero leaks through a closed box |

**Smoke test:** the existing variant check picks up the new fast deltas by itself. Add one check that
the default and live profiles resolve every new knob to its default. Register the knobs in
`config/parameter_registry.yaml` (`class: model_structure`, `status: structural`, `tunable: false`, the
code defaults), which smoke `[16]` verifies.

**Consistency pass before the final run:**
- delete dead branches and temporary prints;
- no duplicated kernel bodies (§0);
- the knob docstrings match `docs/configuration_guide.md`;
- regenerate the fast profile with `write_variant_profiles("config")`, with a description that states
  what it contains (no look claims the session has not measured);
- `colcon build --packages-select blueboat_sss_sim`.

**Final run, once each:**
- `guard.py --check` (0 on every ping);
- `python3 -m test.raycast_test` (full suite; it must pass);
- `python3 -m test.smoke_test` (the Stop hook runs it anyway, about 4 min).

If one fails, fix it and re-run only that suite.

**Docs:**
- `.claude/raycast-live.md`: the fast variant's contents;
- `docs/configuration_guide.md` §2c: the knobs and the fast row;
- `docs/engineering/raycast_design.md`: a short "fast-variant engine paths" section;
- `.claude/TODO.md`;
- the superproject skill `module-gates`: the smoke count;
- this file's status log.

## 4. Step 3 — hand-over to the owner

Report the offline per-ping timings of fast before and after, on the guard's quay, pier and open-water
poses. They are the only speed numbers the session measures. Then ask the owner to run, from MCS:
- the harbour world (`Shiraishijima-wl3p00`) with **Ray-cast — fast**, then the same stretch with
  **Ray-cast** (full), both at Slow-down x0.5;
- comparing wall time and the waterfalls.

What to look at, one item each:
- **item 2b:** the moment a quay enters or leaves the beam. A reused selection can bring a wall echo in
  up to 9 pings (~0.3 m) late;
- **item 3:** the shadow edges behind quay tops and toes, compared with full;
- **item 4:** faint ghosts behind walls (sound leaking through geometry) and grainier speckle;
- **item 1:** nothing visible expected; at most a few bins differ near quays.

If something looks wrong, name the item. Turning its knob off is a one-line change in `RAYCAST_VARIANTS`
plus a regeneration, and the other items stay.

## 4b. Phase 2 — real time and faster (owner order 2026-11-10, same day)

The owner got at most about x0.4 with the phase-1 fast and asked for 2–3× more, ideally faster than real
time. Plan: `~/.claude/plans/find-the-lastest-session-prancy-torvalds.md`.

**Measured before phase 2** (offline, idle machine, both sides per cycle, `Shiraishijima-wl3p00`):
- open water 0.097 s, quay 0.30 s, pier corner 0.62 s;
- 38 % / 27 % / 22 % of the emitted points lie outside the direct beam band;
- with 10 mm scatterers 0.032 / 0.09 / 0.17 s;
- a −30 dB global floor gave 0.057 s open water, but it was not chosen.

A profile at 10 mm then showed open-water ping-sides were **overhead-bound**, per render:
- `floor_coherent` ~13 ms;
- a Python loop over every quay triangle in `_wall_plan` ~5 ms;
- 6 timing syncs;
- 24 array copies, 36 launches and 127 allocations.

Owner decisions: fast at **10 mm**; **no** far-range floor change; runs **above x1**, with **MCS's slider
raised** (an add-only owner order).

**As built:**
- **Fast deltas** (`RAYCAST_VARIANTS`):
  - `sampling.scatterer_spacing_m: 0.010` (look-changing; the `SamplingConfig` docstring records the
    exception to "never relaxed for speed");
  - new engine knob `sampling.band_cull: true`. The sheet's count and emit kernels keep only points
    with `R ≤ r_max` and `|(X − P)·e_a| ≤ w(R)`, through a shared `sheet_keep` and a `cull` bit mask
    (1 unreachable, 2 band). The CPU reference does the same.
  - `band_cull` is exact when a ping keeps no multipath sequence (an out-of-band point is never a direct
    candidate). It is near-exact otherwise: such a point can still be lit through a kept bounce.
- **Exact, every profile (no value changes; guard 1.2e-15):**
  - `GpuScene._wall_plan` vectorised (column arrays from `_wall_lattices`);
  - `_rx` uploads the envelope table once per renderer instead of every call.
- **Exact switch, off in the live node:** `RaycastRenderer.stage_timing` / `GpuMultipath.timing` skip the
  per-stage device syncs; `last_timing` then holds no GPU stages.
- **Fast only, under `paths.reselect_every`:** `floor_coherent` reuses which image pairs were kept (cache
  tag `"floor"` in `_cached_selection`). A reused ping evaluates only those, exactly.
- **Dropped:** the planned `specular.device_select` knob. The floor channel's cost was launches, copies and
  allocations, and the pair reuse removes most of them.
- **Run loop (no image change, every ray-cast run):**
  - the hold pauses at `budget − max(hold_margin_s = 5 ms, 0.1 × budget)` instead of 0.6 × budget
    (`RenderHold.arm_delay_s`);
  - `real_time_factor` is allowed up to **4.0** (`full_mission_launch.RTF_MAX`, the node clamp);
  - MCS: `SimSpeedConfig.rtf_max` 4.0; `slowdown_hint` says "faster than real time" above x1; the
    hold hint reads "x2–x4 for fast"; the station clock's ratio cap is 5.0; the SIM badge shows the
    requested x2–x4.
  - Above x1 the unmodified control stack must keep up: PID is light, the MPC is not measured.
- **Tests:**
  - `[F5]`: band cull exact on a wall-free scene, 46 % fewer points, GPU = CPU;
  - `[F6]`: floor-pair reuse identical;
  - `[F7]`: timing off identical;
  - smoke: the hold pause point, RTF up to 4.0 (4.5 refused), `band_cull` in `ENGINE_KNOBS`;
  - the registry entry;
  - MCS smoke: x4 slider, the hint above x1, the badge at x2.

## 5. Status log

| date | session | item | result | numbers / evidence |
|---|---|---|---|---|
| 2026-11-10 | plan written | — | planned | evidence `~/ros2_ws/sim_runs/raycast/live_opt/` |
| 2026-11-10 | plan restructured (owner) | — | planned | one session, three steps; look gates moved to the owner; diffraction included; no pool variant |
| 2026-11-10 | implementation | 1, 2b, 3, 4 (2a dropped) | done; guard PASS after every item (≤ 1.7e-15 relative); `[F1]`–`[F4]` PASS | today's fast → new fast, per ping-side, harbour `Shiraishijima-wl3p00`: pier corner port 2.04 → 0.29 s (0.17 s on a reused-selection ping); quay port 0.91 → 0.43 / 0.23 s; open water 0.08 → 0.07 s (`fast_opt/timing_before.json`, `timing_after_fp32.json`, `timing_after_fp64.json`); the hybrid v2 correction's `check_core` still passes |
| 2026-11-10 | phase 2 (owner order) | 10 mm, band cull, overhead cuts, x4 run loop | done; guard PASS (1.2e-15); `[F1]`–`[F7]` PASS; `raycast_test` 174 PASS / 0 FAIL / 8 SKIP (world content, 3925 s); smoke 500 PASS | per cycle (both sides, last of 3 reps, `Shiraishijima-wl3p00`, node setting `stage_timing` off): open water x = 32.8 0.097 → **0.019 s**; quay x = 55 0.30 → **0.073 s**; pier corner x = 51.75 0.62 → **0.154 s** (with stage timing on: 0.029 / 0.078 / 0.162 s). At x1 the budget is 0.050 s: open water runs up to ~x2.5, quays stay held. Live speed and look: the owner |
