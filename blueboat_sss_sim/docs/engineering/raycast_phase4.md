# Ray-cast sensor — Phase 4 report: the GPU path

Started 2026-11-04, pulled forward by the owner ("make the computation GPU compatible") because the CPU
reference cannot score the pool validation set inside the pre-registered 24 h compute line. Gate:
`raycast_plan.md` §Phase 4 (G1 on every validation scene, the leak probe on the GPU intersection, the
benchmark table). Patterns credited to DAVE (Choi et al. 2021; `raycast_dave_comparison.md` §7).

## Stack (installed in `~/ros2_ws/sim_runs/raycast/venv` only)

```
python3 -m venv ~/ros2_ws/sim_runs/raycast/venv
~/ros2_ws/sim_runs/raycast/venv/bin/pip install numpy pyyaml warp-lang scipy
```

Versions: Warp 1.19.0 (CUDA 13.4 through NVRTC, no nvcc), numpy 2.5.4, scipy 1.18.1. RTX 4060 Laptop,
8 GiB, driver 595.99. **Never run CUDA under `ulimit -v`**: unified addressing reserves large virtual
ranges, and allocations fail.

## What runs on the GPU (`sonar/raycast/gpu_warp.py`, `backend: warp`)

| Part | Implementation | Check |
|---|---|---|
| visibility oracle | `WarpBVH`: a `wp.Bvh` over the triangles' padded float32 boxes (local origin, `BOX_PAD_M` 1e-5) for candidate traversal, every candidate tested by the CPU's watertight algorithm (Woop, Benthin & Wald 2013) in float64, with the CPU's `eps` and lowest-id tie rule; the module compiled without FMA contraction (below); a drop-in for `TriangleBVH.closest`/`occluded` | G1 leak probe; G1 renders |
| near-field aperture | `near_pattern`: the Gauss–Legendre aperture integral per direction, stable `r − R` | vs the CPU integral on 2·10⁵ directions: 3e-4 relative where \|b\| > 1e-3 (low sidelobes, float32 cancellation), 4e-6 absolute |

Everything else (scatterer lattice, chains, laws, deposits) is still numpy; see the profile below.

## Benchmark (pool world, 1.24 M triangles; `notes/s4/warp_bench.log`)

| Query | GPU, Warp `mesh_query_ray` (leaky, not used) | GPU, `wp.Bvh` + float64 watertight (used) | CPU reference |
|---|---|---|---|
| BVH build | 0.10 s | 0.1 s | ~5–6 s (mesh + BVH) |
| closest hit | 115 M rays/s | 7.0 M rays/s (incl. transfers) | 8–43 k rays/s |
| segment occlusion | 434 M rays/s | 5.7 M rays/s (incl. transfers) | ~same as closest |

The watertight float64 test costs ~20–60× Warp's float32 query. It is still 160–700× the CPU
reference, and exact.

## Profile after the GPU oracle (flat sand, 20 m / 1200, 5 mm lattice)

With only the oracle on the GPU, a ping-side took 17 s, of which 15 s was the near-field aperture
integral (1.5 M directions) plus 1.5 s of re-computed Gauss–Legendre nodes. Both are fixed: the
integral runs on the GPU and the nodes are memoised. The numbers after the change: below.

## Watertightness on the GPU: three failures, three causes

1. **Warp's `mesh_query_ray`** (first version) leaked 24 of 22 049 vertex/edge probes. Its
   watertightness is not documented, and the probe shows it is not watertight. It is replaced by
   a `wp.Bvh` for traversal plus the CPU's own watertight test in float64.
2. **The float64 port then leaked 375 of 22 049, all at edge midpoints.**
   - Isolated case: an axis-aligned ray through a box's face diagonal. The CPU accepts both
     triangles; the GPU rejected both.
   - The GPU's edge function read **+1.7e-18 in both triangles**. Exact negatives are what make
     the test watertight. The CPU reads exactly 0.
   - Cause: NVRTC contracts `a·b − c·d` into `fma(a, b, −c·d)`. That is the exact residual of one
     product, which is not antisymmetric under swapping the edge's ends.
   - Fix: `wp.set_module_options({"fuse_fp": False})` for `gpu_warp`. Every operation is then the
     IEEE one numpy performs (CUDA double division is correctly rounded), so hit decisions and
     tie-breaks equal the reference's. The failing case now gives the CPU's t and triangle id.
   - Tried first and rejected: a canonical vertex order with one expression per branch. Each code
     site fused its own way, and the two triangles still read one sign. Single-site selects closed
     the leak but left the GPU taking one triangle where the CPU takes both: watertight, but not
     equal to the reference. Unfused arithmetic is both, with no extra code.
   - Lesson for any GPU port of a watertight test: the algorithm's guarantee assumes unfused
     arithmetic. It must be compiled that way, or its edge functions made exactly antisymmetric.
3. **56 of 22 049 leaks remained after the FMA fix**, all on the cone's base rim with untilted
   probes.
   - Those rays run in the base plane. The probe puts their origin 2.45e-20 m off the plane
     z = 0, which float64 represents only near zero.
   - The GPU ran its float64 test on coordinates shifted to the mesh's centre (0.25 m away). There
     the offset is below one ulp, so the origin lands exactly on the base, the ray exits at t = 0,
     and `t > t_min` rejects the hit.
   - The CPU's own function gives the same −1.3e-19 in that frame. So the algorithm is fine and the
     frame is the cause.
   - Fix: the float64 test runs on the **global** coordinates (the reference's inputs). The shifted
     frame is kept only for the float32 traversal boxes. Leak probe: **0 of 22 049**.

## Scatterer density: is 5 mm more than needed? (§6, flat sand, GPU)

The owner asked whether less data would give the same result. This is `tools/density_sweep.py`
(`notes/s4/density_sweep.json`), with the §6 criteria stated before the work:
- the mean against an independent midpoint quadrature (3 mm), ε_μ p95 ≤ 0.02;
- texture KS against Exp(1) below its 5 % critical value;
- range and track lag-1 equal to the finest spacing's.

| spacing | ε_μ p95, 1.4 m alt | ε_μ p95, 6 m alt | KS / crit, 1.4 m | KS / crit, 6 m | GPU s per ping-side (1.4 / 6 m) |
|---|---|---|---|---|---|
| 2.5 mm | 0.0024 | 0.0016 | 0.011 / 0.025 | 0.032 / 0.033 | 17.0 / 14.4 |
| **5 mm (default)** | **0.0082** | **0.0055** | 0.014 / 0.025 | 0.010 / 0.033 | **2.5 / 3.3** |
| 7 mm | 0.0177 | 0.0121 | 0.017 / 0.025 | 0.026 / 0.033 | 1.5 / 1.7 |
| 10 mm | 0.0346 ✗ | 0.0234 ✗ | 0.035 ✗ / 0.025 | 0.028 / 0.033 | 1.2 / 0.9 |
| 20 mm | 0.141 ✗ | 0.092 ✗ | 0.012 / 0.025 | 0.012 / 0.033 | 0.6 / 0.2 |

- Range lag-1 is 0.49–0.51 at every spacing; the texture correlation is density-independent, as
  Brown17's Gaussian factor predicts.
- Track lag-1 is −0.07 to −0.12 at every spacing. That is the −1/11 bias of demeaning over 12 pings
  (true value ≈ 0, footprints 0.25 m apart).
- **Verdict: keep 5 mm.** 7 mm passes with no margin at 1.4 m (0.0177 against 0.02) and saves about
  1 s per ping-side; 10 mm fails the mean at both altitudes. The mean error scales as h², as a
  midpoint rule should. Since the GPU port, 5 mm costs 2.5–3.3 s per flat ping-side.

## Pool ping-side: from 968 s to ~133 s (profiled, GPU backend)

`tools/pool_profile.py` profiles the pool's own world (`pool_1w_1`, 1.24 M triangles), mid-pool,
looking at the long wall. There are 6.3 M contributors, 6.06 M of them seabed, and 37 % of those lie
in the direct beam above the pattern floor. Every change below was verified before it was kept:

| Change | Verified by | Effect |
|---|---|---|
| Multipath pairs folded by reciprocity: (i, j) + (j, i) → 2 (i, j) | `sigma_bistatic` reciprocal to 1.2e-15 over 2·10⁵ geometries; gates P2, P4, O4, O5 pass | K(K+1)/2 pairs instead of K² |
| Chain selection bounds on the flat-floor image solution (`chains.build(refine=False)`); the Newton refinement only for kept sequences | `tools/selection_check.py`: the same kept chains, ε_μ = 0.0 against the old refine-everything selection on the pool ping-side | `select_chains` from ~780 s to ~60 s |
| Diffraction queries stacked: one bisection pass for all transitions, the 16 direction rows in one call, `classify`'s 41 samples in one call, `_chord_H` vectorised over its angular samples (chunked) | `tools/diffraction_batch_check.py`: 6,000 legs over a quay and a sill, class and factor **bit-identical** to the per-call originals | oracle calls 47 k → 13 k; diffraction ~142 s → ~64 s |

What remains (cumulative, under the profiler):
- diffraction 64 s, of which ~28 s is GPU call latency (16 k calls from the sector recursion);
- chain selection 59 s (planar unfold, factors and validity over 6.3 M contributors × 31 sequences);
- facets 11 s.

No single hotspot is left. The next factor needs the chain stages themselves on the GPU.

**Compute projection for the pool validation set (gates: stop and ask over 24 h).** p4_score's
pre-registered sampling asks for ~15,200 distinct poses: stride-3 pings for the wall tail ≈ 14,500,
plus 702 box-pass poses. That is ~420 h at ~100 s per ping-side, and still ~42 h at 10 s. The box
statistics alone (702 poses, ~20 h) fit. Asked to the owner on 2026-11-04; the answer is pending.


## The whole pipeline on the GPU (owner, 2026-11-04: "convert everything to GPU computation")

`sonar/raycast/gpu_render.py` runs every per-contributor stage on the device for `backend: warp`. The
CPU modules stay the reference, and `RaycastRenderer.use_gpu_pipeline = False` keeps the CPU stages on
the GPU oracle, which is what the G-gates compare. One fused kernel per stage, contributors as the
thread axis, nothing per-chain stored per contributor:

| Stage | Kernel(s) | Held to the reference by |
|---|---|---|
| scatterers | splitmix64 lattice, bit for bit; sheet (steep cells refined) and walls (cells cut to the r_max box and the beam band) in one allocation | `tools/gpu_stage_check.py scatterers`: **bit-identical** on flat sand (1.83 M), the pool (6.3 M before rule C) and the harbour (23.5 M before, 8.5 M after) |
| direct chain | pattern (near-field integral, node count per point), law, visibility (BVH + float64 watertight, in-kernel), thin-layer transmission | full renders below |
| diffraction | hit fractions, the lit-side cell table (sort; the smallest fraction per cell), `classify` (early exit), `factor` (16 direction rows, the sector refinement on an explicit depth-first stack) | pool render ε_μ 1.2e-7 with float64 samples; float32 samples vs float64 ε_μ 3.0e-8 |
| selection | one bound kernel per sequence; lazy enumeration; a scalar short path for chains of <= 2 bounces (<= 1 floor) | the kept set equal on every scene |
| pairs | every contributor through its chains (light / heavy kernel by kept count; an error, never a truncation, above 32 chains), reciprocity fold, the per-scatterer coherent mean summed exactly | flat ε_μ 1.3e-6, pool 1.2e-7 |
| facets | PO pairs of the smooth solids' patches | `gpu_stage_check.py facets`: field within 2.4e-6 of its peak |
| Newton (floor bounces) | the general solver (LU, partial pivoting) and an analytic 2x2 for one floor bounce | through the renders |
| floor coherent channel | the window lattice (zone index per vertex), cell dilation and the n × n patch tiling; per patch the out and back chains, `amplitudes_dirs` (sinc area, reflection at the facet's own incidence, the J&R coherent fraction, the Fresnel window), the two patterns; the energy bound on the host by the same rule; then visibility, thin-shell transmission and the coherent deposit | `gpu_stage_check.py floor`: flat sand and pool, deposits equal (4741, 4527), field within 1.2e-6 / 2.8e-6 of its peak; `tools/floor_calm_check.py`: calm 0.9 m, field 2.0e-5 of peak, power ε 3.0e-5, and per pair up to order 4 the same patches and live sets, amplitudes within 1.4e-5 of their maximum |

The floor channel's order loop never passes order 0 in these geometries. Any out or back chain
that leaves the transducer toward the surface does so near nadir, behind the baffle, so its pattern
is exactly 0, every order-1 pair is empty, and the loop stops. The floor-first back chains
(F S, F S F S, …) carry coherent floor factors of 1e-9 and below at 450 kHz. Their device code
is therefore checked pair by pair (`floor_calm_check.py`), not through a render. The stage takes
8–21 ms against the CPU's 38–80 ms, and a flat ping-side went from 0.08 s to 0.06 s.

A fix to the CPU reference in the same stage: the coherent floor deposit computed each leg's
thin-shell transmission (`check_visibility` sets `T`) but never applied it, so a leg through a
flooded shell deposited unattenuated. The deposit now carries `T`, as the diffuse and facet paths
always did. Without flooded shells (the fit's flat scenes) nothing changes.

The stage checks' field tolerance is G1's: 1e-4 of the peak, since G1 allows 1e-3 of P in
intensity. The checks were first written with 1e-5. The calm scene's 2.0e-5 comes from the float32
near-field pattern at ~1 m range; its power ε, 3.0e-5, meets G1's 1e-4.

**Precision policy** (the RTX 4060 runs float64 at 1/64 of float32):
- float64 for geometry that sets the carrier phase (positions, unfolded lengths, Newton points) and
  for the reference watertight test of the direct and chain legs, as G1 requires;
- float32 for the seabed law, the boundary factors, the pattern, the carrier's cos / sin after a
  float64 reduction, and diffraction's visibility samples and chord integrals.

**Changes to the CPU reference.** Each makes a result independent of how contributors are batched,
or removes work without changing the result:
- `chains.sequences_by_order`: lazy enumeration. With the other objects' faces the full list was
  combinatorial (facets 18 s → 0.9 s on the CPU). The report's `sequences` now counts the
  enumerated sequences.
- `diffract_direct`: a lit-side leg takes the smallest hit fraction among its blocked neighbours.
  This replaces the first-visited rule of a Python dict, and is vectorised.
- `Diffraction._chord_H`: the quadrature count is per leg, no longer the batch's largest span.
- The neighbour-cell key offset is 2^30, so keys stay positive: signed and unsigned sort orders
  agree.
- `planar_sample`: wall cells are limited to the r_max box. It is the same set, and a 226 m quay
  face no longer means 1.3·10⁸ cells; the CPU refused it with a MemoryError.
- **Steep-cell refinement**, a sampling change gated before adoption:
  - The old rule was ceil(J)² unstratified points. ceil(J) is 2 for any tilt, so every slightly
    sloped cell got 4 points: the harbour's 23.5 M contributors were 5.8 M cells × 4, and the
    textured pool floor 1.6 M × 4.
  - Now a stratified n_x × n_y sub-grid, with n = ceil(stretch / 1.25) per axis.
  - `tools/slope_sweep.py`, §6 mean criterion on ramps: ε_μ p95 is 0.006 / 0.010 / 0.006 / 0.004 at
    20 / 35 / 50 / 70°, all ≤ 0.02.
  - The first run's reference used analytic planes and measured the 5 cm mesh's rounded kinks,
    0.09 for both rules. Corrected to integrate the mesh surface.

**G1 on the full pipeline: one failure, one cause.** G1's quay + sill scene first failed (ε_μ
9.4e-3, intensity 0.40 of P; flat sand passed). Per-contributor pair counts
(`tools/g1_pair_diff.py`) differed on 5 contributors behind the sill and on the quay face, every one
on a leg grazing the sill's top edge. The GPU lifted a chain's bounce points off their planes by
`surface_offset_m` (1 mm), the CPU reference by `check_visibility`'s default 0.1 mm, and on those legs
the 0.9 mm decides visibility. It was a parameter inconsistency, not precision. The offset is now one named
constant, `chains.BOUNCE_OFFSET_M` (0.1 mm), used by both paths; the contributor's own end keeps
`surface_offset_m`. G1 after the fix: quay + sill ε_μ **4.1e-6**, intensity **9.3e-5**; flat sand
9.2e-7 / 4.6e-6; leak probes 0 / 22 049 (`notes/s4/g1_v6.log`). G1 was rerun after every later change
(the traversal cutoff, the floor coherent channel on the device, the reused disc centre). The numbers
were the same: quay + sill 4.1e-6 / 9.3e-5, flat 9.2e-7 / 4.6e-6, leaks 0 / 22 049 (`notes/s4/g1_v7.log`).

**The traversal stops at the segment's end.** `wp.bvh_query_ray` has no maximum distance. Every
visibility test therefore visited every leaf box along the *infinite* ray, far past the segment's end:
the 0.5 m leg from a Fresnel-disc sample to its contributor ran on through the whole floor.
`bvh_query_next(q, idx, max_dist)` now gets the segment's length × (1 + 1e-6) + 1 mm, and a
closest-hit query shrinks it as hits are found (`gpu_warp.CUT_REL`, `CUT_ABS_M`). Hit decisions
are unchanged:
- The traversal ray lies inside every padded box that holds an accepted hit, at the hit's distance.
  That is the premise of `BOX_PAD_M`, which the leak probes check.
- So the exact slab entry is at most t_hit.
- The float32 entry adds ~3 roundings (2e-7 relative), and the float32 ray differs from the float64
  test's ray by ~1e-5 m + 6e-8 t. The margin covers both, so no node holding a hit at or below
  t_max is ever skipped.

Measured: the rendered power is the same to 6.5e-16 (pool) and 8.6e-16 (harbour), and every visibility
test in the pipeline got cheaper. On the pool, classify fell 0.27 → 0.06 s and the diffraction factor
1.09 → 0.29 s.

**Tried and dropped: a wide diffraction factor.** One thread per leg integrates the whole Fresnel
disc (16 rows × 21 samples + 7 per transition, × 2 casts, then the sector refinement). Spreading it
as coarse samples per thread, bisections per thread and sectors per thread was slower:
- leg-slowest thread order: 2.83 s vs 1.08 s, because a warp's rays scattered over one disc and
  traversal coherence was lost;
- leg-fastest: 1.16 vs 1.08 s.

After the cutoff the one-thread-per-leg kernel still won (pool 0.29 vs 0.32 s, harbour 0.97 vs
1.32 s), so the wide version was removed. The lesson: a warp tracing 32 neighbouring contributors'
identical offsets is what makes BVH traversal cheap; parallelism that breaks that coherence costs
more than it brings.

**Per ping-side on the RTX 4060** (one side of one ping; a ping is both sides):

| Scene | Contributors | Session start | Now | Main remaining stages |
|---|---|---|---|---|
| flat sand, 6.2 m, 20 m / 1200 | 1.8 M | 4.2 s | **0.06 s** | pairs 0.014, scatterers 0.013, direct 0.010, select 0.009, floor coherent 0.006 |
| pool, mid-pool, walls + box | 6.3 M → 1.6 M | 968 s | **0.59 s** | diffraction factor 0.27 (49 k edge legs), select 0.14, classify 0.06 |
| harbour 07-21, 35 m / 600, quays | 23.5 M → 8.5 M | (OOM on CPU) / 15.2 s GPU | **2.36 s** | diffraction factor 0.89 (483 k edge legs), select 0.61, pairs 0.44 |

The last step is exact. Sample 0 of every direction row is the Fresnel disc's centre, so each leg tested
the same point 16 times, plus once more in classify. It is now tested once, in classify, and reused
by every row of the factor (`cen`): the diffraction factor went 0.96 → 0.89 s on the harbour. GPU =
CPU on G1's quay + sill scene at 2 cm: diffuse mean ε 1.1e-6, identical report (37 019 diffracted
legs, 3 418 edges).

The harbour's selection bounds 9 sequences over 8.5 M contributors. The bound itself costs ~3.5 ns
per contributor-sequence; the rest is the refined bound of each kept floor sequence, which the
reference's rule also computes.

**Pool validation cost** (`tools/pool_cost.py`: 72 real validation poses, seeded, in their runs'
worlds with the pool installation, one render each; `notes/s4/pool_cost.json`):

| | mean | median | p90 | max |
|---|---|---|---|---|
| before the cutoff (`pool_cost_v1.json`) | 4.59 s | 2.76 s | 4.80 s | 127 s |
| now | **0.99 s** | 0.80 s | 1.26 s | 8.1 s |

The full p4 pool validation renders ~15 200 ping-sides, about **4.2 h** of GPU time. That is under
the owner's strict 24 h bound with the full sampling, so the sampling is not reduced.

Warp lessons, each one a bug found here:
- A `wp.func` receives vectors, matrices and structs **by value**: writes to arguments are lost.
  Return the results.
- `enable_backward: False`: the adjoint codegen lacks in-place float64 matrix element writes. A
  `module="unique"` kernel needs its own options, `fuse_fp: False` included.
- A variable may not change type within a function.
- A kernel in `__main__` compiles with `__main__`'s options.
- Wall scatterers are compacted through an atomic counter, so their order changes between renders.
  Never compare per-index arrays across renders; compare the renders.
- Runtime-indexed local arrays live in local memory. Building a 1-bounce chain through 13×3 matrix
  copies cost 91 ms per 6.3 M contributors; the scalar short path is several times faster.
- `bvh_query_ray` traverses the infinite ray: give `bvh_query_next` its `max_dist`.
- Two GPU processes share 8 GB: a debug run beside the fit took the memory the fit needed, and the
  fit died on a 41 MB allocation. `RaycastRenderer.release_device()` now drops a cached renderer's
  device buffers once its bases are built (`calib.expected_level`). Every long-running ray-cast
  render now waits and retries on a transient device allocation failure, releasing its buffers first
  (`calib._build_retrying`): basis builds, the engine's noisy renders and `scene_replay.render_rows`.
- Spawn worker processes; never fork one after CUDA is initialised ("CUDA error 3: initialization
  error").
