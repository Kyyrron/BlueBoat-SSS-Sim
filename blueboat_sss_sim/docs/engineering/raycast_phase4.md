# Ray-cast sensor — Phase 4 report (IN PROGRESS): the GPU path

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
