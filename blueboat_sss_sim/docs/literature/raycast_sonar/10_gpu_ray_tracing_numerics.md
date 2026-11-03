# 10 — Ray-tracing infrastructure: intersection, acceleration, GPU, random numbers

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| WBW13 | Woop, Benthin, Wald, "Watertight ray/triangle intersection", *J. Computer Graphics Techniques* 2(1):65–82, 2013, https://jcgt.org/published/0002/01/05/paper.pdf | — (JCGT issues no DOI) | record (journal page, PDF link, search 2026-11-02); algorithm implemented and tested |
| MT97 | Möller & Trumbore, "Fast, minimum storage ray-triangle intersection", *J. Graphics Tools* 2(1):21–28, 1997 | 10.1080/10867651.1997.10487468 | record; algorithm standard |
| Wal14 | Wald et al., "Embree: a kernel framework for efficient CPU ray tracing", *ACM TOG* 33(4):143, 2014 | 10.1145/2601097.2601199 | record only |
| Par10 | Parker et al., "OptiX: a general purpose ray tracing engine", *ACM TOG* 29(4):66, 2010 | 10.1145/1778765.1778803 | record only |
| Sal11 | Salmon, Moraes, Dror, Shaw, "Parallel random numbers: as easy as 1, 2, 3" (Philox/Threefry), SC'11 | 10.1145/2063384.2063405 | record only |
| Warp | NVIDIA Warp, Python framework for GPU simulation (`wp.Mesh` BVH, `mesh_query_ray`), https://github.com/NVIDIA/warp | — (software, no DOI) | docs to read at Phase 4 |
| Jar14 | Jarabo et al. 2014 (note 01): time-resolved path integral, time-binned estimators | 10.1145/2661229.2661251 | record |

## What they establish

* **Ray–triangle test.** MT97 is a barycentric solve without a stored plane. It is **not watertight**:
  a ray through an edge shared by two triangles can be rejected by both through round-off. Measured in
  Phase 1: 1 of 7572 axis-aligned probe rays leaked through a cylinder. WBW13 shears the triangle
  into ray space and tests three 2-D edge functions, which neighbours evaluate to exact negations, so
  no ray crossing a closed mesh can leak. **The oracle uses WBW13**; MT97 is kept as a cross-check
  away from edges, where the two agree to 5e-15 (Phase 1).
* **BVH acceleration** (Wal14, Par10): CPU (Embree) and GPU (OptiX, and Warp's `wp.Mesh`, which builds a
  BVH on the device and exposes closest-hit and any-hit queries from Python kernels compiled through
  NVRTC, so no `nvcc` is needed). Expected throughput is 10⁸–10⁹ rays/s on a laptop RTX GPU; Phase 4
  measures it.
* **Counter-based RNG** (Sal11): random numbers are a pure function of (key, counter), so the same
  scatterer gets the same draw on CPU and GPU, in any thread order. Numpy exposes Philox, and the
  project already uses splitmix64 hashing (`renderer._scatter_phase`). Either way, determinism does not
  depend on how threads are scheduled.
* **Transient estimators** (Jar14): accumulate each path's contribution into time bins with a
  reconstruction kernel. Here that kernel is the matched-filter response `s_mf`, never a box.

## Numerical points for the design

* **Phase precision.** `2kr` at 20 m is ≈ 75 000 rad. In float32 the relative precision is 6e-8, so
  the phase error is ≈ 0.005 rad; tolerable alone. Accumulating ~10⁵ phasors per bin in float32 loses
  more, so phases are computed relative to the bin's own reference range and sums are accumulated in
  float64 (or Kahan) on the GPU. The RTX 4060's FP64 rate is low, but accumulation is a small share of
  the work.
* **Atomics.** Float atomics make GPU sums order-dependent at round-off. So "GPU = CPU" is stated as a
  tolerance (`raycast_design.md` §10), and bitwise GPU determinism is documented rather than claimed.
* **Mesh robustness.** Watertight meshes with shared vertices avoid rays leaking through cracks.
  Tessellating patches (`solids.tessellate`) are not watertight; meshes are built per primitive.

## How it maps to the renderer

`bvh_cpu.py` (numpy BVH + MT97, the oracle), `gpu_<backend>.py`; scatterer hashing in `scatterers.py`.
Backends are chosen by measurement in Phase 4.
