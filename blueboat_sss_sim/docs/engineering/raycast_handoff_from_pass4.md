# Hand-off to the ray-cast sensor session: architectural failures found in pass 4

Pass 4 (2026-11-02, field calibration on the pool corpus) traced each symptom to a stage with a
measurement. This note lists the ones whose cause is the renderer's **architecture** (visibility,
multipath, diffraction, transport), not a constant. They are the cases a ray-cast renderer should be
tested against first.

All numbers are sim − real on the pool corpus, FIT (calibration) / VAL (validation) runs. The private
record (scores, tools, logs) is `~/calib/pass4/`. The public summary is `calibration_results.md`
Part III. The model terms named here are in `sonar_model.md` §14.

## 1. Multipath is first order, and images have no visibility of their own

The renderer's mirror sources are one image per wall plus an optional z = 0 image. Pass 4 added the
shallow-water image comb as three terms, each off by default:
- `waveguide_comb_enabled`: the wall echo;
- `seabed_comb_enabled`: the seabed;
- `waveguide_facet_comb`: smooth objects.

It reproduces the pool. With the pool's environment overlay:
- the wall "beyond 0.5–2 m" bias goes from −33.9 / −30.5 dB to +1.9 / +2.5;
- the seabed level-vs-range shape error goes from 15 dB to < 1 dB per band (after the floor's level);
- the boxed weight's echo deficit goes from −11.7 / −12.5 dB to −2.3 / −0.8;
- its echo extent goes from 0 to 0.50 / 0.85 m (real 0.99 / 0.82).

What it cannot do, and a ray caster should:
- **Image paths share the direct path's visibility.**
  - The seabed comb is moved per ground sample with the direct shadow mask.
  - The wall comb and the facet comb are not occluded by anything.
  - Real pool rows with the box on the boat → wall line are darker beyond the face (VAL median −2.85 dB,
    18 % of passes under −6 dB). This is plausibly the box shadowing part of the wall's comb, which the
    comb cannot reproduce.
  - Test case: an opaque object between the boat and a quay in 0.9 m of calm water. Measure the comb
    tail behind the face with and without the object.
- **Flat-floor geometry about nadir** (seabed comb): local depth = the altitude at nadir plus the
  transducer depth. Test case: a sloping floor (the sand altitude sweep of the roadmap).
- **Paths add incoherently.** Within one bin, relative phases of the 2–4 near-coincident Lloyd paths
  (direct / surface) are not modelled. The real plastic-box screenshot shows interference rings.
  Test case: a small plate at 4–10 m in calm water; the fringe spacing follows 2 z_t z_o / r.
- **The FBR is not imaged.** The seabed comb is diffuse only.
  - The real pool's first bottom return ramps from 0.6 to ~0.9 m slant: the direct return, then the
    direct/surface mixed path at (r_D + r_S)/2 ≈ 0.9 m, then the surface-surface path at 1.12 m.
  - The simulator jumps to its crest in 2–4 bins (leading edge +9 dB, pass 3; crest still +1.5 dB in
    pass 4).
  - Test case: nadir at 0.6–0.7 m altitude in 0.9 m of calm water.
- **Cost.** The facet comb is bistatic Kirchhoff per image pair, ~360 ms per ping-side with the pool
  box in view (the live budget is 25 ms). The wall comb adds ~10 ms; the seabed comb ~50 ms in the
  pool.

## 2. Occlusion is per azimuth line in its own vertical plane

- **B1, object shadows beyond a quay face.** They are painted by the wall ghost's first leg (since F2,
  2026-10-21, `issue_triage.md`). Every ghost ray to the seabed beyond the face passes the object's
  column, and the whole ghost is multiplied by its transmission (the flooded shell's 0.11 = −9.6 dB,
  filled to −3…−5 dB by the reverberation floor).
  - A 3 mm quay's ghost is ~100 % the lumped incoherent share (C ≈ e⁻¹²⁸), which a single mirror ray
    cannot represent.
  - The split implemented in pass 4 (`ghost_share_split`): coherent share ray-occluded, incoherent share
    weighted by the face's lit fraction. It moved the simulator away from the data (−1.94 → −0.79 dB
    vs real −2.85) and ships off.
  - Raster (rough) objects cannot be removed from a first leg at all: there is no object-free terrain
    raster in `SceneModel`.
  - A ray caster with real 3-D visibility of diffuse wall scattering is the fix.
- **Facet shadow boundary.** A hard boolean against the centre-line horizon, read 0.1 m in front of
  the object (`facets.py`). No Fresnel-zone softening: at 10 m the first zone is √(λR) ≈ 0.18 m,
  comparable to a 0.3 m object.

## 3. Diffraction and shadows

- After the comb and the seabed comb, the box's shadow-found fraction is 6 % / 7 % (real 6 % / 2 %;
  start 25 % / 19 %). The primary shadow-zone profile (mean over 0–1 m after the echo end) is still
  3.4 / 4.6 dB darker than real. Tested candidates and evidence:
  `calibration_results.md` §III and `~/calib/pass4/step8/`.
- `penumbra_deg` exists (off). A physically scaled penumbra (∝ √(λR)) would need per-object geometry
  the 2.5-D raster lacks.

## 4. Texture and scene content (not constants)

`~/calib/pass4/step1/DB3/REPORT.md`; raw dB.
- **No large-scale component in the simulator.** The 1 m block variance is 0.001–0.002 of the per-bin
  variance, against pool 0.006–0.013 and beach 0.004. The per-ping level std is 0.57 dB against
  0.9–1.5 dB (pool) and 0.9–1.0 dB (harbour, beach). This needs scene content (sediment patches) and a
  per-ping level mechanism (surface interference, source level).
- **The range kernel is a flat 5-bin boxcar.** Lag-2/3 correlation is 0.26 / 0.10 against 0.10–0.17 /
  0.03–0.10 real. The device's response is the pulse convolved with the receive filter (and the
  transducer bandwidth), a bell. A ray caster with a matched-filter receiver gets this for free.

## 5. Device behaviour the renderer does not model

- **The 10 m vs 20 m level step.** −2.1 dB in our pool recordings (our driver) and none at a fixed
  pulse in the SonarView field recordings (−0.01 dB median over 29 within-file switches). A
  zero-parameter receive-window term (`receive_window_level`) predicts the pool and is refuted by the
  field. The test needs both recorders in one session.
- **The nadir ringing** rises at 10 m (+5 dB) where every level term falls.
