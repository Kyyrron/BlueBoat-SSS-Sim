# Realism roadmap

Ordered by (thesis value ÷ effort). Each item plugs into an existing seam
— none requires touching the ROS interface. (The 2026-09 world-builder
rework is done and tracked in `.claude/TODO.md`, not here.)

## 1. Sound-speed profile & ray bending (low effort)
Replace straight-ray `slant = hypot(y, dz)` with a two-layer SVP ray step
in `GeometricRenderer._ping_geometry`. Mostly relevant beyond ~30 m range
or strong thermoclines; low priority for the 20 m shallow regime (the project's
survey range, every condition).

## 2. Wall / surface multipath — **shipped**, re-anchored 2026-09-03
Implemented in `sonar/multipath.py` and `renderer._face_echo_into`, on by
default; world config carries `walls:` with `kind: quay | pontoon`,
`draft_m`, `roughness_m`. See `sonar_model.md` §10–11 and A9/A11. Still
out of scope: composed bounces beyond the first-order mirror set, a rubble
slope at a quay's foot, diffraction over the top, and a corpus-measured
pontoon (no pontoon geometry ground truth exists in the field logs).

## 2b. Sea state — **shipped 2026-09-03**, open items
`sea/` + `sea_state_node` (`docs/sea_state_model.md`). Next, in value
order: wind load on the hull and superstructure (the current model has
no wind at all); spatially varying current beyond the weak eddies shipped
2026-10-08 (`sea/current.py` `EddyField`) — a basin mouth jet;
wave diffraction/reflection off the quays (the wave field is open-water
JONSWAP even inside a basin); a hydrodynamic identification of the
BlueBoat itself (the damping coefficients Gazebo applies are BlueROV2's,
named as such in `hydrodynamics.xacro` — a request to BlueBoat-Control,
which is updated by additions only). The live Gazebo acceptance pass is done (2026-10-08/09,
`test/gazebo_sea_test.py`, `docs/sea_state_validation.md`).

## 3. Mesh-accurate targets (medium)
For cavity-bearing targets (wrecks, pipes on trestles) the 2.5-D stamp is
insufficient. Add a per-object triangle-mesh path in the renderer: ray/
mesh intersection along the ground line only where an object's footprint
is hit; the heightfield remains the fast path everywhere else.

## 4. Intra-ping motion & yaw smear (medium)
Sample the pose at per-bin receive times instead of once per ping;
convolve along-track with the horizontal beam footprint. Makes turns
smear realistically — relevant for imagery taken while the vehicle
maneuvers (turns, and the revisit passes of an adaptive replanner, which is
planned, not built).

## 5. Bottom-type speckle spectra (low)
Correlated speckle shipped 2026-09-02: `Exp(1)` comes from the coherent
ground-scatterer field, correlated along range and track as the corpus
measures (`coherent_scatterers: true`, NC #5, smoke `[2g]`). Still open:
per-material K-distribution parameters. Sharpens
texture-classification realism; detector-level impact is modest.

## 6. Vegetation dynamics (low)
Seagrass sway as a per-ping phase jitter on the seagrass-material texture;
addresses A4 partially without giving up the static-scene architecture.

## 7. GPU / Gazebo-sensor backend (only if needed)
If world sizes or ping rates ever outgrow the CPU renderer, reimplement
`SonarRenderer` on a GPU ray caster (or a Gazebo sensor plugin feeding
ranges + material IDs) behind the same ABC. The interface, noise stack
and encoder layers are already backend-agnostic.

## 8. Sim-to-real calibration over a known bottom
Two passes are done (`docs/sonar_model.md` §6): the encoder carries the
device's per-ping normalisation, and since 2026-09-18 the range law, level
and coherent spreading are fitted on **two** altitudes together (`bath3`,
1.40 m sand, and `reflectionEvidenceFullharbour`, 7.33 m), which is what
measured `lambert_exponent` (1.0) and `tvg_compensation` (0.30) — over
**sand only**. What is still missing is a bottom-*class* lever: the harbour
bottom type was never sampled, the angular law is one global exponent, and
two real passes of the same water disagree by 1.8–3.1 dB RMS against a
0.67 dB reduction floor. Closing that needs captures over **independently
classified, flat** bottoms at three or more altitudes with the acquisition
written down. The lifecycle tooling for it — session manifests, quality
reports, identifiability, versioned calibration artifacts, the effective
profile — exists since 2026-10-21 (`blueboat_sss_sim/calibration/`,
`docs/engineering/`); the fit itself waits for that data.
