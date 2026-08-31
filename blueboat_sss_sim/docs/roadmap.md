# Realism roadmap

Ordered by (thesis value ÷ effort). Each item plugs into an existing seam
— none requires touching the ROS interface or the dataset pipeline.

## 1. Sound-speed profile & ray bending (low effort)
Replace straight-ray `slant = hypot(y, dz)` with a two-layer SVP ray step
in `GeometricRenderer._ping_geometry`. Mostly relevant beyond ~30 m range
or strong thermoclines; low priority for the 15 m shallow regime.

## 2. Wall / surface multipath — **shipped**
Implemented in `sonar/multipath.py`, off by default
(`model.wall_multipath_enabled`); world config carries `walls:`. See
`sonar_model.md` §10 and A9. What remains out of scope, and why it is here
rather than done: the wall's **own** direct echo and shadow, which the 2.5-D
heightfield cannot represent and which needs the mesh path below; and
composed bounces beyond the first-order mirror set.

## 3. Mesh-accurate targets (medium)
For cavity-bearing targets (wrecks, pipes on trestles) the 2.5-D stamp is
insufficient. Add a per-object triangle-mesh path in the renderer: ray/
mesh intersection along the ground line only where an object's footprint
is hit; the heightfield remains the fast path everywhere else.

## 4. Intra-ping motion & yaw smear (medium)
Sample the pose at per-bin receive times instead of once per ping;
convolve along-track with the horizontal beam footprint. Makes turns
smear realistically — relevant for adaptive-replanning imagery where the
vehicle images while maneuvering.

## 5. Correlated speckle & bottom-type spectra (low)
Replace i.i.d. `Exp(1)` with correlated gamma speckle (spatial low-pass on
the field) and per-material K-distribution parameters. Sharpens
texture-classification realism; detector-level impact is modest.

## 6. Vegetation dynamics (low)
Seagrass sway as a per-ping phase jitter on the seagrass-material texture;
addresses A4 partially without giving up the static-scene architecture.

## 7. GPU / Gazebo-sensor backend (only if needed)
If world sizes or ping rates ever outgrow the CPU renderer, reimplement
`SonarRenderer` on a GPU ray caster (or a Gazebo sensor plugin feeding
ranges + material IDs) behind the same ABC. The interface, noise stack,
encoder and dataset layers are already backend-agnostic.

## 8. Sim-to-real calibration over a known bottom
The first pass is done: the encoder now carries the device's per-ping
normalisation, and the range law, noise floor, gain ladder and level
reference are fitted against the Shiraishi-jima harbour corpus
(`docs/sonar_model.md` §6). What that corpus cannot give is the seabed:
`lambert_exponent` is confounded with an unsampled bottom type, and the
residual floor is set by site geometry — two real passes of the same water
disagree by 1.8–3.1 dB RMS, against a 0.67 dB reduction floor. Closing that
needs a capture over a **known, flat** bottom at two or more altitudes with
the acquisition written down; `sss_calibration_report` runs the comparison
as soon as one exists.
