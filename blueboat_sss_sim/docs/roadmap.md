# Realism roadmap

Ordered by (thesis value ÷ effort). Each item plugs into an existing seam
— none requires touching the ROS interface or the dataset pipeline.

## 1. Sound-speed profile & ray bending (low effort)
Replace straight-ray `slant = hypot(y, dz)` with a two-layer SVP ray step
in `GeometricRenderer._ping_geometry`. Mostly relevant beyond ~30 m range
or strong thermoclines; low priority for the 15 m shallow regime.

## 2. Wall / surface multipath (high thesis value)
The enclosed-basin regime's signature artifact (quay walls, pontoons).
Model as mirror-image sources: reflect the transducer across configured
vertical planes (add `walls:` to the world config) and across z = 0,
render each virtual source with a reflection-loss factor, and sum into the
same range bins. Produces the ghost returns and false-positive stimuli the
thesis's C4 characterization needs — and gives the detector hard negatives
for free, with per-ghost ground truth.

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

## 8. Sim-to-real calibration loop
With real Omniscan captures available (`sonar_data.txt` pipeline), fit
`base_scale`, `calibration_db_offset`, `lambert_exponent`, noise floor and
drift parameters by matching per-range intensity histograms between real
and synthetic waterfalls of the same seabed type. Turns the model knobs
from plausible defaults into measured values — the highest-leverage single
step for training-data usefulness.
