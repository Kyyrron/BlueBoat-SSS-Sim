# TODO — BlueBoat-SSS-Sim

Open items only. Settled facts live in `CLAUDE.md`.

Priority marks: **[P0]** blocks use of the package · **[P1]** correctness or
cross-module risk · **[P2]** realism / project value.

---

## Cross-module correctness

### [P1] Transducer lateral offset disagrees across modules
The simulator mounts transducers at `mount_y_abs_m: 0.20`
(`sonar/config.py`, `config/default_sonar.yaml`); `sss_processor_node.py`
still has all four of `TRANSDUCER_X_OFFSET_M`, `TRANSDUCER_Y_OFFSET_PORT_M`,
`TRANSDUCER_Y_OFFSET_STBD_M` and `TRANSDUCER_SUBMERSION_M` at `0.0` with their
"TODO: measure on the physical BlueBoat" comments intact. Until both use the
same numbers, slant-range-corrected geometry from simulated data carries a
systematic across-track bias.

**Needs a physical measurement, and there is no second source for it**:
`blueboat_description` ships a `sonar_snippets.xacro` but never instantiates it
on the BlueBoat model, so the URDF does not carry the geometry either. Measure
on the hull, then set both repositories (the `BlueBoat-SSS` half is that
repo's change — CM-3) and record who measured it, on which hull, from which
reference point. Do not pick a number to make the two agree: an
agreed-but-wrong offset is worse than a recorded disagreement.

### [P2] 1200-bin handling in the GCS is unverified
The ROS half is settled: at `num_results:=1200` the wire contract, throughput and
`sss_processor_node` all hold (recorded in `CLAUDE.md` §6). What was not exercised
is the **visualization app**. `blueboat_gcs` reads `num_results` from the packet
everywhere it matters (`core/svlog.py` sizes its payload as
`_HEAD_SIZE + 2 * num_results`) and hard-codes no bin count, so it should be
bin-count-agnostic by construction — but that is a code reading, not a run. It is
a Qt app whose acquisition starts from a button, so confirming it needs an
interactive session, and the fix for anything found there is `BlueBoat-SSS`'s
(CM-3).

---

## Realism and project value

### [P2] Seabed-dependent constants need a capture over a known bottom
The Shiraishi-jima corpus fitted the encoding, the range law, the noise floor,
the gain ladder and the level reference (`docs/sonar_model.md` §6). It cannot
fit the constants that speak about the **seabed itself**: `lambert_exponent` is
confounded with an unsampled bottom type, and `specular_strength` /
`specular_width_deg` are set by whatever lies under the boat — the real −6 dB
FBR width varies 7× across altitude bands of one recording. Both are held at
values NC #4 gates rather than fitted.

The blocker is site geometry, not tooling. Two *real* passes of the same water,
at the same range setting and altitude band, disagree by **1.8–3.1 dB RMS**,
against a 0.67 dB reduction floor — a flat-seabed model compared against a
harbour survey cannot do better, because the far swath sits over ground whose
depth is nothing like the depth under the boat. Closing this needs a capture
over a **known, flat** bottom at ≥ 2 altitudes with the acquisition written
down. `sss_calibration_report` runs the comparison the moment one exists; it
refuses to score a pairing whose geometry does not overlap.

### [P2] `calibration_db_offset` is determined only to ±6 dB
The device's auto-gain regulates the real reported level, holding `max_pwr_db`
roughly flat against altitude while the model's follows the range law. Pooled
over 14 (range, altitude) cells at gain index 4 the required offset spans
**11.9 dB**; 97.2 is the ping-weighted mean. A capture at a **fixed** gain
index would pin it. Nothing in the current corpus fixes gain.

### [P2] Gain indices 0–3 are unmeasured
`ANALOG_GAIN_TABLE` carries measured values for 4–7 — the only indices the
device's auto-gain ever selected in seven recordings. 0–3 are the original
estimates and would misreport `analog_gain` and the level if a run used them.

---

## Blocked — not verifiable on this machine

Attempted and deliberately not answered. The remaining item needs the
Omniscan 450 hardware and should not be guessed at.

- **20 Hz vs 22 ms ping rate.** The Omniscan 450 spec sheet says ≤20 Hz; the
  field capture's device `timestamp_ms` deltas say 22 ms (45 Hz) per channel at
  15 m. Shipped as `max_ping_rate_hz` (default 20, `0` disables), so either is
  reproducible. Resolve against the device — firmware version, whether the spec
  figure is a max-range guarantee, or a longer capture at several range
  settings — then set the default from evidence and record which is right.
  *NOT VERIFIABLE ON THIS MACHINE (needs the Omniscan 450 hardware).*

---

## Known limitations (accepted, not bugs)

Recorded so they are not "rediscovered" as defects. These match the explicit
assumption list A1–A10 in `docs/sonar_model.md` §7:

- Static acoustic scene during a run; no intra-ping motion (stop-and-hop pings).
- Straight rays, no refraction. Reflections are first order only: the direct
  path plus the two optional multipath models (second-bottom-echo ghost, and
  wall/surface mirror sources), both off by default.
- A wall reflects but does not echo: it produces multipath ghosts and a Gazebo
  visual, but no direct return or shadow of its own, because it is not in the
  height raster.
- 2.5-D heightfield scene: no overhangs or cavities; objects are height +
  reflectivity stamps. Wrecks with internal structure would need a mesh path.
- Uncorrelated inter-ping speckle.
- No biofouling, sediment interaction, or vegetation dynamics.

Settled against a full simulated survey (bundle `~/runs/r3`, seed 7, 906 s
lawnmower, 19 354 pings, mosaic cells 3.4 M):

- **Across-track range, not seabed material, sets large-scale brightness —
  by a wide margin.** Intensity falls **18.5 dB** from just past the bottom
  return to the 12–15 m far edge (measured on the fitted range law at 3.8 m
  mean altitude), against a **3.67 dB** total spread between all five
  materials. Sand→mud is ~0.7 dB and seagrass sits 0.24 dB from sand, so the
  "mud/seagrass image 5–10 dB darker" reading does not hold; the only material
  that separates is **rocks, ~2 dB brighter**. The gradient is the device's own
  pre-TVG range loss, measured against the corpus — dark far-range bands are
  expected output, not a renderer defect. Set `tvg_compensation: 1.0` for a
  flat image; waterfall tiles already remove its smooth part for display.
  **The 3.67 dB material figure and the mosaic numbers below were measured
  under the old range law and have not been re-measured since the fit.**
- **Objects are well resolved in the mosaic; contrast is the limit, not pixel
  size.** At the GCS's `auto_cell_size` (30.5 mm/cell for 15 m / 600 bins)
  manifest objects span **4–106 cells** — a 0.77 m debris object is 26 cells at
  7.9 σ. Of 29 objects, 23 were ensonified and **10 clear ≥3 σ**; the 13 that do
  not are spatially resolved but low-contrast at 4 m altitude (a 2.74 m PVC pipe
  is 90 cells at 2.0 σ). More mosaic resolution would not help — shorter
  altitude would, via shadow length; that is what
  `config/shallow_water_world.yaml` ships for.
  Detector evaluation stays in the **waterfall** domain regardless (root
  `CLAUDE.md` CM-10); there a 0.77 m object spans ~20 bins at 37.5 mm/bin.
