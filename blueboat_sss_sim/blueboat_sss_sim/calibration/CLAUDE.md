# Field calibration — `blueboat_sss_sim/calibration/`

Moved from the module root `.claude/CLAUDE.md` (§2 and NC #6) on 2026-11-10; the rules
(NC #4 FBR envelope, NC #6 level constants) stay there. This file holds the tooling map
and the calibration history behind the current constants.

## The tooling

**Field calibration (2026-10-21):** `blueboat_sss_sim/calibration/`
(ROS-free; `sss_calibration`): the parameter registry
`config/parameter_registry.yaml` (checked against the code by smoke `[16]`
-- a moved constant fails until the registry moves with it), session
manifests (whole-session splits), `.svlog` quality reports, local
identifiability, and versioned calibration artifacts composed into a
self-contained effective profile. A calibration never edits
`default_sonar.yaml`. Design and runbook: `docs/engineering/`.
**The seabed engine (pass 2, 2026-10-30):** `calibration/seabed.py`
(`sss_calibration score | fit | scan`) reduces recordings to strata, renders
their simulated counterpart (noisy, or the noise-free expected mode, with
the recorded attitude replayed), scores level / crest / texture / nadir per
session, fits on calibration sessions only, and holds the NC #4 FBR envelope
every fitted change must pass. Results and the promotion record:
`docs/engineering/calibration_results.md`; second-pass runbook:
`validation_and_experiment_plan.md` §6.

**Shallow-water multipath (pass 4, 2026-11-02):** `sonar/waveguide.py` adds the image comb. It
images the transducer in the surface and the floor and evaluates the echo for every out/back image
pair, with Rayleigh floor and Eckart boundary coherence. There are three terms:
- the wall echo, `waveguide_comb_enabled`;
- smooth objects, `waveguide_facet_comb`;
- the seabed itself, `seabed_comb_enabled`.

Each term, and the non-sand seabed law (`seabed_level_db`), is **off / neutral by default**, which is
bit-identical. They are proved against closed forms in smoke `[17]`; `docs/sonar_model.md` §14 has the
physics. The pool's values (a calm surface 3e-4 m, a painted floor −16.4 dB re sand) are a **site**
overlay, `config/pool_environment_pass4_sonar.yaml`. It is generated, never hand-edited, never a
field profile: it fails the NC #4 envelope on 11 of 82 field cases. Pass-4 record:
`docs/engineering/calibration_results.md` Part III, `~/calib/pass4/`.

## NC #6 history — how the level constants got where they are

It moved on 2026-09-18, on TWO altitudes; the 2026-09-19 image-formation
rework did NOT move it again (its acceptance test: the two-anchor mean
offset shifted 0.20 dB at `bath3` and 0.38 dB at the harbour, inside the
0.67 dB repeatability floor). `calibration_db_offset` (**81.1 dB**, was
97.2) is still the **only** level constant, fit against the reported
level, never a count histogram. A single-altitude fit cannot separate
level from *slope*: the 97.2 dB anchor was right at the harbour corpus's
6–9 m and **18 dB hot** at the beach corpus's 1.2 m. The 2026-09-18 fit
uses `bath3` (1.40 m, 12 371 pings) and `reflectionEvidenceFullharbour`
(7.33 m, 1 445 pings) **together**, moving four constants as one package:
`tvg_compensation` 0 → **0.30** (the device compensates part of its range
loss), `specular_spreading_exponent` → **2.0** (NC #4),
`specular_strength` 30 → **12**, `calibration_db_offset` → 81.1. Result:
seabed dB-vs-slant within **0.6 dB at both altitudes**; the reported
peak's altitude law within **0.2 dB per decade** of the device's
(66.60 − 11.28·log₁₀ alt, measured over 19 gain-4 recordings). Noise
floor (**3.39e-09**) and water column (**4.68e-06**) encode measured
*levels* and moved with the offset without changing them;
`reverberation_db` (**18**, the floor of a shadow) is new, measured on the
corpus quay.

**Moved again, once, on 2026-10-30** (field-calibration pass 2,
bundle B, through its pre-registered gate; `docs/engineering/
calibration_results.md`): `specular_strength` 12 → **1.75** and the
**receiver ceiling** on (`receiver_ceiling_db` **67.9**, the beach
anchor's measured pile-up). The 12 had been fitted to the reported
`max_pwr_db`, which on the device is mostly not the first bottom return;
fitted to the detector-located FBR crest instead, the crest bias fell on
every session and the seabed level moved <= 0.07 dB.
`calibration_db_offset`, `tvg_compensation`, `lambert_exponent` did NOT
move: bundle A (a pulse-length level term + refit) failed its gate and is
provisional in `config/field_candidate_v1_sonar.yaml`. The old residual
("peak ~5 dB hot") is gone: within 4 dB of the device law at every
altitude, mean +0.7 (`[2c]`, re-anchored).
