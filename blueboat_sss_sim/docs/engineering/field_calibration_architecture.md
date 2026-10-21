# Field-calibration architecture

**Goal.** Repeatable synthetic data that are realistic enough for our tasks
(survey policy comparison, detector training), with quantified limits, and
a credible, auditable way to tune the model against real recordings and
real boat experiments. **Non-goal:** a general full-wave acoustic solver.

```
controlled field experiment
   -> immutable recordings + session manifest          calibration/manifest.py  (built)
   -> quality checks                                   calibration/quality.py   (built)
   -> identifiability of the planned fit               calibration/identifiability.py (built, local)
   -> staged fits, one parameter group at a time       NOT BUILT (needs the private data)
   -> versioned calibration artifact                   calibration/artifact.py  (built: schema + validator)
   -> held-out validation on untouched sessions        metrics: analysis/calibration.py, svlog_compare.py
   -> effective, self-contained sonar profile          artifact.effective_config / write_effective_profile (built)
   -> reproducible synthetic data (seeded, recorded)   node `noise_seed`, world seed, profile on disk
```

Every box is ROS-free and runs from `python3 -m blueboat_sss_sim.calibration`
(`sss_calibration`). Nothing in it edits a calibrated constant.

## 1. Parameter classes

Machine-readable in `config/parameter_registry.yaml`, human view in
[`parameter_registry.md`](parameter_registry.md).

| Class | What | May a calibration set it? | Per session? |
|---|---|---|---|
| `hardware_fixed` | specification, physics (aperture, frequency, spreading, sound speed) | no | no |
| `installation` | lever arm, transducer depth below still water, bracket tilt, (future) timing latency | yes | **yes** — a re-mounted transducer is a real per-session change |
| `device` | level offset, TVG share, coherent lobe, ringing, floors, gain table | yes, if `tunable` | no |
| `environment` | seabed law and materials, roughness, transmission, walls | yes, if `tunable` | no |
| `vehicle_control` | hull mass, damping (sea-state wrench) | yes (own artifact or section; not applied to the sonar profile) | no |
| `session_nuisance` | reserved: per-session level offset, clock offset, heading bias | yes, once a data reduction justifies one | yes |
| `augmentation` | gain drift, drops, scatterer seed | **never** | — |
| `acquisition` | the six real-driver run parameters | no (run setting) | per run |
| `model_structure` | switches and numerics | no | no |
| `test_injection` | fault knobs | no | no |

**Guardrail against hidden model error.** A per-session value is accepted
only for `installation`/`session_nuisance`. If every session leans the same
way, `validate_artifact` warns that the systematic part belongs in the
global value, or in the model. A device or environment term is never
per-session.

## 2. Precedence when composing a run's model

1. dataclass defaults (`SonarModelConfig`, `AcquisitionParams`)
2. the sonar profile YAML (default: `config/default_sonar.yaml`)
3. the calibration artifact's global `parameters`
4. the artifact's `sessions.<id>` values (installation/nuisance only)
5. per-run acquisition overrides (`acquisition.*` only)

`effective_config` returns the composed `SonarConfig` and a provenance map
naming the layer of each value. Registry parameters that live outside the
sonar profile (materials, hull) are returned as `not_applied`, with their
owner: they are never dropped silently. `write_effective_profile` writes a
**self-contained** profile (all 6 + 68 fields plus a `provenance:` block
that the loader ignores), so a run, or a dataset, is reproducible from one
file. Augmentation stays outside calibration: a dataset generator records
its own seeds and knobs next to the effective profile.

## 3. Calibration artifact (`blueboat_calibration_artifact/1`)

Schema and rules: [`dataset_and_calibration_contracts.md`](dataset_and_calibration_contracts.md) §5.
In short:

- every value carries its unit (checked against the registry), its bounds,
  its status (`measured | joint | estimated`), and an **uncertainty**
  (`sd`, `interval`, `equivalence_range` or `unidentified: <why>`);
- every artifact names its calibration and validation sessions
  (whole-session split, disjoint), the fit method and objective, the
  assumptions and the excluded data;
- validation results may only cite validation sessions;
- no position fixes and no raw arrays anywhere in the artifact;
- artifacts are write-once (`save_artifact` refuses to overwrite).

**Versioning.** `format` carries the major version. A reader accepts only
the formats it knows, so a `/2` artifact fails loudly on a `/1` reader. The
registry is versioned the same way. An artifact names its base profile;
re-applying it to a different profile is allowed, but the provenance block
records both.

## 4. Staged calibration order

Do not optimise everything in one pass: a later stage must not absorb an
earlier stage's error. The details, metrics and experiments are in
[`validation_and_experiment_plan.md`](validation_and_experiment_plan.md).

1. **Geometry and synchronisation:** transducer lever arm and depth below
   still water, timing latency, navigation-to-sonar alignment,
   known-target position.
2. **USV/control dynamics:** calm-water manoeuvring from command + motion
   logs, independently of acoustics; then the disturbance response where
   the logs constrain it.
3. **Device response:** pulse/range PSF, ringing, gain steps (incl. 0–3),
   normalisation, ping interval, channel behaviour.
4. **Seabed mean response:** level, TVG share, angular law, beam tilt,
   jointly over ≥ 3 altitudes and several headings, per independently
   classified bottom.
5. **Objects and labels:** reference targets of known size, material, pose
   and position uncertainty — direct echo, edge width, footprint, shadow.
6. **Noise and texture:** intensity distribution and tails, speckle
   correlation per sediment, floors, session variability.
7. **Environment and augmentation:** only after 1–6 are credible.
8. **Blind validation:** untouched sessions by site, regime and day.

Before each fit, run `sss_calibration identifiability` on the planned
data geometry. If a pair is a ridge (|correlation| ≥ 0.95), constrain one
member with independent evidence, change the experiment, or report the
pair's equivalence range rather than individual values.

## 5. What exists, and what does not

| Piece | State |
|---|---|
| Registry (64 parameters) + drift check against the code | built, gated (smoke `[16]`) |
| Manifest schema + leakage checks | built, gated |
| `.svlog` quality report | built, gated on synthetic data; run read-only on the three attached samples |
| Local identifiability (seabed profile statistic) | built, gated (reproduces `[2e]`'s ridge) |
| Artifact schema, validator, write-once I/O, composition, effective profile | built, gated |
| Staged fitters | **not built**. Each needs the private datasets to design its reduction against; the existing `analysis.calibration` reductions are the starting point |
| Navigation/bag readers for USV dynamics | `analysis/sea_validation.read_bag` (sim bags); real run bags live in Control (`run_bag.py`) |
| Global sensitivity, Bayesian discrepancy model | not built; the identifiability tool is local and noise-free |

## 6. Literature anchors

See [`literature_review.md`](literature_review.md).
Kennedy & O'Hagan (2001): keep parameter uncertainty and model discrepancy
separate. Fossen (2021): fit vessel dynamics separately from acoustics.
Jackson & Richardson (2007): literature values are priors and ranges, not
calibrations.
