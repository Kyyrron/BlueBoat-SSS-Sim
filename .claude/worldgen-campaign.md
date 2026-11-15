# Generated harbour worlds: campaign generator, object set version 2, ground truth

Read this before touching `worldgen/campaign.py`, `population.py`, `domain.py`, `groundtruth.py`,
`compounds.py` (the version 2 builders), `depth/layers/heterogeneity.py`, or the metadata that
`build_world_folder` writes. Written 2026-11-15 (completion round, session 02). Sources and numbers:
`docs/seabed_model.md` §9–§12; keys: `docs/configuration_guide.md` §3.1, §4; evidence:
`~/ros2_ws/sim_runs/sim_acceptance/02_world_builder/` (this machine only).

## What exists

- `worldgen_campaign --spec <yaml> --seeds 1,2,3 --out <root>` (`worldgen/campaign.py`; example
  `config/campaign_example.yaml`): one ordinary world folder per (case, seed), built through
  `build_world_folder`. About 5–9 s per world of 140 × 130 m.
- A world's seabed: the reference class `sand` plus a patch field of a finer and a coarser class, micro-relief
  at two scales, pits and mounds, drag marks, small litter, cobbles (six layer kinds). The presets
  `HETEROGENEITY` (none | low | nominal | high) are **fitted on the FIT recording**: do not retune one without
  redoing the fit (`analysis/seabed_stats.py`, the proxies of the evidence folder).
- The object set version 2: 11 classes (`objects.V2_CLASSES`). Parts in `compounds.py`, cards in
  `population.py`.
- Ground truth for every new world (hand-made too): `metadata.yaml` schema version 2 (additive; `format` stays
  `blueboat_world_meta/1`), `ground_truth_rasters.npz`, a content digest.

## Rules that are easy to break

1. **Version 2 parts are opt-in per instance** (`params.spec: 2`) for the seven classes that existed before
   (`_BUILDERS_V2`). The construction an instance *without* `spec` gets is a build path kept on purpose (owner,
   2026-11-15), not a kept class: changing it changes existing worlds and session 01's evidence. **New worlds always
   carry `spec: 2`.** New classes (`trap_pot`, `net_bundle`) have one construction.
1b. **The catalogue is the 11 classes of the set** (since Stage 6, 2026-11-15). A deleted class is refused by name
   with its replacement (`objects.REMOVED_CLASSES`, `check_class`): by `SceneModel.load`, the ray-cast mesher and
   the placement. Do not bring a deleted class back to load an old world: rebuild the world as a new folder with
   version 2 classes (`02_world_builder/worlds_src/build_sea_validation_v2.py` is the pattern).
2. **Everything generated is pinned**: the population model writes complete `objects.explicit` entries. A new
   parameter read by a builder goes into `objects._CONSTRUCTION_PARAMS`, or a re-pinned entry loses it.
3. **Random streams**: terrain `[seed, 0]`, objects `[seed, 1]`, layers `[seed, 2, layer]`, population
   `[seed, 3]`, basin `[seed, 4]`. A draw added in the middle of `plan_basin` or `_instance` changes every
   world of every campaign: add draws at the end of a branch, and bump `campaign.GENERATOR_VERSION`.
4. **The content digest must repeat**: same specification and seed, same digest, in any folder. Nothing that
   depends on the clock, the folder path or dict order may enter the arrays, the manifest or the metadata
   (`created`, `content_digest` and `generator` are the only keys left out). Smoke `[20]` builds a world twice.
5. **The reference sediment is `sand`**, the only class the harbour sensor's correction is calibrated on. The
   generator refuses another; a world declares its patch classes in `world.seabed` so that the correction stays
   on over them.
6. **No brightness anywhere** (NC #17): a patch is a sediment class, an object is parts with materials, R6 is a
   construction (tyre) or constants with a registered bracket (fibre).
7. **Structures are rough steel, concrete or rough timber.** A smooth tall pile is over a million mirror
   patches (13.7 s a ping, measured).
8. **`metadata.yaml` is a cross-module contract** (MCS, Belief and the Replanner's emulator test the `format`
   string): add keys, never rename or retype one. The root interface tables are updated by a change request.
9. A world folder is immutable: the generator skips an existing (case, seed) and never writes `_1`.

## Known limits (stated in the handoff; the owner chose no correction, 2026-11-15)

- The presets lower the mean level of the 16–20 m band by 0.45 dB (`nominal`) and 1.08 dB (`high`).
- The real harbour's mooring blocks as concrete objects are 2–3 dB brighter in the near bands than as terrain.
- `high`'s lag-3 correlation sits 0.049 over the real maximum; its fine relief (12 mm) is at its floor.

- The simulated speckle at 40 m / 600 bins is more correlated along range than the device's: the dark-region
  rate cannot be matched at the FIT acquisition by any seabed parameter (amendment A-04: region rates are reported,
  not judged).
- A lying drum is never sediment-filled; a pipe is a closed shell; nets do not drape; no moored hulls, no
  vegetation; burial ranges hold for a sheltered basin with normal mud.
- An Edit re-save of a generated world keeps the scene exactly but not the `campaign` / `conditions` /
  `population` records of its metadata.
