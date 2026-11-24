# The harbour population model (mission-like placement)

*Model version 1.0.0, 2026-11-21. Package `blueboat_harbour_population` (beside `blueboat_sss_sim` in this
repository, outside the simulator's code digest). Evidence: `EVIDENCE.md` beside this file. Every number:
`blueboat_harbour_population/blueboat_harbour_population/data/evidence_parameters.yaml` (value, tag, bracket, sources,
the function that reads it). How to run it: §11.*

## 1. What it does

`populate(environment, params, seed) -> PlacementPlan` fills a harbour's seabed the way documented harbours are
filled: targets (the seven classes of the object set), debris, small litter and the structures the harbour type
needs, where such objects lie, at a chosen density level 1–10. The same environment, block and seed give the same
plan, byte for byte, on any machine.

The **environment** is what the earlier screens (or a specification) define: survey water, depth and sediment on a 1 m
planning grid, and the harbour's **features** with stable ids (quay walls, pontoons, breakwaters, berths, outlets,
mooring blocks, piles, drawn areas and lanes). The **plan** holds instances in the frozen generator's explicit-instance
format (every drawn field pinned), groups, structures, seabed layers and a report. The simulator never runs the model:
its adapter (`sim_adapter.py`) builds the environment with the frozen build pipeline and hands the plan to the frozen
`build_world_folder`.

## 2. Sources and kernels

**A feature becomes a source through its use** (a source *tag*). The use is, in order: the user's
(`placement.feature_tags`, or a double-click on a wall on Screen 5), the feature's own (a specification may give it),
the profile's default for the feature's kind (`profiles.<p>.kind_use`). `none` makes a feature no source (a pile, by
default). Tags: fender line, urban quay, working quay, fishing berth, marina pontoon, mooring block, mooring field,
traffic lane, fishing ground, dumping spot, breakwater toe, outfall, angling pier, slipway; and the six `generic_*`
tags of the targets document's model.

A tag carries:

- **a class mix** (`emits`: relative emission of each target class, and of debris);
- **a kernel**: where its objects lie against the distance d to the feature (beyond its solid body). A *near* part —
  half-normal, exponential, a bump at a distance (a city quay: things are thrown a few metres) or uniform within a
  radius (a spot, a lane, a ground) — holds `1 − broad_share` of the objects, a flat part over the reach the rest;
- **hot spots** along it (`patch`): litter is lost at particular berths, ladders and racks, and derelict pots sit in
  tight clusters. A share of the emission sits in Gaussian patches round spots drawn on the feature (per 100 m of
  line, or per hectare of area), the rest is even. The spots are drawn once per feature from its own stream: the same
  spots at every level and for every class. The fishing-ground values are fitted to Port Angeles (§6);
- **group processes** (§4).

**Reach and ownership.** A line source reaches the water within the active band (50 m) of it, a point source within 15
m, a flat-kernel source within its own radius. A cell reached by n sources belongs to each for 1/n. A source's budget
= level density × the area it owns × its activity (a user's weight 0–5; a dumping spot is 4 by default). So:
- the mean density over the active band is the level's, whatever the harbour's geometry;
- changing one feature's use or activity changes that feature's budget and no other.

Water no source reaches is the **far field**, at 2 % of the level's density [DESIGN; Venice lagoon-wide over Grand Canal
= 0.65 %].

## 3. Sampling

An inhomogeneous Poisson process, sampled by thinning a dominating one, in metres and vectorised:

1. One random stream per `(placement seed, feature id, class, process)` (`numpy.random.SeedSequence` with SHA-256 keys
   of the strings).
2. Each stream draws N ~ Poisson(600 per ha × the source's reach) candidates, each with a thinning mark, a priority
   mark, a cell (from the source's kernel), four jitters inside it, a heading and a row of attribute uniforms.
3. A candidate is active when its thinning mark is under (expected count / N). So a lower level, a smaller class
   share or a lower activity is a **subset of the same candidates**: the levels are nested.
4. Its position is the first of its jitters where the object fits (on allowed water, its footprint clear of walls,
   structures and the objects already there).
5. **Hard core** (Matérn type II): a single or a group is deleted when another lies nearer than the spacing (3 m between
   targets, 1 m otherwise, never less than the two footprints plus 0.3 m) and has a smaller priority mark. Whether an
   object survives depends only on the candidates within its own radius.
6. The spacing rule removes candidates; a **retention factor per level**, measured on 2,400 plans over 12 campaign
   basins, gives them back (0.98 up to level 5, 0.855 at level 10).

`count: fixed` (Advanced) replaces the Poisson count by its rounded mean, per stream.

**What "stability" means (tested, `test/test_stability.py`).** An object's place and attributes depend only on its own
stream. Changing one feature's use or activity changes that feature's objects and, at most, objects of other sources
within the spacing distance of one of them. Deselecting a class makes the other classes share the total (as the owner
asked): they gain objects, and none of theirs moves or changes. Making a feature no source lets its neighbours own
its water: they gain objects, none moves. Strict "nothing else changes" cannot hold together with a fixed total and a
global hard core; this is the precise statement. Not covered: the seeded micro-shape of nets, debris and lattices,
which the frozen generator draws from an object's index in the world (frozen behaviour).

**Optional redistribution** (`advanced.redistribution`, off by default): 30 % of the light single objects (boxes, nets)
move 2–8 m down the local slope, when the place is free [DESIGN; storms carry light gear into lows, R15]. Scour rims are
not modelled.

## 4. Groups

| Process | Members | Layout | Evidence |
|---|---|---|---|
| Tyre line | 2–6 tyres | 0.5–2 m apart along the feature, ±0.3 m off it | Pomos 7–9 tyres per berth; Plymouth groups up to 15 at 5 m linking |
| Dumping pile | 5–30 items of every class of its source | inside a disc of 2–6 m | The 60 m³ Whelan's Reef pile (R18) |
| Gear ball | 2–6 traps, a net or rope with 70 % chance | inside a disc of 0.6–1.5 m | Gear balls of Boothbay (R15) |
| Trap pair | 2–3 traps | 9–27 m apart in a line | Maine inshore groundline (R39) |

An emission event of a source with a group process is a group with chance p(level) × the process weight (p = 0.05 at
level 1 to 0.30 at level 10 [DESIGN]); the event rate is lowered so that the expected number of objects stays the
budget. Members keep their own records and share a `group_id`; the group gets a record (type, source, centre, radius,
members, classes). Spacing applies between groups and singles, never inside a group.

**Scoring a detection on a group (a proposal for the evaluation code; nothing is implemented in another module).**
Score per member, as for singles, and report per group beside it: a group counts as found when at least one member is
detected (the operator would revisit it), with its member count. Members within one footprint of each other (a tyre
line at 0.5 m) may yield one detection; the matcher should then credit the nearest member and mark the others as
"covered by a group detection", neither missed nor false. This keeps the per-family recall of the thesis comparable
with and without groups.

## 5. Density scale, class selection, realism guard

**Level.** λ̄(s) per ha in the active band: 1.0, 1.78, 3.16, 5.62, **10**, 15.8, 25.1, 39.8, 63.1, 100 (companion §2.2;
anchors re-verified in EVIDENCE.md §2). The level scales every source's budget, the far field, the small litter
(0.01 → 0.06 → 0.5 items per m² at levels 1 / 5 / 10: the frozen sensor's verified clutter presets) and the group
chance. It does not change the mix, the kernels, the hot spots or the poses.

**Class selection.** The level fixes the total. The selected target classes share the target part of it:
- `realistic`: the profile's class weights × a per-class factor g × the tag's emission. g is solved once per profile
  (iterative proportional fitting over the seven classes) so that a harbour with the profile's *reference* composition
  of sources has exactly the profile's weights; a harbour with other sources departs from them, and the report shows
  the expected share of each class;
- `balanced`: equal expected counts of the selected classes;
- a mapping: those weights.

A deselected class is absent and does not reduce the total. Switching debris off removes the debris part only (the
targets are the same objects). The panel shows the expected share of every class and the expected counts.

**Target share.** Each profile's share of targets among sonar-size objects (0.4 working port to 0.7 trap harbour),
moved per source by the tag's relative emission of debris (a working quay emits more debris than a fishing berth).

**Realism guard.** When a class's expected density over the placement water exceeds 10 times its highest documented
area mean (traps 4.0 per ha, tyres 500 per ha, all litter 29 per ha), the plan warns and the world records
`stress_world: true`. A world can be dense; it cannot be silently implausible.

## 6. What the model reproduces (validation, Part 4)

Two hundred placement seeds per (profile, level), spread over 12 campaign basins in the sensor's altitude band
(`~/ros2_ws/sim_runs/worldgen_v11/stats/final/`; figures in `figures/`).

| Measure | Model | Evidence |
|---|---|---|
| Density in the active band | within ±10 % of λ̄(s) at every level and profile (level 10: 91–109 per ha) | The scale's anchors (§5) |
| Clark–Evans ratio, edge-corrected, levels 5–10 | 0.66–0.83 | Measured 0.62 (Port Angeles pots), 0.62–0.70 (St Mary's), 0.87–1.0 (Weymouth) |
| Pots at Port Angeles density (1.2–1.3 per ha) | median nearest neighbour 14.8 m, 67 % single at 10 m, Clark–Evans 0.60 | 17 m, 63 %, 0.62 (fitted) |
| Tyres in a fishing harbour, level 5 | median nearest neighbour 11.9 m, 64 % single at 5 m | Plymouth Sound 8.4 m, 65 % (not fitted) |
| Class shares | follow the profiles; at level 8 the spacing rule removes some tyres from crowded fender lines (fishing harbour: 0.41 realised against 0.48 expected) | — |
| Targets among sonar-size objects, level 8 | 0.40–0.54 by profile (trap harbour 0.49: the basins have no fishing ground) | profiles 0.4–0.7 |
| Share within 30 m of a source | 0.85–0.95 | Targets document 0.7 (0.5–0.9) |

**The one miss, explained:** the share within 30 m sits at or over the targets document's bracket because the
campaign basins are small (1.6–1.8 ha, 140 × 130 m with quays on up to three sides): a uniform scatter over their
placement water already lies within 30 m of a feature for 70–90 % of its points. The measure says more about the
basin than about the model; in a large world it falls (a 9 ha trap ground: 0.70).

## 7. Profiles

| Profile | Class weights T1…T7 (box, drum, block, trap, pipe, tyre, net) | Targets among sonar-size | Default sources | Typical levels | Candidate harbours |
|---|---|---|---|---|---|
| `fishing_harbour` | 0.10 / 0.02 / 0.05 / 0.10 / 0.03 / 0.50 / 0.20 | 0.6 | fender lines, fishing berths, breakwater toe, slipway | 5–8 | Cherbourg fishing port; the Pomos and Paphos type |
| `trap_harbour` | 0.05 / 0.01 / 0.04 / 0.65 / 0.03 / 0.07 / 0.15 | 0.7 | fishing ground, traffic lanes, fishing berths, mooring blocks | 3–6 | Provincetown, Boothbay, Port Angeles, Port Gardner, Charleston |
| `city_harbour` | 0.15 / 0.02 / 0.05 / 0.35 / 0.05 / 0.25 / 0.13 | 0.6 | urban quays, marina pontoons | 4–8 | Marseille Vieux-Port, Venice canals, Plymouth waterfront |
| `marina` | 0.15 / 0.01 / 0.10 / 0.05 / 0.04 / 0.35 / 0.30 | 0.5 | marina pontoons, mooring blocks, slipway | 2–5 | Chantereyne (Cherbourg), Plymouth marinas |
| `working_port` | 0.05 / 0.05 / 0.15 / 0.05 / 0.25 / 0.30 / 0.15 | 0.4 | working quays, breakwater toe, mooring blocks | 5–8 | Thessaloniki, Mangalia, New Bedford, Boston inner harbour |
| `generic` | 0.15 / 0.05 / 0.10 / 0.15 / 0.10 / 0.30 / 0.15 | 0.5 | one exponential kernel (15 m) per kind, session 02's affinities | any | Session 02's campaign worlds |
| `auto` | each source's own mix | 0.55 | each feature's own use | any | real harbours with several uses |

The weights are [DESIGN] (no study counts sonar-size harbour litter by class); they are swept in E5. In `city_harbour`,
`trap_pot` instances of an urban quay stand for bicycles (63 %) and e-scooters (37 %) [DERIVED from Copenhagen 2020]
and say so (`stand_in_for`). The ten candidate harbours map to profiles in `harbours` of the parameter file; a world
built there takes that profile unless told otherwise, and its card lists the gaps (wood for the Puget Sound harbours).

## 8. Per instance

Class cards: the frozen simulator's own (`worldgen/population.py`, the targets document §4), restated in the parameter
file and compared with the simulator by `test/test_frozen_cards.py`: size, state, pose and burial (U(0, upper end) for
the sediment under the object, × 0.5 / 1 / 1.5 for the burial preset, capped at 0.9; over 0.7 = effectively buried).
Every instance carries `params.spec: 2`, `role`, `nominal_target` (largest dimension ≥ 0.5 m), its pose, and the
placement record: `source {feature, use, process, index}`, `group_id`, `stand_in_for`, `depth_m`, `altitude_m`,
`sediment_under`, `on_mud`, `in_envelope`. One row of uniforms per instance: its shape depends on nothing else.

Not representable by the frozen classes, so on the gaps list (EVIDENCE.md §6): bicycles and trolleys as shapes,
damaged traps, truck tyres (the builder accepts the size, the class card does not), long cables and nets, wood,
lobster traps, per-object fouling.

## 9. Envelope

Objects lie only where the sonar altitude, with a 0.10 m heave margin, is within 4.5–7.0 m (the transducer is 0.22 m
deep: water 4.82–7.12 m), on survey water, clear of walls and structures and 2 m inside the world's limits.
`advanced.outside_band: true` lifts the band for showcase worlds; every object outside it says so (`in_envelope`) and
the world records `showcase_world: true`. Mud is allowed and flagged per object (`on_mud`). The report gives the share
of the world's water in the band.

## 10. Seeds, splits, structures, small litter, determinism

- **Seeds.** The placement seed is separate from the world seed (which seeds the seabed): Roll on Screen 5 never moves
  the seabed. Streams are keyed by strings (feature ids, classes, processes), never by list positions.
- **Splits** (`data/splits.yaml`): development 1–99,999; dataset 100,000–199,999; tuning 200,000–299,999; evaluation
  300,000–399,999 [DESIGN; none existed]; and a harbour split (dataset: Provincetown, Boston, Venice; tuning:
  Charleston, Cherbourg; evaluation: Port Angeles, Plymouth, Boothbay; Port Gardner and Marseille unassigned, the
  owner decides). A seed or a harbour of another split is refused unless `split_override` gives a reason, which the
  world records.
- **Structures** (`structures: auto`): a mooring field (blocks 20–30 m apart, 1–1.5 swing radii, with their risers) for
  the profiles that have one, when the world has no mooring; guide piles along pontoons that have none. They are
  obstacles and sources, never targets. `keep` adds none; `none` also ignores the features they would be.
- **Small litter**: the frozen sensor's `small_litter` seabed layer, at the level's density, 4 times denser within 10
  m of quays and pontoons (Sydney piers: 10 times), with at least a quarter of the mean everywhere, capped at the
  verified clutter maximum; two layers with reserved ids (`ml_`) and their own seeds. The world's seabed declaration
  lists their hard class (the sensor's near-nadir correction needs it).
- **Determinism**: numpy supplies uniform doubles only (PCG64 and SeedSequence are stable across numpy versions);
  decisions use + − × ÷, square roots and comparisons, a fixed polynomial for exp, sequential sums; outputs are
  rounded to 1 mm and 1e-5 rad. Checked: the same plans from two processes and from numpy 1.26 and 2.4
  (`test/test_model.py`), and from a world rebuilt from its folder (`test/test_adapter.py`).

## 11. Metadata and how to run

**Per world**, `metadata.yaml` gets a `placement` block (additive to schema version 2; `format` unchanged): mode, model
{package, version}, plan and environment digests, profile, level, λ̄, classes, mix, weights, seed, split, clutter,
structures, advanced, the features with their uses and budgets, the groups, the small litter, the envelope, expected
and realised counts and statistics, warnings, `stress_world`, `showcase_world`, gaps. No code digest and no path:
`metadata.yaml` is inside the content digest. The model's code and evidence digests are in `builder_state.yaml`, the
build manifest and the world card. **Per object**, the placement record of §8 under `objects[i].params`; the simulator
lifts `role`, `nominal_target`, `pose`, `hotspot` (the source's id) to the object. Checked by `schema.py`.

```bash
cd BlueBoat-SSS-Sim/blueboat_harbour_population
python3 -m blueboat_harbour_population check
python3 -m blueboat_harbour_population populate --env synthetic --profile fishing_harbour --level 5 --seed 7 --card /tmp/card
python3 -m blueboat_harbour_population build --spec examples/mission_like_example.yaml --seeds 1,2,3 --out <worlds root>
python3 -m blueboat_harbour_population batch --envs <environments.yaml> --split dataset --levels 2,5,8 --worlds 30 --out <folder>
python3 -m blueboat_harbour_population card --world <world folder>
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest            # the model's tests
```

A specification is a `blueboat_campaign/1` file with a `placement:` block (and, for a real depth file, an
`environment:` block: `depth_file`, `values`, `water_level_m`, `smooth_sigma_m`, `shift_m` or `compress {k ≥ 0.5,
d_lo, to}`, `walls`, `features`, `sediment`). Without `placement:`, or with `mode: legacy`, `build` hands it to the
frozen `worldgen_campaign` unchanged. In the World Builder: Screen 5, "Mission-like".
