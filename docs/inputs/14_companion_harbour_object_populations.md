# Harbour seabed object populations: evidence and metrics for the mission-like placement mode

*Companion to session prompt 14. Version 1, 2026-11-19. Web sources were read on 2026-11-19 (how each was read is
given in §9). It builds on `target_families_and_surrogates.md` v2 (the object set, class cards and first
population model; "targets doc") and copies from `real_harbour_bathymetry_candidates.md` v2 ("harbours doc") only
what the placement mode needs (§7). It is a starting point: the session is expected to verify, extend and correct
it.*

**Tags**

| Tag | Meaning |
|---|---|
| **[R n]** | Read in source n of §9 |
| **[TD]** | From the targets doc v2, which read its sources in full |
| **[HD]** | From the harbours doc v2 (web facts read 2026-11-19 by an earlier session) |
| **[PROJ]** | A project fact |
| **[DERIVED]** | Computed here from referenced values; the computation is shown |
| **[DESIGN]** | A modelling choice with its reason: a default with a bracket, never a measurement |
| **[CHECK]** | Not verified. Verify before relying on it |

"Sonar-size" means at least about 0.5 m, the size floor of the target set (IHO Exclusive Order, [TD §1]).
Densities are per hectare of water unless stated.

---

## 0. Ten findings the mode must reproduce

1. **Litter stays close to where it entered the water.** Sinking litter mostly stays near its emission point;
   storms can move it up to about 4 km [TD, Schernewski 2023]. Litter peaks near urban quays and harbour inlets
   [TD, Venice]; it was slightly denser close to the breakwater at Thessaloniki [R1]; derelict lobster gear
   "tends to be nearshore, nestled in valleys or the backs of ridges after storms" [R15].
2. **Each harbour use has its own sources and its own objects** (detail in §4):
   - *Fender lines, tyres.* 180 tyres lifted from one small fishing refuge, where boats use them as fenders
     against the docking stations [R22]. 32 car, 8 motorcycle and 4 truck wheels in a two-hour dive in Kato
     Paphos harbour [R23]. Tyre fenders in Maltese bays "with the first storm … washed away into the sea";
     a local group reported "650 tyres and counting" found that year [R24].
   - *City quays (the sources count the items but do not say where they lay).* Copenhagen, 2020, ten days of
     diving: 299 bicycles, 178 e-scooters, 44 tyres,
     10 safes [R19]. Marseille Vieux-Port, 2018: at least 4,333 objects, among them 42 bicycles, 15 skateboards
     and scooters, 7 prams and 1 motorbike; glass bottles mostly along one quay (Rive Neuve), "more polluted"
     than the opposite one [R20].
   - *Berths.* Engine batteries found "under the boats" in the Vieux-Port, 2025 [R21].
   - *Fishing grounds crossed by traffic.* Port Angeles Harbor West, sheltered and bounded by log-booming grounds,
     with heavy tug, barge, tanker and recreational traffic that may cut float lines: 206 derelict pots in
     1.53 km². The adjoining East area: none [R3]. In the Boothbay region, gear concentrates in traffic lanes,
     where boats sever buoys [R15].
   - *Working quays and shipyards.* "Many debris and some wrecks" near the shipyard docks of Mangalia harbour
     [R8]. Metal pieces, car tyres and wires at Thessaloniki [R1]. Pipelines and cables, mooring blocks, chain,
     anchors and "other dropped objects" in New Bedford Harbor [R10].
   - *Dumping spots.* Mount Batten (Plymouth) "appears to have been a favourite place for dumping junk of all
     types", with "a mess of wire, steel and tyres"; Whelan's Reef is a pile of about 60 m³ and 50 t of tyres,
     steel scrap, net, rope, lobster pots, a dinghy and a fridge, gathered by trainee divers in the 1980s [R18].
   - *Log booms and rivers.* Logs and woody debris heavily concentrated near the log-boom grounds of Port Angeles
     [R3]; 9 of 47 debris anomalies woody in the Saco River estuary [R9]; Swedish sonar returns were mostly
     branches and ropes [R13].
   - *Angling piers.* Harbour walls "draped in decades of lost fishing line" [R18]; a net "half-buried" outside a
     pier at about 4 m [R13].
3. **Densities span three orders of magnitude; harbour basins cluster near 10 per ha** (§2).
4. **Neighbouring areas differ by 10× to 100×.** Port Angeles West 134 pots/km², East 0 [R3]. In an Adriatic
   anchorage the mean was 1.3–1.7 items per 100 m², with three hotspots of at least 4 items/m² [R11]. Clustering
   is the norm; a uniform scatter is not realistic.
5. **Cleaning halves the stock, and it comes back.** Port Gardner: 136.0 derelict-gear targets/km² in 2004,
   63.6 in 2008 after removals; the cleared area was re-surveyed to measure the yearly pot loss [R4].
6. **Burial is common on soft floors.** Divers cut the mesh of pots "more than half buried" [R3]; debris and a
   frame "partially buried" or "partially embedded in river bed" [R9]; junk on sand and mud "may be partly or
   completely buried" [R18]; a half-buried net [R13]; litter buried in sludge in a fishing port [TD].
7. **Old items are fouled.** Colonisation and encrustation by biota [R1]; tunicates, barnacles and hydroids on
   traps [R5]; barnacles on derelict gear [R15].
8. **Many derelict traps are intact and still fishing.** 82 % of removed traps [R5]; 28 of 52 (54 %) [R3]; 37 %
   [R4]. The rest are damaged (propeller or drag damage in Port Angeles West [R3]), collapsed or buried.
9. **Trap arrangement depends on the fishery.** Puget Sound crab pots: commercial and sport about half each
   (71 and 74 removed in Port Gardner [R4]; 20 and 32 in Clallam County [R3]); shrimp pots on longlines, two per
   buoy [R3]. Maine: inside the exemption line any configuration, including singles; pairs from the line to
   3 miles; longer trawls further out [R16, R17]. Tangled "gear balls" of traps and rope form, the largest about
   3,700 lb [R15].
10. **Small litter is about a hundred times denser than sonar-size litter.** 0.1 items/m² on average in German
    marinas (0.04–1.75), 0.7 in North African marinas, more than 3 in city-centre harbours [R12]; 1.3–1.7 items
    per 100 m² in an anchorage [R11]. This is the C2 layer, not targets.

---

## 1. Case scenarios

| # | Site (type) | Method | Area, depth | Counted | Count | Per ha [DERIVED] | Spatial, state and size notes | Src |
|---|---|---|---|---|---|---|---|---|
| 1 | Port of Thessaloniki, GR (commercial port) | USV, 455/800 kHz side-scan, 10 cm mosaic; ROV with USBL check | 120 × 160 m = 1.92 ha; depth not stated | Acoustic anomalies (man-made: metal pieces, car tyres, wires) | 56 | **29.2** (56 / 1.92) | "Largely homogeneous", slightly denser near the breakwater; equivalent diameter bimodal, peaks 0.6 and 1.5 m; 33 (59 %) at 0.5–0.7 m, 7 (12.5 %) over 1 m; colonised and encrusted | R1 |
| 2 | Venice, Grand Canal, IT (urban canal) | Multibeam | 30–70 m wide, about 5 m deep [HD] | Macro-litter | — | **11.6** (1,161/km²) | Quays on both sides; tyres, rubber fenders, containers, rip-rap debris, dock piles; scour holes 40 × 15 × 1.5 m at water-bus stops, up to 120 × 80 × 3.5 m at the cruise terminal | TD |
| 3 | Venice lagoon channels, all | Multibeam | About 40 km² | Macro-litter | — | 0.075 (7.5/km²) | Hotspots around Venice, Chioggia and the inlets | TD |
| 4 | Port Angeles Harbor West, WA (harbour crab ground) | 600 kHz side-scan, 50 m per side | 1.53 km²; pots at about 9–29 m, many near 10 m | Derelict crab pots | 206 | **1.34** | Traffic cuts float lines; 20 of 35 recovered inactive, with possible propeller or drag damage; some more than half buried; logs heavily concentrated near the log booms (not counted) | R3 |
| 5 | Port Angeles Harbor East | Same | 1.20 km² | Pots | 0 | 0 | Adjoins #4 | R3 |
| 6 | Clallam County, 7 areas | Same | 8.79 km²; 4.6–30.5 m | Pots (292 crab, 4 shrimp) | 296 | 0.34 overall; by area 0 to 1.34 | Dungeness Bay 0.20, Sequim Bay 0.37, Freshwater Bay 0.03; 11 octopus traps made of tyres also recovered | R3 |
| 7 | Port Gardner, WA, 2008 (bay crab ground, after removals) | 600 kHz side-scan | 2.61 km²; 3–32 m | Derelict-gear targets | 166 | **0.64** (2004: 1.36) | 145 pots removed, 71 commercial and 74 sport; 54 (37 %) still fishing; rot cord disintegrated on 120 of 128 | R4 |
| 8 | Lower York River, VA (estuary crab ground) | 600 kHz side-scan, 100 m swaths, 20 % overlap | 33.5 km² | Derelict traps | 635–676 | 0.19–0.20 | Sarah Creek (0.2 km²): 12–16 traps, 0.6–0.8/ha; 16 of 17 targets confirmed; 82 % of removed traps still fishing; fouled by tunicates, barnacles, hydroids | R5 |
| 9 | Mangalia harbour, RO (port and shipyard) | 400 and 1,250 kHz side-scan | 114.7 ha of water; up to 9 m | Anthropogenic targets (not counted) | — | — | Debris and wrecks near the shipyard docks; tyre 0.7 ± 0.1 m; metal frame 3.84 × 0.40 × 0.40 m; rope 1.75 m; buoy anchor 3.5 m across; a rock mound beside the channel; anchor-drag traces, oil hoses, sand ripples | R8 |
| 10 | Saco River estuary, ME (near a planned dredging area) | Multibeam, 0.5 m grids; camera tows | About 8 km reach; tidal range 2.76 m | Debris anomaly areas | 47 | — | 6 suspected tyres, 9 woody (poles, planks), metal cables, 1 mooring block, frames; several partly buried | R9 |
| 11 | New Bedford Harbor, MA (industrial port) | Side-scan, 50 ft (15 m) line spacing | 103 nautical miles of track | Side-scan anomalies | 87 | — | Pipelines and cables, mooring blocks, chain, anchors, other dropped objects, pier and wharf remnants, sunken vessels, rail track, boulders | R10 |
| 12 | Marseille Vieux-Port, FR (urban marina basin, 24 ha [HD]) | Divers, yearly | 24 ha, partly covered; 5–6 m [HD] | Large items lifted, 2018 | ≥ 65 (42 bicycles, 15 boards and scooters, 7 prams, 1 motorbike; tyres not counted) | ≥ 2.7 (a lower bound) | ≥ 4,333 objects in total, mostly small (1,613 glass bottles, mainly along Rive Neuve); 2019: 92 e-scooters [CHECK: a later caption says the 2019 event was cancelled]; 2025: "many tyres", bicycles and e-scooters "by the dozen", waste containers, engine batteries under the boats, scrap metal "en masse" | R20, R21 |
| 13 | Copenhagen Harbour, DK (city harbour), 2020 | Divers, 10 days | Not stated | Large items | 531 | — | 299 bicycles and 178 e-scooters (90 %), 44 tyres (8 %), 10 safes | R19 |
| 14 | Pomos fishing refuge, CY (small fishing shelter), 2022 | Divers | Small refuge [CHECK: measure the basin] | Tyres | 180 | [CHECK] | Used as fenders; about 1,500 kg in total with plastics, metal, old nets and cans | R22 |
| 15 | Kato Paphos harbour, CY, 2025 | Divers, 2 h | Not stated | Wheels and tyres | 44 | — | 32 car, 8 motorcycle, 4 truck; nearly 2 t of debris | R23 |
| 16 | Vathy (Ithaca), GR, 2026 | Divers, 2 days | Not stated | Large items (no counts) | — | — | Tyres of various sizes, batteries, nets and fish traps, bicycles, chairs, cones, anchors, boat parts, metal, an almost whole plastic fishing boat; 1,300 kg | R25 |
| 17 | Selehovica Bay, Krk, HR (anchorage) | ROV at 1 m altitude; side-scan over 24,000 m² | 2.2 ha; 3–25 m | All litter, mostly bottles and cans | 305 → 384 | 130–170 (small litter) | Three hotspots ≥ 4 items/m²; rubber 2.6–4.4 % of items, half or more of them tyres; crates, boxes and baskets 0.6–2 % of plastics | R11 |
| 18 | 14 marinas and city ports, DE and North Africa | Camera on a pole | — | Litter particles | — | 1,000 (400–17,500) in German marinas; 7,000 North African; > 30,000 city centres | Plastics 73–74 %; ropes and textiles about 42 % in sport-boat marinas | R12, TD |
| 19 | Boothbay region, ME (lobster harbour) | Grapple, divers and sonar | — | Ghost traps removed | > 1,000 from Boothbay Harbor in 2021; on course for > 4,000 by end 2024 | — | Nearshore; valleys and backs of ridges after storms; traffic lanes; gear balls up to about 3,700 lb; a crew hauls 30–40 traps a day | R15, HD |
| 20 | Plymouth Sound, UK | Side-scan, divers | — | Tyres, cables, junk | "Several hundred" tyres identified; 500 lifted by Nov 2025; "hundreds" of cables in Jennycliff Bay | — | Dumping spot; tyres "alongside and within the seagrass"; partly or completely buried on sand and mud; a 60 m³ pile | R18, HD |
| 21 | Buzzards Bay and Cape Cod Bay, MA (open bays; trial) | Towed 132/445 kHz side-scan, 100 m range | 1.3 and 2.91 km²; depth unclear in the report | Placed and active lobster pots | — | 0.12–0.17 (trial pots, not derelict) | Pots set singly, at least 15.2 m apart; detected at a true-positive rate of 0.18 (0.32 on simple, 0.05 on complex floor) | R7 |
| 22 | Kagawa and the Seto Inland Sea, JP | Divers; fishers' collection | — | Seabed litter | — | — | By count: food wrappers and bags 67.7 %, metals 2.6 %, rubber 1.0 %; tyres and appliances occasional; litter buried under sand and mud; zones closed to trawling held 2.1 times the items; scattered deposition hotspots | R26, TD |

**What the table shows**
- Only one site (Thessaloniki) gives an all-class count of sonar-size items over a known harbour area.
- The other densities are single-class (pots), lifted items (lower bounds), or small litter.
- Composition is known by count for city harbours (Copenhagen) and partly for fishing shelters (Pomos, Paphos).

---

## 2. Density, and the 1–10 scale

### 2.1 What the numbers say (sonar-size litter per ha)

| Setting | Per ha | Sites |
|---|---|---|
| Open water, lagoon-wide, open bays | 0.03–0.4 | Venice lagoon 0.075; Clallam bays 0.03–0.37; York River 0.2 |
| Fishing grounds inside harbours, pots only | 0.6–1.4 | Port Gardner 0.64 (after removals) and 1.36 (before); Port Angeles West 1.34; Sarah Creek 0.6–0.8 |
| Urban and working harbour basins | 3–30 | Vieux-Port ≥ 2.7 (lifted items, lower bound); Venice Grand Canal 11.6; Thessaloniki 29.2 |
| Fender-tyre shelters, dumping spots | above 30, perhaps about 100 | Pomos (180 tyres in a small refuge) [CHECK area]; Plymouth dumping spots; a 60 m³ pile |

**Central value [DERIVED].** The geometric mean of the four harbour-basin values (1.34, 2.7, 11.6 and 29.2) is
5.9 per ha; two of the four are lower bounds (one class only, or lifted items only). Adding the fender shelter at
about 100 per ha (itself [CHECK]) gives 10.4. The targets doc's nominal of **10 per ha** sits between the two and is kept as level 5.

### 2.2 The scale [DESIGN, anchored]

The level s ∈ {1, …, 10} sets λ̄(s), the mean density of sonar-size litter (targets plus C1 debris; not
structures, not C2) in the **active band**: water within 50 m of a source feature (§4). Steps are geometric, ×10
from 1 to 5 and ×10 from 5 to 10:

- λ̄(s) = 10^((s − 1)/4) per ha for s ≤ 5;
- λ̄(s) = 10 · 10^((s − 5)/5) per ha for s ≥ 5.

| Level | λ̄ per ha | In a 0.8 ha tile | Reads as | Nearest documented case |
|---|---|---|---|---|
| 1 | 1.0 | 0.8 | Cleaned within the last year; strictly managed marina | Port Gardner after removals, 0.64 (pots only) |
| 2 | 1.8 | 1.4 | Managed harbour | Port Angeles West, 1.34 (pots only) |
| 3 | 3.2 | 2.5 | Lightly littered | Vieux-Port lifted items, ≥ 2.7 |
| 4 | 5.6 | 4.5 | | Four-site geometric mean, 5.9 |
| **5** | **10** | **8** | **Typical working harbour, never systematically cleaned** | Venice Grand Canal 11.6; five-site geometric mean 10.4 |
| 6 | 16 | 13 | | |
| 7 | 25 | 20 | Heavily littered port | Thessaloniki 29.2 (between 7 and 8) |
| 8 | 40 | 32 | | Targets doc "dense" preset, 30 |
| 9 | 63 | 50 | Neglected basin | |
| 10 | 100 | 80 | Fender-tyre shelter, dumping ground | Pomos [CHECK area] |

The "tile" column assumes the whole tile is in the active band, as for a tile against a quay.

**Throughput check.** The densest tile measured on the frozen simulator held 54 objects and ran at 0.64 × real
time [HD, freeze decision §4]. Tiles at levels 9–10 exceed it: measure before using them in campaigns.

**What changes with the level [DESIGN]**
- Every source intensity and the far field, by the same factor. The mix, kernels and poses do not change.
- C2 small litter, by the same factor: about 0.03 items/m² at level 1, 0.3 at level 5, 3 at level 10 (German
  marina minimum 0.04, German mean 0.1, North African mean 0.7, city centres above 3 [R12]).
- The chance that an emission event forms a group (tyre line, pile, gear ball): about 0.05 at level 1, rising
  to about 0.3 at level 10. Neglected harbours have piles; cleaned ones do not [findings 2 and 5].

**Class selection [DESIGN].** The level fixes the total. The selected target classes share the target part of it:
realistic weights of the profile renormalised over the selection, or equal counts ("balanced"). A deselected
class is absent; it does not reduce the total. The interface shows the expected count per class.

**Realism guard [DESIGN].** Warn, and write `stress_world: true` in the metadata, when a class's density over the
world exceeds the highest documented density for that class by more than a set factor (for example only drums at
level 10). Highest documented area-averages: pots 1.34 per ha (Port Angeles West); all litter 29 per ha
(Thessaloniki); tyres probably far higher in fender shelters [CHECK Pomos area]. The session sets the factors.

---

## 3. Per-class evidence and defaults

Class geometry, sizes, poses and burial ranges are in the targets doc §4 and are not repeated. Shares are
realistic weights per profile (§5).

| Class | Where it comes from | How it lies | Sizes and states (beyond the targets doc) | Gaps |
|---|---|---|---|---|
| **T6 tyre** | Fender lines [R22–R24]; dumping [R18]; urban quays [R19, R21]; tyre octopus traps [R3] | Along the quay toe below fender lines, often several together [DESIGN]; in piles at dumping spots [R18]; singles elsewhere | Car tyres dominate; motorcycle 8 and truck 4 of 44 wheels at Paphos [R23]; truck and tractor tyres in Plymouth and Marseille [TD, HD]; one 0.7 ± 0.1 m tyre at Mangalia [R8]. Partly buried on sand and mud [R18] | Truck tyres (0.9–1.1 m) only if the class accepts that size [CHECK] |
| **T4 trap or pot** | Fishing grounds crossed by traffic [R3, R15]; storms [R15]; fishing berths [TD] | Areal over the fishing ground, singles in Puget Sound and inside Maine's exemption line, pairs or more further out [R16, R17]; gear balls [R15]; set at least 15.2 m apart in one trial [R7] | Still fishing, hence intact, 37–82 % [R3–R5]; otherwise damaged, collapsed or more than half buried [R3] | Class is the 0.61 m Maryland pot [TD]; rectangular lobster traps and round Dungeness pots are other geometries. Bicycles, e-scooters and trolleys (90 % of Copenhagen's large items [R19]) are lattices that the targets doc maps acoustically to T4; whether a trolley variant exists is [CHECK] |
| **T7 net or rope bundle** | Fishing berths; angling piers [R18, R13]; mooring lines; ropes and textiles about 42 % of litter by count in sport-boat marinas [TD] | Near berths and piers; snagged on structures; gear balls; Puget Sound gillnets mostly on high-relief rock and boulders, most ≤ 1,000 m² [R14] | A 1.75 m rope at Mangalia [R8]; a half-buried net outside a pier [R13]; on sand only the lead line may show [TD] | Long nets (tens of metres [R13]) exceed the class's 2 m envelope |
| **T1 plastic box** | Abandoned crates [TD, Cherbourg]; waste containers [R21]; fish boxes at fishing berths [DESIGN] | Singles near berths and quays | Crates, boxes and baskets 0.6–2 % of plastic items in an anchorage [R11] | |
| **T2 drum** | Dumping; working quays [TD] | Singles, rarely piles | No harbour count found | Share is a pure [DESIGN] |
| **T3 concrete block** | Mooring sinkers, rubble, rip-rap debris [TD]; mooring blocks among the dropped objects [R9, R10] | Around mooring fields and quays | A rock mound by the channel [R8] (a feature, not a target) | |
| **T5 pipe** | Metal pieces and wires [R1]; cables [R9, R10, R18]; frames [R8]; rail track [R10] | Near working quays; long cables cross the basin [R18] | Frame 3.84 m [R8]; cables 2–4 cm thick [R9] | Cables of tens of metres are not a T5 shape |
| **C1 debris** | Scrap metal "en masse" [R21]; shipyard debris [R8]; rubble | Everywhere, more at working quays and breakwaters | | |
| **C2 small litter** | Every quay, berth and outfall; glass concentrated along one quay [R20] | Seabed layer | §2.2 | |
| **S1 to S3 structures** | Mooring fields; piles; dropped mooring blocks [R9, R10]; a buoy anchor 3.5 m across [R8] | Mooring chain at 2:1 to 3:1 scope in one harbour's mooring code [R27] | | Surface buoys may collide with the boat in Gazebo [CHECK] |

**Not in the object set, but common in the evidence** (record as gaps; adding a class is new geometry and a
separate owner decision): logs, poles and planks [R3, R9, R13]; bicycles and e-scooters [R19, R20]; long cables
and nets [R9, R13, R18]; lobster traps and round crab pots; car batteries [R21]; boats and dinghies [R18, R25].

---

## 4. Spatial model (recommendation)

### 4.1 Sources and kernels

Emission is tied to how a feature is used, not just to its presence. Kernels are [DESIGN] defaults with brackets.

| Source | Shape | Emits | Kernel | Arrangement | Evidence |
|---|---|---|---|---|---|
| Fender line (berth along a quay or pontoon) | Line | T6 mostly; T7, T1 | Half-normal from the wall, σ 2–4 m | Lines of 2–6 tyres along the toe; singles | R22–R24 |
| Urban quay, promenade, bridge | Line | Lattices (T4 stand-in), T6, T1, C1; much C2 | 0–10 m from the wall, mode at 2–4 m | Singles; one activity weight per quay (one side can be much dirtier) | R19, R20 |
| Working quay, shipyard, crane | Line | C1 mostly; T5, T3, T2, T6 | Exponential, L 10–20 m | Singles, small heaps | R1, R8, R10 |
| Fishing berth | Line | T4, T7, T1, T6 | Exponential, L 10–15 m | Singles, gear balls | TD, R15 |
| Marina pontoon or finger | Line | T7, T6, T1; small items under the boats | Exponential, L 5–10 m; the berth footprint | Singles | R12, R21 |
| Mooring field | Area | S1 and S2; T4 (cut lines); T7 | Uniform over the field, plus around each block (L ≈ 5 m) | Singles | R10, R15 |
| Traffic lane across a fishing ground | Band | T4 | Uniform in the band | Singles or pairs | R3, R15 |
| Fishing ground | Area | T4 mostly; T7 | Uniform with patchiness | Singles, pairs, short trawls | R3–R5, R17 |
| Dumping spot | Point | Mixed piles: T6, C1, T7, T4, T5 | Radius 3–10 m | Overlapping piles | R18 |
| Breakwater or rip-rap toe | Line | C1, T3; T7 snagged | Exponential, L 5–10 m | Singles | TD, R1 |
| Log boom, river mouth, outfall | Point or area | Wood (gap), C2, C1 | Exponential, L 20–50 m | Scattered | R3, R9 |
| Angling pier, slipway | Line or point | T7 (lines), C2, T6 | Exponential, L ≈ 5 m | Singles | R13, R18 |
| Far field | Everywhere | All, few | Uniform: 0.5–10 % of λ̄ | Singles | Venice lagoon-wide against the Grand Canal: 0.6 % [DERIVED 0.075 / 11.6] |

The targets doc's single-kernel model (decay 15 m, bracket 5–40; 70 % of items within 30 m of a hotspot, bracket
0.5–0.9) is the special case with one source type. Keep it reproducible as the `generic` profile.

### 4.2 Redistribution (optional)

- **Storms.** A fraction of the light items (crates, nets, ropes) moves into local lows and to the lee of ridges,
  walls and breakwaters [R15]. [DESIGN] Move 0–30 % of light items down the local depth gradient by a few
  metres, or weight their placement by the local concavity of the floor.
- **Propeller scour** at berths keeps its own floor bare and pushes items to the rim [TD Venice scours; DESIGN].
- **Sheltered corners** collect fine sediment, so burial is deeper there; scoured berths bury less [DESIGN].

### 4.3 What to measure on generated worlds

- Realised density in the active band against λ̄(s), per level.
- Share of items within 30 m of a source (targets doc 0.7, bracket 0.5–0.9).
- Distance to the nearest source, per class.
- Clustering: the Clark–Evans ratio or the pair-correlation function at 2–20 m. Real litter is clustered
  (findings 1–4), so the generated ratio should be below 1.
- Class shares against the profile.
- Share of targets effectively buried, per sediment.

---

## 5. Harbour profiles

A profile gives default use tags to the world's features and a default class mix. All weights are [DESIGN],
informed by §1; the session sets the final values and records why. Order: T1 box, T2 drum, T3 block, T4 trap,
T5 pipe, T6 tyre, T7 net.

| Profile | Default sources | Realistic target mix (T1…T7) | Target share among sonar-size items | Typical levels | Candidate harbours [HD] |
|---|---|---|---|---|---|
| `fishing_harbour` (small fishing shelter, Mediterranean or Japanese type) | Fender lines, fishing berths, slipway, breakwater toe | 0.10 / 0.02 / 0.05 / 0.10 / 0.03 / 0.50 / 0.20 | 0.6 | 5–8 | Cherbourg fishing port; the Pomos and Paphos type; our Shiraishi-jima harbour, if it is one [PROJ, CHECK] |
| `trap_harbour` (lobster or crab harbour) | Fishing ground, traffic lanes, mooring field, fishing berths | 0.05 / 0.01 / 0.04 / 0.65 / 0.03 / 0.07 / 0.15 | 0.7 | 3–6 | Provincetown, Boothbay and Rockland, Port Angeles, Port Gardner, Charleston, Boston Harbor (ghost-gear surveys) |
| `city_harbour` (city-centre quays, canals) | Urban quays, bridges, marina pontoons | 0.15 / 0.02 / 0.05 / 0.35 / 0.05 / 0.25 / 0.13 | 0.6 | 4–8 | Marseille Vieux-Port, Venice canals, Plymouth waterfront |
| `marina` (sport-boat marina) | Pontoons, mooring field, fuel dock, slipway | 0.15 / 0.01 / 0.10 / 0.05 / 0.04 / 0.35 / 0.30 | 0.5 | 2–5 | Chantereyne (Cherbourg), Plymouth marinas |
| `working_port` (commercial quays, shipyard) | Working quays, cranes, mooring blocks, breakwater | 0.05 / 0.05 / 0.15 / 0.05 / 0.25 / 0.30 / 0.15 | 0.4 | 5–8 | Thessaloniki, Mangalia, New Bedford, Cherbourg commercial port, Boston's inner harbour, Port Angeles piers |
| `generic` (targets doc §5.2) | One hotspot type | 0.15 / 0.05 / 0.10 / 0.15 / 0.10 / 0.30 / 0.15 | 0.5 | any | Reproduces the session-02 campaign worlds |
| `auto` | From each feature's own tag | Mix by source | By source | any | Real harbours with several uses |

- **`city_harbour` T4 share (0.35)** stands for bicycles, e-scooters and trolleys. If no lattice variant exists,
  either keep T4 as a stand-in and write `stand_in_for` in the metadata, or lower the share. Owner's choice.
- **Pacific Northwest harbours** have wood as their main false-alarm source [R3]. It is not in the object set:
  record it as a gap in their world cards.

---

## 6. Groups, poses, burial and fouling

- **Groups [DESIGN].**
  - Tyre lines: 2–6 tyres, 0.5–2 m apart along a fender line.
  - Dumping piles: 5–30 items within a 2–6 m radius, overlapping allowed.
  - Gear balls: 2–10 traps with rope, entangled [R15].
  - Trap pairs and short trawls: 2–3 traps inshore [R17]; groundline spacing not found [CHECK].
  - Nets draped over structures (P2 in the targets doc).
- **Ground truth for groups.** Each member keeps its own record; the group gets a `group_id` and a type. How a
  detection on a group is scored (one opportunity per group or per member) is a rule for the evaluation code,
  not for the generator: propose it, do not implement it in other modules.
- **Minimum spacing.** The targets doc keeps targets at least 3 m apart so that their echoes do not merge. Apply it
  between singles and between groups, not inside a group.
- **Trap state [DESIGN from R3–R5].** Intact and upright about 0.6 (bracket 0.4–0.8); damaged or collapsed
  about 0.25; on a side about 0.15.
- **Burial.** The targets doc's per-class ranges and presets; optionally deeper in sheltered mud corners and
  shallower in scoured berths (§4.2).
- **Fouling.** Keep the frozen mechanism (the validity conditions draw it per world, U(0.2, 1.0)). If per-object
  fouling exists, older items are more fouled.

---

## 7. What the mode needs from the harbours doc

The harbours doc is a download guide; the session does not need it. These are the rules that affect placement:

- **World sizes.** Tiles 0.6–1 ha, at least about 70 m on each side (one 20-minute two-pass mission); large
  worlds 5–25 ha; showcase worlds 25–100 ha and more. Placement must scale to about 100 ha (about 1,000 objects at
  level 5).
- **Depth band.** Water 4.65–7.15 m deep at the world's water level, so the sonar altitude is 4.5–7.0 m with
  a transducer depth of 0.15 m [CHECK the frozen value on the freeze card]. At least 95 % of a world in the band;
  weak-class worlds at 4.65–6.15 m. Place targets only inside the band.
- **Depth from real grids.** Source water up to about 12 m is mapped into the band by a recorded shift
  (d′ = d − Δ) or compression (d′ = 4.65 + k (d − d_lo), with k = 2.5 / (d_hi − d_lo)). Prefer the shift; keep
  k ≥ 0.5 where possible. The transform applies to the depth layer before objects are placed, so rest poses and
  burial follow the new floor.
- **Clean grids.** Real grids are low-passed (σ ≈ 2–3 m), holes filled and real objects masked before use: an
  unfiltered tyre in a survey grid becomes an unlabelled target. Quays and walls come from the chart or
  OpenStreetMap.
- **Water level.** Chart depths refer to a low datum; record the world's water level above it (η), the
  smoothing σ and the transform with the world.
- **Splits.** Prefer dataset and evaluation worlds from different harbours (for example dataset: Provincetown,
  Boston, Venice; evaluation: Port Angeles, Plymouth, Boothbay), on top of disjoint seed ranges.
- **Documented positions** (1000 Tyres GIS, pot lists, the Venice litter map) may check the model's densities.
  They are never labels.
- **Open question carried over.** Can the headless campaign generator read a depth file (harbours doc §2.1)? If
  not, the world builder has to gain that ability before real-harbour datasets can be made without the GUI.

---

## 8. Gaps and resource requests

| Missing | Why it matters | Where it may be |
|---|---|---|
| Area of the Pomos refuge (and of other clean-up sites) | Turns 180 tyres into a density; level-10 anchor | OpenStreetMap or satellite imagery (the session can measure it) |
| A second all-class count of sonar-size items over a known harbour area | Only Thessaloniki has one; it anchors levels 5–8 | Papatheodorou et al. (University of Patras), target list behind [R1] |
| Distance of litter from quay walls | Every kernel width here is [DESIGN] | Diver clean-up logs with GPS (Marseille nautical clubs, Copenhagen); pre-dredging target lists |
| Tyre positions in Plymouth Sound | Checks the fender and dumping kernels | SHIPS Project, 1000 Tyres GIS |
| Derelict-pot positions in Puget Sound | Checks areal sources; singles against strings | Northwest Straits Foundation, Snohomish MRC |
| Groundline spacing of inshore lobster trawls | Trap strings | Maine DMR; lobstermen's associations |
| Sonar-size counts in Japanese fishing ports | A Japan profile | Okayama and Kagawa prefectures; fisheries cooperatives (Ushimado collects seabed litter [TD]) |
| Count of sonar-size objects in our own harbour recordings | A local anchor | July 2026 Shiraishi-jima data (read-only; owner approval; outside any training or held-out role) |
| How often drums occur in harbours | T2 share | Port authority debris reports |
| Per-class counts of the Venice survey; per-marina tables of Schernewski 2023 | Profile weights | Full texts [TD REF 3, REF 6] |

---

## 9. References

How read: **[full]** full text; **[page]** web page or abstract only; **[partial]** part of a long document.
All read on 2026-11-19.

1. Papatheodorou, G., et al. (2021). Benthic megalitter detection using unmanned surface vehicle (USV) and
   automatic target detection: a case study in the Port of Thessaloniki. *CEST2021*, paper 00599.
   https://cms.gnest.org/sites/default/files/Proceedings/cest2021_00599/cest2021_00599.pdf [full]
2. Madricardo, F., et al. (2019). Assessing the human footprint on the sea-floor of coastal systems: the case of
   the Venice Lagoon. *Scientific Reports* 9:6615. doi:10.1038/s41598-019-43027-7 [via the targets doc, REF 3;
   not re-read: the PMC page was behind a browser check]
3. Natural Resources Consultants (2003). Clallam County derelict gear removal, final report (Northwest Straits
   project CLA-2003-0009; survey of June–July 2003).
   https://nwstraits.org/sites/default/files/2025-10/cla-2003-0009-derelict-gear-removal-nrc-final-report.pdf
   [full]
4. Snohomish County Marine Resources Committee (2008). 2008 Port Gardner derelict fishing gear removal.
   https://www.snocomrc.org/media/1306/2008-port-gardner-derelict-fishing-gear-removal.pdf [full]
5. Havens, K. J., Bilkovic, D. M., Stanhope, D., Angstadt, K., & Hershner, C. (2008). The effects of derelict blue
   crab traps on marine organisms in the lower York River, Virginia. *North American Journal of Fisheries
   Management* 28, 1194–1200. https://ccrm.vims.edu/publications/pubs/Havens%20et%20al%20NAJFM08.pdf [full]
6. Bilkovic, D. M., Havens, K., Stanhope, D., & Angstadt, K. (2014). Derelict fishing gear in Chesapeake Bay,
   Virginia: spatial patterns and implications for marine fauna. Abstract record: https://oneearth-oneocean.com/research/2014/donna-marie-bilkovic-kirk-havens-david-stanhope-kory-angstadt-derelict-fishing-gear-in-chesapeake-bay-virginia-spatial-patterns-and-implications-for-marine-fauna/
   [page: "widely distributed, but with notable hotspot areas"; almost 32,000 pots recovered, 2008–2012]
7. ScanPot project (2017). Development of side-scan sonar methodology to survey derelict lobster pots. Report
   published on mass.gov. https://www.mass.gov/files/2017-08/2017%20ScanPot%20Lobster%20Pot%20Report_0.pdf [full]
8. Lazăr, A. V., Bugheanu, A. D., Ungureanu, G. V., & Ionescu, A. D. (2013). Side-scan sonar mapping of
   anthropogenically influenced seafloor: a case-study of Mangalia Harbor, Romania. *Geo-Eco-Marina* 19, 59–64.
   https://journal.geoecomar.ro/geo-eco-marina/article/download/04_2013/51 [full]
9. Maine Department of Marine Resources. Saco River submerged debris, preliminary report.
   https://www1.maine.gov/dmr/sites/maine.gov.dmr/files/docs/Saco_River_submerged_debris_preliminary_report_final.pdf
   [full; date not noted]
10. City of New Bedford Harbor Development Commission (2012). Draft final sampling and analysis plan, Lower Harbor
    CAD cell, with the 2001 marine geophysical survey report. https://semspub.epa.gov/work/01/507227.pdf [partial]
11. Maglić, L., Maglić, L., & Blažina, A. (2025). Nautical tourism vessels as a source of seafloor litter: an ROV
    survey in the North Adriatic Sea. *Journal of Marine Science and Engineering* 13(6), 1012.
    https://www.mdpi.com/2077-1312/13/6/1012 [full]
12. Schernewski, G., et al. (2023). Marine macro-litter (plastic) pollution of German and North African marina and
    city-port sea floors. *Applied Sciences* 13(20), 11424. https://www.mdpi.com/2076-3417/13/20/11424 [page; full
    text read for the targets doc, REF 6]
13. ReFish project (2026). Diving for lost fishing gear in Finland and Sweden. Interreg Central Baltic.
    https://centralbaltic.eu/wp-content/uploads/2026/08/ReFish_Diving-for-Lost-Fishing-Gear-in-Finland-and-Sweden.pdf
    [full]
14. The haunted fjord: patterns and implications of ghost nets in Puget Sound and the Northwest Straits. American
    Fisheries Society annual meeting 2011, abstract 123-8. https://afs.confex.com/afs/2011/webprogram/Paper3403.html
    [page]
15. Boothbay Register. Ghost (gear) hunting. https://www.boothbayregister.com/article/ghost-gear/266899 [page]
16. NOAA Fisheries. Minimum traps per trawl, Northeast lobster and Jonah crab trap/pot fisheries.
    https://www.fisheries.noaa.gov/new-england-mid-atlantic/marine-mammal-protection/minimum-traps-trawl-northeast-lobster-jonah-crab-trap-pot-fisheries
    [page]
17. Fishermen's Voice (September 2014). Trawling up. https://fishermensvoice.com/archives/201409TrawlingUp.html
    [page]
18. The SHIPS Project, 1000 Tyres Project pages: about.html, record.html, whelansreef.html,
    gallery_waterfront3.html, under https://1000tyres.org.3hconsulting.com/ [page]
19. The Copenhagen Post (1 July 2020). Scooters out: Copenhagen Harbour cleaning ends. https://cphpost.dk/?p=115538
    [page]
20. Made in Marseille (October 2019). Nettoyage du Vieux-Port 2019 (with the 2018 results).
    https://madeinmarseille.net/a-la-une-marseille/54049-nettoyage-vieux-port-2019/ [page]
21. Made in Marseille (2025). Des tonnes de déchets sorties du Vieux-Port par les sociétés nautiques.
    https://madeinmarseille.net/actualite/marseille/189356-des-tonnes-de-dechets-sorties-du-vieux-port-par-les-societes-nautiques/
    [page]
22. Cyprus Mail (1 November 2022). Over 1,500 kilos of rubbish removed from Pomos harbour.
    https://cyprus-mail.com/2022/11/01/over-1500-kilos-of-rubbish-removed-from-pomos-harbour [page]
23. Cyprus Mail (17 November 2025). Tonnes of underwater debris removed in Paphos.
    https://cyprus-mail.com/2025/11/17/tonnes-of-underwater-debris-removed-in-paphos [page]
24. Guide Me Malta (9 October 2019). Local eco groups plead to ban tyre fenders on local beaches.
    https://www.guidememalta.com/en/watch-local-eco-groups-plead-to-ban-tyre-fenders-on-local-beaches [page]
25. Euronews (28 October 2026). Ithaca's natural horseshoe harbour is picturesque, but it hides a dirty secret.
    https://www.euronews.com/2026/10/28/ithacas-natural-horseshoe-harbour-is-picturesque-but-it-hides-a-dirty-secret
    [page]
26. Ministry of the Environment, Japan (about 2022). Chapter II of a report supporting a manual for marine-litter
    collection by fishers. https://www.env.go.jp/content/000192517.pdf [full]
27. threeharbors.com. Mooring specification sheet (Wychmere Harbor, Allen Harbor, Round Cove, Herring River).
    https://www.threeharbors.com/mooringspecsheet.htm [page; table garbled]

**Project sources:** `target_families_and_surrogates.md` v2 (2026-11-14); `real_harbour_bathymetry_candidates.md`
v2 (2026-11-19); `SIM_VALIDITY_CONDITIONS.md` v1.0; the freeze decision (2026-11-18).
