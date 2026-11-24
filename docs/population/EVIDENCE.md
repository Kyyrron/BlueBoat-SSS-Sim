# Harbour seabed litter: the evidence behind the mission-like placement

*Session 14, 2026-11-21. It verifies and extends the companion `docs/inputs/14_companion_harbour_object_populations.md`
(version 1, 2026-11-19). Every number here was read on 2026-11-21 in the source named, or computed from such numbers
(the computation is shown). The reading notes, with verbatim quotes and page references, are in
`~/ros2_ws/sim_runs/worldgen_v11/evidence/notes_1_anchors.md` to `notes_4_conversions_checks.md`; downloaded copies
are in `~/ros2_ws/sim_runs/worldgen_v11/downloads/`. The model's numbers, each with its tag, bracket and sources, are
in `blueboat_harbour_population/blueboat_harbour_population/data/evidence_parameters.yaml`; the model is described in
`POPULATION_MODEL.md` beside this file.*

**Tags** (as in the parameter file): **REF** read in a source; **DERIVED** computed from referenced values (shown);
**DESIGN** a modelling choice with a default, a bracket and a reason; **CHECK** not verified.
**How read:** [full] the whole text; [partial] part of it; [page] a web page or an abstract only.
"Sonar-size" means about 0.5 m or more (the size floor of the object set).

---

## 1. What changed against the companion

| Companion statement | Verified finding | Consequence for the model |
|---|---|---|
| Thessaloniki: 56 anomalies in 1.92 ha = 29.2 per ha | Confirmed [R1, full]. 8 of the 56 are 0.3–0.4 m (read from its Fig. 2d), so **25 per ha at ≥ 0.5 m** | Level 7–8 anchor stays; quote 25 per ha for sonar-size |
| Pomos: 180 tyres "in a small refuge", about 100 per ha [CHECK] | Counts confirmed [R22, page]. Basin measured on OpenStreetMap: **0.36 ha** (0.26–0.47), berthing frontage about 120 m, 20–25 vessels [DERIVED] → **about 500 tyres per ha** (380–690), **7 to 9 per berth**, about **1.5 per metre of quay** | Level 10 (100 per ha) is not the ceiling of documented harbours: a never-cleaned fender shelter is 5 times denser, over a basin that is all fender line. Reproduced by level 10 with a fender line's activity raised (§2) |
| Port Angeles West: "pots at about 9–29 m, many near 10 m" | Appendix 2 lists every pot [R3, full]: **median depth 13.4 m**, 90 % between 9 and 18 m | Depth plays no part in the model; noted |
| Port Angeles: pots "more than half buried" | That phrase is a rule of the removal plan (Appendix 1), not an observation. No buried pot is reported in Clallam County [R3] | Burial evidence for traps comes from North Carolina instead: 51 % of 92 derelict pots buried, mean depth 7.9 cm [R30, full] |
| Marseille 2019: 92 e-scooters [CHECK: "event cancelled"] | The 2019 edition took place; the "cancelled" caption is a re-used photograph about 2020 [R20, R21, page] | Resolved |
| Venice: about 40 km² surveyed | 40 km² is the lagoon's channel area; the surveyed area is not stated [R2, full] | The lagoon-wide 7.5 per km² stays a far-field reference only |
| Venice: per-class counts in the full text | **Not there**: neither the article nor its supplement gives counts per class [R2, full]. Its Fig. 5 says tyres, "commonly used as fenders", are the dominant macro-litter | Tyres dominate the city and fishing profiles |
| Schernewski 2023: litter "stays near its emission point", storms "up to 4 km" | These are particle-model results for small plastics, which the authors call a strong simplification [R12, full] | Not applied to heavy objects. What R12 does show: every sample lies within about 3 m of a quay or pier, so its densities are quay-foot densities |
| Small litter about 100 times denser than sonar-size litter | In the Krk anchorage it is **16 to 28 times** [R11, full] | Level 5: 600 small items per ha against 10 sonar-size (60 times): inside the bracket of the measured ratios once quay-foot densities are counted |
| Only one all-class count over a known harbour area | Two more, both archaeological sonar assessments of never-cleaned small UK harbours: **Weymouth 0.64 per ha proud (2.0 with buried ferrous)** [R33, full], **St Mary's (Scilly) 0.50 per ha, 0.20 per ha man-made** [R32, full] | Level 1–2 anchors; both lower bounds (kelp masking, survey rectangles include land) |
| Highest documented trap density 1.34 per ha | **3.4 probable (4.0 recovered) per ha** in the densest polygons of the Mullica estuary [R31, full] | Realism guard for traps raised to 4.0 per ha |
| Groundline spacing of inshore lobster trawls: not found | Maine, zones A–G: inshore **singles, pairs and triples**; **5 to 20 fathoms (9 to 37 m)** of groundline between traps, most often 10 to 15 fathoms (18 to 27 m) [R39, full] | Trap pairs: 2–3 traps, 9–27 m apart (REF) |
| No spatial statistics of seabed litter | None published (Ripley's K, pair correlation, Clark–Evans) for seabed macro-litter or derelict gear. **Derived here** from two published position lists: Port Angeles pots and Plymouth tyres (§4) | The model's clustering is fitted to, and checked against, those two patterns |
| Kernel widths are design values | Still true for quays: **no source gives a measured distance of sonar-size litter from a quay wall.** Two measured scales are upper bounds (§4.1) | Quay kernels stay DESIGN, with the shape now evidenced |

## 2. Density anchors and the 1–10 scale

The level sets the mean density of sonar-size litter (targets and debris; not structures, not small litter) over the
**active band**: placement water within 50 m of a source feature (15 m of a point feature). λ̄(s) = 10^((s−1)/4)
per ha up to level 5, 10·10^((s−5)/5) from 5 (the companion's scale, kept).

| Setting | Per ha | Site and how it was counted | Src |
|---|---|---|---|
| Never-cleaned small harbours, all sonar targets | 0.20–0.64 (2.0 with buried ferrous) | St Mary's, Scilly, 15 anomalies in ≤ 30 ha (6 man-made); Weymouth harbour mouth, 11 sonar-visible sites, 37 in all, ≤ 18.6 ha | R32, R33 |
| Fishing grounds inside harbours, pots only | 0.64–1.34 (to 3.4–4.0 in hot polygons) | Port Gardner 0.64 (after removals; 1.36 before); Port Angeles West 1.34; North Carolina water bodies 0.06–3.0, mean 1.05; Mullica estuary 1.07 mean, 0.28–3.4 by polygon | R4, R3, R30, R31 |
| Lagoon-wide | 0.075 | Venice lagoon channels | R2 |
| Urban canal and harbour basins | 2.7–25 | Vieux-Port lifted items ≥ 2.7 (lower bound); Venice Grand Canal 11.6; Paris Canal Saint-Martin about 24 Vélib' per ha after 8.5 years [DERIVED: about 100 in about 4.1 ha]; Thessaloniki 25 (≥ 0.5 m) to 29 | R20, R2, R38, R1 |
| Tyre dumping grounds, densest areas | 17.6–37 | Plymouth Sound: 936 mapped tyres; 37 per ha in the densest hectare, 17.6 per ha over the densest 9 ha [DERIVED from the position file] | R18 |
| Fender shelter | about 500 | Pomos: 180 tyres in a 0.36 ha basin [DERIVED, OSM area] | R22 |

**Central value [DERIVED].** Geometric mean of the basin values 1.34, 2.7, 11.6 and 25: **6.3 per ha**; with the
fender shelter (500): **13.9 per ha**. The nominal **10 per ha** of the targets document lies between and is kept as
level 5. Level 1 (1 per ha) sits on the cleaned fishing grounds and the never-cleaned UK harbours; level 7–8 on
Thessaloniki and the densest 9 ha of Plymouth Sound; level 10 (100 per ha) between the densest Plymouth hectare and
the Pomos basin.

**Cross-checks of the scale (no parameter is set from them):**
- Amsterdam: 12,000–15,000 bicycles recovered a year from about 100 km of canals [R37, page] = 12–15 per 100 m of
  canal per year [DERIVED]; a canal 25 m wide holds a quay band of about 12 m on each side, so a yearly stock of
  lattice objects of about 50 per ha in the band: **level 9 for the stand-in class alone**. City canals sit at
  levels 7–10.
- Pomos: 7–9 tyres per berth over 39 years of no cleaning [R22, DERIVED]; Paphos: 32 car, 8 motorcycle and 4 truck
  wheels in a two-hour dive [R23, page] (car 73 %, motorcycle 18 %, truck 9 % [DERIVED]); Malta, St George's Bay:
  104 tyres in one 4-hour dive (October 2020) and over 400 more in four sessions of one month (2021), "many more
  still" [R42, page]. Fender tyres dominate every Mediterranean fishing-harbour clean-up read.
- In Japan most seabed tyres were boats' own fenders, lost at the berth and under way [R43, partial].

**What the level does not set:** the class mix, the kernels and the poses. It sets every source's budget, the far
field (2 % of the level's density outside the band [DESIGN; Venice lagoon-wide over Grand Canal = 0.65 %]), the
small litter (the frozen R2 presets low / nominal / high at levels 1 / 5 / 10, geometric between) and the chance that
an emission event is a group (0.05 at level 1 to 0.30 at level 10 [DESIGN]).

## 3. Case table: what was counted where

New or corrected cases only; the companion's §1 lists the others, all re-read (R1–R27).

| Site (type) | Method | Area or length | Counted | Count | Per ha / per 100 m [DERIVED] | Spatial notes | Src |
|---|---|---|---|---|---|---|---|
| St Mary's Harbour, Scilly (small harbour, never cleaned) | Side-scan, 2004, pre-construction | ≤ 30 ha | All anomalies | 15 (4 apparent + 2 possible man-made) | 0.50; 0.20 man-made | All 6 man-made in one 47 × 43 m patch at the quay head | R32 |
| Weymouth harbour mouth, UK (marina site) | Side-scan + magnetometer, 2007 | ≤ 18.6 ha | Anomalies | 11 sonar-visible sites, 37 in all | 0.6; 2.0 with buried ferrous | Larger objects at the pier and channel edges | R33 |
| Mullica River–Great Bay, NJ (estuary trap fishery) | Side-scan, 50 m range | 20.78 km² | Probable derelict traps | 2,218 | 1.07; 0.28–3.4 by polygon | 12× between polygons | R31 |
| North Carolina sounds (estuarine trap fishery) | Side-scan, 201 cells of 2.4 ha | 482 ha scanned [DERIVED] | Derelict crab pots | 106 | 1.05 mean; 0.06–3.0 by water body | No difference within 50 m of shore; 51 % of 92 pots buried, mean 7.9 cm | R30 |
| Sydney Harbour piers (urban estuary) | Diver transects | 5 piers × pier and off-pier transects | Benthic debris (mostly small) | — | 1.22 per m² under piers, 0.12 per m² about 50 m off the pier end | 10× at the pier | R28 |
| Florida Keys reefs | Diver transects of 100 m | — | Debris position along each transect | — | — | 36–47 % of debris (by class) within 10 m of the reef edge | R29 |
| Vigo, dársena 2 Berbés (A Laxe) (working basin) | Divers, three days | Berth perimeter | Waste by weight | 74 tyres; 18.3 t in all | Tyres 9.5 t and rubber fenders 3.3 t: **70 % of the weight** [DERIVED] | "the whole perimeter of the quay where the berths are"; many items "half buried in the mud" | R34 |
| Korea, 129 sites (national, 1999–2000) | Side-scan, converted to weight | — | All debris | ≥ 34,636 t | Ropes 26 %, metal 23 %, wire rope 16 %, wood 8 %, tyres 5 % by weight | — | R35 |
| Korea, East Sea, 39 ports (2021) | Survey (abstract read only) | — | All debris | — | Tyres 58 % by weight | Tyres dense at breakwaters and berthing facilities; plastics also outside the port | R36 |
| Paris, Canal Saint-Martin (urban canal), 2016 | Drained, 14 years since the last | About 1.5 km, about 4.1 ha [DERIVED from 90,000 m³ / 2.2 m] | Vélib' (first week); waste | About 100; 300 t | About 24 Vélib' per ha; 7 per 100 m of canal | Bicycles mostly in less frequented stretches | R38 |
| Amsterdam canals | Grab boat, yearly | About 100 km | Bicycles | 12,000–15,000 per year | 12–15 per 100 m of canal per year | Most "at the bicycle racks" | R37 |

## 4. Spatial pattern

### 4.1 Distance from structures

| Statement | Src | Grade | What it supports |
|---|---|---|---|
| Small debris 1.22 per m² under piers against 0.12 per m² about 50 m off the pier end | R28 | Measured, two points | An exponential with L ≤ 21.5 m from the structure [DERIVED: 50 / ln 10.2]: an upper bound for the quay kernels (3–15 m) |
| 36–47 % of debris within 10 m of a reef edge | R29 | Measured (rugose natural edge) | Storm redistribution against rugose edges, L about 16–23 m [DERIVED]: the breakwater toe (7 m) and the optional storm move |
| All man-made targets of St Mary's in one patch at the quay head | R32 | Measured from coordinates, small n | Emission at particular berths, not evenly along a quay: the **hot spots along a source** of the model |
| Divers cleared "the whole perimeter of the quay where the berths are" | R34 | Primary, qualitative | A band at the berth face |
| Tyres dense at breakwaters and berthing facilities | R36 | Abstract | Fender-line and breakwater sources for tyres |
| Derelict pots: no difference between "within 50 m of shore" and other habitats | R30 | Measured null | Traps are an areal source with no shore kernel: the fishing ground |
| Bicycles "at the bicycle racks"; most in less frequented stretches | R37, R38 | Anecdotal | Point-like emission along city quays: hot spots |

**No source gives a measured distance of sonar-size litter from a quay wall.** Every quay kernel width stays a design
value (bracket in the parameter file); what is now evidenced is the shape (a band at the berth face, emission at
particular places along it) and two measured scales that bound it from above.

### 4.2 Mapped patterns (derived here from published position lists)

| Pattern | Density | Median nearest neighbour | Single when points closer than r are joined | Clark–Evans | Src |
|---|---|---|---|---|---|
| Derelict crab pots, Port Angeles Harbor West (206 positions) | 1.34 per ha | 17 m | 63 % at r = 10 m (129 singles, 26 pairs, 4 triples, 3 fours) | about 0.62 | R3 Appendix 2 |
| Tyres, Plymouth Sound (936 positions) | to 37 per ha (densest hectare) | 8.4 m | 65 % at r = 5 m; groups up to 15 | — | R18 position file |
| All sonar targets, St Mary's (15) | 0.5 per ha | 7 m | 10 of 15 have a neighbour within 10 m | 0.70 (0.62 edge-corrected) | R32 gazetteer |
| Sonar-visible targets, Weymouth (11 sites) | 0.6 per ha | — | — | 1.00 (0.87 edge-corrected) | R33 gazetteer |

Between areas: 12× between the polygons of the Mullica estuary [R31]; 50× between North Carolina water bodies [R30];
134 against 0 pots per km² between Port Angeles West and East [R3]. Counts per area are over-dispersed: variance over
mean 91–142 between Mullica polygons [DERIVED, R31]. The standard count model in the litter literature is the
negative binomial [R28, R29]; neither paper prints its dispersion in the text read.

How the model meets these: §6 of `POPULATION_MODEL.md` (the fitted fishing-ground hot spots reproduce Port Angeles; the
fishing-harbour tyres reproduce Plymouth at level 5).

## 5. Per class

| Class | Where it comes from | How it lies | What the evidence adds | Src |
|---|---|---|---|---|
| T6 tyre | Fender lines (quay-hung, or the boats' own); dumping | Along the berths; tyre lines; piles at dumping spots | Dominant by weight in every Mediterranean and Korean port read (Vigo 52 %, Marsamxett Harbour 81 %, East Sea 58 %); Pomos 7–9 per berth; Paphos wheels car 73 % / motorcycle 18 % / truck 9 %; Plymouth recovery log car about 80 %, truck class about 19 % | R34, R42, R36, R22, R23, R18 |
| T4 trap | Fishing grounds; lines cut by traffic | Areal, in tight hot spots; singles, pairs, triples inshore | Groundline 9–37 m; 51 % of North Carolina pots buried (mean 7.9 cm); Port Gardner 92 % of dived targets were pots; Dungeness pots set 61–91 m apart, each on its own buoy, "sanded down" under 15 m [R44] | R39, R30, R4, R44 |
| T4 as stand-in | Bicycles, e-scooters, trolleys at city quays | Near racks and promenades; quiet stretches | 90 % of Copenhagen's large items; Amsterdam 12,000–15,000 bicycles a year; Paris 24 Vélib' per ha | R19, R37, R38 |
| T7 net or rope | Fishing berths, mooring lines, angling piers | Near berths; snagged | Ropes 26 % and wire rope 16 % of Korean port debris by weight | R35 |
| T1 plastic box | Fish boxes, containers | Near berths | Containers 0.8 % of the Vigo weight; crates 0.6–2 % of the plastic items at Krk | R34, R11 |
| T2 drum | Working quays, dumping | Singles | **Still no harbour count**: its share stays a pure DESIGN value | — |
| T3 block, C1 debris | Rubble, sinkers, scrap | Working quays, breakwater toes, moorings | Metal 23 % of Korean port debris by weight; mooring sinkers are meant to be **buried** to hold [R41] | R35, R41 |
| T5 pipe | Working quays | Singles | Metal pieces and wires at Thessaloniki | R1 |
| S1–S3 structures | Moorings, pontoons | Mooring fields | Swing radius about 1.7 D + L: 16.5–23.6 m for 5–8 m of water and 8–12 m boats [R40]; blocks of 1–2 t for boats up to 9 m [R41]; guide-pile spacing is left to each project's structural design in every design guide read [R45] | R40, R41, R45 |

## 6. Gaps

Classes the evidence wants and the object set lacks (each is new geometry and a separate owner decision; the world
card of every world names those its sources would have emitted): wood (logs, poles, planks: the main false-alarm
source of the Puget Sound harbours [R3]); bicycles and e-scooters as shapes (stood in for by `trap_pot`, recorded as
`stand_in_for`); long cables, chains and nets over 2–3 m; lobster traps and round Dungeness pots (the class is a
square wire pot); truck and tractor tyres over 0.8 m (9–19 % of the tyres where types were logged); car batteries;
boats and dinghies; damaged or collapsed traps (only the pose is modelled); fouling per object (one value per world).

Evidence still missing, and who probably holds it: `~/ros2_ws/sim_runs/worldgen_v11/RESOURCE_REQUESTS.md`.

## 7. References

R1–R27 are the companion's sources (its §9), all re-read on 2026-11-21 except where stated; R28 on were found in this
session. **TD**: `target_families_and_surrogates.md` v2 (superproject root). **HD**: `real_harbour_bathymetry_candidates.md`
v2, through the companion §7 only. **FRZ**: the frozen simulator (`SIM_VALIDITY_CONDITIONS.md`, its code). **S02**:
the simulator's session-02 population code (`worldgen/population.py`). **MODEL**: this model's own statistics
(`~/ros2_ws/sim_runs/worldgen_v11/stats/`).

| Key | Source | How read |
|---|---|---|
| R1 | Papatheodorou G. et al. (2021). Benthic megalitter detection using USV and automatic target detection: the Port of Thessaloniki. CEST2021, paper 00599. Read from the Internet Archive copy of cms.gnest.org/sites/default/files/Proceedings/cest2021_00599/cest2021_00599.pdf (the live URL returns 404) | [full] |
| R2 | Madricardo F. et al. (2019). Assessing the human footprint on the sea-floor of coastal systems: the Venice Lagoon. Scientific Reports 9:6615. https://pmc.ncbi.nlm.nih.gov/articles/PMC6488697/ and its supplement | [full] |
| R3 | Natural Resources Consultants (2003). Clallam County derelict gear removal, final report, with Appendix 2 (position list). https://nwstraits.org/sites/default/files/2025-10/cla-2003-0009-derelict-gear-removal-nrc-final-report.pdf | [full] |
| R4 | Snohomish County MRC (2008). 2008 Port Gardner derelict fishing gear removal. https://www.snocomrc.org/media/1306/2008-port-gardner-derelict-fishing-gear-removal.pdf | [full] |
| R5 | Havens K. J. et al. (2008). Derelict blue crab traps in the lower York River. North American Journal of Fisheries Management 28, 1194–1200 | [full] |
| R7 | ScanPot project (2017), mass.gov | companion, not re-read |
| R8 | Lazăr A. V. et al. (2013). Side-scan sonar mapping of Mangalia Harbor. Geo-Eco-Marina 19 | companion, not re-read |
| R9, R10 | Maine DMR, Saco River debris report; New Bedford Harbor sampling plan | companion, not re-read |
| R11 | Maglić L. et al. (2025). Nautical tourism vessels as a source of seafloor litter. JMSE 13(6):1012, read from mdpi-res.com | [full] |
| R12 | Schernewski G. et al. (2023). Marine macro-litter pollution of German and North African marina and city-port sea floors. Applied Sciences 13(20):11424, read from mdpi-res.com | [full] |
| R13–R17 | ReFish 2026; AFS 2011 abstract; Boothbay Register; NOAA traps per trawl; Fishermen's Voice 2014 | companion, [page] |
| R18 | The SHIPS Project, The 1000 Tyres Project (Plymouth Sound): 13 pages and the public position file behind its map. https://1000tyres.org.3hconsulting.com/ | [page]; position file parsed |
| R19 | The Copenhagen Post (1 July 2020). Scooters out: Copenhagen Harbour cleaning ends | [page] |
| R20, R21 | Made in Marseille (2019, 2025): Vieux-Port clean-ups | [page] |
| R22 | Cyprus Mail (1 Nov 2022). Over 1,500 kilos of rubbish removed from Pomos harbour; with the basin measured on OpenStreetMap (way 684750565, © OpenStreetMap contributors, ODbL) and an aerial image | [page]; OSM geometry |
| R23 | Cyprus Mail (17 Nov 2025). Tonnes of underwater debris removed in Paphos | [page] |
| R24–R27 | Guide Me Malta 2019; Euronews 2026 (Ithaca); Ministry of the Environment Japan (c. 2022); threeharbors.com mooring sheet | companion |
| R28 | Sherow B., Johnston E. L., Clark G. F. (2022). Piers are hotspots for benthic marine debris in an urbanised estuary. PLoS ONE 17(12): e0274512 | [full] (text) |
| R29 | Renchen G. F., Butler C. B., Matthews T. R. (2021). Marine debris knows no boundaries: the Florida Keys. Marine Pollution Bulletin (accepted manuscript). https://repository.library.noaa.gov/view/noaa/57520/noaa_57520_DS1.pdf | [partial] |
| R30 | Voss C. M., Browder J. A., Wood A., Michaelis A. (2015). Factors driving the density of derelict crab pots in North Carolina waters. Fishery Bulletin 113(4): 378–390. https://repository.library.noaa.gov/view/noaa/12548/noaa_12548_DS1.pdf | [full] |
| R31 | Sullivan M. et al. (2019). Ghost fishing gear in the Mullica River–Great Bay Estuary. Marine Pollution Bulletin 138: 37–48. https://repository.library.noaa.gov/view/noaa/57758/noaa_57758_DS1.pdf | [full] |
| R32 | Wessex Archaeology (2004). St Mary's Harbour geophysical survey, ref. 58300.01. https://archaeologydataservice.ac.uk/catalogue/adsdata/arch-425-1/dissemination/pdf/wessexar1-502233_105006.pdf | [full] |
| R33 | Wessex Archaeology (2008). Weymouth Pavilion Marina: assessment of geophysical data, ref. 69000.01. https://archaeologydataservice.ac.uk/catalogue/adsdata/arch-425-1/dissemination/pdf/wessexar1-502175_104960.pdf | [full] |
| R34 | Northcomdiving S.L. for CETMAR and the Autoridad Portuaria de Vigo (2020). Informe de los residuos retirados ... dársenas nº 2 Berbés (A Laxe) y Julio Verne. https://www.apvigo.es/descargas/descargar/5206/ | [full] |
| R35 | Kang W.-S. et al. (2005). Marine litter in the ports of the Republic of Korea. ISOPE 2005, 638–644. https://www.kriso.re.kr/sciwatch/handle/2021.sw.kriso/9066 | [page] (abstract) |
| R36 | Jeong M. et al. (2021). Compositions and spatial distributions of submerged marine debris in the East Sea. J. Korean Society of Marine Environment and Safety 27(2): 295–307. https://db.koreascholar.com/Article/Detail/407366 | [page] (abstract) |
| R37 | Amsterdam: NH Nieuws (9 July 2023), Waternet crews; Guinness World Records, "Most bicycles recovered from waterways (city)" | [page] |
| R38 | Ville de Paris, "Le canal Saint-Martin fête ses 200 ans"; Euronews (12 Jan 2016) | [page] |
| R39 | McCarron P., Tetreault H. (2012). Lobster pot gear configurations in the Gulf of Maine. Maine Lobstermen's Association. https://repository.library.noaa.gov/view/noaa/37957/noaa_37957_DS1.pdf | [full] (gear paragraphs of every zone) |
| R40 | Greater Wellington Regional Council, Harbour Master (2021). Mooring construction guidelines. https://archive.gw.govt.nz/assets/Documents/2021/10/Mooring-construction-guidelines.pdf | [full] |
| R41 | OCEL Consultants NZ (2014/2015). Swing mooring design report, for Waikato and Bay of Plenty regional councils. https://www.boprc.govt.nz/media/456922/ocel-swing-mooring-design-report-v2.pdf | [partial] |
| R42 | Lovin Malta (21 Oct 2020; 7 June 2021): St George's Bay tyre clean-ups; Malta Daily (6 July 2025): Marsamxett Harbour | [page] |
| R43 | Ministry of the Environment, Japan (2021). 漁業者の協力による海底ごみ回収実証業務 報告書 (seabed litter recovery with fishers), and file 900543422 (section on large items) | [partial] |
| R44 | Hipkins F. W. (1972). Dungeness crab pots. Fishery Facts 3, NMFS. https://spo.nmfs.noaa.gov/Fishery%20Facts/fishfact03.pdf | [partial] |
| R45 | US DoD (2009/2012). UFC 4-152-07, Small craft berthing facilities; Queensland TMR (2015). Design criteria for floating walkways and pontoons | [partial] |

Sources tried and not readable on 2026-11-21 (behind a browser check, timed out or not public) are listed in
`notes_2_spatial.md` §S30 and in the resource requests.
