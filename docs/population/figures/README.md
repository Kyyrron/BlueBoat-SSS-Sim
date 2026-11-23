# Figures of the population model (captions for the thesis)

*Made by `~/ros2_ws/sim_runs/worldgen_v11/stats/make_figures.py` from the statistics of 2026-11-21 (model 1.0.0, code
digest `15c33763a2a771f3`, evidence digest `534a2c0b3db629f2`). Vector files: `.svg`, `.pdf`; `.png` is a preview
(ignored by git). Reference keys: `../EVIDENCE.md` §7.*

**Figure 1 (`fig1_density_levels`).** Density of sonar-size objects (targets and debris) in the active band against
the density level. Line: the level's density λ̄(s). Points: mean over 200 placement seeds of each harbour profile on 12
campaign basins in the sensor's altitude band; bars: 10th to 90th percentile. Dashed lines: documented densities —
St Mary's Harbour all sonar targets 0.2–0.5 per ha (R32), Weymouth 0.64 (R33) and Port Gardner pots 0.64 (R4), Port
Angeles West pots 1.34 (R3), Vieux-Port lifted items ≥ 2.7 (R20), Venice Grand Canal 11.6 (R2), Plymouth Sound tyres
17.6 over the densest 9 ha and 37 in the densest hectare (R18, derived from the position file), Thessaloniki 25 at
≥ 0.5 m (R1), the Pomos fender shelter about 500 tyres per ha (R22 with the basin measured on OpenStreetMap).

**Figure 2 (`fig2_class_shares`).** Shares of the seven target classes at level 8 by profile: expected by the model
(grey) against realised (colour), 200 seeds × 12 basins. The weights are design values informed by EVIDENCE.md §5
(no study counts sonar-size harbour litter by class); the spacing rule removes some tyres from crowded fender lines.

**Figure 3 (`fig3_distance_to_source`).** Cumulative share of objects against the distance to the nearest source
feature, levels 5 and 10. The dashed line marks 30 m (targets document: 70 % within 30 m, bracket 50–90 %). The quay
kernels are design values bounded above by the Sydney piers (10× denser at the pier than 50 m away, R28).

**Figure 4 (`fig4_clustering`).** Clark–Evans ratio (Donnelly edge correction; median over 200 seeds) by level and
profile. Dashed: values derived from published position lists — derelict pots of Port Angeles Harbor West 0.62 (R3
Appendix 2) and the sonar targets of St Mary's Harbour 0.62 (R32), Weymouth 0.87 (R33); 1 is a random pattern. The
fishing-ground hot spots are fitted to the Port Angeles pots (nearest neighbour 14.8 m against 17 m, 67 % single at
10 m against 63 %); the fishing-harbour tyres at level 5 are not fitted and match Plymouth Sound (11.9 m against
8.4 m, 64 % single at 5 m against 65 %, R18).

**Figure 5 (`fig5_gallery`).** One campaign basin (two quays, a pontoon with its piles, a mooring block; world seed 7)
and one placement seed (14001) for the six profiles at levels 2, 5 and 8: the profiles share their random numbers, so
the panels differ only by the profile and the level. Light grey: land behind a quay; off-white: water outside the
altitude band (none here). Class markers: box, drum, block, trap, pipe, tyre, net; debris grey; structures open squares.
