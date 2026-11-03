# Literature corpus — the ray-cast sensor (`renderer: raycast`)

Phase 0 of `docs/engineering/raycast_plan.md`. This corpus underpins the second simulated sensor: a
3-D ray-cast side-scan renderer next to `GeometricRenderer`. It builds on, and does not repeat:
- `docs/sonar_model.md`, `docs/seabed_model.md`;
- `docs/engineering/literature_review.md` (the calibration literature, 2026-10-21);
- `docs/engineering/calibration_results.md`;
- `~/calib/pass4/literature.md`.

The design decisions taken from it are in `docs/engineering/raycast_design.md`.

## How each reference was verified (2026-11-02)

Every DOI below was resolved on **2026-11-02** through `https://api.crossref.org/works/<DOI>`
(script `~/ros2_ws/sim_runs/raycast/tools/crossref_check.py`; cached records in
`~/ros2_ws/sim_runs/raycast/crossref/`). Title, authors, year, journal, volume and pages were compared
with the intended work.
- Where a guessed DOI resolved to another paper, the right one was found with a Crossref bibliographic
  search (`crossref_search.py`) and re-checked.
- Four guesses were wrong and are **not used**: `10.1109/URAI.2017.7992729`, `10.1121/1.421087`,
  `10.1121/1.421245`, `10.1121/1.427171`. So is pass 4's `10.1121/1.385685`.
- Web documents without a DOI (the device pages, and the JCGT paper WBW13, whose journal issues no DOIs) are listed with their URL and the date read.

"Read" says how much of each source was read:
- **full** — the full text was read where it is used (local PDF in `~/Downloads/research_paper/` or
  open access);
- **record** — only the bibliographic record or the abstract. A record-only source never fixes a
  constant; it can only motivate a choice whose numbers come from elsewhere;
- **wanted** — the full text is needed before the source can justify what it is cited for (list at
  the end).

**No reference here is UNVERIFIED.** The APL-UW TR 9407 handbook has no DOI and was not read. It is
cited only where Jackson & Richardson cite it, never as the source of a constant.

## Topic notes

| # | Note | Renderer modules |
|---|---|---|
| 01 | `01_simulation_and_transient_rendering.md` — SSS ray tracing, GPU simulators, point scatterers, transient rendering | `renderer`, `scatterers`, `paths`, `receiver` |
| 02 | `02_ray_theory_image_method_lloyd.md` — ray theory, image method, Lloyd's mirror, straight-ray validity | `paths`, `propagation` |
| 03 | `03_seabed_scattering.md` — SPM, Kirchhoff, composite roughness, SSA, volume, impedance | `scattering` |
| 04 | `04_reverberation_statistics.md` — speckle, K, correlation, coherent summation | `scatterers`, `receiver` |
| 05 | `05_object_scattering.md` — PO closed forms, corners, thin shells, object–seabed paths | `scattering`, `paths` |
| 06 | `06_edge_diffraction.md` — knife edge, Biot–Tolstoy/BTM, GTD/UTD/PTD | `diffraction` |
| 07 | `07_propagation_loss.md` — Francois–Garrison absorption per site, spreading, sound speed | `propagation` |
| 08 | `08_surface_bubbles_volume.md` — Eckart surface, surface reverberation, bubbles, volume | `scene_mesh`, `scattering` |
| 09 | `09_sss_system_physics.md` — Omniscan facts, near-field aperture, pulse, receiver, noise, motion | `beam`, `receiver`, `motion` |
| 10 | `10_gpu_ray_tracing_numerics.md` — ray–triangle, BVH, GPU, counter-based RNG, precision | `bvh_cpu`, `gpu_*` |

## References

| Key | Short reference | DOI / URL | Read | Notes |
|---|---|---|---|---|
| AB79 | Allen & Berkley 1979, JASA 65(4):943 | 10.1121/1.382599 | record | 02 |
| Ain10 | Ainslie 2010, *Principles of Sonar Performance Modelling* | 10.1007/978-3-540-87662-5 | record | 09 |
| AL02 | Abraham & Lyons 2002, IEEE JOE 27(4):800 | 10.1109/JOE.2002.804324 | full | 04 |
| AM98 | Ainslie & McColm 1998, JASA 103(3):1671 | 10.1121/1.421258 | record | 07 |
| Bell97a | Bell & Linnett 1997, IEE Proc. RSN 144(4):219 | 10.1049/ip-rsn:19971311 | full | 01, 02, 09 |
| Bell97b | Bell 1997, Opt. Eng. 36(6):1806 | 10.1117/1.601325 | record | 01 |
| BL03 | Brekhovskikh & Lysanov 2003, *Fundamentals of Ocean Acoustics* | 10.1007/b97388 | record | 02, 05 |
| Bl09 | Blondel 2009, *Handbook of Sidescan Sonar* | 10.1007/978-3-540-49886-5 | full (local) | 09 |
| Brown17 | Brown, Johnson, Olson 2017, JASA 141(3):EL210 | 10.1121/1.4976584 | full (owner PDF, read 2026-11-03) | 01, 04 |
| BT57 | Biot & Tolstoy 1957, JASA 29(3):381 | 10.1121/1.1908899 | record | 06 |
| BW93 | Bilaniuk & Wong 1993, JASA 93(3):1609 | 10.1121/1.406819 | record | 02, 07 |
| Car09 | Carey 2009, Acoustics Today 5(2):14 | 10.1121/1.3182842 | record | 02 |
| Cer17 | Cerqueira et al. 2017, Computers & Graphics 68:66 | 10.1016/j.cag.2017.08.008 | record | 01 |
| Choi21 | Choi et al. 2021, Front. Robot. AI 8:706646 | 10.3389/frobt.2021.706646 | full (open access) | 01 |
| Coi09a | Coiras & Groen 2009, *Advances in Sonar Technology* | 10.5772/39406 | record | 01 |
| Coi09b | Coiras, Ramirez-Montesinos, Groen 2009, OCEANS-Europe | 10.1109/OCEANSE.2009.5278213 | record | 01 |
| COJ92 | Cobra, Oppenheim, Jaffe 1992, IEEE JOE 17(3):252 | 10.1109/48.153442 | full (local) | 09 |
| CP89 | Commander & Prosperetti 1989, JASA 85(2):732 | 10.1121/1.397599 | record | 08 |
| DeM15 | DeMarco, West, Howard 2015, OCEANS | 10.23919/OCEANS.2015.7404349 | record | 01 |
| Eck53 | Eckart 1953, JASA 25(3):566 | 10.1121/1.1907123 | record | 08 |
| FG82a | Francois & Garrison 1982, JASA 72(3):896 | 10.1121/1.388170 | formula via JR07 B.3 | 07 |
| FG82b | Francois & Garrison 1982, JASA 72(6):1879 | 10.1121/1.388673 | formula via JR07 B.3 | 07 |
| FL77 | Folds & Loggins 1977, JASA 62(5):1102 | 10.1121/1.381643 | full (owner PDF, read 2026-11-03) | 05 |
| FS77 | Fisher & Simmons 1977, JASA 62(3):558 | 10.1121/1.381574 | record | 07 |
| Goo20 | Goodman 2020, *Speckle Phenomena in Optics*, 2nd ed. | 10.1117/3.2548484 | record | 04 |
| Gue10 | Guériot & Sintes 2010, OCEANS Sydney | 10.1109/OCEANSSYD.2010.5603867 | record | 01 |
| Gwon17 | Gwon et al. 2017, URAI, pp. 662–665 | 10.1109/URAI.2017.7992789 | record | 01 |
| Hwang15 | Hwang, Hong, Song, Kwon 2015, J. Appl. Math. Phys. 3(2):166 (PO RCS; GO paths + PO at the last bounce, effective area) | 10.4236/jamp.2015.32026 | full (owner PDF, read 2026-11-03) | 05 |
| Iva98 | Ivakin 1998, JASA 103(2):827 | 10.1121/1.421243 | record | 03 |
| Jar14 | Jarabo et al. 2014, ACM TOG 33(6) | 10.1145/2661229.2661251 | record | 01, 10 |
| Jar17 | Jarabo et al. 2017, Visual Informatics 1(1):65 | 10.1016/j.visinf.2017.01.008 | record | 01 |
| JB92 | Jackson & Briggs 1992, JASA 92(2):962 | 10.1121/1.403966 | record | 03 |
| JH90 | Johnson & Helferty 1990, Rev. Geophys. 28(4):357 | 10.1029/RG028i004p00357 | record | 09 |
| JKPS11 | Jensen, Kuperman, Porter, Schmidt 2011, *Computational Ocean Acoustics* | 10.1007/978-1-4419-8678-8 | record | 02 |
| JP76 | Jakeman & Pusey 1976, IEEE TAP 24(6):806 | 10.1109/TAP.1976.1141451 | record | 04 |
| JR07 | Jackson & Richardson 2007, *High-Frequency Seafloor Acoustics* | 10.1007/978-0-387-36945-7 | full (local) | 03, 04, 07, 09 |
| JWI86 | Jackson, Winebrenner, Ishimaru 1986, JASA 79(5):1410 | 10.1121/1.393669 | record | 03 |
| Kel62 | Keller 1962, JOSA 52(2):116 | 10.1364/JOSA.52.000116 | record | 06 |
| KP74 | Kouyoumjian & Pathak 1974, Proc. IEEE 62(11):1448 | 10.1109/PROC.1974.9651 | record | 06 |
| KST04 | Knott, Shaeffer, Tuley 2004, *Radar Cross Section* | 10.1049/SBRA026E | record — wanted | 05 |
| Kwak15 | Kwak et al. 2015, IEEE EEEIC | 10.1109/EEEIC.2015.7165431 | record | 01 |
| LA99 | Lyons & Abraham 1999, JASA 106(3):1307 | 10.1121/1.428034 | full (owner PDF, read 2026-11-03) | 04 |
| Love77 | Love 1977, JASA 62(6):1397 | 10.1121/1.381672 | record | 08 |
| Lur10 | Lurton 2010, *An Introduction to Underwater Acoustics* | 10.1007/978-3-642-13835-5 | record | 09 |
| Mac61 | Mackenzie 1961, JASA 33(11):1498 | 10.1121/1.1908482 | full (local) | 03 |
| Mac81 | Mackenzie 1981, JASA 70(3):807 | 10.1121/1.386920 | record | 02, 07 |
| MC98 | Medwin & Clay 1998, *Fundamentals of Acoustical Oceanography* (chapters) | 10.1016/b978-012487570-8/50010-6, …/50013-1 | record | 08 |
| McD93 | McDaniel 1993, JASA 94(4):1905 | 10.1121/1.407514 | full (owner PDF, read 2026-11-03) | 08 |
| Med77 | Medwin 1977, Ultrasonics 15(1):7 | 10.1016/0041-624X(77)90005-1 | record | 08 |
| Med81 | Medwin 1981, JASA 69(4):1060 | 10.1121/1.385684 | record | 06 |
| Meng23 | Meng et al. 2023, Sensors 23(6):3083 | 10.3390/s23063083 | record | 01 |
| Mel52 | Mellen 1952, JASA 24(5):478 | 10.1121/1.1906924 | record | 09 |
| MT97 | Möller & Trumbore 1997, J. Graphics Tools 2(1):21 | 10.1080/10867651.1997.10487468 | record | 10 |
| Pail09 | Pailhas et al. 2009, OCEANS-Europe | 10.1109/OCEANSE.2009.5278111 | record | 01 |
| Par10 | Parker et al. 2010, ACM TOG 29(4) | 10.1145/1778765.1778803 | record | 10 |
| PB87 | Porter & Bucker 1987, JASA 82(4):1349 | 10.1121/1.395269 | record | 02 |
| Pie19 | Pierce 2019, *Acoustics*, 3rd ed. | 10.1007/978-3-030-11214-1 | record | 05 |
| Pot22 | Potokar et al. 2022, IROS | 10.1109/IROS47612.2022.9981119 | record | 01 |
| Pot24 | Potokar et al. 2024, IEEE JOE 49(4):1322 | 10.1109/JOE.2024.3410290 | record | 01 |
| Ric96 | Richards, Heathershaw, Thorne 1996, JASA 100(3):1447 | 10.1121/1.415991 | record | 07, 08 |
| Sab02 | Sabol et al. 2002, Estuaries 25(1):133 | 10.1007/BF02696057 | record | 03 |
| Sac15 | Saç et al. 2015, Turk. J. EECS 23:2289 | 10.3906/elk-1305-188 | record | 01 |
| Sal11 | Salmon et al. 2011, SC'11 | 10.1145/2063384.2063405 | record | 10 |
| Sel85 | Selfridge 1985, IEEE Trans. Sonics Ultrason. 32(3):381 | 10.1109/T-SU.1985.31608 | full (local) | 03, 05 |
| SFV99 | Svensson, Fred, Vanderkooy 1999, JASA 106(5):2331 | 10.1121/1.428071 | full (owner PDF, read 2026-11-03) | 06 |
| Sta89 | Stanton 1989, JASA 86(4):1499 | 10.1121/1.398711 | record | 05 |
| Sung20 | Sung et al. 2020, IJCAS 18(3):523 | 10.1007/s12555-019-0691-3 | abstract (local) | 01 |
| Tho88 | Thorsos 1988, JASA 83(1):78 | 10.1121/1.396188 | record | 03, 08 |
| TJ89 | Thorsos & Jackson 1989, JASA 86(1):261 | 10.1121/1.398342 | record | 03 |
| Tur60 | Turin 1960, IRE Trans. IT 6(3):311 | 10.1109/TIT.1960.1057571 | record | 09 |
| TVF94 | Trevorrow, Vagle, Farmer 1994, JASA 95(4):1922 | 10.1121/1.408706 | full (owner PDF, read 2026-11-03) | 08 |
| Ufi14 | Ufimtsev 2014, *Fundamentals of the PTD*, 2nd ed. | 10.1002/9781118753767 | record | 06 |
| Vel12 | Velten et al. 2012, Nat. Commun. 3:745 | 10.1038/ncomms1747 | record | 01 |
| Wal14 | Wald et al. 2014, ACM TOG 33(4) | 10.1145/2601097.2601199 | record | 10 |
| Wan26 | Wang et al. 2026, Ocean Eng. 366:127967 | 10.1016/j.oceaneng.2026.127967 | abstract (local) | 09 |
| Wil10 | Williams et al. 2010, JASA 127(6):3356 | 10.1121/1.3419926 | full (owner PDF, read 2026-11-03) | 05 |
| WBW13 | Woop, Benthin, Wald 2013, J. Computer Graphics Techniques 2(1):65 | https://jcgt.org/published/0002/01/05/paper.pdf (no DOI) | record; implemented | 10 |
| WJ98 | Williams & Jackson 1998, JASA 103(1):169 | 10.1121/1.421109 | record (summarised in JR07 §13.2.4) | 03 |
| OISST | NOAA 0.25° Daily OISST v2.1 dataset (NOAA NCEI) | 10.25921/RE9P-PT57 (DataCite; verified by DOI content negotiation 2026-11-03) | data read via ERDDAP 2026-11-03 | 07 |
| Huang21 | Huang et al. 2021, J. Climate 34(8):2923 (DOISST v2.1) | 10.1175/JCLI-D-20-0166.1 | record | 07 |
| Rey07 | Reynolds et al. 2007, J. Climate 20(22):5473 (daily OISST) | 10.1175/2007JCLI1824.1 | record | 07 |
| OS-CAD | Cerulean Omniscan transducer CAD (STEP) bounding box, given by the owner 2026-11-03: 318 × 29 × 20 mm | — | owner measurement of the CAD model | 09 |
| OS-BR | Blue Robotics Omniscan 450 SS page | https://bluerobotics.com/store/the-reef/cerulean-sidescan-sonar/ | read 2026-11-02 | 09 |
| OS-CS | Cerulean Omniscan 450 SS page + SonarView docs | https://ceruleansonar.com/products/omniscan-450-ss | read 2026-11-02 | 09 |
| PP-OS | Ping Protocol Omniscan450 messages | https://docs.bluerobotics.com/ping-protocol/pingmessage-omniscan450/ | read 2026-11-02 | 09 |
| Warp | NVIDIA Warp | https://github.com/NVIDIA/warp | to read at Phase 4 | 10 |
| APL94 | APL-UW TR 9407 (1994) | — | not read | cited via JR07 only |

## Full texts still wanted

Updated 2026-11-03. The owner supplied Brown17, SFV99, Wil10, FL77, LA99, McD93 and TVF94, and the PO
paper Hwang15 in place of KST04. All are now read; their findings are in the notes. Still wanted:

1. **Mackenzie 1981**, JASA 70(3):807, doi:10.1121/1.386920 (or any printed copy of its nine
   coefficients). Blocks: replacing the J&R B.10 sound speed (±3–4 m/s bias, stated).
2. **Knott, Shaeffer, Tuley 2004** (optional now). The dihedral and trihedral closed forms follow from
   Hwang15's effective-area method (σ = 4π A_eff²/λ², A_eff = √2·ab for a 90° dihedral at its
   bisector, L²/√3 for a triangular trihedral on axis). Knott would confirm the table values.
3. A **primary source for coastal high-frequency volume scattering strength `S_v` bounds**. TVF94 gives
   ship-wake values only.
4. A **primary seagrass scattering source** (high frequency). LA99 shows Posidonia sites' statistics
   at 80 kHz, not a level law.
