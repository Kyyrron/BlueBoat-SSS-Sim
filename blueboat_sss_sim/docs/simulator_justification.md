# Is the simulator justified for the thesis? Choices, evidence, references

Written 2026-11-11 for the ray-cast sensor in its harbour profiles
(`config/raycast_harbour_*_sonar.yaml`). The geometric renderer is legacy and is not
argued here. Every number below was measured in this repository; where a choice rests on
judgement it says so. The detailed records are `docs/engineering/raycast_assessment.md`,
`docs/engineering/independent_review_2026-11-11.md` and `.claude/raycast-live.md`.

## 1. Verdict

| Use in the thesis | Verdict | Why |
|---|---|---|
| Seabed appearance (texture, level, range trend) at the survey acquisition | **Justified** | Speckle statistics within 0.5 dB and level within 3 dB of a real recording at the same acquisition (§4). |
| Geometry (range, shadow length, occlusion, platform motion) | **Justified** | Closed-form gates; shadows end within 2 bins of geometry; the device's 1500 m/s range axis is reproduced. |
| Device chain and interface (Ping-protocol frames, gain, ping timing, ROS topics) | **Justified** | Byte-valid frames, the real driver's parameters and defaults, the downstream stack runs unmodified. |
| Reproducibility | **Justified** | Seeded, repeatable to 1e-9, every run records its build, profile, world and seeds. |
| Bottom tracking by the real processor | **Justified, with a known bias** | Locked on 99.7 % of pings; the tracked bottom is 0.28 m late (median), as a threshold tracker is on the real ramp. |
| Object detection results (highlight vs aspect, detector training) | **Conditional** | The object model is physics from cited constants and passes closed forms, but no simulated object has been compared with a measured one. Results that depend on it are model-conditional (superproject CM-13) until the real heading-response experiment. |
| Near-nadir level | **Empirical** | One site, one recording, outside its own acceptance rule (§6). |

## 2. What the thesis asks of the simulator

From `project_synthesis.md` (superproject): §2 the scientific spine (a surface vessel cannot
choose its altitude, shadows starve in shallow water, detection falls back on
aspect-dependent highlights); §4 the evidence layers (E1 plausible imagery in a real
geometry, E2 a detector trained on simulated data, E3 the aspect check that bounds the
simulator, E4 the policy comparison in Gazebo); §4.1 the headline claim, stated "in a
calibrated simulation environment"; §6.1, §6.3 the simulator's validation table.

| Requirement | How it is met | Where |
|---|---|---|
| The real sonar's ROS interface, unchanged downstream | Same node name, topics, QoS, parameters and defaults as the real driver | `.claude/CLAUDE.md` §3; smoke `[4b]` |
| A georeferenced harbour with real bathymetry | World Builder on GPS-anchored paths and soundings | `builder/`, `worldgen/` |
| Highlights that depend on aspect | Kirchhoff patches on smooth parts, a diffuse share from their roughness | §3.2 |
| Shadows from object height and altitude | Ray visibility on true 3-D meshes | `raycast_test [O6]`, `[L1]`–`[L7]` |
| Ground truth for a simulation-trained detector | Contacts read off the renderer's own mesh and visibility oracle | `sonar/raycast/labels.py` |
| A bottom line the processor tracks | Verified offline with the processor's own code | `analysis/raycast_bottomline.py` |
| Tens of closed-loop runs | About real time in open water on one laptop GPU | `.claude/raycast-live.md` |

## 3. Model choices and what each rests on

Status: **M** measured on the project's recordings · **C** cited physics or constants ·
**D** documented by the maker · **A** assumed or judged · **E** empirical fit.
References are in §7.

### 3.1 Seabed

| Choice | Status | Basis |
|---|---|---|
| Diffuse law: small-slope roughness scattering + sediment volume | C | [JR07] eqs 13.9–13.25, 14.6–14.9 |
| Sand and mud parameters | C, extrapolated | [JR07] Table 13.1, measured at 35–40 kHz and used at 450 kHz ([JR07] §6.5 caveat) |
| Gravel +3 dB, rock +6.5 dB above sand (a level, not a spectrum) | C | [JR07] §12.1 Lambert fits and frequency slopes; ordering confirmed at 290–440 kHz by [Fon25] |
| Coherent mean field of the seabed (first bottom return) | C | [Eck53] coherent fraction; [JR07] §13; near-vertical check [NC94], [NC98] |
| Speckle from world-fixed point scatterers, complex Gaussian | C | [Bro17]; statistics [Goo20], [AL02], [LA99] |
| Relief = the world raster at its cell (0.1–0.2 m); finer roughness only inside the law | A | Resolution of the authored world |
| Seagrass | not modelled | No scattering law found ([Sab02]); renders as sand, with a warning |

### 3.2 Objects and walls: one surface rule

A surface is a material on a triangle. The material gives a normal-incidence reflection
|R| (from density and sound speed) and an RMS roughness σ. The echo splits into a mirror
share `R·√C` (Kirchhoff patches) and a diffuse share `1 − C` (the sand law scaled by
`|R/R_sand|²`), with `C = exp(−(2kσ cos θ)²)`.

| Choice | Status | Basis |
|---|---|---|
| Mirror share: physical optics on planar patches | C | [Hwa15], [Kno04]; gates against plate, cylinder, sphere closed forms (smoke `[14]`) |
| Coherent / diffuse split by roughness | C | [Eck53]; Lambert's law does not hold on a mirror-smooth object [CG09] |
| Diffuse law of a manufactured surface = sand law × `|R/R_sand|²` | A | First-order Kirchhoff scaling; sand reference R = 0.370 from [JR07] Table 4.3. Not checked on a measured object |
| Densities and sound speeds | C | [Sel85] (PVC, HDPE, PP); [KFCS] (steel, concrete, glass, rubber). Wood and fired brick are stated without a source |
| Layered and flooded shells (plastic box, cement-filled weight, tyre tread) | C | Fluid-layer and three-medium formulas [KFCS] §6.3, [BL03]; transmission through thin shells [FL77] |
| Roughness of each material (0.05 mm metal … 3 mm tread) | A | Judged from manufacture, not measured (`worldgen/materials.py`) |
| Geometry: every class as primitives (pallet boards, bicycle tubes, tyre band) | A | Dimensions from vendor and JIS sizes for 13 classes (`target_families_and_surrogates.md`), judgement for the rest |
| Car tyre tread = rubber on its steel belts, no absorption | A (upper bound) | No absorption value for tyre rubber at 450 kHz was read at a source |
| Fouling: a fraction of the surface scatters like the seabed, no mirror | A | A sensitivity axis, default 0. No measurement at 450 kHz found; [SERDP] states the relation is not well understood |
| Wall kinds: concrete (gravity) or steel sheet pile | C | Of 965 Japanese quay walls (−7.5 m and deeper) analysed by structural type: 536 gravity, 333 sheet pile, 96 piled pier [MLIT] |
| Object–seabed corner paths by images | C | [Wil10]; image method [JKPS], [AB79] |
| Hollow bodies, sheet-pile corrugation | not modelled | Stated limits |

### 3.3 Transducer, propagation, receiver

| Choice | Status | Basis |
|---|---|---|
| Beam = product of two line apertures, near field included | C | [BL97]; far-field condition [JR07] App. F |
| Along-track width 0.5°, aperture ≈ 24.6 cm | D | Maker's pages [OS]; housing length measured by the owner |
| Vertical pattern (uniform strip from "50°", floor 0.217) | A | The maker's figure is unqualified; the large-angle pattern is unmeasured and drives the near-nadir level |
| Mounting tilt 20°, depth 0.22 m, offsets | M | Depth and offsets from the pool calibration; the tilt estimated on one recording (`.claude/sensor-pose.md`) |
| Absorption | C | [FG82a], [FG82b], as written in [JR07] App. B |
| Sound speed from temperature and salinity | C | [JR07] eq. B.10; site temperature from [OISST], not measured in situ |
| Delays at the water's speed, bins at the device's 1500 m/s | D | As the real device; `range_scale` on `ground_truth/sensor` |
| Pulse length, ping timing, gain ladder | M | Every corpus recording (`AcquisitionParams`) |
| CW burst, boxcar receive filter, TVG law | A | Undocumented by the maker |
| Frames | D | Ping Protocol, Omniscan messages [PP] |
| Multipath by image chains; diffraction on the direct leg | C | [JKPS], [AB79]; [SFV99], [Med81] |
| Visibility: watertight ray–triangle test on a BVH | C | [Woop13]; 0 leaks, GPU = CPU to 1e-4 |

### 3.4 Empirical layers (harbour profiles only)

| Layer | Status | Basis and limit |
|---|---|---|
| System gain, TVG slope, sand spectrum strength | E | Fitted on the harbour calibration recording's far field |
| Near-nadir table (up to +15 dB over r/altitude 0.85–2.0) | E | One recording, one site, 5.5–11.3 m; status "failed: every group excluded" by its own pre-registered rule; kept because it improves every band |
| Its fade-in after the first echo (0.8 m) | E | Fitted on the first 1 500 pings of a second recording at the survey acquisition (§4) |
| Shadow floor 18 dB under the ambient seabed | M as a level | Quay shadows measure 17–25 dB; a stand-in for reverberation, not physics |
| Ringing and water column | M | At 1200 bins / 44.3 µs |

### 3.5 Vessel motion and sea state

Gazebo owns the hull; the sea state adds current and a wave wrench. Spectra and forcing:
[Fos11], [New77], [Has73], [KC92]; platform-motion distortion of side-scan: [Cob92].
Known gaps: simulated pitch about 1.4 times the real one; hull mass 16 kg against about
20 kg (`docs/sea_state_model.md`).

## 4. The harbour run against real water

Simulated run `sessions/2026_11_11-11_54_29` (harbour world, harbour-fast profile, install
equal to source) against the first 1 500 pings of the real `toOptimize-harbourScan`
recording: same 1200 bins and 44 µs pulse, 18–20 m range, 4.4–5.7 m of water (simulated
4.4–9.0 m).

| Quantity | Real | Simulated |
|---|---|---|
| Speckle p1 / p10 / p50 / p90 / p99, dB re local mean | −20.2 / −10.1 / −1.9 / +3.7 / +7.0 | −21.0 / −10.2 / −1.7 / +3.6 / +6.7 |
| Speckle contrast (std / mean) | 1.05–1.10 | 1.01–1.02 |
| Lag-1 correlation, along track / along range | 0.51–0.57 / 0.64–0.66 | 0.49–0.50 / 0.63–0.64 |
| Level at 8 / 12 / 16 m slant, dB (port) | 43.8 / 39.0 / 33.5 | 45.2 / 41.5 / 36.5 |
| First return: level at the onset, re far seabed | −9.5 dB | −8.7 dB |
| First return: profile over the next 1.3 m | reference | within 2.6 dB at every offset, 1.4 dB rms with this update's fade, computed by applying it to the recorded run (to be confirmed on the next live run); before: a 2-bin step with a +4 dB crest |
| Bottom tracker jumps p50 / p90 | 0.16 / 0.88 m | 0.13 / 0.42 m (the recorded track replayed offline through the current code and the processor's tracker) |

Differences that remain, seen in the pictures: the real seabed is patchy at the scale of
metres where the simulated one is uniform (the world has one sediment and no reflectivity
map); nothing is simulated in the water column (fish, wakes); real pings are occasionally
lost.

Internal consistency (all pass): the plate, cylinder and sphere closed-form gates (smoke
`[14]`, `test/raycast_test.py`); material ordering equal to `20·log₁₀(R_a/R_b)` within 0.1 dB; a fully fouled block equal
to a sand-equivalent block to 1e-9; ground-truth shadow within 2 bins of geometry;
default, live and fast profiles unchanged to 1e-15 by this update.

## 5. Why not an existing simulator

Compared on the same requirements (details and sources in
`docs/engineering/holoocean_reference.md` and `raycast_dave_comparison.md`).

| Simulator | Side-scan sonar | Why it was not adopted |
|---|---|---|
| HoloOcean 2.4 [Pot22], [Pot24] | Octree side-scan: `R²·cos` mean per bin, no spreading, absorption, beam pattern or TVG. Ray-cast and GPU side-scan (2.4.0): law undocumented, source behind an Epic-linked account | No validation against real side-scan published; no BlueBoat; no bathymetry or GIS import; the ROS 2 bridge publishes no sonar topic; Unreal Engine does not fit the project's laptop (5 GB free disk at the time) |
| DAVE [Choi21] | Forward-looking multibeam only: first hit, constant scattering coefficient, fixed absorption | No side-scan sensor; it confirms the point-scatterer coherent sum this design uses [Bro17] |
| Stonefish | Rasterised side-scan, `cos × restitution` | GPL-3.0; no multipath, no device chain; another dynamics engine |
| OceanSim, Gazebo sensors | None | — |
| Published side-scan simulators [BL97], [CG09], [Coi09], [Pai09], [GS10], [Cer17], [Cer20], [BF21] | Ray or tube tracing, GPU rasterisation, learned generators | Not available as maintained code for this instrument; used as design references |

What only the custom sensor has, and the thesis needs: the Omniscan's own chain (pulse,
bins, gain, frames) so the real processor and station run unmodified; a near-field line
aperture; a coherent first bottom return the real tracker locks on; georeferenced worlds
from real soundings; ground truth from the same mesh; and source that can be gated
against closed forms and field recordings.

## 6. What is not justified, and what bounds it

1. **No object has been validated.** Aspect response and shadow contrast are physics from
   cited constants with judged roughness. The clean-material response is the upper bound
   of aspect sensitivity; `raycast.objects.fouling` sweeps towards the lower one. The real
   heading-response experiment is what bounds the simulator (`project_synthesis.md` §7.2).
2. **Near nadir is empirical**: one site, one recording, and the vertical beam pattern is
   unmeasured. Results within about one water depth of nadir carry that caveat.
3. **450 kHz seabed parameters are extrapolated** tenfold in frequency from 35–40 kHz
   tables; the far-field level is then fitted, so the level is right at this site and the
   angular law is inherited.
4. **Waveform, receive filter and TVG are assumed** (undocumented by the maker).
5. **Not modelled**: water-column reverberation, bubbles and wakes, seabed patchiness,
   hollow objects, sheet-pile corrugation, absorption in tyre rubber, seagrass, refraction.
6. **Shallow water** (under about 2 m) is a secondary domain: the empirical layers are off
   there and the pool recordings show image-comb effects the fast profiles cut to first order.

The thesis should therefore state its results as holding "in a calibrated simulation
environment" and lead its limitations with items 1 and 2.

## 7. References

Check status as recorded by the project: ✓ = DOI resolved and checked against Crossref on
the date shown in the cited project file; "read" = read in full; "url" = web page read on
the date shown.

**Seabed and speckle**
- [JR07] Jackson, Richardson, *High-Frequency Seafloor Acoustics*, Springer 2007. doi:10.1007/978-0-387-36945-7 ✓ read (`docs/literature/raycast_sonar/INDEX.md`)
- [NC94] Novarini, Caruthers, IEEE J. Ocean. Eng. 19(2):200–207, 1994. No DOI recorded; read.
- [NC98] Novarini, Caruthers, IEEE J. Ocean. Eng. 23(3):157–166, 1998. No DOI recorded; pp. 157–160 read.
- [Fon25] Fonseca, Lurton, Fezzani, Roche, Front. Remote Sens. 6:1619218, 2025. doi:10.3389/frsen.2025.1619218 ✓ (`docs/seabed_model.md`)
- [Sab02] Sabol et al., Estuaries 25(1):133, 2002. doi:10.1007/BF02696057 ✓ record only
- [Bro17] Brown, Johnson, Olson, J. Acoust. Soc. Am. 141(3):EL210, 2017. doi:10.1121/1.4976584 ✓ read
- [Goo20] Goodman, *Speckle Phenomena in Optics*, 2nd ed., SPIE 2020. doi:10.1117/3.2548484 ✓ record only
- [AL02] Abraham, Lyons, IEEE J. Ocean. Eng. 27(4):800, 2002. doi:10.1109/JOE.2002.804324 ✓ read
- [LA99] Lyons, Abraham, J. Acoust. Soc. Am. 106(3):1307, 1999. doi:10.1121/1.428034 ✓ read

**Reflection, materials, objects**
- [Eck53] Eckart, J. Acoust. Soc. Am. 25(3):566, 1953. doi:10.1121/1.1907123 ✓ record only
- [Sel85] Selfridge, IEEE Trans. Sonics Ultrason. SU-32(3):381, 1985. doi:10.1109/T-SU.1985.31608 ✓ read
- [KFCS] Kinsler, Frey, Coppens, Sanders, *Fundamentals of Acoustics*, 4th ed., Wiley 2000. No DOI; not in the project's checked lists.
- [BL03] Brekhovskikh, Lysanov, *Fundamentals of Ocean Acoustics*, 3rd ed., Springer 2003. doi:10.1007/b97388 ✓ record only
- [FL77] Folds, Loggins, J. Acoust. Soc. Am. 62(5):1102, 1977. doi:10.1121/1.381643 ✓ read
- [Hwa15] Hwang, Hong, Song, Kwon, J. Appl. Math. Phys. 3(2):166, 2015. doi:10.4236/jamp.2015.32026 ✓ read
- [Kno04] Knott, Shaeffer, Tuley, *Radar Cross Section*, 2nd ed., 2004. doi:10.1049/SBRA026E ✓ record only
- [Wil10] Williams et al., J. Acoust. Soc. Am. 127(6):3356, 2010. doi:10.1121/1.3419926 ✓ read
- [CG09] Coiras, Groen, in *Advances in Sonar Technology*, 2009. doi:10.5772/39406 ✓
- [SERDP] Lee, "Investigation of Acoustic Scattering from UXO with Long-term Environmental Aging", SERDP project MR21-3582, project page read 2026-11-11 (url; the reports themselves were not retrieved).
- [MLIT] Ministry of Land, Infrastructure, Transport and Tourism, Ports and Harbours Bureau, 「資料-4 点検診断実績の整理」 (inspection and diagnosis records, maintenance database as of the end of fiscal 2022), mlit.go.jp/kowan/content/001726361.pdf, read 2026-11-11 (url).

**Beam, propagation, receiver**
- [BL97] Bell, Linnett, IEE Proc. Radar Sonar Navig. 144(4):219, 1997. doi:10.1049/ip-rsn:19971311 ✓ read
- [FG82a] Francois, Garrison, J. Acoust. Soc. Am. 72(3):896, 1982. doi:10.1121/1.388170 ✓ (formula read in [JR07])
- [FG82b] Francois, Garrison, J. Acoust. Soc. Am. 72(6):1879, 1982. doi:10.1121/1.388673 ✓
- [OISST] NOAA OISST v2.1, doi:10.25921/RE9P-PT57; Huang et al., J. Climate 34(8):2923, 2021, doi:10.1175/JCLI-D-20-0166.1 ✓
- [OS] Blue Robotics and Cerulean Sonar product pages for the Omniscan 450 SS, read 2026-11-02 and 2026-11-06 (url).
- [PP] Blue Robotics Ping Protocol, Omniscan450 messages, read 2026-11-02 (url).
- [JKPS] Jensen, Kuperman, Porter, Schmidt, *Computational Ocean Acoustics*, 2nd ed., Springer 2011. doi:10.1007/978-1-4419-8678-8 ✓ record only
- [AB79] Allen, Berkley, J. Acoust. Soc. Am. 65(4):943, 1979. doi:10.1121/1.382599 ✓ record only
- [SFV99] Svensson, Fred, Vanderkooy, J. Acoust. Soc. Am. 106(5):2331, 1999. doi:10.1121/1.428071 ✓ read
- [Med81] Medwin, J. Acoust. Soc. Am. 69(4):1060, 1981. doi:10.1121/1.385684 ✓ record only
- [Woop13] Woop, Benthin, Wald, J. Comput. Graph. Tech. 2(1), 2013. No DOI.

**Sea state and motion** (`docs/sea_state_model.md` §10, checked 2026-10-08)
- [Fos11] Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, Wiley 2011. doi:10.1002/9781119994138 ✓
- [New77] Newman, *Marine Hydrodynamics*, MIT Press 1977. doi:10.7551/mitpress/4443.001.0001 ✓
- [Has73] Hasselmann et al., JONSWAP report, 1973. No DOI.
- [KC92] Kahma, Calkoen, J. Phys. Oceanogr. 22:1389, 1992. doi:10.1175/1520-0485(1992)022<1389:RDITOG>2.0.CO;2 ✓
- [Cob92] Cobra, Oppenheim, Jaffe, IEEE J. Ocean. Eng. 17(3), 1992. doi:10.1109/48.153442 ✓

**Simulators and sonar image simulation**
- [Pot22] Potokar et al., IROS 2022, pp. 8450–8456. doi:10.1109/IROS47612.2022.9981119 ✓
- [Pot24] Potokar et al., IEEE J. Ocean. Eng. 49(4):1322, 2024. doi:10.1109/JOE.2024.3410290 ✓
- [Choi21] Choi et al., Front. Robot. AI 8:706646, 2021. doi:10.3389/frobt.2021.706646 ✓ read
- [Coi09] Coiras et al., OCEANS 2009 Europe. doi:10.1109/OCEANSE.2009.5278213 ✓
- [Pai09] Pailhas et al., OCEANS 2009 Europe. doi:10.1109/OCEANSE.2009.5278111 ✓ record only
- [GS10] Guériot, Sintes, OCEANS 2010 Sydney. doi:10.1109/OCEANSSYD.2010.5603867 ✓ record only
- [Cer17] Cerqueira et al., Computers & Graphics 68:66, 2017. doi:10.1016/j.cag.2017.08.008 ✓ record only
- [Cer20] Cerqueira et al., Graphical Models 111:101086, 2020. doi:10.1016/j.gmod.2020.101086
- [BF21] Bore, Folkesson, IEEE J. Ocean. Eng. 46(1):195, 2021. doi:10.1109/JOE.2020.2980456
