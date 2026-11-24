# Literature map for the calibration work

A short, task-specific map. These sources guide method, not parameter
values.

**How each entry was checked (2026-10-21).** The bibliographic record was
resolved through Crossref (`api.crossref.org/works/<DOI>`). Where noted, the
abstract or description was read on the author's page, Semantic Scholar or
the publisher. Full texts were **not** read for this map. "Supported" means
supported by that record or abstract; anything more specific is marked as
our inference.

| # | Reference (verified record) | DOI |
|---|---|---|
| 1 | Potokar, Lay, Norman, Benham, Neilsen, Kaess, Mangelson. "HoloOcean: Realistic Sonar Simulation." *IEEE/RSJ IROS 2022*, pp. 8450–8456. | [10.1109/IROS47612.2022.9981119](https://doi.org/10.1109/IROS47612.2022.9981119) |
| 2 | Potokar, Lay, Norman, Benham, Ashford, Peirce, Neilsen, Kaess, Mangelson. "HoloOcean: A Full-Featured Marine Robotics Simulator for Perception and Autonomy." *IEEE J. Oceanic Eng.* 49(4):1322–1336, 2024. | [10.1109/JOE.2024.3410290](https://doi.org/10.1109/JOE.2024.3410290) |
| 3 | Meng, Xu, Shen, Guo. "A High-Efficiency Side-Scan Sonar Simulator for High-Speed Seabed Mapping." *Sensors* 23(6):3083, 2023. | [10.3390/s23063083](https://doi.org/10.3390/s23063083) |
| 4 | Coiras, Groen. "Simulation and 3D Reconstruction of Side-Looking Sonar Images." In *Advances in Sonar Technology*, I-Tech, 2009. (Crossref lists the second author as "J. Groe", a metadata typo.) | [10.5772/39406](https://doi.org/10.5772/39406) |
| 5 | Coiras, Ramirez-Montesinos, Groen. "GPU-based simulation of side-looking sonar images." *OCEANS 2009-Europe*, pp. 1–6. | [10.1109/OCEANSE.2009.5278213](https://doi.org/10.1109/OCEANSE.2009.5278213) |
| 6 | Jackson, Richardson. *High-Frequency Seafloor Acoustics*. Springer, 2007. | [10.1007/978-0-387-36945-7](https://doi.org/10.1007/978-0-387-36945-7) |
| 7 | Kennedy, O'Hagan. "Bayesian Calibration of Computer Models." *J. R. Stat. Soc. B* 63(3):425–464, 2001. | [10.1111/1467-9868.00294](https://doi.org/10.1111/1467-9868.00294) |
| 8 | Fossen. *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd ed., Wiley, 2021. (Wiley lists the April 2021 book as the second edition; Crossref's edition field reads "1".) | [10.1002/9781119575016](https://doi.org/10.1002/9781119575016) |

## Per source: scope, lesson, limits

**[1] HoloOcean sonar (IROS 2022).**

- *Supported (author's page abstract):* upgrades to HoloOcean's sonar
  simulation with cluster-based multipath ray tracing, probabilistic noise
  models and material dependence, for side-scan, single-beam and
  multibeam profiling sonars.
- *Lesson:* compare acoustic effects and the validation method, not
  screenshots.
- *Limits:* nothing in the record ties it to the Omniscan 450, to its
  per-ping normalisation, or to our corpus. It is not evidence that
  either simulator is more accurate for our sensor.

**[2] HoloOcean full simulator (JOE 2024).**

- *Supported (abstract):*
  - a UE4-based multi-agent simulator with a Python interface and
    customisable agent dynamics;
  - a sonar framework built on an octree, cluster-based multipath ray
    tracing, probabilistic noise and reflecting-surface properties;
  - side-scan, single-beam and multibeam profiling sonar.
- *Lesson:* a general comparator for scene, sensor and robot architecture.
  A common benchmark (same mesh, path, settings; geometry first, then
  noise) is the fair comparison.
- *Limits:* the abstract reports no field validation. It neither
  establishes a validated surface-USV wave response nor Cerulean protocol
  compatibility, and it gives no reason to replace our stack.

**[3] Meng et al. 2023.**

- *Supported (Crossref abstract):*
  - a two-level network architecture;
  - a polyline path model for the propagation delay under high-speed
    motion;
  - a seabed simplification based on an energy function;
  - comparison with actual experiments.
- *Lesson:* check whether our stop-and-hop approximation (A5) matters at
  our speed, range and beam before modelling intra-ping motion. The memo
  estimates about 9 cm at the far edge at 10°/s yaw *(inference, not
  measured)*.
- *Limits:* not a USV dynamics or ROS/device integration, and its
  conclusions are for high-speed mapping, not ~1 m/s litter surveys.

**[4] Coiras & Groen 2009 (chapter).**

- *Supported (record):* side-looking sonar image formation, used forward
  for simulation and inverted for 3-D reconstruction.
- *Lesson:* geometry (bathymetry, orientation) and radiometry
  (reflectivity, angular law) are coupled in the image. Never tune object
  brightness in isolation; NC #17 already encodes this.
- *Limits:* a method chapter, not a benchmark for our device.

**[5] Coiras et al. 2009 (OCEANS).**

- *Supported (record):* GPU-based side-looking sonar image simulation. The
  record carries no abstract, so our summary of its contents (stages,
  noise, multipath, PSF) relies on the review's description and is
  unverified here.
- *Lesson:* treat the PSF and runtime as explicit design dimensions. Our
  measured cost is about 6–7 ms per ping-side, so a GPU port is not
  needed at 20 Hz.
- *Limits:* older, generic.

**[6] Jackson & Richardson 2007.**

- *Supported (record, and its use in `docs/seabed_model.md`):* the
  reference monograph on seafloor geoacoustics, roughness and volume
  scattering, and backscatter statistics. It is the source of
  `sand_lambert_mu_db` (Fig. 12.3) and of `materials_literature.yaml`.
- *Lesson:* use its values as **priors and ranges**, with the site spread
  stated (±5 dB for µ_sand), and calibrate to our frequency, geometry and
  bottom.
- *Limits:* it does not make any simplified reflectivity table correct for
  a given site.

**[7] Kennedy & O'Hagan 2001.**

- *Supported (abstract, via search of the published abstract):*
  - Bayesian calibration that carries the remaining uncertainty of the
    fitted parameters into prediction;
  - an explicit model-inadequacy (discrepancy) term, revealed when even
    the best-fitting parameters do not reproduce the observations;
  - the form "observation = model(θ) + discrepancy + error".
- *Lesson:*
  - calibration artifacts carry an uncertainty per value, never a bare
    number;
  - validation residuals beyond the repeatability floor are reported as
    discrepancy, not removed by re-fitting;
  - the local identifiability tool here is a first step, not this full
    framework.
- *Limits:* Gaussian-process discrepancy modelling needs more data than
  the current corpus. It is not implemented.

**[8] Fossen 2021.**

- *Supported (publisher description):* hydrodynamic models of marine
  craft; wind, wave and current models; stability; guidance, navigation
  and control.
- *Lesson:* identify vessel and controller dynamics from command and
  motion logs **separately** from the acoustics (stage 2). An acoustic fit
  must never compensate a wrong trajectory or a wrong yaw response.
- *Limits:* it gives model structure, not this BlueBoat's coefficients.
  The Gazebo plant carries the BlueROV2 table (Control CLAUDE.md §8). The
  repository's sea-state docs cite the 2011 edition for the current
  model.
