# HoloOcean technical reference for the BlueBoat harbour side-scan simulation

**Status:** verified on 2026-11-11 against the official documentation of release 2.4.0 and the source that
could be reached. Supporting material for the architecture decision recorded in
`independent_review_2026-11-11.md`; it is not a recommendation by itself. It updates the owner's
starting reference (`~/Desktop/HoloOceanRef.txt`), whose open questions are answered in §12.

Every statement carries its evidence class:

| Tag | Meaning |
|---|---|
| **[doc]** | stated in the official HoloOcean documentation (URL given) |
| **[src]** | read in source code, with the version named |
| **[paper]** | stated in a published paper |
| **[inf]** | this document's own inference or assessment, not a HoloOcean statement |

**The main limit of this reference.** The current HoloOcean source repository is only visible to GitHub
accounts linked to an Epic Games account; without one it returns "Page Not Found" **[doc]**
(<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/usage/installation.html>; confirmed by a
maintainer in <https://github.com/byu-holoocean/holoocean-ros/issues/15>). No 2.x engine source was read.
Source-level facts below come from three public stand-ins: the v0.5.0 engine (2022, Unreal 4.27) on
Bitbucket, a third-party copy of the 2.2.2 Python client, and the fully public `holoocean-ros` bridge.

## 1. Releases, engine and platform

- **Latest release: 2.4.0, dated 2026-11-03** **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/changelog/changelog.html>). It adds
  `RaycastSidescanSonar` and `GPUSidescanSonar`, FFT waves with mesh-based buoyancy, and material
  (impedance) support for the raycast and GPU sonars.
- Documented versions: 2.4.0, 2.3.0, 2.2.x, 2.1.0, 2.0.x, 1.0.0, an Unreal-4.27 archive and `develop`
  **[doc]** (<https://byu-holoocean.github.io/holoocean-docs/versionList.html>). The `release` branch is
  the latest stable release; `develop` "may not be stable" **[doc]** (installation page).
- **Unreal Engine 5.3** for every 2.x release **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/develop/develop.html>). Development also needs
  Visual Studio 2019 / 2022 **[doc]** (`develop/start.html`).
- **Operating system:** "64-bit Linux or Windows"; no Ubuntu version is listed for native installs. The
  Docker images moved to Ubuntu 24.04 and CUDA 12.9 in 2.4.0 **[doc]**. The docs add that the Unreal
  development tools are "more stable and straightforward in Windows", and that on Linux Unreal 5.3 may
  lack the ISPC compiler the FFT waves need **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/develop/troubleshooting.html>).
- **Python ≥ 3.11, numpy ≥ 2** **[doc]**. This workspace's ROS 2 Jazzy interpreter carries numpy 1.26, so
  the client would need its own environment **[inf]**.
- **Hardware:** a Vulkan-capable GPU; recommended a dedicated GPU, 8 GB RAM, a quad-core CPU; no VRAM
  figure **[doc]**.
- **Disk:** the prebuilt Ocean 2.4.0 package is 5.96 GB compressed for Linux (HTTP `HEAD` on
  <https://robots.et.byu.edu/holo/Ocean/v2.4.0/Linux.zip>, 2026-11-11); octree caches "can often take up
  several gigabytes" **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/sensors/docs/octree.html>).
- **Prebuilt worlds** install as binaries with `holoocean.install("Ocean")` **[doc]**. Their editable
  projects are not distributed: "Because the other packaged worlds distributed with HoloOcean contain
  purchased assets, they are only available to official BYU FROST Lab members. Other users will have to
  develop their own worlds" **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/develop/env-docs/package-env.html>).

## 2. Custom harbour environments

- Workflow: clone the gated repository, work in the Unreal 5.3 editor, build the seabed with Unreal's
  Landscape tools, import meshes as FBX, tag the water plane, package the project, then copy
  `config.json` and `materials.csv` by hand **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/develop/env-docs/create-env.html>).
- **No bathymetry, heightmap or GIS import is documented.** The 2.4.0 documentation index contains no
  bathymetry, GeoTIFF, UTM, WGS or latitude terms; the 2.0 preview paper only says a bathymetry-map
  input was being tested **[doc] [paper]** (<https://arxiv.org/abs/2510.06160>).
- The sonar reads an object's **collision** mesh ("use complex collision as simple"); if it is coarser
  than the visual mesh the sonar image of that object "will be inaccurate" **[doc]** (same page).
- Octree sonars need their octree regenerated whenever the scene changes; raycast and GPU sonars need
  none **[doc] [paper]**.

## 3. Coordinates and georeferencing

- Right-handed, x forward, y left, z up, metres; rotations as roll, pitch, yaw in degrees about fixed
  axes; Unreal itself is left-handed and in centimetres **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/usage/units-and-coordinates.html>).
- The GPS sensor returns a local `[x, y, z]` in metres, only near the surface; no latitude or longitude
  **[doc]**.
- This project's world frame is ENU about a GPS anchor (`core/geo.py`); a rotation and an anchor
  transform would be needed and tested **[inf]**.

## 4. Surface vessels and dynamics

- The only surface agent is `SurfaceVessel`, "a simple surface vessel with 2 thrusters in the rear"
  **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/agents/agents/surface-vessel-agent.html>); the
  preview paper bases it on the WAM-V **[paper]**. **There is no BlueBoat model.**
- Control schemes: thruster forces, a PD controller, or custom dynamics (global-frame accelerations),
  for which "all other forces and torques (including gravity, buoyancy, and damping) have been disabled"
  **[doc]**.
- Fossen-based dynamics: `fossen_model: "otter"` with the Otter's parameters; it "does not support to
  configure the parameters yet from the scenario", and the graphics asset "is not linked to the fossen
  model" **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/agents/docs/fossen-based-dynamics.html>).
- A credible BlueBoat would need its own model (mass, inertia, added mass, damping, thrusters) and a
  re-tuning of this project's controllers, which are tuned on Gazebo's thruster and hydrodynamics
  plugins **[inf]**.

## 5. Waves and currents

- FFT waves, new in 2.4.0; "the FFT waves will impact the buoyancy of the surface vessel", with a
  mesh-based buoyancy the docs call "an estimation" **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/env_appearance/waves/waves.html>).
- Currents are a global velocity vector per vehicle; "ocean currents do not work when using Fossen
  Dynamics" **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/agents/docs/currents.html>).
- Whether waves move a vessel driven by custom or Fossen dynamics is not stated; since those modes
  disable buoyancy, probably not **[inf]**.

## 6. Side-scan sonar implementations

Three classes exist in 2.4.0 **[doc]**
(<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/sensors/sensors/sidescan-sonar-sensor.html>):

| Class | Method | Sides | Algorithm known? |
|---|---|---|---|
| `SidescanSonar` | octree of surface leaves | both, from one sensor | yes, from the v0.5.0 source and the IROS 2022 paper |
| `RaycastSidescanSonar` | CPU ray casting on collision meshes | one per sensor | **no**: intensity law, beam pattern and output type are undocumented |
| `GPUSidescanSonar` | "utilizes the rendering pipeline" | one per sensor | **no** |

**Octree `SidescanSonar`** **[src v0.5.0]**
(<https://bitbucket.org/frostlab/holoocean-engine/src/v0.5.0/Source/Holodeck/Sensors/Private/SidescanSonar.cpp>,
`HolodeckSonar.cpp`, `Octree.cpp`); the same law is eq. 4–6 of the IROS 2022 paper **[paper]**
(<https://www.cs.cmu.edu/~kaess/pub/Potokar22iros.pdf>):
- leaves inside the range / azimuth / elevation window are kept when they face the sensor;
- per leaf, `val = R² · cos(incidence)` with `R = (z − z_water)/(z + z_water)` and `z` = density × sound
  speed;
- shadowing: leaves are sorted by range inside each angular bin, and everything after the first range gap
  larger than `ShadowEpsilon` is dropped;
- a bin is the **mean** of its leaves × (1 + Gaussian noise) + Rayleigh noise;
- **no spreading or absorption loss, no time-varied gain, no beam-pattern weighting, no multipath.**

For the 2.x octree sonars the documentation says "Material dependency is only available for raycasting
and GPU rendering sonars" **[doc]**
(<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/sensors/docs/material-dependency.html>).

**Validation.** The IROS 2022 paper validates the imaging sonar against an Oculus M1200d and the
echosounder against a Blue Robotics Ping in a tank; the side-scan is shown simulated only **[paper]**.
No paper found validates any HoloOcean side-scan model against real side-scan data, and none publishes a
per-ping cost for it. A 2026 BYU thesis on the side-scan implementations exists
(<https://scholarsarchive.byu.edu/etd/11165>); its text could not be retrieved.

**Against this project's instrument** (Cerulean Omniscan 450 SS, 450 kHz, 0.5° two-way, 1200 bins,
20 Hz) **[inf]**: none of the three reproduces the device chain this project's downstream stack reads
(Ping-protocol frames, gain ladder, per-ping 16-bit normalisation, transmit ringing, water column); an
adapter would have to rebuild it. The documented ray-count rule for `RaycastSidescanSonar` gives about
6 000–14 000 rays per side per ping at 40 m and 1200 bins for 5–10 m altitude.

## 7. Acoustic materials

- `materials.csv` lists `Material, Density, Speed of Sound`; an unlisted material is appended with
  10 000 for both, "which results in full acoustic reflection" **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/packages/docs/package-structure.html>).
- In v0.5.0 the name matched is the Unreal **render** material of the hit component, and a landscape
  carries one material **[src v0.5.0]**.
- Roughness, incidence-angle laws beyond `cos`, sediment types, frequency dependence and coherent
  effects are not documented for any side-scan class **[doc, by absence]**.

## 8. ROS 2 integration

`holoocean-ros` (<https://github.com/byu-holoocean/holoocean-ros>, MIT, version 0.0.1, public):
- "tested on Humble and Jazzy" (README); the HoloOcean docs name Humble **[doc]**;
- a background thread loops `act` then `tick`; the simulation is free-running, with no external step
  service; `/clock` is published after every tick and stamps use simulation time **[src]**
  (`holoocean_main/holoocean_main/holoocean_node.py`);
- **no TF publisher** (a `TODO` in the code) **[src]**;
- **no sonar encoder at all**: encoders exist for IMU, DVL, depth, pose, velocity, GPS, cameras and a
  few others; an imaging-sonar message is defined but unused; an open pull request adds a
  profiling-sonar message only **[src]** (`interface/sensor_data_encode.py`, pull request 26);
- commands: agent command vectors, depth / heading / speed set-points, a reset service **[src]**.

For this project a bridge node would have to publish `OmniscanProfile` and raw frames with the real
driver's names, QoS and parameters (CM-1 / CM-2) **[inf]**.

## 9. Ground truth

Pose, velocity and dynamics sensors; a semantic camera and a semantic lidar; bounding boxes only through
an editor script. **No semantic side-scan and no per-ping contact list** **[doc]**
(<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/sensors/sensors.html>).

## 10. Licensing and cost

- Last public licence text (Bitbucket, 2024): MIT for the client and the `Holodeck` engine source; the
  engine binaries and Unreal content fall under the Epic EULA
  (<https://bitbucket.org/frostlab/holoocean/src/master/LICENSE>). The current file is gated and was not
  read.
- The Unreal EULA primary pages could not be retrieved (HTTP 403). Check the terms that apply to
  Unreal 5.3 and to the institution before any adoption.
- Costs beyond licences **[inf]**: an Epic-linked GitHub account, an Unreal 5.3 editor installation for
  any custom level, a Windows machine or a troubleshooting budget on Linux, harbour asset authoring, a
  BlueBoat dynamics model, a sonar bridge, labels, and re-validation of every downstream module.

## 11. Performance

- `ticks_per_sec` defaults to 30; a sensor rate must be below it. The docs claim "simulation speeds of
  up to 2x real time are possible" overall **[doc]**
  (<https://byu-holoocean.github.io/holoocean-docs/v2.4.0/index.html>).
- Single-beam sonar in the 2.0 preview paper: octree 0.055–0.351 s per tick, raycast 0.012 s per tick,
  hardware not given **[paper]**.
- An independent comparison measured the HoloOcean 1.0 imaging sonar at 1.5–3.8 frames per second after
  317–502 s of cache building on an RTX A6000 **[paper]** (<https://arxiv.org/abs/2503.01074>).
- No measured figure exists for any side-scan class.

## 12. The starting reference's open questions, answered

| Question in `HoloOceanRef.txt` | Answer on 2026-11-11 |
|---|---|
| Do `SidescanSonar`, `RaycastSidescanSonar`, `GPUSidescanSonar` exist? | Yes, in 2.4.0 **[doc]** |
| Octree two-sided, raycast and GPU one-sided? | Confirmed **[doc] [src v0.5.0]** |
| Does a sonar model acoustics or geometric visibility? | Octree: visibility + `R² cos` shading + noise **[src v0.5.0]**. Raycast / GPU: not documented |
| Are material density and sound speed used? | Raycast / GPU only, as an impedance; how it combines with angle is not documented **[doc]** |
| Unreal version | 5.3 for all 2.x **[doc]** |
| A BlueBoat model? | No; WAM-V based vessel with optional Otter dynamics **[doc]** |
| Do waves drive vessel motion? | Through mesh buoyancy for the built-in vessel **[doc]**; under custom / Fossen dynamics unknown, probably not **[inf]** |
| ROS 2 distributions | Humble and Jazzy per the bridge README **[src]** |
| Does the bridge publish side-scan data, `/clock`, TF? | `/clock` yes; side-scan **no**; TF **no** **[src]** |
| GPS in WGS84? | No, local Cartesian **[doc]** |
| Georeferenced bathymetry workflow? | None documented **[doc, by absence]** |

## 13. Other simulators checked (side-scan only)

| Simulator | Side-scan | Notes |
|---|---|---|
| Stonefish (<https://github.com/patrykcieslak/stonefish>) | yes, OpenGL raster: `cos × restitution`, heuristic noise | GPL-3.0; ROS 2 images |
| DAVE / POSIM (<https://github.com/IOES-Lab/POSIM>) | no (multibeam only) | |
| OceanSim (<https://github.com/umfieldrobotics/OceanSim>) | no (imaging sonar) | Isaac Sim, no vehicle dynamics |
| Gazebo Harmonic / Jetty | no plugin found | |

## 14. A minimal proof of concept, if HoloOcean is ever reconsidered

1. Obtain an Epic-linked GitHub account; record the exact release and engine build.
2. Install the prebuilt Ocean package on a machine with at least 30 GB free; run the stock side-scan
   example.
3. Read the 2.4.0 source of `RaycastSidescanSonar` / `GPUSidescanSonar`; write down the intensity law,
   beam weighting, range law and noise.
4. In a stock world place the `SurfaceVessel` over a flat area with two primitive targets of known size;
   measure the range bin of each target and the shadow length against geometry.
5. Measure the time per ping at 1200 bins and 20 Hz on the available GPU.
6. Only then estimate the custom harbour level, the BlueBoat model and the bridge.

Decision rule used on 2026-11-11: no migration unless step 3–5 show a measurable advantage over the
current renderer on this project's own criteria (`independent_review_2026-11-11.md` §7).

## 15. Could not be verified

Any 2.x engine source (algorithms and file paths of the raycast and GPU side-scan, whether the 2.x
octree side-scan still matches v0.5.0, the current LICENSE and changelog files, the main repository's
issues); the 2026 BYU thesis text; the Unreal EULA primary pages; the Ubuntu versions supported
natively; the hardware behind the preview paper's timing table.
