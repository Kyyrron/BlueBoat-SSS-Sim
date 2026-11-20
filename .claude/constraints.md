# BlueBoat-SSS-Sim — non-negotiable constraints (full text)

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 4. NON-NEGOTIABLE CONSTRAINTS

Violating any of these breaks another module, the hardware swap, or the
scientific argument.

1. **Interface fidelity is absolute.** Topic names, message types, QoS
   (BEST_EFFORT/KEEP_LAST/10), parameter names, ping-enable semantics = the
   real `sss_node.py`. **Never define a new message type** — reuse
   `blueboat_interfaces`; extra simulation data goes on new additive JSON
   topics (as `ground_truth/contacts`).

2. **`raw` frames must be byte-valid Ping Protocol, checksum included** —
   downstream rebuilds `.svlog` files from them for SonarView. Layout:
   `'B''R' | u16 payload_len | u16 msg_id=2198 | u8 src | u8 dst | 52-byte
   fixed payload | u16[num_results] | u16 checksum(sum of all prior bytes)`;
   frame length `8 + 52 + 2·num_results + 2` (1262 B at 600 bins).
   `parse_frame()` is the round-trip oracle; keep it working.

3. **Both sides must be published from the same timer tick** — the
   simulator's own rule (downstream no longer forces it): the two channels'
   ping counters stay in lockstep, every simulated row two-sided by
   construction.

4. **The first bottom return must survive every change to the acoustic
   model.** Six coupled pieces make downstream bottom tracking lock; remove
   any and the tracker fails silently: (1) the vertical beam weight at nadir
   (at tilt 20°/FWHM 50° the Gaussian tail alone gives 0.00436 there, so the
   FBR rides on it and `beam_sidelobe_floor` (0.004) is the guard when the
   fan is rolled up >0.55°, the tilt reduced or the aperture narrowed; keep
   w(nadir) ≥ ~0.004); (2) the specular near-nadir lobe
   (`specular_strength`; 1.75 since 2026-10-30 = the FBR-crest fit raised to
   the smallest value with no regression of `calibration.seabed.fbr_envelope`
   -- sand and seagrass, 0.6-22 m, the corpus acquisitions; any acoustic
   change must keep that envelope, which caught the first pass-2 candidate
   losing the FBR at 7-21 m); (3) the specular component rendered and speckled
   **separately** from the diffuse field, low CV (`specular_looks`); (4)
   pulse-length range smearing (`pulse_smearing`); (5) (2026-09-18) that
   component's **own range law** (`specular_spreading_exponent` 2.0: a
   coherent reflection is an image source, spreading once over the folded
   path, not twice like the diffuse field) — it makes the FBR follow the
   device's measured altitude law instead of falling two to three times too
   fast; set equal to `spreading_exponent`, the pre-2026-09-18 behaviour is
   back exactly; (6) (2026-09-19) the coherent channel keeps the per-sample
   **mean** normalisation under `image_formation: energy` (`_Acc.den_s`) —
   an image source's aperture is a Fresnel zone, not a resolution cell;
   area-weighting it puts the nadir `r/y` growth on the FBR (measured: the
   reported peak **8.6 dB** off the corpus anchor). The smoke test
   re-verifies FBR lock with every object family (open-frame lattice classes
   included) in the scene; `[2c]` gates the altitude law itself.

5. **Speckle statistics are load-bearing, not decoration.** Diffuse:
   multiplicative `Exp(1)`; coherent specular: `Gamma(L, 1/L)`, low CV.
   Since 2026-09-02 `Exp(1)` comes from the renderer's coherent
   ground-scatterer field (`Ping.texture`, `coherent_scatterers: true`), so
   it is also *correlated* along range (pulse) and track (footprint overlap)
   as the corpus measures; `Ping.power` stays the noiseless mean,
   `speckle: false` is untouched. Smoke `[2g]` gates mean, CV and both lag-1
   correlations.

6. **Power calibration is anchored to the field corpus, and moves once** —
   through a pre-registered gate, on **both** anchors (`bath3` 1.40 m and
   `reflectionEvidenceFullharbour` 7.33 m; a single-altitude fit cannot
   separate level from slope). Current values: `calibration_db_offset`
   **81.1 dB** (the **only** level constant, fit against the reported level,
   never a count histogram), `tvg_compensation` **0.30**,
   `specular_spreading_exponent` **2.0**, `specular_strength` **1.75**,
   `receiver_ceiling_db` **67.9**, noise floor **3.39e-09**, water column
   **4.68e-06**, `reverberation_db` **18**. Changing any of them silently
   invalidates every comparison against real data. `docs/sonar_model.md` §6
   records which constants are measured, jointly determined, undetermined —
   keep it current whenever one moves. How they got here (2026-09-18,
   2026-10-30, the failed bundle A in `field_candidate_v1_sonar.yaml`):
   `blueboat_sss_sim/blueboat_sss_sim/calibration/CLAUDE.md`.

7. **Never raise launch's global log level to suppress noise.** Filter by
   message content on the screen handler instead.

8. **Never modify the control stack or other packages.** Integrate by
   serving identical interfaces, not by patching neighbours (MCS trajectory
   discovery included: `builder/paths.py` reimplements the predicate rather
   than importing `mcs.*`).

9. **The renderer's ground-sample step must stay coupled to the slant-bin
   size.** A fixed step leaves range bins unpopulated.

10. **World folders are immutable snapshots** — never overwritten by Create
    (name dedupe), Edit (re-saves take a new `_i` folder) or tooling
    (`mission_metrics` etc. write only into `--out`). Regenerate rather than
    edit; `builder_state.yaml` makes regeneration a load-away. Legacy mission
    bundles keep loading (their frozen `sonar.yaml`/`trajectory.yaml` are the
    resolver fallbacks) and are equally write-once.

11. **Project-level (owner's decision, 2026-10-20; superproject CM-11):** the
    headline detector is a YOLO model in BlueBoat-Perception, trained on
    *simulated* datasets that Perception's dataset mode labels from this
    module's ground truth, with its confidence calibrated on held-out
    simulated worlds. Real water validates this simulator (boat motion and
    sensor), runs the heading-response experiment and serves the final
    demonstration; a labelled real dataset blocks nothing. The consequence
    stays stated: a simulation-trained detector inherits its aspect
    sensitivity from this module's model, so the model is what the real
    heading-response experiment bounds, and the module's data are also for
    the policy comparison and development.

12. **Limits never reflect.** World-limit borders are Gazebo collision-only
    geometry, in neither `scene.walls`, the rasters, nor the manifest. Only
    authored walls are acoustic.

13. **No return from land, and no objects on it.** Terrain at or above the
    transducer (a dry beach, an awash bank) scatters nothing back into the
    water (`renderer._ping_geometry`, `dz > 0` mask; the horizon cull hides
    what lies beyond the waterline); random placement never lands above
    `DRY_HEIGHT_M` (−0.05 m). A fully wet world is bit-identical either way.

14. **Nothing ticked means nothing placed.** `objects.classes: {}` (no class
    enabled in the builder) yields zero objects; only a config *without* the
    key falls to the legacy random composition (2026-09-05 fix — an empty
    selection used to scatter 29 objects).

15. **A shadow is a measurement, not a hole** (2026-09-18; extended
    2026-09-19: the floor is referenced to the **ambient** seabed level at
    that range, not the culled bin's own would-be response, which collapses
    to zero on a back-facing flank or over land, put the worst bin 40 dB
    under the device's and drew a hard black band between a simulated object
    and its shadow; `reverberation_reference: unoccluded` restores it).
    Three corpus-anchored rules, each reversible by one constant: a culled
    bin holds the **reverberation floor** (`reverberation_db` 18 below the
    lit seabed at that range, with the seabed's own speckle — the quay
    measures 17–25; `0` restores the white receiver floor that made every
    shadow 30 dB too deep); a **transmitting** material attenuates its own
    shadow rather than zeroing it (`pp_shell` 0.11, measured on the field
    box's 9.6 dB shadow; every other material opaque, as every object was);
    **ground truth is read off the rendered mask**, never from
    `h·R/altitude`, so a label cannot contradict its picture. A detector
    learns a shadow's statistics; a merely *empty* shadow teaches nothing the
    real device will confirm.

16. **A world can have dry land in it, and it is authored, not carved.** The
    `land` layer lifts a drawn area over the waterline and carries the
    seabed up to it, the outline **rounded** by an opening-closing disc
    (`round_m`) so a handful of clicked vertices reads as a shoreline, not a
    polygon. A dredged basin with a negative floor is the hard-edged
    alternative and composites normally rather than being silently swallowed
    by its "deepen" blend. NC #13 does the rest: dry cells return nothing,
    the horizon cull hides what lies beyond.

17. **Objects are rendered from material physics, never tuned per object**
    (2026-09-28, the user's rule: "apply it to any kind of object/material
    ... we don't have to tune this ever again"). Only **material** and
    **geometry** decide how an object returns:
    * **rough** materials (RMS height >= `COHERENT_ROUGHNESS_LIMIT_M`,
      1.6 mm: concrete, rope, tyre tread, generic) and **aggregates** of many
      members (lattice classes, rope, chain, anchor, debris, coils; (2026-11-18) rope and anchor are among the 22
      classes deleted in object set version 2) stay on
      the raster's Lambert path — the right law for a wavelength-rough
      scatterer, bit-identical to before;
      (fired brick, 1 mm, is on the smooth side: C = 6e-7 at normal
      incidence, 0.5 at 77° off it — a facet object, smoke `[14]`);
    * **smooth solid primitives** (`objects.is_facet_object`: a smooth
      material on a box / cylinder / capsule / cone / annulus — any primitive
      `solids.tessellate` knows — proud of the seabed) scatter in the **facet
      layer** (`sonar/facets.py`): coherent Kirchhoff patches scaled by the
      material's reflection coefficient and the Eckart coherent fraction,
      plus the incoherent residual; glints, their aspect dependence (a wall
      within ~1 deg of broadside), edges and layover EMERGE from the
      geometry. In the raster they only occlude (`facet_bump`; the seabed
      beneath dimmed by their two-way transmission, 0 when opaque);
    * a material's acoustics are **physical constants with citations**
      (`_ACOUSTIC` in `worldgen/materials.py`: density, sound speed,
      backing, wall), plus MEASURED properties labelled as such (a shell's
      transmission from a field shadow). The one constant tying an object's
      cross-section to the calibrated seabed is literature
      (`sand_lambert_mu_db`, J&R), never fitted to an object.

    **Adding a class** = its `Solid` primitive (or aggregate mask) and, when
    it is not one solid, its parts (`worldgen/compounds.py`); **adding
    a material** = its physical constants, with a citation, never a
    brightness. A brightness knob on an object or a class is a violation.
    **Ray-cast, `raycast.objects.mode: solid` (2026-11-11, the harbour profiles): one surface rule
    for objects, walls and seabed.** The sheet is the terrain (`depth.npz`); every object is meshed
    at load from the manifest as its parts (`worldgen/compounds.py`: one primitive, or a drum's
    hoops and chimes, a wire pot's rods and mesh, a car tyre's steel-belted tread band ...), each with its material
    (`sonar/raycast/object_mesh.py`); a wall is built of its `material` (concrete | metal, chosen
    in the World Builder). A material is its reflection |R| and RMS roughness: the surface mirrors
    `R·√C` (Kirchhoff patches) and scatters `1 − C` diffusely (`scattering.DiffuseLaw`), C =
    Eckart's coherent fraction; rough materials have C = 0. `raycast.objects.fouling` (0..1, per
    run: node `fouling`) covers that fraction of every object and wall with sediment (seabed-like,
    no mirror) — a sensitivity axis, not a calibration. Still no brightness knob: a class gains
    realism by a better **geometry**, never by a level. `raster` stays the default elsewhere.
    Not modelled: hollow bodies (pipe bore, drum), sheet-pile corrugation, absorption in a tyre's
    rubber (its tread echo is an upper bound).
    New object physics is proved first against textbook closed forms (plate,
    cylinder, sphere — smoke `[14]`), then against the corpus at the field's
    own geometry; `docs/sonar_model.md` §12 carries the derivation, the
    evidence and the limits.

---
