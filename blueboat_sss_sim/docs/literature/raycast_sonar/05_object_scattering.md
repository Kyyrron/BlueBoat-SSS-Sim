# 05 — Object scattering: physical optics, closed forms, corners, thin shells, object–seabed paths

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| KST04 | Knott, Shaeffer, Tuley, *Radar Cross Section*, 2nd ed., SciTech/IET, 2004 — PO closed forms (plate, cylinder, sphere, dihedral, trihedral) | 10.1049/SBRA026E | record only — **full text wanted** for the corner-reflector formulas |
| Pie19 | Pierce, *Acoustics*, 3rd ed., Springer, 2019 — Kirchhoff/PO, plane-wave reflection | 10.1007/978-3-030-11214-1 | record only |
| Sta89 | Stanton, "Simple approximate formulas for backscattering of sound by spherical and elongated objects", *JASA* 86(4):1499–1510, 1989 | 10.1121/1.398711 | record only |
| FL77 | Folds & Loggins, "Transmission and reflection of ultrasonic waves in layered media", *JASA* 62(5):1102–1109, 1977 | 10.1121/1.381643 | record only — **full text wanted** (elastic layer at oblique incidence) |
| BL03 | Brekhovskikh & Lysanov 2003, ch. 2 — fluid layer and fluid–fluid reflection | 10.1007/b97388 | record (pass 4) |
| Wil10 | Williams et al., "Acoustic scattering from a solid aluminum cylinder in contact with a sand sediment: measurements, modeling, and interpretation", *JASA* 127(6):3356–3371, 2010 | 10.1121/1.3419926 | record only — **full text wanted** (object–seabed multipath decomposition) |
| Sel85 | Selfridge 1985 (note 03) — densities and speeds of the object materials | 10.1109/T-SU.1985.31608 | full text (local) |
| Project | `sonar/facets.py` (PO patches, `po_amplitude`, `po_bistatic`), `worldgen/materials.py` (`_ACOUSTIC`, `acoustic_reflection`), smoke `[14]` | — | code |

## What they establish

* **Physical optics (Kirchhoff) for smooth bodies** (KST04, Pie19). The scattered field is the integral
  over the lit surface of the tangent-plane field. Monostatic, per patch:
  `(k/2π) cos θ A sinc(k a g·u) sinc(k b g·v) e^{2ikr}`. The cross-section is `σ = 4π|Σ|²`. This is
  exactly `facets.po_amplitude`, and smoke `[14]` holds it against the closed forms below to 0.15 dB.
  Bistatic: `facets.po_bistatic` (pass 4).
* **Closed forms** (`ka ≫ 1`, normal / broadside incidence; KST04):
  - flat plate of area `A`: `σ = 4π A² / λ² = k² A² / π`;
  - circular cylinder, radius `a`, length `L`, broadside: `σ = k a L²`;
  - sphere: `σ = π a²`;
  - dihedral corner (two `a × b` plates at 90°, on its symmetry plane): `σ = 8π a² b² / λ²`;
  - trihedral corner, triangular faces of edge `L`: `σ_max = 4π L⁴ / (3 λ²)`; square faces:
    `12π L⁴ / λ²`.
  The corner values are quoted from standard RCS tables and must be **checked against KST04's text**
  before they gate a test. The ray caster's double- and triple-bounce PO must reproduce them.
* **Aspect dependence** follows from the sinc. A flat face glints within `≈ λ/(2a)` rad of normal:
  0.95° half-width for a 10 cm face at 450 kHz, as the facet layer records ("a box wall glints within
  about 1° of broadside").
* **Elongated and spherical objects** (Sta89): approximate formulas spanning `ka` from Rayleigh to
  geometric. At 450 kHz every catalog object has `ka > 50`, which is the geometric limit, where PO holds.
* **Thin layers** (BL03, FL77). A fluid layer of thickness `d` and impedance `Z₂` between `Z₁` and `Z₃`
  reflects with the three-medium formula. It is transparent at `d = nλ₂/2` (2 mm PP: half-wavelength
  3.0 mm, `worldgen/materials.py`). FL77 extend this to an **elastic** layer at oblique incidence, where
  shear waves add angle-dependent transmission and coincidence features that the fluid-layer formula
  lacks.
* **Object–seabed multipath** (Wil10). A target on sand returns four groups: direct, target→bottom,
  bottom→target, and bottom→target→bottom, plus elastic responses. The paths through the bottom are
  coherent images in the seabed. Sand below its critical grazing angle (30° for SAX99, note 03) reflects
  `|V| → 1` minus loss, times the Eckart coherence `exp(−(2kσ sin θ)²)`. With σ = 5 mm at 4° grazing,
  `C` ≈ 0.18; the dihedral term is strong only where the floor is smooth (the painted pool) or the
  grazing angle is small.

## Project facts

* `pp_shell` transmission 0.11 two-way is **measured** (beach box shadow −9.6 dB; re-measured −8.2 /
  −7.4 dB). The fluid-layer formula gives a 2 mm wall `|R|²` = 0.16, i.e. 0.84 transmitted one way per
  crossing at normal incidence. The difference is the oblique lid and floor ("forward mirrors −4 to
  −8 dB per crossing") and the contents. In the ray caster this has to **emerge** from tracing the
  transmitted rays through every wall at its real angle. The 0.11 becomes a validation number, not an
  input.
* Pool target `pe_cement`: a PE layer on a cement half-space (`_ACOUSTIC`, backing concrete). The
  pass-3/4 boxed weight is a nested object, a container with contents. The ray caster gets both as
  meshes, with the layer formula on each interface.

## How it maps to the renderer

`scattering.py` reuses `po_amplitude`/`po_bistatic` for the coherent PO patches of every smooth
triangle set (objects, walls, the calm surface, a smooth floor). `paths.py` provides the double- and
triple-bounce specular chains (dihedral, trihedral, object–seabed images) with full visibility.
`transmission` lives in `scattering.py`: thin-layer `R`/`T` from `_ACOUSTIC` per crossing angle. Thin
walls are zero-thickness interfaces carrying the layer formula; the 2–3 mm offset is below the bin
size. Rays continue inside flooded bodies, so the contents and inner faces scatter.

## Caveats

* PO misses edge diffraction (note 06) and creeping waves. Creeping waves are negligible at `ka > 50`
  except in deep shadow.
* Elastic resonances and Lamb waves in shells are **excluded** in v1, with a reason to check: PP and PE
  shear speeds (~1100–1300 m/s) are below water's, so there is no shear coincidence. The compressional
  plate modes radiate as the layer formula already describes. Needs FL77 to confirm.
* The corner-reflector closed forms are not yet text-checked (KST04 full text wanted).

## Full-text reading, 2026-11-03 (Wil10, FL77, Hwang15; owner-supplied PDFs)

* **Wil10** (a 61 cm × 30.5 cm aluminium cylinder on flattened sand, 1–30 kHz): target strength vs
  aspect and frequency, and SAS images. The physical-acoustics interpretation and the FE models use
  **image cylinders** for the target/interface interaction; a finite-range receiver and a fuller
  Green's function refine it.
  - This validates the mirror (image) construction of this sensor's object–seabed paths.
  - Its numbers are low-frequency elastic responses (low ka) and do not transfer to 450 kHz.
* **FL77:** the transmission and reflection of plane waves at **oblique incidence** through n parallel
  solid layers, with **shear and longitudinal** waves, in a transfer form for computation. Measured
  at 100–700 kHz on one to three plates in water, agreeing with the computation. This is the method
  for thin elastic shells (`pp_shell`, PE) in the transmission step, replacing the fluid-layer formula
  where shear matters.
* **Hwang15:** the PO RCS of polygonal plates (the Kirchhoff/Gordon edge-sum form, exact for flat
  polygons) and **multi-bounce by GO path finding + PO at the last bounce, with an effective area**.
  - This is the scheme of this sensor's specular chains.
  - The corner closed forms follow from it: dihedral `8π a²b²/λ²`, trihedral `4π L⁴/(3λ²)` (O4, O5).

