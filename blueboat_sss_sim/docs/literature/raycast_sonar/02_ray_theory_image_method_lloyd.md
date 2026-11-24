# 02 — Ocean-acoustic ray theory, the image method, Lloyd's mirror, straight-ray validity

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| JKPS11 | Jensen, Kuperman, Porter, Schmidt, *Computational Ocean Acoustics*, 2nd ed., Springer (MASP), 2011 — §2.4 ideal waveguide / image sum, ch. 3 ray theory | 10.1007/978-1-4419-8678-8 | record; the image sum is already implemented and closed-form tested in `sonar/waveguide.py` (pass 4) |
| PB87 | Porter & Bucker, "Gaussian beam tracing for computing ocean acoustic fields", *JASA* 82(4):1349–1359, 1987 (BELLHOP) | 10.1121/1.395269 | record only |
| AB79 | Allen & Berkley, "Image method for efficiently simulating small-room acoustics", *JASA* 65(4):943–950, 1979 | 10.1121/1.382599 | record (pass 4) |
| BL03 | Brekhovskikh & Lysanov, *Fundamentals of Ocean Acoustics*, 3rd ed., Springer, 2003, ch. 2 | 10.1007/b97388 | record (pass 4) |
| Car09 | Carey, "Lloyd's mirror — image interference effects", *Acoustics Today* 5(2):14, 2009 | 10.1121/1.3182842 | record only |
| Bell97a | Bell & Linnett 1997 (note 01) — constant-gradient layered ray tracing for SSS | 10.1049/ip-rsn:19971311 | full text |
| Mac81 | Mackenzie, "Nine-term equation for sound speed in the oceans", *JASA* 70(3):807–812, 1981 | 10.1121/1.386920 | record; formula in J&R App. B.2 |
| BW93 | Bilaniuk & Wong, "Speed of sound in pure water as a function of temperature", *JASA* 93(3):1609–1612, 1993 | 10.1121/1.406819 | record only |

## What they establish

* **Ray (WKB) approximation.** It is valid when the sound speed varies little over a wavelength and the
  ray's radius of curvature is much larger than λ. At 450 kHz λ = 3.3 mm, so both hold everywhere in
  water (Bell97a uses this to justify ray tracing for imaging sonar). The ray approximation fails at
  caustics and shadow boundaries; that is diffraction, note 06.
* **Constant-gradient layers** (Bell97a, JKPS11 ch. 3): a ray is a circular arc of radius
  `R_c = c / (g cos θ)`, so travel time and arrival angle are closed form per layer.
* **Image method** (AB79, JKPS11 §2.4). For a source between two parallel planar boundaries (the
  surface `z = 0` and a flat floor at depth `D`), the field is the sum over images at
  `z = 2mD ± z_s`. Each image is weighted by `R_surf^{n_s} R_floor^{n_f}` and by its own spreading and
  phase. In a closed room, mirroring repeatedly in every wall gives every specular path; each needs a
  **visibility check**, because an image is valid only where its reflection point lies inside the
  reflecting face and every leg is unobstructed. (AB79 do not do this for convex rooms; it is the
  standard extension for non-convex geometry.)
* **Lloyd's mirror** (Car09, JKPS11). A source at depth `z_t` below a pressure-release surface (R ≈ −1)
  and a point at depth `z_o`, range `r`. Direct and surface-image paths differ by
  `Δ ≈ 2 z_t z_o / r` for `r ≫ z_t, z_o`, so the one-way intensity is
  `|1 − e^{ikΔ}|²/r² = 4 sin²(k z_t z_o / r)/r²`. It has lobes at `r_n = 2 z_t z_o / ((n + ½) λ)` and a
  null on the surface. For our transducer, `z_t = 0.22 m` and `λ = 3.33 mm`:
  - on the floor at 0.9 m depth (pool): `2 z_t z_o / λ = 119 m`, far beyond the 20 m swath, so the
    whole floor is in the **near-interference region** (r < 2 z_t z_o / λ). There the two paths add
    with a phase that rotates many times per range bin across the beam, and the fringes are
    range-dependent;
  - for a small object near the floor or proud, fringes in range with spacing
    `δr ≈ r² λ / (2 z_t z_o)`: ≈ 0.21 m at 5 m range, 0.84 m at 10 m, for `z_o = 0.9 m`;
  - two-way, the monostatic echo has three distinct delays (direct–direct, direct–surface twice,
    surface–surface), which interfere inside one resolution cell only when
    `Δ < c τ / 2 = 3.3 cm`, i.e. at `r > 2 z_t z_o / 0.033 ≈ 12 m` for `z_o = 0.9 m`. Nearer, they
    are separate arrivals. This is exactly the pool FBR ramp of the pass-4 hand-off:
    direct at 0.6 m, mixed at ≈ 0.9 m, surface–surface at 1.12 m.
* **Sound speed** (Mac81, J&R App. B.2; BW93 for fresh water). Seawater: `c ≈ 1412 + 3.21 T + 1.19 S`
  `+ 0.0167 z` (the simplified form used inside Francois–Garrison, J&R eq. B.10; the full nine-term
  equation is Mac81). Fresh water at 20–25 °C is about 1482–1497 m/s (BW93). `dc/dT ≈ 3 m s⁻¹ °C⁻¹`.

## Straight-ray validity here (numbers, `~/ros2_ws/sim_runs/raycast/notes/phase0_numbers.txt`)

Constant gradient `g = (dc/dT)(dT/dz) ≈ 3 dT/dz s⁻¹`; a 20 m leg's vertical sagitta is `x²/(2R_c)`;
the angle changes by `x/R_c` against the 4.3° grazing angle at 20 m slant from 1.5 m altitude.

| dT/dz | R_c | sagitta at 20 m | Δangle at 20 m | effect on a sin²-type law at 4.3° |
|---|---|---|---|---|
| 0.02 °C/m (well-mixed pool) | 25 km | 0.8 cm | 0.05° | 0.1 dB, negligible |
| 0.1 °C/m | 5 km | 4 cm | 0.23° | ≈ 0.5 dB |
| 0.5 °C/m (calm, sunny harbour surface layer) | 1 km | 20 cm | 1.15° | +2.1 / −2.7 dB at 20 m (sign of the gradient) |
| 1 °C/m | 0.5 km | 40 cm | 2.3° | +3.7 / −6.7 dB at 20 m, range-dependent |

**Decision.** Refraction is *include-optional*: constant-gradient layers, off by default, a profile
given per site. It matters only at far range and only under a real near-surface thermocline. **No
temperature profile exists for any corpus session** (needs a measurement: a thermistor string, or the
autopilot's water-temperature log).

## How it maps to the renderer

`paths.py` builds the eigenray set to every contributor:
- images in `z = 0`, in planar walls, and in the local tangent plane of the floor, refined by Fermat;
- a visibility ray per leg against the whole BVH (`bvh_cpu.py` / GPU);
- an optional constant-gradient curved-leg variant.

`propagation.py` holds the sound speed: Mackenzie for seawater, Bilaniuk–Wong for fresh water. The
ideal-waveguide image sum (`waveguide.ideal_image_sum_closed`) is the closed form the comb path is
gated against.

## Caveats

* For a non-planar floor, the image construction is an approximation, and its error has to be
  measured. Fermat iteration over the true mesh is the reference.
* Gaussian beam tracing (PB87) is not needed at these ranges, since there are no caustics without a
  gradient. It would be the upgrade if refraction ever matters near a caustic.
* Lloyd interference only appears if the surface's coherent fraction is high. That holds in a calm pool
  and fails in waves (note 08).
