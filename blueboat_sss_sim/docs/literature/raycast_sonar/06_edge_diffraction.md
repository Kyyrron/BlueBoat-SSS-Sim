# 06 — Diffraction at shadow boundaries and edges

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| BT57 | Biot & Tolstoy, "Formulation of wave propagation in infinite media by normal coordinates with an application to diffraction", *JASA* 29(3):381–391, 1957 | 10.1121/1.1908899 | record only |
| SFV99 | Svensson, Fred, Vanderkooy, "An analytic secondary source model of edge diffraction impulse responses", *JASA* 106(5):2331–2344, 1999 | 10.1121/1.428071 | record only — **full text wanted** |
| Med81 | Medwin, "Shadowing by finite noise barriers", *JASA* 69(4):1060–1064, 1981 | 10.1121/1.385684 | record only |
| Kel62 | Keller, "Geometrical theory of diffraction", *JOSA* 52(2):116–130, 1962 | 10.1364/JOSA.52.000116 | record only |
| KP74 | Kouyoumjian & Pathak, "A uniform geometrical theory of diffraction for an edge in a perfectly conducting surface", *Proc. IEEE* 62(11):1448–1461, 1974 | 10.1109/PROC.1974.9651 | record only |
| Ufi14 | Ufimtsev, *Fundamentals of the Physical Theory of Diffraction*, 2nd ed., Wiley, 2014 | 10.1002/9781118753767 | record only |

(Pass 4 rejected `10.1121/1.385685` because it resolves to another paper. Medwin's barrier paper is
`10.1121/1.385684`, verified here.)

## What they establish

* **Fresnel–Kirchhoff knife edge** (textbook form; Med81 compares it with exact theory for barriers). A
  straight edge at distance `d₁` from the source and `d₂` from the receiver, with clearance `h` (positive
  when the edge cuts the line of sight). Fresnel parameter `v = h √(2(d₁ + d₂)/(λ d₁ d₂))`. One-way
  field ratio `F(v) = ((1 + i)/2) ∫_v^∞ e^{−iπ t²/2} dt` (Fresnel integrals). It gives |F| = ½ (−6 dB)
  on the geometric shadow boundary, an oscillating lit side, and a monotonic decay into the shadow
  (≈ −20 dB at v ≈ 2.4).
* **Exact wedge diffraction** (BT57). The time-domain wedge solution, rewritten by SFV99 as a
  distribution of **secondary sources along the edge** with closed-form strengths. That makes it
  directly usable for finite edges in a ray/path framework, where it is the reference "BTM" model.
  Med81 found the Fresnel–Kirchhoff barrier prediction close to BT57 except at large diffraction
  angles. The specific bounds need the full text.
* **GTD/UTD** (Kel62, KP74): diffraction coefficients for rays hitting edges, uniform across shadow
  boundaries (UTD). **PTD** (Ufi14): PO plus a fringe-wave edge correction.

## Scale here (`phase0_numbers.txt`)

The first Fresnel radius is `√(λR)` = 5.8 cm at 1 m, 12.9 cm at 5 m, 18.3 cm at 10 m and 25.8 cm at
20 m. Catalog objects are 0.1–1 m, so a 0.3 m object at 10 m spans ~1.6 Fresnel radii and its shadow
edge is softened over a width comparable to the object. Pass 4 measured the geometric renderer's hard
boolean shadows 3.4 / 4.6 dB too dark over the first metre (`raycast_handoff_from_pass4.md` §3).
Diffraction is **included**.

## Model chosen (simplest one valid at √(λR) ≈ 0.1–0.2 m)

**Ray-traced Fresnel–Kirchhoff on each leg.** This is the Kirchhoff diffraction integral over a plane
perpendicular to the leg at the occluder. Visibility through that plane is sampled by rays against the
**real** geometry, so it works for any occluder shape: box edge, wall top, pontoon draft, a pile of
rough objects.
`U/U₀ = (−i / (λ d_eff)) ∬ V(ξ) e^{iπ|ξ|²/(λ d_eff)} d²ξ`, with `d_eff = d₁ d₂ / (d₁ + d₂)`.
- `V` is the ray-cast visibility of source → ξ → receiver, sampled over a disc of a few Fresnel radii;
- for an unobstructed disc it returns 1, and for a straight edge it returns `F(v)`, which is the gate;
- monostatic two-way: the out and back legs pass the same edge, so the leg factor enters twice
  (amplitude squared).

It is only evaluated for legs whose sampled Fresnel disc is partly occluded (a cheap ring test first),
so fully lit and deep-shadow legs cost one ray. The SFV99 secondary-source model is the upgrade if the
Kirchhoff error at large diffraction angles matters (deep shadow behind walls). That decision is made
by the knife-edge-vs-BTM comparison once the SFV99 text is in hand.

## How it maps to the renderer

`diffraction.py`: the ring test and the Fresnel-disc quadrature, with stratified samples, a count set by
the Phase 5 study, and visibility from the same BVH oracle. It is applied in `paths.py` to every leg.

## Validation (Phase 2 gate)

* Straight knife edge (a vertical plate in the mesh): `|U/U₀|` within 0.5 dB of `|F(v)|` for
  v ∈ [−2, 3], phase within 0.1 rad.
* Shadow profile of a box on a flat floor: the soft edge width scales as `√(λR)` (fitted slope 0.5 ± 0.05
  in log–log over R = 2–20 m).

## Caveats

* Kirchhoff diffraction ignores the edge's material and its polarisation analogues. For a
  pressure-release vs rigid wedge the BTM coefficients differ, and the knife edge is the soft/hard
  average. Acceptable at the 0.5 dB level only away from deep shadow.
* The shadow *floor* is not a diffraction question. It is reverberation, sidelobes, multipath and volume
  scattering (notes 08, 09), all of which the sensor renders.

## Full-text reading, 2026-11-03 (SFV99, owner-supplied PDF)

Equations, read in the text:
* Biot–Tolstoy edge-diffraction impulse response of a rigid wedge (eq. 1):
  `h(τ) = −(c ν / 2π) β(τ) / (r_S r_R sinh η(τ)) H(τ − τ0)`, with `ν = π/θ_w` and
  `τ0 = L0/c`, `L0 = [(r_S + r_R)² + z_R²]^{1/2}`. Here:
  - `β = β₊₊ + β₊₋ + β₋₊ + β₋₋` (eq. 2);
  - `β±± = sin[ν(π ± θ_S ± θ_R)] / (cosh[ν η] − cos[ν(π ± θ_S ± θ_R)])` (eq. 3);
  - `η = cosh⁻¹[(c²τ² − (r_S² + r_R² + z_R²)) / (2 r_S r_R)]` (eq. 4);
  - pressure-release (soft) wedge: `β_soft = −β₊₊ + β₊₋ + β₋₊ − β₋₋` (eq. 5).
* **Secondary edge sources** (eq. 19):
  `h(τ) = −(ν/4π) β[α(τ), γ(τ), θ_S, θ_R] / (m(τ) l(τ)) · dz/dτ`, with
  `η(α, γ) = cosh⁻¹[(1 + sin α sin γ)/(cos α cos γ)]` (eq. 16), `sin α = z/m`, `sin γ = (z − z_R)/l`.
  - α and γ are exchangeable, so reciprocity holds.
  - The two branches either side of the apex contribute equally; a finite wedge's infinite-wedge part
    scales by ½ (eq. 20).
* **Use here:** the frequency-domain edge integral `P_d = −(ν/4π) ∫ β e^{−ik(m + l)}/(m l) dz`,
  a quadrature of secondary sources along each finite mesh edge (box edges, wall tops, pontoon
  drafts). It is the reference the Fresnel–Kirchhoff method is checked against (D1). It replaces
  that method where the knife-edge differs by > 0.5 dB at large diffraction angles.

