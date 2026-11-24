# Surveying for the "depth from sonar recording" tool

`world_builder` → Screen 3 → **From sonar recording (.svlog)** turns a
recording into the world's bathymetry. This note says what the tool
derives from a recording, and therefore how to sail so the map it makes
is worth building a simulation on. The mechanics live in
`blueboat_sss_sim/analysis/bathymetry.py` (soundings) and
`worldgen/depth/smooth.py` (the surface); the configuration keys in
`configuration_guide.md` ("depth", mode `svlog`).

## 1. What the tool measures — and what it cannot

The Omniscan's **first bottom return** (FBR) is the shortest slant range
to the seabed. For a transducer a few centimetres off the hull's
centreline that is, to a few centimetres, the **depth under the boat**:
one sounding per ping, placed at the boat's GPS fix, depth = FBR
altitude + the transducer's submersion (`sensor_depth_m`, 0.15 m) + the
datum offset you enter (tide / draft). Between the lines you sailed the
surface is **inferred** — a smoothing thin-plate spline through the
along-track cell medians — and beyond the outermost line the slope
measured at the edge is **continued and damped** (it flattens out over
the *trend length*, 15 m by default).

Across the swath the imagery does carry depth information, in its
shading (2026-09-16, *Relief between lines: shading*): on a Lambertian
seabed brightness is slope, so the tool reads the across-track tilt out
of every lit bin, integrates it outward from the depth under the boat
and keeps the **relief** about the along-ray trend — sand waves, scour,
a bank — which replaces the guessed undulation where the swath was lit
(`seabed_model.md` §7.1). Its *amplitude* is only known to a factor of
about 1–2 (the sonar's Lambert exponent is measured over sand only —
`sonar_model.md` §6 — not over the bottom being surveyed), so
leave the relief gain at 0 to have it **calibrated on the tie lines**:
wherever another line's bottom return crosses a swath, the relief the
FBR measured along that line is what the shading relief is fitted to.
Where you chose `synthetic` instead, a gentle undulation whose size is
set by how rough the recording itself measured (the residual about the
fit: 5–15 cm on the field corpus) is added between the lines, tapered to
zero within 2 m of a sounding, so a flat beach stays flat and a rippled
harbour floor ripples.

Consequences:

* the map is trustworthy **along the lines**, plausible **between** them
  if they are close enough, and a damped guess **beyond** them;
* walls must be drawn on Screen 2 first — soundings on one side of a
  quay never influence the other side, and the trend never crosses one;
* the dry margin of a beach is never measured (the boat cannot go
  there): it comes from the continued slope, or from the depth brush.

## 2. Pattern

**A lawnmower whose primary lines run along the depth gradient**
(shore-normal on a beach, across a channel), with two or more tie lines
along the contours. The dense direction should be the one that changes
most: along a line the tool gets a sounding every 5 cm (20 pings/s at
~1 m/s), across lines it gets whatever spacing you chose.

| Setting | Harbour basin, gentle slopes | Beach / channel bank |
|---|---|---|
| Primary line spacing | 5–10 m | 3–5 m |
| Primary line direction | across the basin's long axis | shore-normal (down the slope) |
| Tie lines | 2, across the primaries | 2–3, shore-parallel at different depths |
| Trend length to accept | 15 m (default) | 10 m |

Rules of thumb:

* keep the spacing at or below the trend length you will accept, and
  below twice the size of any feature you want to see (a 6 m-wide
  dredged channel needs ≤ 3 m lines across it);
* cover to within ~5 m of the limits you will draw: beyond the last
  line the map is the damped continuation, which saturates at
  `trend length × slope`;
* sail straight lines and put the turns *outside* the area — turns are
  fine for the FBR but the GPS lags the boat through them;
* cross every primary with at least one tie line: the tool reports the
  **crossover residual** (depth difference where lines meet more than a
  minute apart) on Screen 3 and in `metadata.yaml`; a residual above
  ~0.2 m means a tide change, a bad fix or a wrong datum offset.

## 3. Boat and sonar settings

* **Speed ≤ 1 m/s** (the corpus surveys ran at 0.9 m/s): one ping every
  50 ms is a sounding every 5 cm.
* **Range** 15–20 m at the 3–9 m depths of a harbour (about 3–4 × the
  deepest expected depth): the bottom return must sit past the
  transmit ringing (the first ~1 m of slant range) and well inside the
  range. Keep `range_length_mm` constant for the whole survey.
* **Gain**: auto-gain is fine for depth, and for the reflectivity layer
  too — every bin is brought back to the calibration gain through the
  measured ladder. A fixed `gain_index` (4, the calibration rung) keeps
  every bin off the ladder entirely; the calibration's old ±6 dB auto-gain
  caveat was retired on 2026-09-18 (`sonar_model.md` A8).
* **Tie lines** matter twice now: they give the crossover residual *and*
  calibrate the shading relief's amplitude. Two or three, across the
  primaries, at least a minute apart from them.
* **GPS**: a 3-D fix before you start, fixes at ≥ 5 Hz (the corpus
  carries 25 Hz `GLOBAL_POSITION_INT`). Heading comes from the ping
  itself, not from the GPS.
* **Recording software**: a SonarView-native recording carries a live
  autopilot clock and the tool interpolates every ping onto it. A
  recording written by `sss_processor_node` carries one *frozen*
  `time_boot_ms` on every navigation packet (3 of the 7 corpus files):
  the tool then pairs each ping with the most recent fix in file
  order, which is within one ping period (5 cm) of the truth at 25 Hz.
  Screen 3 shows which regime it used (`clock live` / `clock file
  order`). The frozen clock is a `BlueBoat-SSS` item.
* **Water level**: note the tide / water level at survey time and enter
  it as the **datum offset** (positive when the water stood higher than
  the level you want the world referenced to).

## 4. What "good" looks like on Screen 3

| Readout | Expect | If not |
|---|---|---|
| pings with a bottom return | ≥ 60 % | range too long / too short, or the boat sat still over one spot |
| altitude range | 2–15 m | outliers: a quay face or a wake inside the swath; they are rejected, but check the map |
| cells | tens to a few thousand | a single straight line gives a few tens: the tool then holds the nearest value sideways (no across-track information — sail a second line) |
| crossover residual | < 0.2 m | datum / tide, or a bad fix |
| clock | live | file order is fine; it only says the recording came through the processor |
| coverage | as much of the area as you can | the share of the world within 15 m (the trend length) of a sounding; below 35 % the readout warns, and the rest of the seabed is the edge slope continued, not a measurement |
| relief from shading | most of the survey hull; passes disagree by < 0.3 m; gain calibrated on tie soundings | a low share means a dark swath (short range, a floor reached late); a large pass disagreement means the shading is not slope there (a moving object, a wake, a wall); "default" for the gain means no tie line crossed a swath — sail one |
| reflectivity (Screen 3 layer) | thousands of cells, a 6–25 dB spread | a narrower spread is a uniform bottom; the classes are relative to the survey median (assumed sand) — change the reference material if the site is mud |

Expected accuracy on the lines: ±0.1–0.2 m (FBR bin size 2.5 cm, fit
residual 5–15 cm measured on the field corpus).

## 5. Checklist

1. Plan lines along the depth gradient, 3–10 m apart, plus two tie
   lines; extend them 5 m past the area you will draw.
2. GPS fixed, range 15–20 m constant, gain 4 if you want the
   backscatter layer, speed ≤ 1 m/s.
3. Record the water level and time.
4. In the World Builder: pick the anchored path, draw limits and walls,
   choose the recording, enter the datum offset, check the readout,
   then paint the dry margin or any unsurveyed corner with the depth
   brush (depth ≤ 0 is land).

## 6. Troubleshooting

**"no depth sample lands inside the world limits — the recording and the
drawn rectangle do not overlap"**

The world is anchored on the **path** chosen at Screen 1, and a recording
places its soundings by their own GPS. The two must be of the same place.
The message names both: what the recording covers, what the world covers,
and how far apart the two centres are. Read the distance first.

| Distance in the message | What happened | Fix |
|---|---|---|
| kilometres | the recording is of another site, or another path was selected at Screen 1 | pick the recording of this area, or the path this recording was sailed on |
| tens of metres | right site, wrong rectangle — the track passes just outside the limits | go back to Screen 2 and redraw the rectangle over the track |
| "carries no positioned depth sample at all" | the file has no usable GPS fix | use a recording made with a GPS fix; the tool cannot place soundings without one |

A recording that only *partly* covers the area is not an error: it builds,
and the coverage line says how much of the world was measured. Below 35 %
it warns. Everything outside is the damped trend — the slope at the edge
of the data, continued and saturating over the trend length — so treat it
as a plausible seabed, not as a survey.

The same rules apply to a CSV or GeoTIFF depth file; the message says
"the file" instead of "the recording".

**"the depth looks like the boat's track, printed"**

It was, before 2026-09-18. Each pass leaves a hole at its own nadir (the
water column carries no bin) and meets its neighbour at a seam, and
rasterising the assembled relief raw turned both into ridges that follow
the survey lines. The relief is now blended over its own coverage
(`relief_smooth_m`, default 0.75 m — a coverage-weighted Gaussian, which
bridges the holes without pulling the surrounding relief toward zero) and
each ray's across-track bias is removed before assembly. What survives is
the bedform band the method can resolve: 0.75 m keeps 90 % of a 10 m
ripple. If a survey still prints its lines, the passes disagree — read
`overlap_rms_m` in the quality line, and see "the two passes disagree"
below.

**"selecting the recording takes minutes, every time"**

It did, and it also killed the builder outright on large files: a 180 MB
survey inverted to several gigabytes of samples with no cache, so every
parameter change re-did it. Since 2026-09-18 the inversion is cached per
(file, modification time, size, anchor, settings) and runs in along-track
chunks, so the first read of a 129 MB recording costs about 1.6 GB and
every later one is free. A file you *edit* is re-read: the cache key
carries its modification time.
