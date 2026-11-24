"""``reflectivity_survey`` -- the seabed reflectivity a recording measured,
as a layer over the recording's own depth source.

The layer reads the ``.svlog`` of the source below it (``LayerContext.
svlog_path``), inverts every profile bin with
:mod:`blueboat_sss_sim.analysis.backscatter` (the calibrated forward model
removed per bin, the near-nadir specular cone and the shadow bins left
out, the residual angular slope removed as Coiras' regularisation does
in its one-parameter form) and rasterises Lambert's coefficient per
ground cell. Two channels come out of that raster:

* **gain** -- ``mu - median(mu)`` in dB: the recording's median ground
  renders as the ``reference`` material (sand unless told otherwise, the
  survey median being the best guess of what most of the seabed is) and
  every cell brighter or darker than it is a continuous reflectivity
  gain, through the compositor's gain channel;
* **material** -- a coarse classification by the contrasts the
  literature measures between sediment classes at hundreds of kHz:
  Jackson & Richardson 2007 [B2] par. 12.1 fit Lambert's coefficient at
  mud -19.7, sand -20.2, gravel -11.7 and rock -5.6 dB over the compiled
  measurements (grazing < 60 deg), with frequency slopes of +3 / +6-8 /
  +3 / 0 dB per decade (par. 12.1.2); carried to 450 kHz from the
  ~30 kHz compilation that puts sand about 5 dB above mud, gravel ~3 dB
  and rock 5-8 dB above sand -- the same ordering and size the ESAB
  multifrequency model measures at 290-440 kHz (Fonseca et al. 2025
  [B4]: sandy mud 3-5 dB below sand in the oblique regime, bedrock and
  maerl highest). The thresholds are the layer's parameters, so a survey
  with a known bottom can move them.

Both are honest only within what the physics allows: the within-class
spread of scattering strength is +-5-8 dB ([B2]: "sediment name is not a
good predictor of scattering strength"; a hard/soft separation is what
is defensible), the absolute level carries the calibration's +-6 dB AGC
caveat, and the relative map is good to the 1-2 dB the single-look
speckle leaves after averaging ~10^3 bins per cell.

The layer covers only what was insonified: its default mask is the
measured cells feathered over ``feather_m``, so a partial survey shows
the base material everywhere else.
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

from ...materials import MaterialLibrary
from .. import geometry as geo
from . import LayerContext, LayerOutput, LayerSpec, register

_NAMES = MaterialLibrary().seabed_names

SURVEY_LAYER_ID = "svlog_backscatter"     # the builder's reserved row id

_SCHEMA = (
    ("reference", "Reference material (the survey median)",
     "choice:sand|mud|gravel|rocks", "sand", 0, 0,
     "The material the recording's median reflectivity is taken to be; "
     "brighter and darker ground is rendered relative to it."),
    ("mud_below_db", "Mud below [dB]", "float", -3.5, -20.0, 0.0,
     "Cells this many dB (or more) below the survey median are classed as "
     "mud: sand sits ~5 dB above mud at 450 kHz (Jackson & Richardson 2007 "
     "par. 12.1; Fonseca et al. 2025), half of it is the boundary."),
    ("gravel_above_db", "Gravel above [dB]", "float", 2.5, 0.0, 20.0,
     "Cells this many dB above the survey median are classed as gravel "
     "(gravel ~3 dB above sand at 450 kHz, Jackson & Richardson 2007 "
     "par. 12.1)."),
    ("rocks_above_db", "Rocks above [dB]", "float", 5.5, 0.0, 30.0,
     "Cells this many dB above the survey median are classed as rock "
     "(rock 5-8 dB above sand at 450 kHz, Jackson & Richardson 2007 "
     "par. 12.1; the within-class spread is +-5-8 dB, so treat the class "
     "as hard/soft rather than a sediment name)."),
    ("cell_m", "Cell [m]", "float", 1.0, 0.25, 10.0,
     "Size of the ground cell every inverted bin is averaged into "
     "(~10^3 single-look bins per 1 m cell on a 20 Hz survey)."),
    ("smooth_m", "Smoothing [m]", "float", 3.0, 0.0, 20.0,
     "Gaussian smoothing of the measured map before it is applied and "
     "classified; zero keeps the raw cells. The within-class spread of "
     "scattering strength is +-5-8 dB (Jackson & Richardson 2007 par. "
     "12.1), so structure finer than a few metres in the class map is not "
     "defensible; smoothing before the thresholds also stops single bright "
     "cells speckling into islands of another class."),
    ("feather_m", "Coverage feather [m]", "float", 4.0, 0.0, 20.0,
     "How far beyond the insonified cells the layer fades out over the "
     "base material."),
)


def _smooth_nan(field: np.ndarray, sigma_cells: float) -> np.ndarray:
    """NaN-aware Gaussian smoothing (normalised convolution)."""
    if sigma_cells <= 0.0:
        return field
    ok = np.isfinite(field)
    filled = np.where(ok, field, 0.0)
    w = ok.astype(np.float64)
    num = ndimage.gaussian_filter(filled, sigma_cells)
    den = ndimage.gaussian_filter(w, sigma_cells)
    out = np.where(den > 0.05, num / np.maximum(den, 1e-6), np.nan)
    return np.where(ok, out, np.nan)


def survey_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    from ....analysis.backscatter import mu_raster, resample_to_grid, summarize
    if not ctx.svlog_path:
        raise ValueError("reflectivity_survey needs a recording (.svlog) "
                         "depth source below it")
    if ctx.anchor is None:
        raise ValueError("reflectivity_survey needs a GPS-anchored world")
    r = mu_raster(ctx.svlog_path, ctx.anchor, float(p["cell_m"]))
    mu = resample_to_grid(r, grid)
    mu = _smooth_nan(mu, float(p["smooth_m"]) / grid.resolution)
    finite = np.isfinite(mu)
    if not finite.any():
        raise ValueError("the recording insonified nothing inside the world "
                         "limits -- no reflectivity to apply")
    ref = float(np.nanmedian(mu))
    rel = np.where(finite, mu - ref, 0.0)
    # Material classes relative to the survey median (see module doc).
    names = [str(p["reference"])]
    for extra in ("mud", "gravel", "rocks"):
        if extra not in names:
            names.append(extra)
    ids = np.full((grid.ny, grid.nx), -1, dtype=np.int16)
    ids[finite] = names.index(str(p["reference"]))
    ids[finite & (rel <= float(p["mud_below_db"]))] = names.index("mud")
    ids[finite & (rel >= float(p["gravel_above_db"]))] = names.index("gravel")
    ids[finite & (rel >= float(p["rocks_above_db"]))] = names.index("rocks")
    d = geo.edt_outside(finite, grid.resolution)
    mask = geo.band_weight(d, 0.0, float(p["feather_m"]))
    info = {"backscatter": dict(summarize(r, str(p["reference"])),
                                cells_in_world=int(finite.sum()),
                                reference_db=ref)}
    return LayerOutput(material=(ids, names), material_edge="dithered",
                       gain_db=rel, default_mask=mask, info=info)


register(LayerSpec(
    kind="reflectivity_survey", fn=survey_fn,
    label="Reflectivity from recording", family="material", schema=_SCHEMA,
    geometry=(), channels=("gain", "material"),
    reference="[33] Jackson & Richardson 2007 par. 2.3, 12.1; [39] Mackenzie "
              "1961; [41] Fonseca et al. 2025; [45] Coiras et al. 2007",
    consequence="The recording's own light and dark ground, rendered as a "
                "continuous gain about the reference material, with hard "
                "(gravel, rock) and soft (mud) patches classed by the "
                "literature contrasts.",
    blend="normal"))
