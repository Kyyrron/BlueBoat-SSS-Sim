"""Can a dataset separate these parameters? -- local identifiability.

Before any constant is fitted, ask whether the planned data can tell it
apart from its neighbours. The method is the textbook local one: render
the statistic the fit will compare (here the seabed's absolute dB against
slant range, per altitude -- what ``analysis.calibration`` reduces from a
recording), differentiate it numerically with respect to each candidate
parameter, and read the conditioning of that Jacobian. Under Gaussian
residuals of standard deviation ``sigma_db`` the linearised posterior
covariance is ``sigma^2 (J^T J)^-1``; its correlation matrix names the
ridges and its diagonal the best precision the data could give. A ridge
(|correlation| close to 1) means "constrain one of the pair with
independent evidence, or report the pair, never the individual values".

This reproduces, as a generic tool, what smoke ``[2e]`` pins for one case:
on a flat seabed at ONE altitude ``tvg_compensation`` and
``lambert_exponent`` both act as a slope in log range and trade off almost
exactly; two altitudes separate them (docs/sonar_model.md §6).

It is local (one operating point), noise-free (a deterministic render, so
a ridge here is a property of the parameterisation, not of a sample) and
says nothing about model discrepancy -- a well-conditioned fit of a wrong
model is still wrong (Kennedy & O'Hagan 2001). ROS-free.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass

import numpy as np

from ..core.types import GridSpec, Pose3D, Side
from ..sonar.config import AcquisitionParams, SonarModelConfig

#: The reduction's own repeatability floor (analysis.calibration), used as
#: the default residual scale.
DEFAULT_SIGMA_DB = 0.67


def calibration_acquisition() -> AcquisitionParams:
    """The acquisition every calibration gate measures at (600 bins, pulse
    0.002, gain 4 -- the anchors'; smoke ``_cal_config``)."""
    return AcquisitionParams(range_length_mm=20000, num_results=600,
                             pulse_len_percent=0.002, gain_index=4)


def _flat_scene(depth_m: float, reflectivity: float = 0.55,
                half_m: float = 25.0, res: float = 0.1):
    from ..worldgen.scene import SceneModel
    n = int(round(2 * half_m / res)) + 1
    g = GridSpec(origin_x=-half_m, origin_y=-half_m, resolution=res,
                 nx=n, ny=n)
    return SceneModel(grid=g, height=np.full((n, n), -float(depth_m)),
                      reflectivity=np.full((n, n), float(reflectivity)),
                      material_id=np.zeros((n, n), dtype=np.uint8),
                      material_names=["sand"], objects=[], walls=[], seed=0,
                      config_snapshot={})


def seabed_profile_db(model: SonarModelConfig, acq: AcquisitionParams,
                      altitude_m: float, probes_m: np.ndarray,
                      window_m: float = 0.5) -> np.ndarray:
    """Noise-free absolute seabed level [dB, as the device reports it at
    gain 4] at each probe slant range, over flat sand at ``altitude_m``.

    Mean of the bins within ``window_m`` of each probe (bin centres at
    ``start + (i + 0.5) * bin``, the renderer's convention).
    """
    from ..sonar.renderer import GeometricRenderer
    scene = _flat_scene(altitude_m + model.sensor_depth_m)
    ping = GeometricRenderer(scene, acq, model).render(
        Side.PORT, Pose3D(0.0, 0.0, 0.0), 0.0).ping
    p = np.asarray(ping.power, dtype=np.float64)
    if ping.specular is not None:
        p = p + np.asarray(ping.specular, dtype=np.float64)
    db = 10.0 * np.log10(np.maximum(p, 1e-300)) + model.calibration_db_offset
    r = acq.range_start_mm / 1000.0 + (np.arange(acq.num_results) + 0.5) \
        * acq.bin_size_m
    out = np.empty(len(probes_m))
    for i, pr in enumerate(probes_m):
        sel = np.abs(r - pr) <= window_m / 2.0
        out[i] = float(db[sel].mean()) if sel.any() else np.nan
    return out


def default_probes(altitude_m: float, r_max: float = 14.5,
                   n: int = 12) -> np.ndarray:
    """Probe ranges over the calibration fit's own window: the seabed tail
    past 1.4 x altitude (the near-nadir return is excluded on purpose),
    up to ``r_max``."""
    lo = max(1.4 * altitude_m, 1.0)
    return np.linspace(lo, max(r_max, lo + 1.0), n)


@dataclass
class Identifiability:
    """Result of :func:`analyse` (all arrays ordered as ``params``)."""

    params: list[str]
    altitudes_m: list[float]
    steps: list[float]
    singular_values: list[float]
    condition_number: float
    correlation: list[list[float]]
    sd: list[float]               # linearised posterior sd at sigma_db
    sigma_db: float
    ridges: list[tuple[str, str, float]]
    n_residuals: int

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


def analyse(params: list[str], altitudes_m: list[float],
            model: SonarModelConfig | None = None,
            acq: AcquisitionParams | None = None,
            steps: dict[str, float] | None = None,
            sigma_db: float = DEFAULT_SIGMA_DB,
            ridge_threshold: float = 0.95,
            acqs: list[AcquisitionParams] | None = None,
            sites: list[str] | None = None,
            check_registry: bool = True) -> Identifiability:
    """Local identifiability of ``params`` (``model.<field>`` names) from
    seabed dB-vs-slant profiles at ``altitudes_m``.

    ``steps`` are the central-difference half-steps in each parameter's own
    unit (default 2 % of the value, or 0.02 for a zero value). The
    condition number is that of the Jacobian with columns scaled by those
    steps (sensitivity to a comparable change of each parameter); the
    correlations and sd do not depend on that scaling.

    ``acqs`` (one per altitude) renders each altitude at its own
    acquisition -- the pulse and range a session actually recorded; ``acq``
    is the single acquisition otherwise. ``sites`` (one label per altitude)
    adds a free level per site, ``site_level:<label>`` in ``params``: what
    the data look like when each site carries an unmodelled offset (an
    un-modelled pulse term, a gain error, another bottom). With one pulse
    per site, a pulse-level exponent is then confounded with the site
    levels by construction (2026-10-23). Parameter names are checked
    against the registry unless ``check_registry`` is False.
    """
    model = model or SonarModelConfig()
    if acqs is not None and len(acqs) != len(altitudes_m):
        raise ValueError("acqs needs one acquisition per altitude")
    if sites is not None and len(sites) != len(altitudes_m):
        raise ValueError("sites needs one label per altitude")
    acq_list = list(acqs) if acqs is not None else \
        [acq or calibration_acquisition()] * len(altitudes_m)
    keys = []
    reg = None
    if check_registry:
        from .registry import load_registry
        reg = load_registry()
    for name in params:
        if not name.startswith("model."):
            raise ValueError(f"{name}: only model.* parameters are rendered")
        k = name.split(".", 1)[1]
        if not hasattr(model, k):
            raise KeyError(f"{name}: no such SonarModelConfig field")
        if reg is not None and name not in reg.parameters:
            raise KeyError(f"{name}: not in the parameter registry "
                           "(config/parameter_registry.yaml)")
        keys.append(k)
    hs = []
    for name, k in zip(params, keys):
        v = float(getattr(model, k))
        hs.append(float((steps or {}).get(name, 0.02 * abs(v) if v else 0.02)))
    probes = [default_probes(a, r_max=min(14.5, 0.725 * q.range_max_m))
              for a, q in zip(altitudes_m, acq_list)]

    def stack(m: SonarModelConfig) -> np.ndarray:
        return np.concatenate([seabed_profile_db(m, q, a, pr)
                               for a, q, pr in zip(altitudes_m, acq_list,
                                                   probes)])

    cols = []
    for k, h in zip(keys, hs):
        v = float(getattr(model, k))
        up = stack(dataclasses.replace(model, **{k: v + h}))
        dn = stack(dataclasses.replace(model, **{k: v - h}))
        cols.append((up - dn) / (2.0 * h))
    names = list(params)
    if sites is not None:
        row_site = [s for s, pr in zip(sites, probes) for _ in pr]
        for s in dict.fromkeys(sites):
            cols.append(np.array([1.0 if r == s else 0.0 for r in row_site]))
            hs.append(1.0)                       # one dB of site level
            names.append(f"site_level:{s}")
    jac = np.column_stack(cols)
    ok = np.all(np.isfinite(jac), axis=1)
    jac = jac[ok]
    scaled = jac * np.asarray(hs)[None, :]
    sv = np.linalg.svd(scaled, compute_uv=False)
    cond = float(sv[0] / sv[-1]) if sv[-1] > 0 else float("inf")
    jtj = jac.T @ jac
    cov = sigma_db ** 2 * np.linalg.pinv(jtj)
    sd = np.sqrt(np.maximum(np.diag(cov), 0.0))
    with np.errstate(invalid="ignore", divide="ignore"):
        corr = cov / np.outer(sd, sd)
    corr = np.where(np.isfinite(corr), corr, 0.0)
    ridges = [(names[i], names[j], float(corr[i, j]))
              for i in range(len(names)) for j in range(i + 1, len(names))
              if abs(corr[i, j]) >= ridge_threshold]
    return Identifiability(
        params=names, altitudes_m=[float(a) for a in altitudes_m],
        steps=hs, singular_values=[float(s) for s in sv],
        condition_number=cond, correlation=corr.round(6).tolist(),
        sd=[float(s) for s in sd], sigma_db=float(sigma_db), ridges=ridges,
        n_residuals=int(jac.shape[0]))
