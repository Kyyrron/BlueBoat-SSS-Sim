"""Procedural depth patterns for the environments this project surveys.

Each preset is a pure ``fn(grid, params, rng) -> height`` (z, negative
down) building the *base* bathymetry; dunes/roughness texture from
:mod:`..terrain` is layered on top by ``generate_scene``. Shapes follow
the literature in ``docs/seabed_model.md`` (Dean 1991 for the beach,
Keller & Melhorn 1978 and Ahmed et al. 2024 for the river, the Fisheries
Agency / OCDI / USACE standards for the dredged basins); the World
Builder's layer editor (``.layers``) generalises them to drawn shapes:

* harbour basin -- flat dredged floor at a maintained depth, silt
  shoaling toward the edges, tanh scarp at the dredge line, optional
  deeper navigation channel;
* river channel -- asymmetric parabolic cross-section extruded along the
  flow axis (thalweg offset toward one bank), sinusoidal pool--riffle
  undulation along it;
* beach profile -- concave-up Dean equilibrium ``h = A * x^(2/3)`` from
  the shallow edge, optional longshore sandbar;
* canal -- trapezoidal cross-section extruded along the axis;
* flat / sloped plane -- the two trivial references;
* dredged channel -- a navigation cut through a basin with spoil shoals
  along both sides;
* marina -- pontoon rows over a basin, deeper berth pockets between them
  and scour pits at the piles;
* rocky outcrop -- seeded rounded rock heads over a base depth (heights
  only: the heads keep the base material; the layered editor's
  "outcrops" layer also stamps the rocks material);
* sandbar ripples -- a periodic longshore bar field.

``DEPTH_PRESETS`` maps name -> (fn, label, schema rows); the schema rows
are exactly what the builder's SchemaForm consumes, so a new preset
registered here appears in the GUI with no GUI change. Unknown parameter
keys raise by name (the same quiet-wrong-answer guard as the sonar and
wall configs).
"""

from __future__ import annotations

from typing import Any, Callable, Mapping

import numpy as np

from ...core.types import GridSpec

PresetFn = Callable[[GridSpec, Mapping[str, Any], np.random.Generator],
                    np.ndarray]

#: Schema row: (key, label, kind, default, min, max); kind is
#: "float" | "int" | "bool" | "choice:a|b|c".
SchemaRow = tuple


def _params(params: Mapping[str, Any], schema: tuple[SchemaRow, ...],
            preset: str) -> dict[str, float]:
    allowed = {row[0] for row in schema}
    bad = set(params) - allowed
    if bad:
        raise KeyError(
            f"Unknown depth preset parameter(s) for '{preset}': "
            f"{sorted(bad)} (accepted: {sorted(allowed)})")
    out: dict[str, Any] = {row[0]: row[3] for row in schema}
    out.update({k: params[k] for k in params})
    return out


def _grid_coords(grid: GridSpec) -> tuple[np.ndarray, np.ndarray]:
    xs = grid.origin_x + np.arange(grid.nx) * grid.resolution
    ys = grid.origin_y + np.arange(grid.ny) * grid.resolution
    return np.meshgrid(xs, ys)


def _axis_coords(grid: GridSpec, heading_deg: float
                 ) -> tuple[np.ndarray, np.ndarray]:
    """(along, across) coordinates about the grid centre for an axis at
    ``heading_deg`` (world frame, degrees CCW from +x)."""
    xx, yy = _grid_coords(grid)
    xmin, ymin, xmax, ymax = grid.extent
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    th = np.radians(heading_deg)
    c, s = np.cos(th), np.sin(th)
    along = c * (xx - cx) + s * (yy - cy)
    across = -s * (xx - cx) + c * (yy - cy)
    return along, across


# ----------------------------------------------------------------- presets
_HARBOUR_SCHEMA: tuple[SchemaRow, ...] = (
    ("basin_depth", "Basin depth [m]", "float", 2.5, 0.5, 20.0),
    ("edge_shoal_m", "Edge shoaling [m]", "float", 0.8, 0.0, 5.0),
    ("shoal_width_m", "Shoal width [m]", "float", 6.0, 0.5, 50.0),
    ("channel_enabled", "Dredged channel", "bool", False, 0, 1),
    ("channel_depth", "Channel depth [m]", "float", 4.0, 0.5, 30.0),
    ("channel_width_m", "Channel width [m]", "float", 15.0, 2.0, 100.0),
    ("channel_heading_deg", "Channel heading [deg]", "float", 0.0, -180.0, 180.0),
    ("scarp_smooth_m", "Dredge scarp smoothing [m]", "float", 2.0, 0.2, 10.0),
)


def preset_harbour_basin(grid: GridSpec, params: Mapping[str, Any],
                         rng: np.random.Generator) -> np.ndarray:
    p = _params(params, _HARBOUR_SCHEMA, "harbour_basin")
    xx, yy = _grid_coords(grid)
    xmin, ymin, xmax, ymax = grid.extent
    # Distance to the nearest world edge -> silt shoaling ramp at the toes.
    d_edge = np.minimum(np.minimum(xx - xmin, xmax - xx),
                        np.minimum(yy - ymin, ymax - yy))
    ramp = np.clip(1.0 - d_edge / max(p["shoal_width_m"], 1e-6), 0.0, 1.0)
    depth = p["basin_depth"] - p["edge_shoal_m"] * ramp ** 1.5
    if p["channel_enabled"]:
        _, across = _axis_coords(grid, p["channel_heading_deg"])
        extra = max(p["channel_depth"] - p["basin_depth"], 0.0)
        # tanh scarp at the dredge line (measured 6-18 deg in the field).
        depth = depth + extra * 0.5 * (1.0 + np.tanh(
            (p["channel_width_m"] / 2.0 - np.abs(across))
            / max(p["scarp_smooth_m"], 1e-6)))
    return -depth


_RIVER_SCHEMA: tuple[SchemaRow, ...] = (
    ("axis_heading_deg", "Flow axis heading [deg]", "float", 0.0, -180.0, 180.0),
    ("max_depth", "Thalweg depth [m]", "float", 3.0, 0.5, 15.0),
    ("bank_depth", "Bank depth [m]", "float", 0.5, 0.1, 5.0),
    ("thalweg_offset_frac", "Thalweg offset (frac of half-width)", "float",
     0.3, -0.9, 0.9),
    ("pool_riffle_wavelength_m", "Pool-riffle wavelength [m]", "float",
     40.0, 5.0, 500.0),
    ("pool_riffle_amplitude_m", "Pool-riffle amplitude [m]", "float",
     0.4, 0.0, 3.0),
)


def preset_river_channel(grid: GridSpec, params: Mapping[str, Any],
                         rng: np.random.Generator) -> np.ndarray:
    p = _params(params, _RIVER_SCHEMA, "river_channel")
    along, across = _axis_coords(grid, p["axis_heading_deg"])
    half = 0.5 * (across.max() - across.min())
    u_t = float(np.clip(p["thalweg_offset_frac"], -0.9, 0.9)) * half
    # Asymmetric parabola: each side normalised by its own bank distance,
    # so the offset thalweg makes the near bank steep and the far one flat.
    span_lo = max(u_t - across.min(), 1e-6)
    span_hi = max(across.max() - u_t, 1e-6)
    v = np.where(across < u_t, (u_t - across) / span_lo,
                 (across - u_t) / span_hi)
    v = np.clip(v, 0.0, 1.0)
    depth = p["bank_depth"] + (p["max_depth"] - p["bank_depth"]) * (1.0 - v ** 2)
    if p["pool_riffle_amplitude_m"] > 0:
        # Deepest before the "bend": longitudinal undulation, weighted
        # toward the thalweg so the banks stay put.
        depth = depth + (p["pool_riffle_amplitude_m"]
                         * np.sin(2.0 * np.pi * along
                                  / max(p["pool_riffle_wavelength_m"], 1e-6))
                         * (1.0 - v))
    return -np.maximum(depth, 0.1)


_BEACH_SCHEMA: tuple[SchemaRow, ...] = (
    ("deepening_direction_deg", "Offshore direction [deg]", "float",
     0.0, -180.0, 180.0),
    ("dean_A", "Dean parameter A [m^(1/3)]", "float", 0.10, 0.02, 0.40),
    ("min_depth", "Depth at shallow edge [m] (<= 0: dry beach)", "float",
     0.4, -5.0, 3.0),
    ("bar_enabled", "Longshore sandbar", "bool", False, 0, 1),
    ("bar_distance_m", "Bar distance from shallow edge [m]", "float",
     40.0, 5.0, 500.0),
    ("bar_height_m", "Bar height [m]", "float", 0.6, 0.0, 3.0),
    ("bar_width_m", "Bar width [m]", "float", 8.0, 1.0, 50.0),
    ("berm_enabled", "Submerged berm (step) at the shore", "bool", False, 0, 1),
    ("berm_distance_m", "Berm distance from shallow edge [m]", "float",
     6.0, 0.0, 100.0),
    ("berm_height_m", "Berm height [m]", "float", 0.3, 0.0, 2.0),
    ("berm_width_m", "Berm width [m]", "float", 3.0, 0.5, 20.0),
)


def _shoal_floor(min_depth: float, frac: float) -> float:
    """Shallowest depth a bar/berm may leave: ``frac * min_depth`` for a
    wet edge (bit-identical to the historical floor), the edge itself for
    a dry or awash one (``min_depth <= 0``)."""
    m = float(min_depth)
    return frac * m if m > 0.0 else m


def preset_beach_profile(grid: GridSpec, params: Mapping[str, Any],
                         rng: np.random.Generator) -> np.ndarray:
    p = _params(params, _BEACH_SCHEMA, "beach_profile")
    along, _ = _axis_coords(grid, p["deepening_direction_deg"])
    off = along - along.min()          # 0 at the shallow (shoreward) edge
    depth = p["min_depth"] + p["dean_A"] * off ** (2.0 / 3.0)
    if p["bar_enabled"] and p["bar_height_m"] > 0:
        bar = p["bar_height_m"] * np.exp(
            -((off - p["bar_distance_m"]) / max(p["bar_width_m"], 1e-6)) ** 2)
        # The bar may rise to 60 % of the shallow-edge depth; with a dry
        # margin (min_depth <= 0) it may reach the shoreline level, not above.
        depth = np.maximum(depth - bar, _shoal_floor(p["min_depth"], 0.6))
    if p["berm_enabled"] and p["berm_height_m"] > 0:
        # The swash step: a short shore-parallel ridge near the shallow
        # edge, the same Gaussian form as the bar but closer in.
        berm = p["berm_height_m"] * np.exp(
            -((off - p["berm_distance_m"]) / max(p["berm_width_m"], 1e-6)) ** 2)
        depth = np.maximum(depth - berm, _shoal_floor(p["min_depth"], 0.5))
    return -depth


_CANAL_SCHEMA: tuple[SchemaRow, ...] = (
    ("axis_heading_deg", "Canal axis heading [deg]", "float", 0.0, -180.0, 180.0),
    ("channel_depth", "Channel depth [m]", "float", 2.0, 0.5, 10.0),
    ("channel_width_m", "Flat channel width [m]", "float", 12.0, 2.0, 100.0),
    ("side_slope", "Side slope [m/m]", "float", 0.5, 0.05, 3.0),
    ("bank_depth", "Minimum depth at banks [m]", "float", 0.4, -5.0, 5.0),
)


def preset_canal(grid: GridSpec, params: Mapping[str, Any],
                 rng: np.random.Generator) -> np.ndarray:
    p = _params(params, _CANAL_SCHEMA, "canal")
    _, across = _axis_coords(grid, p["axis_heading_deg"])
    rise = np.clip(np.abs(across) - p["channel_width_m"] / 2.0, 0.0, None)
    depth = np.maximum(p["channel_depth"] - p["side_slope"] * rise,
                       p["bank_depth"])
    return -depth


_FLAT_SCHEMA: tuple[SchemaRow, ...] = (
    ("depth", "Depth [m] (<= 0: dry land)", "float", 3.0, -5.0, 50.0),
)


def preset_flat(grid: GridSpec, params: Mapping[str, Any],
                rng: np.random.Generator) -> np.ndarray:
    p = _params(params, _FLAT_SCHEMA, "flat")
    return np.full((grid.ny, grid.nx), -float(p["depth"]))


_SLOPED_SCHEMA: tuple[SchemaRow, ...] = (
    ("depth_centre", "Depth at the area centre [m]", "float", 3.0, 0.1, 50.0),
    ("grade", "Grade [m/m] (deepening along the direction)", "float",
     0.02, 0.0, 1.0),
    ("direction_deg", "Deepening direction [deg]", "float", 0.0, -180.0, 180.0),
    ("min_depth", "Minimum depth [m] (<= 0: dry margin)", "float",
     0.3, -5.0, 10.0),
)


def preset_sloped_plane(grid: GridSpec, params: Mapping[str, Any],
                        rng: np.random.Generator) -> np.ndarray:
    p = _params(params, _SLOPED_SCHEMA, "sloped_plane")
    along, _ = _axis_coords(grid, p["direction_deg"])
    depth = p["depth_centre"] + p["grade"] * along
    return -np.maximum(depth, p["min_depth"])


_DREDGED_SCHEMA: tuple[SchemaRow, ...] = (
    ("basin_depth", "Basin depth [m]", "float", 2.0, 0.3, 20.0),
    ("channel_depth", "Channel depth [m]", "float", 4.5, 0.5, 30.0),
    ("channel_width_m", "Channel width [m]", "float", 20.0, 2.0, 200.0),
    ("channel_heading_deg", "Channel heading [deg]", "float", 0.0, -180.0, 180.0),
    ("scarp_smooth_m", "Cut scarp smoothing [m]", "float", 2.0, 0.2, 10.0),
    ("shoal_height_m", "Spoil shoal height [m]", "float", 0.5, 0.0, 5.0),
    ("shoal_width_m", "Spoil shoal width [m]", "float", 6.0, 0.5, 50.0),
)


def preset_dredged_channel(grid: GridSpec, params: Mapping[str, Any],
                           rng: np.random.Generator) -> np.ndarray:
    """A navigation cut through a shallow basin: tanh scarps at the cut
    line and a Gaussian spoil shoal (dredge material) hugging each side."""
    p = _params(params, _DREDGED_SCHEMA, "dredged_channel")
    _, across = _axis_coords(grid, p["channel_heading_deg"])
    half = p["channel_width_m"] / 2.0
    extra = max(p["channel_depth"] - p["basin_depth"], 0.0)
    depth = p["basin_depth"] + extra * 0.5 * (1.0 + np.tanh(
        (half - np.abs(across)) / max(p["scarp_smooth_m"], 1e-6)))
    if p["shoal_height_m"] > 0:
        w = max(p["shoal_width_m"], 1e-6)
        shoal = p["shoal_height_m"] * np.exp(
            -((np.abs(across) - (half + w)) / w) ** 2)
        depth = depth - shoal
    return -np.maximum(depth, 0.1)


_MARINA_SCHEMA: tuple[SchemaRow, ...] = (
    ("basin_depth", "Basin depth [m]", "float", 2.5, 0.3, 20.0),
    ("pocket_depth", "Berth pocket depth [m]", "float", 3.2, 0.3, 30.0),
    ("pontoon_pitch_m", "Pontoon row pitch [m]", "float", 16.0, 4.0, 100.0),
    ("pontoon_heading_deg", "Pontoon heading [deg]", "float", 0.0, -180.0, 180.0),
    ("pocket_width_frac", "Pocket width (fraction of pitch)", "float",
     0.6, 0.1, 0.95),
    ("edge_smooth_m", "Pocket edge smoothing [m]", "float", 1.0, 0.1, 10.0),
    ("pile_pitch_m", "Pile pitch along a pontoon [m]", "float", 6.0, 1.0, 50.0),
    ("pit_depth_m", "Pile scour pit depth [m]", "float", 0.3, 0.0, 3.0),
    ("pit_radius_m", "Pile scour pit radius [m]", "float", 1.2, 0.2, 10.0),
)


def preset_marina(grid: GridSpec, params: Mapping[str, Any],
                  rng: np.random.Generator) -> np.ndarray:
    """Pontoon rows at a pitch along the across axis; berth pockets
    (deeper, propeller-scoured) between them; scour pits at the piles."""
    p = _params(params, _MARINA_SCHEMA, "marina")
    along, across = _axis_coords(grid, p["pontoon_heading_deg"])
    pitch = max(p["pontoon_pitch_m"], 1e-6)
    # u in [-0.5, 0.5): 0 at the pocket centre, +-0.5 on a pontoon line.
    u = np.mod(across / pitch + 0.5, 1.0) - 0.5
    half_pocket = 0.5 * p["pocket_width_frac"] * pitch
    extra = max(p["pocket_depth"] - p["basin_depth"], 0.0)
    depth = p["basin_depth"] + extra * 0.5 * (1.0 + np.tanh(
        (half_pocket - np.abs(u) * pitch) / max(p["edge_smooth_m"], 1e-6)))
    if p["pit_depth_m"] > 0:
        pp = max(p["pile_pitch_m"], 1e-6)
        d_along = along - np.round(along / pp) * pp
        d_across = (0.5 - np.abs(u)) * pitch          # distance to the line
        r2 = (d_along ** 2 + d_across ** 2) / max(p["pit_radius_m"], 1e-6) ** 2
        depth = depth + p["pit_depth_m"] * np.exp(-r2)
    return -np.maximum(depth, 0.1)


_ROCKY_SCHEMA: tuple[SchemaRow, ...] = (
    ("base_depth", "Base depth [m]", "float", 4.0, 0.3, 50.0),
    ("count", "Number of outcrops", "int", 12, 1, 500),
    ("height_m", "Typical outcrop height [m]", "float", 1.0, 0.05, 10.0),
    ("radius_m", "Typical outcrop radius [m]", "float", 3.0, 0.3, 50.0),
    ("min_depth", "Minimum depth over a rock [m]", "float", 0.3, -5.0, 10.0),
)


def preset_rocky_outcrop(grid: GridSpec, params: Mapping[str, Any],
                         rng: np.random.Generator) -> np.ndarray:
    """Seeded rounded rock heads (super-Gaussian bumps, size and height
    jittered +-40 %) over a flat base. Positions come from the terrain
    RNG stream, so they never move the objects."""
    p = _params(params, _ROCKY_SCHEMA, "rocky_outcrop")
    xx, yy = _grid_coords(grid)
    xmin, ymin, xmax, ymax = grid.extent
    n = int(p["count"])
    cx = rng.uniform(xmin, xmax, n)
    cy = rng.uniform(ymin, ymax, n)
    rad = p["radius_m"] * rng.uniform(0.6, 1.4, n)
    hgt = p["height_m"] * rng.uniform(0.6, 1.4, n)
    bump = np.zeros_like(xx)
    for k in range(n):
        r2 = ((xx - cx[k]) ** 2 + (yy - cy[k]) ** 2) / max(rad[k], 1e-6) ** 2
        bump = np.maximum(bump, hgt[k] * np.exp(-(r2 ** 1.5)))
    return -np.maximum(p["base_depth"] - bump, p["min_depth"])


_SANDBAR_SCHEMA: tuple[SchemaRow, ...] = (
    ("depth", "Mean depth [m]", "float", 3.0, 0.3, 50.0),
    ("bar_spacing_m", "Bar spacing [m]", "float", 25.0, 2.0, 500.0),
    ("bar_height_m", "Bar height (crest to trough) [m]", "float",
     0.6, 0.0, 5.0),
    ("heading_deg", "Bar crest heading [deg]", "float", 0.0, -180.0, 180.0),
    ("asymmetry", "Crest asymmetry (0 sine .. 1 sawtooth)", "float",
     0.0, 0.0, 1.0),
)


def preset_sandbar_ripples(grid: GridSpec, params: Mapping[str, Any],
                           rng: np.random.Generator) -> np.ndarray:
    """Periodic bars with crests along ``heading_deg``; ``asymmetry``
    skews each crest toward a sawtooth (lee-side steepening)."""
    p = _params(params, _SANDBAR_SCHEMA, "sandbar_ripples")
    _, across = _axis_coords(grid, p["heading_deg"])
    phase = 2.0 * np.pi * across / max(p["bar_spacing_m"], 1e-6)
    a = float(np.clip(p["asymmetry"], 0.0, 0.95))
    # Skewed wave: phase advanced by a fraction of its own sine.
    wave = np.sin(phase + a * np.sin(phase))
    depth = p["depth"] - 0.5 * p["bar_height_m"] * wave
    return -np.maximum(depth, 0.1)


#: One-sentence hover help per parameter name (shared across presets where
#: the name means the same thing); the builder shows it on the row.
_PARAM_HELP: dict[str, str] = {
    "basin_depth": "Water depth over the main basin floor.",
    "edge_shoal_m": "How much shallower the floor gets towards the area edges.",
    "shoal_width_m": "Width of the shallow rim along the edges.",
    "channel_enabled": "Cuts a deeper dredged channel through the basin.",
    "channel_depth": "Water depth in the dredged channel.",
    "channel_width_m": "Width of the channel floor.",
    "channel_heading_deg": "Compass heading the channel runs along.",
    "scarp_smooth_m": "Distance over which the channel walls blend into the basin floor.",
    "axis_heading_deg": "Compass heading the channel or river runs along.",
    "max_depth": "Depth of the deepest line (the thalweg) of the river.",
    "bank_depth": "Water depth at the banks, the shallowest part of the profile; the canal accepts zero or negative for dry banks.",
    "thalweg_offset_frac": "Pushes the deepest line towards one bank (negative) or the other (positive).",
    "pool_riffle_wavelength_m": "Distance between successive deep pools along the flow.",
    "pool_riffle_amplitude_m": "How much deeper the pools are than the riffles between them.",
    "deepening_direction_deg": "Compass direction in which the beach deepens (towards open water).",
    "dean_A": "Steepness of the Dean equilibrium profile: larger A means coarser sand and a steeper beach.",
    "min_depth": "Shallowest water allowed anywhere in the area; zero or negative makes a dry margin above the waterline.",
    "bar_enabled": "Adds a sandbar running along the shore.",
    "bar_distance_m": "Distance of the sandbar crest from the shallow edge.",
    "bar_height_m": "Height of the sandbar above the surrounding profile.",
    "bar_width_m": "Width of the sandbar across the shore.",
    "berm_enabled": "Adds a submerged step near the shoreline.",
    "berm_distance_m": "Distance of the step from the shallow edge.",
    "berm_height_m": "Height of the step.",
    "berm_width_m": "Width of the step across the shore.",
    "side_slope": "Steepness of the canal banks in metres of rise per metre of run.",
    "depth": "Water depth of the flat floor (zero or negative is dry land) or the mean depth of the bar field.",
    "depth_centre": "Water depth at the centre of the area.",
    "grade": "How fast the floor deepens along the direction, in metres per metre.",
    "direction_deg": "Compass direction in which the plane deepens.",
    "shoal_height_m": "Height of the dredge-spoil ridges either side of the cut.",
    "pocket_depth": "Water depth in the berth pockets under the pontoons.",
    "pontoon_pitch_m": "Distance between successive pontoon rows.",
    "pontoon_heading_deg": "Compass heading the pontoon rows run along.",
    "pocket_width_frac": "Share of each row pitch taken by the deeper berth pocket.",
    "edge_smooth_m": "Distance over which a pocket edge blends into the basin floor.",
    "pile_pitch_m": "Distance between mooring piles along a pontoon.",
    "pit_depth_m": "Depth of the scour pit around each pile.",
    "pit_radius_m": "Radius of the scour pit around each pile.",
    "base_depth": "Water depth of the floor between the outcrops.",
    "count": "Number of rock outcrops scattered over the area.",
    "height_m": "Typical height of an outcrop above the floor.",
    "radius_m": "Typical footprint radius of an outcrop.",
    "bar_spacing_m": "Distance between successive bar crests.",
    "heading_deg": "Compass heading the bar crests run along.",
    "asymmetry": "0 makes symmetric bars, 1 makes sawtooth bars with a steep lee side.",
}


def with_help(schema: tuple) -> tuple:
    """Attach the 7th (help) element to every row that lacks one."""
    out = []
    for row in schema:
        row = tuple(row)
        if len(row) >= 7 and row[6]:
            out.append(row)
        else:
            out.append(row[:6] + (_PARAM_HELP.get(row[0], ""),))
    return tuple(out)


DEPTH_PRESETS: dict[str, tuple[PresetFn, str, tuple[SchemaRow, ...]]] = {
    "harbour_basin": (preset_harbour_basin, "Harbour basin (dredged)",
                      with_help(_HARBOUR_SCHEMA)),
    "river_channel": (preset_river_channel, "River channel", with_help(_RIVER_SCHEMA)),
    "beach_profile": (preset_beach_profile, "Beach (Dean profile)",
                      with_help(_BEACH_SCHEMA)),
    "canal": (preset_canal, "Canal (trapezoid)", with_help(_CANAL_SCHEMA)),
    "flat": (preset_flat, "Flat", with_help(_FLAT_SCHEMA)),
    "sloped_plane": (preset_sloped_plane, "Sloped plane", with_help(_SLOPED_SCHEMA)),
    "dredged_channel": (preset_dredged_channel,
                        "Dredged channel (spoil shoals)", with_help(_DREDGED_SCHEMA)),
    "marina": (preset_marina, "Marina (pontoons, berth pockets)",
               with_help(_MARINA_SCHEMA)),
    "rocky_outcrop": (preset_rocky_outcrop, "Rocky outcrops (seeded)",
                      with_help(_ROCKY_SCHEMA)),
    "sandbar_ripples": (preset_sandbar_ripples, "Sandbar field",
                        with_help(_SANDBAR_SCHEMA)),
}
