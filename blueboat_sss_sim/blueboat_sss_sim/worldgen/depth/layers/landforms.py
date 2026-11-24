"""Landforms -- the engineered and natural shapes of harbours, rivers,
canals and beaches, drawn on the map and cut into (or laid over) the
composite beneath.

* ``basin``     dredged polygon: a designed floor plus over-dredge and
                dredge-pass relief, a sloped rim beyond it (JFA 2023;
                OCDI 2020; USACE CEM V-5).
* ``channel``   a cross-section template extruded along a drawn
                centreline in curvilinear (s, n) coordinates, the thalweg
                following the lagged curvature (Ahmed et al. 2024;
                Dietrich & Smith 1983), pools every 5-7 widths (Keller &
                Melhorn 1978), an optional coarse lag in the thalweg.
* ``hole``      a scour hole or pockmark: berth scour of the order of the
                propeller diameter (Mujal-Colilles et al. 2017; Hamill et
                al. 1999; PIANC 2015), pier scour of 1-2.4 pier widths
                (HEC-18).
* ``mound``     spoil, rubble or a silt wedge.
* ``bar``       a bar and trough along a shoreline, cut by rips (Wright &
                Short 1984; Holman et al. 2014; Short 1985).
* ``shelf``     a step across a line (a rock platform edge, a swash step).
* ``siltation`` inward shoaling and fining from a harbour entrance
                (Winterwerp 2005).
"""

from __future__ import annotations

import numpy as np

from ...noise import value_noise
from .. import geometry as geo
from ..paint import smoothstep
from . import (MATERIAL_CHOICE, LayerContext, LayerOutput, LayerSpec,
               register)


# ---------------------------------------------------------------- basin
_BASIN = (
    ("floor_depth", "Design floor depth [m]", "float", 3.0, -5.0, 30.0,
     "Water depth of the dredged floor: the design vessel's draft plus 1 m of "
     "keel clearance, planned in 0.5 m steps. A zero or negative depth puts "
     "the floor at or above the waterline, which is one way to cut a dry "
     "area out of a world -- though the `land` layer, whose outline is "
     "rounded and whose shore width is a beach face rather than a dredge "
     "slope, is the intended tool for that."),
    ("overdredge_m", "Over-dredge [m]", "float", 0.3, 0.0, 2.0,
     "Extra depth below the design floor left by advance maintenance and "
     "dredging tolerance (0.3-1.5 m in practice)."),
    ("rim_slope", "Rim slope [m run per m rise]", "float", 3.0, 1.0, 12.0,
     "How far the dredged rim runs outward for each metre it rises toward "
     "the natural bed (3 is the usual slope in sand)."),
    ("relief_m", "Dredge-pass relief [m]", "float", 0.15, 0.0, 1.0,
     "Height of the low ridges successive dredge passes leave on the floor."),
    ("relief_spacing_m", "Dredge-pass spacing [m]", "float", 4.0, 0.5, 50.0,
     "Distance between successive dredge passes across the floor."),
    ("relief_heading_deg", "Dredge-pass heading [deg]", "float", 0.0,
     -180.0, 180.0, "Compass heading the dredge passes run along."),
    ("feather_m", "Rim feather [m]", "float", 1.0, 0.0, 20.0,
     "Distance over which the outer edge of the rim blends into the "
     "surrounding bed."),
)


def basin_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    D = p["floor_depth"] + p["overdredge_m"]
    run = D * p["rim_slope"]
    inside, sd = geo.polygon_signed_distance(g["points"], grid,
                                             run + p["feather_m"] + 1.0)
    depth = np.where(inside, D, D - np.maximum(sd, 0.0) / p["rim_slope"])
    if p["relief_m"] > 0.0 and inside.any():
        d_line = geo.line_family_distance(grid, p["relief_heading_deg"],
                                          p["relief_spacing_m"])
        wave = np.cos(np.pi * d_line / max(0.5 * p["relief_spacing_m"], 1e-6))
        mod = 1.0 + 0.5 * value_noise((grid.ny, grid.nx), 6, rng)
        depth = np.where(inside, depth + p["relief_m"] * 0.5 * (1.0 + wave) * mod,
                         depth)
    w = geo.feather_weight(sd - run, p["feather_m"])
    w[inside] = 1.0
    # A design floor at or above the waterline is not a dredge any more --
    # it is a way of cutting a dry area out of a world (the `land` layer is
    # the intended tool). "Deepen" takes the minimum, under which such a
    # floor is a silent no-op, so this result is composited normally.
    return LayerOutput(height=-depth, default_mask=w,
                       blend=("normal" if D <= 0.0 else None))


register(LayerSpec(
    kind="basin", fn=basin_fn, label="Dredged basin", family="landform",
    schema=_BASIN, geometry=("polygon",), channels=("height",),
    reference="[24] JFA 2023; [25] OCDI 2020; [26] USACE CEM V-5",
    consequence="A flat floor at a round design depth with faint dredge "
                "ridges, rimmed by a slope that reads as a soft dark band.",
    blend="deepen"))


# -------------------------------------------------------------- channel
_CHANNEL = (
    ("template", "Cross-section", "choice:trapezoid|parabola|asymmetric",
     "asymmetric", 0, 0, "Shape across the channel: a dredged trapezoid, a "
     "natural parabola, or a river section whose deepest line hugs the outer "
     "bank of each bend."),
    ("width_m", "Top width [m]", "float", 20.0, 1.0, 500.0,
     "Width of the channel between the bank tops."),
    ("depth_m", "Thalweg depth [m]", "float", 4.0, 0.2, 40.0,
     "Water depth along the deepest line of the channel."),
    ("bank_depth_m", "Bank-top depth [m]", "float", 0.5, -5.0, 20.0,
     "Water depth at the bank tops (zero or negative is a dry bank)."),
    ("side_slope", "Bank slope [m run per m rise]", "float", 3.0, 0.5, 12.0,
     "Trapezoid only: how far each bank runs for every metre it rises."),
    ("thalweg_frac", "Thalweg offset (frac of half-width)", "float", 0.5,
     0.0, 0.9, "Asymmetric only: how far the deepest line moves toward the "
     "outer bank of a bend, as a fraction of the half-width."),
    ("curvature_lag_m", "Bend lag [m]", "float", 0.0, 0.0, 500.0,
     "Distance downstream of a bend at which its effect on the section is "
     "strongest (0 uses one channel width)."),
    ("pool_spacing_w", "Pool spacing [widths]", "float", 6.0, 0.0, 20.0,
     "Distance between successive pools along a straight reach, in channel "
     "widths (5-7 in nature; 0 keeps only the pools at bends)."),
    ("pool_amplitude_m", "Pool depth [m]", "float", 0.5, 0.0, 5.0,
     "How much deeper the pools are than the riffles between them."),
    ("feather_m", "Bank feather [m]", "float", 2.0, 0.0, 50.0,
     "Distance beyond the bank top over which the channel blends into the "
     "surrounding bed."),
    ("lag_material", "Coarse lag in the thalweg", "bool", False, 0, 1,
     "Paints the deepest band with a coarser seabed material and the inner "
     "bar of each bend with a finer one."),
    ("thalweg_material", "Thalweg material", MATERIAL_CHOICE, "gravel", 0, 0,
     "Material of the coarse lag along the deepest line."),
    ("bar_material", "Bar material", MATERIAL_CHOICE, "sand", 0, 0,
     "Material of the point bar on the inner bank of each bend."),
)


def channel_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    W = p["width_m"]
    half = 0.5 * W
    reach = half + p["feather_m"] + grid.resolution
    fr = geo.polyline_frame(g["points"], grid, reach,
                            smooth_m=max(W, 4.0 * grid.resolution))
    lag = p["curvature_lag_m"] if p["curvature_lag_m"] > 0.0 else W
    k = fr.kappa_lagged(lag)
    kmax = max(float(np.abs(fr.kappa_dense).max()), 1e-9)
    bend = np.clip(np.abs(k) / kmax, 0.0, 1.0)
    n = fr.n
    if p["template"] == "asymmetric":
        n_t = -np.sign(k) * p["thalweg_frac"] * half * bend
    else:
        n_t = np.zeros_like(n)
    lo = np.maximum(n_t + half, 1e-6)
    hi = np.maximum(half - n_t, 1e-6)
    u = np.where(n < n_t, (n_t - n) / lo, (n - n_t) / hi)
    u = np.clip(u, 0.0, 1.0)
    bank, dmax = p["bank_depth_m"], p["depth_m"]
    if p["template"] == "trapezoid":
        depth = np.minimum(dmax, bank + np.maximum(half - np.abs(n), 0.0)
                           / max(p["side_slope"], 1e-6))
    else:
        depth = bank + (dmax - bank) * (1.0 - u ** 2)
    if p["pool_amplitude_m"] > 0.0:
        periodic = np.zeros_like(u)
        if p["pool_spacing_w"] > 0.0:
            lam = max(p["pool_spacing_w"] * W, 1e-6)
            periodic = 0.5 * (1.0 + np.sin(2.0 * np.pi * fr.s / lam))
        depth = depth + p["pool_amplitude_m"] * (1.0 - u) * np.maximum(bend, periodic)
    w = geo.band_weight(fr.dist, half, p["feather_m"])
    w[~fr.valid] = 0.0
    out = LayerOutput(height=-depth, default_mask=w)
    if p["lag_material"]:
        ids = np.full((grid.ny, grid.nx), -1, dtype=np.int16)
        thal = fr.valid & (np.abs(n - n_t) <= 0.15 * W)
        inner = fr.valid & (bend > 0.2) & (n * n_t < 0.0) & (np.abs(n) > 0.5 * half)
        ids[inner] = 1
        ids[thal] = 0
        out.material = (ids, [p["thalweg_material"], p["bar_material"]])
        out.material_edge = "hard"
    return out


register(LayerSpec(
    kind="channel", fn=channel_fn, label="Channel along a line",
    family="landform", schema=_CHANNEL, geometry=("polyline",),
    channels=("height", "material"),
    reference="[13] Leopold & Wolman 1957; [14] Keller & Melhorn 1978; "
              "[15] Dietrich & Smith 1983; [16] Ahmed et al. 2024; "
              "[17] Merwade et al. 2006; [22] Rijkswaterstaat 2020",
    consequence="A dark deep band that swings to the outer bank of every "
                "bend, with brighter coarse lag in it and a pale bar opposite.",
    blend="deepen"))


# ----------------------------------------------------------------- hole
_HOLE = (
    ("depth_m", "Hole depth [m]", "float", 1.0, 0.05, 20.0,
     "Depth of the hole below the surrounding bed at its centre; berth scour "
     "is about one propeller diameter, pier scour one to two pier widths."),
    ("profile", "Profile", "choice:gaussian|bowl", "gaussian", 0, 0,
     "Gaussian for a scour hole that fades out, bowl for a pockmark with a "
     "definite rim."),
    ("rim_m", "Spoil rim height [m]", "float", 0.0, 0.0, 5.0,
     "Height of the ridge of displaced material around the hole."),
    ("lag_material", "Coarse lag on the floor", "bool", False, 0, 1,
     "Paints the hole floor with a coarser material, as scour winnows the "
     "fines away."),
    ("material", "Floor material", MATERIAL_CHOICE, "gravel", 0, 0,
     "Material of the hole floor when the coarse lag is on."),
)


def hole_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    rho = geo.ellipse_rho(grid, g["centre"], g["rx_m"], g["ry_m"], g["angle_deg"])
    if p["profile"] == "gaussian":
        shape = np.exp(-3.0 * rho ** 2)
    else:
        shape = np.clip(1.0 - rho ** 2, 0.0, 1.0)
    delta = -p["depth_m"] * shape
    if p["rim_m"] > 0.0:
        delta = delta + p["rim_m"] * np.exp(-((rho - 1.0) / 0.3) ** 2)
    out = LayerOutput(delta=delta)
    if p["lag_material"]:
        ids = np.where(shape > 0.4, 0, -1).astype(np.int16)
        out.material = (ids, [p["material"]])
    return out


register(LayerSpec(
    kind="hole", fn=hole_fn, label="Scour hole / pockmark", family="landform",
    schema=_HOLE, geometry=("ellipse",), channels=("height", "material"),
    reference="[29] Mujal-Colilles et al. 2017; [28] Hamill et al. 1999; "
              "[27] PIANC 2015; [21] HEC-18",
    consequence="A dark depression with a bright far rim and a shadow on the "
                "near side, brighter inside when the lag is coarse.",
    blend="add"))


# ---------------------------------------------------------------- mound
_MOUND = (
    ("height_m", "Mound height [m]", "float", 0.8, 0.05, 20.0,
     "Height of the mound above the surrounding bed at its centre."),
    ("sharpness", "Flatness of the top", "float", 2.0, 1.0, 8.0,
     "2 is a rounded Gaussian hump; larger values flatten the top and "
     "steepen the sides."),
    ("material_enabled", "Paint the mound", "bool", False, 0, 1,
     "Gives the mound its own seabed material (rubble, spoil, silt)."),
    ("material", "Mound material", MATERIAL_CHOICE, "rocks", 0, 0,
     "Material of the mound when painting is on."),
)


def mound_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    rho = geo.ellipse_rho(grid, g["centre"], g["rx_m"], g["ry_m"], g["angle_deg"])
    shape = np.exp(-(rho ** 2) ** (0.5 * p["sharpness"]))
    out = LayerOutput(delta=p["height_m"] * shape)
    if p["material_enabled"]:
        out.material = (np.where(shape > 0.3, 0, -1).astype(np.int16),
                        [p["material"]])
    return out


register(LayerSpec(
    kind="mound", fn=mound_fn, label="Mound / spoil heap", family="landform",
    schema=_MOUND, geometry=("ellipse",), channels=("height", "material"),
    reference="[26] USACE CEM V-5; [30] Winterwerp 2005",
    consequence="A bright near face and a shadow behind it; rubble reads "
                "speckled, silt smooth.",
    blend="add"))


# ------------------------------------------------------------------ bar
_BAR = (
    ("side", "Offshore side", "choice:left|right|both", "left", 0, 0,
     "Which side of the drawn shoreline is the water."),
    ("distance_m", "Bar distance [m]", "float", 40.0, 1.0, 1000.0,
     "Distance of the bar crest from the shoreline."),
    ("height_m", "Bar height [m]", "float", 0.6, 0.0, 5.0,
     "Height of the bar crest above the profile it sits on."),
    ("width_m", "Bar width [m]", "float", 8.0, 0.5, 200.0,
     "Width of the bar across the shore."),
    ("trough_m", "Trough depth [m]", "float", 0.3, 0.0, 5.0,
     "Extra depth of the trough on the shoreward side of the bar."),
    ("rip_spacing_m", "Rip spacing [m]", "float", 0.0, 0.0, 2000.0,
     "Distance between rip channels cutting through the bar along the "
     "shore (0 makes a continuous bar)."),
    ("rip_width_m", "Rip width [m]", "float", 15.0, 1.0, 200.0,
     "Width of each rip channel."),
)


def bar_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    reach = p["distance_m"] + 3.0 * p["width_m"] + grid.resolution
    fr = geo.polyline_frame(g["points"], grid, reach)
    d, n = fr.dist, fr.n
    w_ = max(p["width_m"], 1e-6)
    crest = p["height_m"] * np.exp(-((d - p["distance_m"]) / w_) ** 2)
    trough = p["trough_m"] * np.exp(-((d - (p["distance_m"] - 1.5 * w_)) / w_) ** 2)
    rip = np.zeros_like(d)
    if p["rip_spacing_m"] > 0.0:
        sp = p["rip_spacing_m"]
        ds = np.abs((fr.s % sp) - 0.5 * sp)
        rip = smoothstep(1.0 - ds / max(0.5 * p["rip_width_m"], 1e-6))
    delta = crest * (1.0 - rip) - trough - 0.5 * p["height_m"] * rip * (
        crest > 0.05 * max(p["height_m"], 1e-9))
    delta = np.where(fr.valid, delta, 0.0)
    if p["side"] != "both":
        keep = (n >= 0.0) if p["side"] == "left" else (n <= 0.0)
        delta = np.where(keep, delta, 0.0)
    return LayerOutput(delta=delta)


register(LayerSpec(
    kind="bar", fn=bar_fn, label="Sandbar and trough", family="landform",
    schema=_BAR, geometry=("polyline",), channels=("height",),
    reference="[5] Wright & Short 1984; [6] Holman et al. 2014; [7] Short 1985",
    consequence="A bright shore-parallel ridge with a dark trough behind it, "
                "broken by rip channels.",
    blend="add"))


# ---------------------------------------------------------------- shelf
_SHELF = (
    ("step_m", "Step height [m]", "float", 0.5, 0.0, 20.0,
     "Height of the step across the line."),
    ("width_m", "Transition width [m]", "float", 2.0, 0.1, 100.0,
     "Distance over which the bed drops through the step."),
    ("deep_side", "Deeper side", "choice:left|right", "right", 0, 0,
     "Which side of the drawn line is the deeper one."),
)


def shelf_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    xmin, ymin, xmax, ymax = grid.extent
    reach = float(np.hypot(xmax - xmin, ymax - ymin)) + 1.0
    fr = geo.polyline_frame(g["points"], grid, reach)
    sgn = -1.0 if p["deep_side"] == "right" else 1.0
    delta = -p["step_m"] * smoothstep(0.5 + sgn * fr.n / max(p["width_m"], 1e-6))
    return LayerOutput(delta=delta)


register(LayerSpec(
    kind="shelf", fn=shelf_fn, label="Step / platform edge", family="landform",
    schema=_SHELF, geometry=("polyline",), channels=("height",),
    reference="[5] Wright & Short 1984 (swash step); [11] Friedrichs 2011",
    consequence="A sharp bright edge facing the sonar or a shadow line "
                "facing away from it.",
    blend="add"))


# ------------------------------------------------------------ siltation
_SILT = (
    ("shoal_m", "Shoaling far from the entrance [m]", "float", 0.8, 0.0, 10.0,
     "How much shallower the bed becomes where the silt has settled, at the "
     "end of the gradient."),
    ("length_m", "Gradient length [m]", "float", 80.0, 1.0, 5000.0,
     "Distance from the entrance line over which the shoaling builds up."),
    ("profile", "Profile", "choice:linear|smoothstep|exp", "smoothstep", 0, 0,
     "How the shoaling grows with distance from the entrance."),
    ("side", "Basin side", "choice:left|right|both", "both", 0, 0,
     "Which side of the drawn entrance line is the basin."),
    ("material_enabled", "Grade the seabed material", "bool", True, 0, 1,
     "Fines the seabed material from the entrance inward."),
    ("near_material", "Material at the entrance", MATERIAL_CHOICE, "sand",
     0, 0, "Seabed material where the exchange flow keeps the bed coarse."),
    ("far_material", "Material in the basin", MATERIAL_CHOICE, "mud", 0, 0,
     "Seabed material where the fines settle."),
)


def siltation_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    xmin, ymin, xmax, ymax = grid.extent
    reach = float(np.hypot(xmax - xmin, ymax - ymin)) + 1.0
    fr = geo.polyline_frame(g["points"], grid, reach)
    d = fr.dist
    if p["profile"] == "exp":
        u = 1.0 - np.exp(-d / max(p["length_m"], 1e-6))
    else:
        u = np.clip(d / max(p["length_m"], 1e-6), 0.0, 1.0)
        if p["profile"] == "smoothstep":
            u = smoothstep(u)
    default = None
    if p["side"] != "both":
        default = ((fr.n >= 0.0) if p["side"] == "left"
                   else (fr.n <= 0.0)).astype(np.float64)
    out = LayerOutput(delta=p["shoal_m"] * u, default_mask=default)
    if p["material_enabled"]:
        pick = rng.uniform(size=u.shape) < u
        out.material = (np.where(pick, 1, 0).astype(np.int16),
                        [p["near_material"], p["far_material"]])
        out.material_edge = "hard"
    return out


register(LayerSpec(
    kind="siltation", fn=siltation_fn, label="Siltation from an entrance",
    family="landform", schema=_SILT, geometry=("polyline",),
    channels=("height", "material"),
    reference="[30] Winterwerp 2005; [23] Canal & River Trust",
    consequence="The floor shoals and darkens away from the entrance as sand "
                "gives way to mud.",
    blend="add"))


# ----------------------------------------------------------------- land
_LAND = (
    ("height_m", "Height above the waterline [m]", "float", 0.3, -5.0, 5.0,
     "Height of the area relative to still water (z = 0). Positive is dry "
     "land -- a beach berm, a bank, an island -- which the renderer sees as "
     "no return at all (a cell at or above the transducer scatters nothing "
     "back); a small negative value makes a shoal or a tidal pool instead."),
    ("round_m", "Corner rounding [m]", "float", 3.0, 0.0, 50.0,
     "Radius of the disc the outline is opened and closed with, so no part "
     "of the water's edge turns sharper than it. A drawn polygon is a "
     "handful of clicked vertices; rounding is what makes it read as a "
     "shoreline instead of a polygon. Zero keeps the drawn corners."),
    ("shore_m", "Shore width [m]", "float", 4.0, 0.0, 200.0,
     "Width of the face that carries the land down to the seabed below it. "
     "A natural beach runs 10-40 m for each metre of height (Dean's "
     "profile); a quay wall or a dyke is a metre or two."),
    ("material_enabled", "Set the material", "bool", True, 0, 0,
     "Give the area its own seabed material (the foreshore is usually "
     "coarser than the bed offshore of it)."),
    ("material", "Material", MATERIAL_CHOICE, "sand", 0, 0,
     "Material stamped where the area is above the surrounding bed."),
)


def land_fn(grid, g, p, rng, ctx: LayerContext) -> LayerOutput:
    kind = str(g.get("type", "polygon"))
    if kind == "ellipse":
        inside = geo.ellipse_rho(grid, g["centre"], g["rx_m"], g["ry_m"],
                                 g["angle_deg"]) <= 1.0
    elif kind == "rect":
        inside = geo.rect_inside(grid, g["x_min"], g["y_min"],
                                 g["x_max"], g["y_max"])
    else:
        inside = geo.polygon_inside(g["points"], grid)
    inside = geo.round_region(inside, grid.resolution, p["round_m"])
    if not inside.any():
        return LayerOutput()
    sd = geo.signed_distance_of(inside, grid.resolution)
    w = geo.feather_weight(sd, p["shore_m"])
    out = LayerOutput(height=np.full((grid.ny, grid.nx), float(p["height_m"])),
                      default_mask=w)
    if p["material_enabled"]:
        out.material = (np.where(inside, 0, -1).astype(np.int16),
                        [p["material"]])
    return out


register(LayerSpec(
    kind="land", fn=land_fn, label="Land / dry area (shoreline)",
    family="landform", schema=_LAND,
    geometry=("polygon", "ellipse", "rect"), channels=("height", "material"),
    reference="[2] Dean 1991 equilibrium profile; [50] Serra 1982 ch. 2 "
              "(the outline's opening-closing radius filter); [33] Jackson "
              "& Richardson 2007 par. 12.1 (foreshore sediment)",
    consequence="Dry ground: no acoustic return from it at all, the swath "
                "ends at the waterline, and the shore face carries the "
                "seabed up to it. This is how a world gets a beach, a bank "
                "or an island rather than water everywhere.",
    blend="normal"))
