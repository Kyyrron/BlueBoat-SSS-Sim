"""The world card: one page that says what a generated world holds
(``world_card.md``) and a top view of it (``world_card.png``).

A card is written beside the worlds, never into a world folder (a world
folder is an immutable snapshot): ``<worlds root>/.population/<path name>/
<world name>/``. It can be made from a plan and its environment (at build
time: the top view then shows the expected density too) or from a built
world folder alone (``metadata.yaml`` holds the placement record and every
object).

Colours: the seven target classes take the seven first slots of the
project's validated categorical palette, in the fixed order of the object
set, and each class also has its own marker shape, so that a class is never
told by colour alone. Debris and structures are neutral. Features are bands
in the colour of their use and are labelled.

matplotlib (Agg); nothing else of the package needs it.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .data import DEBRIS, TARGETS

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
LAND = "#d9d8d2"
WATER_OUT = "#ecebe6"
#: class -> (colour, marker, label): palette slots 1..7 in the set's order
CLASS_STYLE = {
    "plastic_box": ("#2a78d6", "s", "plastic box (T1)"),
    "drum_200l": ("#eb6834", "o", "drum (T2)"),
    "concrete_block": ("#1baf7a", "D", "concrete block (T3)"),
    "trap_pot": ("#eda100", "X", "trap / pot (T4)"),
    "pipe_steel": ("#e87ba4", "d", "pipe (T5)"),
    "tire_car": ("#008300", "P", "tyre (T6)"),
    "net_bundle": ("#6250d6", "*", "net / rope bundle (T7)"),
    DEBRIS: ("#7c7b76", "v", "debris (C1)"),
}
STRUCTURE_STYLE = ("#2f2f2d", "s", "structure")
#: use -> colour (a fixed mapping: a use keeps its colour whatever else is
#: on the map); the other uses are drawn in ink and told by their label
USE_COLOUR = {
    "fender_line": "#2a78d6", "fishing_berth": "#eb6834", "marina_pontoon": "#1baf7a",
    "urban_quay": "#eda100", "working_quay": "#e87ba4", "breakwater_toe": "#008300",
    "mooring_block": "#6250d6", "mooring_field": "#6250d6",
    "fishing_ground": "#e34948", "traffic_lane": "#e34948",
}
SEQ_BLUE = ("#fcfcfb", "#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#104281")
#: the same slots stepped for a dark surface (the world builder's map)
CLASS_COLOUR_DARK = {
    "plastic_box": "#3987e5", "drum_200l": "#d95926", "concrete_block": "#199e70", "trap_pot": "#c98500",
    "pipe_steel": "#d55181", "tire_car": "#0ca30c", "net_bundle": "#9085e9", DEBRIS: "#9a9993",
}
USE_COLOUR_DARK = {
    "fender_line": "#3987e5", "fishing_berth": "#d95926", "marina_pontoon": "#199e70",
    "urban_quay": "#c98500", "working_quay": "#d55181", "breakwater_toe": "#0ca30c",
    "mooring_block": "#9085e9", "mooring_field": "#9085e9",
    "fishing_ground": "#e66767", "traffic_lane": "#e66767",
}
#: short names of the classes for a narrow panel
CLASS_SHORT = {"plastic_box": "Box", "drum_200l": "Drum", "concrete_block": "Block", "trap_pot": "Trap",
               "pipe_steel": "Pipe", "tire_car": "Tyre", "net_bundle": "Net / rope", DEBRIS: "Debris"}


def use_colour(use: str) -> str:
    return USE_COLOUR.get(str(use), INK_2)


def _fmt(v: Any, nd: int = 2) -> str:
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{nd}f}".rstrip("0").rstrip(".") if nd else f"{v:.0f}"
    return str(v)


def card_markdown(meta: Mapping[str, Any], *, model: Mapping[str, Any] | None = None,
                  picture: str | None = "world_card.png") -> str:
    """The card's text from a world's ``metadata.yaml`` (as a mapping)."""
    pl = dict(meta.get("placement") or {})
    rz = dict(pl.get("realised") or {})
    ex = dict(pl.get("expected") or {})
    env = dict(pl.get("envelope") or {})
    name = meta.get("world_name", "world")
    L: list[str] = [f"# World card: {name}", ""]
    L += ["| | |", "|---|---|",
          f"| Content digest | `{str(meta.get('content_digest', ''))[:16]}` |",
          f"| Placement | {pl.get('mode', '—')}, profile `{pl.get('profile', '—')}`, density level "
          f"**{pl.get('density_level', '—')}** ({_fmt(pl.get('lambda_active_per_ha'))} per ha in the active band) |",
          f"| Classes, mix | {', '.join(pl.get('classes') or []) or '—'}; {pl.get('mix', '—')} |",
          f"| Placement seed, split | {pl.get('seed', '—')}, {pl.get('split', '—')}"
          + (f", harbour {pl['harbour']}" if pl.get("harbour") else "") + " |",
          f"| World seed | {meta.get('seed', '—')} |",
          f"| Model | {(pl.get('model') or {}).get('package', '—')} {(pl.get('model') or {}).get('version', '')}, "
          f"plan digest `{pl.get('plan_digest', '—')}` |"]
    if model:
        L.append(f"| Model code / evidence digest | `{model.get('code_digest', '—')}` / "
                 f"`{model.get('evidence_digest', '—')}` |")
    gen = dict(meta.get("generator") or {})
    if gen:
        L.append(f"| Simulator generator digest | `{str(gen.get('code_digest', ''))[:16]}` |")
    run = dict(meta.get("run") or {})
    if "sonar_fouling" in run:
        L.append(f"| Fouling (one value per world) | {run['sonar_fouling']} |")
    L += [f"| Stress world | {_fmt(bool(pl.get('stress_world')))} |",
          f"| Showcase world (objects outside the band) | {_fmt(bool(pl.get('showcase_world')))} |", ""]
    if picture:
        L += [f"![top view]({picture})", ""]
    L += ["## Counts", "", "| Class | Expected | Placed |", "|---|---|---|"]
    per_e, per_r = dict(ex.get("per_class") or {}), dict(rz.get("per_class") or {})
    for c in list(TARGETS) + [DEBRIS]:
        if c in per_e or c in per_r:
            L.append(f"| {c} | {_fmt(per_e.get(c, 0.0))} | {per_r.get(c, 0)} |")
    L += [f"| **targets** | {_fmt(ex.get('targets'))} | {rz.get('targets', 0)} |",
          f"| **debris** | {_fmt(ex.get('debris'))} | {rz.get('debris', 0)} |",
          f"| **total** | {_fmt(ex.get('total'))} | {rz.get('targets', 0) + rz.get('debris', 0)} |", "",
          f"- Detectable targets (nominal, not effectively buried): **{rz.get('detectable_targets', '—')}**; "
          f"effectively buried: {rz.get('effectively_buried', 0)}.",
          f"- Groups: {rz.get('groups', 0)} ({rz.get('grouped_objects', 0)} objects in them). "
          f"Structures added: {rz.get('structures', 0)}.",
          f"- On mud: {rz.get('on_mud', 0)}. Outside the altitude band: {rz.get('outside_band', 0)}.",
          f"- Removed by the spacing rule: {rz.get('removed_by_spacing', 0)}; no room: {rz.get('no_room', 0)}."]
    if rz.get("stand_ins"):
        L.append("- Stand-ins: " + ", ".join(f"{v} `trap_pot` for {k}" for k, v in sorted(rz["stand_ins"].items()))
                 + ".")
    L += ["", "## Density and pattern", "",
          "| Measure | Value |", "|---|---|",
          f"| Density in the active band (placed) | {_fmt(rz.get('density_active_per_ha'))} per ha "
          f"(level: {_fmt(pl.get('lambda_active_per_ha'))}) |",
          f"| Density over the placement water | {_fmt(rz.get('density_world_per_ha'))} per ha |",
          f"| Share within 30 m of a source | {_fmt(rz.get('share_within_30m'))} |",
          f"| Median distance to the source | {_fmt(rz.get('median_distance_to_source_m'))} m |",
          f"| Clark–Evans ratio (under 1: clustered) | {_fmt(rz.get('clark_evans'))} |",
          f"| Water in the altitude band {env.get('altitude_band_m', '')} m | "
          f"{_fmt(100.0 * float(env.get('share_in_band', 0.0)), 0)} % "
          f"(depth {env.get('depth_band_m', '')} m, transducer {env.get('transducer_depth_m', '')} m) |", ""]
    feats = pl.get("features") or []
    if feats:
        L += ["## Sources", "", "| Feature | Kind | Use | Length [m] | Area owned [ha] | Expected | Placed |",
              "|---|---|---|---|---|---|---|"]
        for f in feats:
            L.append(f"| {f.get('id')} | {f.get('kind')} | {f.get('use')} | {_fmt(f.get('length_m'), 1)} | "
                     f"{_fmt(f.get('owned_ha'), 3)} | {_fmt(f.get('expected'))} | {f.get('placed', 0)} |")
        L.append("")
    sl = dict(pl.get("small_litter") or {})
    if sl:
        L += ["## Small litter (seabed layer, not objects)", "",
              (f"- World mean {_fmt(sl.get('mean_per_m2'), 3)} hard items per m² (class `{sl.get('material')}`): "
               f"{_fmt(sl.get('base_per_m2'), 3)} per m² everywhere, {_fmt(sl.get('band_per_m2'), 3)} per m² within "
               f"{_fmt(sl.get('band_width_m'), 0)} m of the quays and pontoons."
               if sl.get("enabled") else "- None."), ""]
    if pl.get("warnings"):
        L += ["## Warnings", ""] + [f"- {w}" for w in pl["warnings"]] + [""]
    if pl.get("gaps"):
        L += ["## Not in the object set (state as a limitation)", ""] + [f"- {g}" for g in pl["gaps"]] + [""]
    return "\n".join(L)


def _feature_lines(meta: Mapping[str, Any], environment=None) -> list:
    """``(points, use, id, is line)`` of the features to draw, in the frame
    of the picture (the world frame of ``metadata.yaml``, or the
    environment's own frame)."""
    out = []
    uses = {str(f.get("id")): str(f.get("use")) for f in ((meta.get("placement") or {}).get("features") or [])}
    if environment is not None:
        for f in environment.features:
            out.append((np.asarray(f.points, float), uses.get(f.id, "none"), f.id, len(f.points) > 1))
        return out
    seen = set()
    by_base: dict[str, list] = {}
    for w in meta.get("walls") or []:
        base = w.get("curve") or w.get("name")
        by_base.setdefault(str(base), []).append(w["local"])
    for base, segs in by_base.items():
        pts = [segs[0][0]] + [s[1] for s in segs]
        out.append((np.asarray(pts, float), uses.get(base, "none"), base, True))
        seen.add(base)
    for s in meta.get("structures") or []:
        if "points" in s and str(s.get("name")) not in seen:
            pts = np.asarray(s["points"], float).reshape(-1, 2)
            out.append((pts, uses.get(str(s["name"]), "none"), str(s["name"]), len(pts) > 1))
    return out


def top_view(path: str | Path, meta: Mapping[str, Any], *, environment=None, plan=None,
             heat: np.ndarray | None = None, depth: Mapping[str, Any] | None = None,
             title: str | None = None, dpi: int = 150) -> Path:
    """The top view. With an ``environment`` it is drawn in the
    environment's frame from the plan; without, in the world frame from
    ``metadata.yaml`` (and ``depth``: the world's ``depth.npz`` arrays)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap, ListedColormap
    from matplotlib.lines import Line2D

    path = Path(path)
    two = heat is not None and environment is not None
    fig, axes = plt.subplots(1, 2 if two else 1, figsize=(13.0 if two else 8.0, 6.4), facecolor=SURFACE)
    axes = np.atleast_1d(axes)
    ax = axes[0]
    # ---- the water
    extent = None
    if environment is not None:
        env = environment
        ny, nx = env.shape
        extent = (env.x0, env.x0 + nx * env.cell_m, env.y0, env.y0 + ny * env.cell_m)
        alt = env.altitude()
        lo, hi = (4.5, 7.0)
        pl = dict(meta.get("placement") or {})
        band = (pl.get("envelope") or {}).get("altitude_band_m")
        m = float((pl.get("envelope") or {}).get("heave_margin_m", 0.1))
        if band:
            lo, hi = float(band[0]), float(band[1])
        cls = np.where(~env.water, 0, np.where((alt - m >= lo) & (alt + m <= hi), 2, 1))
    elif depth is not None:
        h = np.asarray(depth["height"], float)
        res = float(depth["resolution"])
        x0, y0 = float(depth["origin_x"]) - 0.5 * res, float(depth["origin_y"]) - 0.5 * res
        extent = (x0, x0 + h.shape[1] * res, y0, y0 + h.shape[0] * res)
        z = float((meta.get("domain") or {}).get("transducer_depth_m", 0.22))
        alt = -h - z
        cls = np.where(h >= -0.05, 0, np.where((alt - 0.1 >= 4.5) & (alt + 0.1 <= 7.0), 2, 1))
    else:
        cls = None
    if cls is not None:
        ax.imshow(cls, origin="lower", extent=extent, cmap=ListedColormap([LAND, WATER_OUT, SURFACE]),
                  vmin=0, vmax=2, interpolation="nearest", zorder=0)
    # ---- the path
    if environment is not None and environment.path_xy is not None and len(environment.path_xy) > 1:
        ax.plot(environment.path_xy[:, 0], environment.path_xy[:, 1], color=INK_2, lw=0.8, ls=(0, (4, 3)),
                zorder=2)
    # ---- features: bands in the colour of their use, labelled
    used = []
    n_lab = 0
    for pts, use, fid, is_line in _feature_lines(meta, environment):
        col = use_colour(use)
        if use == "none":
            # no source: a thin line for a wall, a small square for a pile or a block; no label
            if is_line:
                ax.plot(pts[:, 0], pts[:, 1], color=INK_2, lw=1.5, alpha=0.6, solid_capstyle="butt", zorder=3)
            else:
                ax.plot(pts[:, 0], pts[:, 1], marker="s", ms=4, mfc="none", mec=INK_2, mew=1.0, ls="none",
                        zorder=3)
            continue
        if is_line:
            ax.plot(pts[:, 0], pts[:, 1], color=col, lw=5.0, alpha=0.55, solid_capstyle="butt", zorder=3)
            mid = pts[len(pts) // 2] if len(pts) > 2 else 0.5 * (pts[0] + pts[-1])
        else:
            ax.plot(pts[:, 0], pts[:, 1], marker="o", ms=11, mfc="none", mec=col, mew=2.0, ls="none", zorder=3)
            mid = pts[0]
        # labels alternate above and below, so that two features that meet do not write over each other
        dy = 5 if n_lab % 2 == 0 else -11
        n_lab += 1
        ax.annotate(f"{fid} · {use.replace('_', ' ')}", xy=(mid[0], mid[1]), xytext=(5, dy),
                    textcoords="offset points", fontsize=6.5, color=INK_2, zorder=6)
        if use not in used:
            used.append(use)
    # ---- objects
    if plan is not None:
        objs = [(e["type"], e["x"], e["y"], e["params"].get("role", ""), e["params"].get("group_id"))
                for e in plan.instances]
        objs += [(e["type"], e["x"], e["y"], "structure", None) for e in plan.structures]
        groups = plan.groups
    else:
        objs = [(o["type"], o["x"], o["y"], o.get("role", ""), (o.get("params") or {}).get("group_id"))
                for o in meta.get("objects") or []]
        groups = (meta.get("placement") or {}).get("groups") or []
    shown = []
    for cls_name in list(TARGETS) + [DEBRIS]:
        pts = np.array([(x, y) for t, x, y, r, _ in objs if t == cls_name and r != "structure"]).reshape(-1, 2)
        if not len(pts):
            continue
        col, mk, _ = CLASS_STYLE[cls_name]
        ax.scatter(pts[:, 0], pts[:, 1], s=46 if cls_name != DEBRIS else 22, c=col, marker=mk,
                   edgecolors=SURFACE, linewidths=0.8, zorder=5)
        shown.append(cls_name)
    st = np.array([(x, y) for t, x, y, r, _ in objs if r == "structure" and t != "chain"]).reshape(-1, 2)
    if len(st):
        ax.scatter(st[:, 0], st[:, 1], s=40, marker=STRUCTURE_STYLE[1], facecolors="none",
                   edgecolors=STRUCTURE_STYLE[0], linewidths=1.3, zorder=4)
    for g in groups:
        ax.add_patch(plt.Circle((g["x"], g["y"]), float(g["radius_m"]) + 0.6, fill=False, ec=INK_2, lw=0.8,
                                ls=(0, (2, 2)), zorder=4))
    handles = [Line2D([], [], marker=CLASS_STYLE[c][1], ls="none", ms=7, mfc=CLASS_STYLE[c][0], mec=SURFACE,
                      label=CLASS_STYLE[c][2]) for c in shown]
    if len(st):
        handles.append(Line2D([], [], marker="s", ls="none", ms=7, mfc="none", mec=STRUCTURE_STYLE[0],
                              label="structure"))
    if groups:
        handles.append(Line2D([], [], marker="o", ls="none", ms=10, mfc="none", mec=INK_2, label="group"))
    for u in used:
        handles.append(Line2D([], [], color=use_colour(u), lw=5, alpha=0.55, label=u.replace("_", " ")))
    if cls is not None:
        handles += [Line2D([], [], marker="s", ls="none", ms=8, mfc=WATER_OUT, mec=LAND,
                           label="water outside the altitude band"),
                    Line2D([], [], marker="s", ls="none", ms=8, mfc=LAND, mec=LAND, label="land, behind a quay")]
    if handles:
        leg = ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7, frameon=False,
                        labelcolor=INK)
        leg.set_zorder(10)
    ax.set_aspect("equal")
    ax.set_xlabel("east [m]", color=INK_2, fontsize=8)
    ax.set_ylabel("north [m]", color=INK_2, fontsize=8)
    ax.tick_params(colors=INK_2, labelsize=7)
    for sp in ax.spines.values():
        sp.set_color(LAND)
    ax.set_facecolor(SURFACE)
    ax.set_title(title or str(meta.get("world_name", "")), color=INK, fontsize=10, loc="left")
    if extent:
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
    if two:
        a2 = axes[1]
        cmap = LinearSegmentedColormap.from_list("seq_blue", SEQ_BLUE)
        hm = np.where(np.isfinite(heat) & (heat > 0), heat, np.nan)
        im = a2.imshow(hm, origin="lower", extent=extent, cmap=cmap, interpolation="nearest", zorder=1)
        a2.imshow(np.where(environment.water, np.nan, 0.0), origin="lower", extent=extent,
                  cmap=ListedColormap([LAND]), interpolation="nearest", zorder=0)
        if plan is not None and len(plan.instances):
            xy = plan.xy()
            a2.scatter(xy[:, 0], xy[:, 1], s=9, c=INK, marker="o", linewidths=0, zorder=3)
        cb = fig.colorbar(im, ax=a2, fraction=0.035, pad=0.02)
        cb.set_label("expected objects per hectare", color=INK_2, fontsize=8)
        cb.ax.tick_params(colors=INK_2, labelsize=7)
        a2.set_aspect("equal")
        a2.set_xlabel("east [m]", color=INK_2, fontsize=8)
        a2.tick_params(colors=INK_2, labelsize=7)
        for sp in a2.spines.values():
            sp.set_color(LAND)
        a2.set_title("Expected density (points: the objects placed)", color=INK, fontsize=10, loc="left")
    fig.savefig(path, dpi=dpi, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)
    return path


def write_card(world_dir: str | Path, out_dir: str | Path | None = None, *, plan=None, environment=None,
               heat: np.ndarray | None = None) -> Path:
    """Write ``world_card.md`` and ``world_card.png`` for a built world; the
    path of the markdown file. ``out_dir``: default
    ``<worlds root>/.population/<path name>/<world name>/``."""
    world_dir = Path(world_dir).expanduser()
    meta = yaml.safe_load((world_dir / "metadata.yaml").read_text(encoding="utf-8")) or {}
    if out_dir is None:
        out_dir = world_dir.parent.parent / ".population" / world_dir.parent.name / world_dir.name
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    model = plan.model if plan is not None else None
    if model is None and (world_dir / "builder_state.yaml").is_file():
        st = yaml.safe_load((world_dir / "builder_state.yaml").read_text(encoding="utf-8")) or {}
        model = (st.get("placement") or {}).get("model")
    depth = None
    if environment is None and (world_dir / "depth.npz").is_file():
        with np.load(world_dir / "depth.npz", allow_pickle=False) as z:
            depth = {k: z[k] for k in z.files}
    if environment is not None and plan is not None:
        # the plan is in the design frame; so is the environment: draw both there
        top_view(out_dir / "world_card.png", meta, environment=environment, plan=plan, heat=heat)
    else:
        top_view(out_dir / "world_card.png", meta, depth=depth)
    md = out_dir / "world_card.md"
    md.write_text(card_markdown(meta, model=model), encoding="utf-8")
    return md
