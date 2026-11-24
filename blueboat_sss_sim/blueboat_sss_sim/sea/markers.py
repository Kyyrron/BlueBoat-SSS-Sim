"""Marker descriptions for the two visual indicators, ROS-free.

* :func:`rviz_markers` -- plain dicts the ROS shell copies into
  ``visualization_msgs/Marker`` (arrows at the boat for the current and
  the wave propagation direction, a text label above).
* :func:`gz_marker_request` -- the protobuf-text request for gz-gui's
  ``/marker_array`` service (``gz.msgs.Marker_V``), sent by the node with
  ``gz service`` because this workspace has no python gz bindings.

Arrows point *toward* where the water / waves are going (the direction of
motion), which is what an operator reads off a chart; the text carries the
"from" convention explicitly to avoid the ambiguity.
"""

from __future__ import annotations

import math

from ..core.geometry import compass_deg_to_enu_yaw
from .presets import SeaState, compass_name

NS = "sea_state"
ID_CURRENT, ID_WAVES, ID_TEXT = 1, 2, 3
CURRENT_RGBA = (0.10, 0.60, 1.00, 0.95)
WAVE_RGBA = (0.20, 0.90, 0.55, 0.95)
TEXT_RGBA = (1.0, 1.0, 1.0, 1.0)


def summary_text(state: SeaState, waves_status: str = "active") -> str:
    c, w = state.current, state.waves
    cur = ("current: none" if c.speed_mps <= 0.0 else
           f"current {c.speed_mps:.2f} m/s from {compass_name(c.from_deg)} "
           f"-> {compass_name(c.from_deg + 180)}")
    if w.hs_m <= 0.0:
        wav = "waves: calm"
    else:
        wav = (f"waves Hs {w.hs_m:.2f} m Tp {w.tp_s:.1f} s from "
               f"{compass_name(w.from_deg)} -> {compass_name(w.from_deg + 180)}")
        if waves_status != "active":
            wav += f" [{waves_status}]"
    return f"{cur} | {wav}"


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def rviz_markers(state: SeaState, boat_xy: tuple[float, float],
                 waves_status: str = "active", frame_id: str = "world",
                 z: float = 0.6) -> list[dict]:
    """Marker dicts: ``{ns, id, type, action, pose{x,y,z,qx,qy,qz,qw},
    scale(x,y,z), rgba, text}``; type 0 = ARROW, 9 = TEXT_VIEW_FACING;
    action 0 = ADD, 2 = DELETE."""
    x, y = float(boat_xy[0]), float(boat_xy[1])
    c, w = state.current, state.waves
    out = []
    if c.speed_mps > 0.0:
        yaw = compass_deg_to_enu_yaw(c.from_deg + 180.0)
        out.append({"ns": NS, "id": ID_CURRENT, "type": 0, "action": 0,
                    "pose": (x, y, z, *_yaw_quat(yaw)),
                    "scale": (1.0 + 3.0 * min(c.speed_mps, 1.0), 0.15, 0.15),
                    "rgba": CURRENT_RGBA, "text": ""})
    else:
        out.append({"ns": NS, "id": ID_CURRENT, "type": 0, "action": 2,
                    "pose": (x, y, z, 0.0, 0.0, 0.0, 1.0), "scale": (1, 1, 1),
                    "rgba": CURRENT_RGBA, "text": ""})
    if w.hs_m > 0.0 and waves_status == "active":
        yaw = compass_deg_to_enu_yaw(w.from_deg + 180.0)
        out.append({"ns": NS, "id": ID_WAVES, "type": 0, "action": 0,
                    "pose": (x, y, z + 0.4, *_yaw_quat(yaw)),
                    "scale": (1.0 + 4.0 * min(w.hs_m, 1.0), 0.12, 0.12),
                    "rgba": WAVE_RGBA, "text": ""})
    else:
        out.append({"ns": NS, "id": ID_WAVES, "type": 0, "action": 2,
                    "pose": (x, y, z, 0.0, 0.0, 0.0, 1.0), "scale": (1, 1, 1),
                    "rgba": WAVE_RGBA, "text": ""})
    out.append({"ns": NS, "id": ID_TEXT, "type": 9, "action": 0,
                "pose": (x, y, z + 1.4, 0.0, 0.0, 0.0, 1.0),
                "scale": (0.0, 0.0, 0.35), "rgba": TEXT_RGBA,
                "text": summary_text(state, waves_status)})
    for m in out:
        m["frame_id"] = frame_id
    return out


def _pb_color(rgba) -> str:
    r, g, b, a = rgba
    return f"{{r: {r:.3f} g: {g:.3f} b: {b:.3f} a: {a:.3f}}}"


def _pb_arrow(mid: int, x: float, y: float, z: float, yaw: float,
              length: float, rgba, axis: str, lifetime_s: int) -> str:
    # gz ARROW is drawn along its local axis (+Z by default); rotate that
    # axis onto the horizontal direction `yaw`.
    if axis == "x":
        qx, qy, qz, qw = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    else:
        # +Z -> horizontal at `yaw`: pitch by +90° about y, then yaw.
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        cp, sp = math.cos(math.pi / 4.0), math.sin(math.pi / 4.0)
        qw = cy * cp
        qx = -sy * sp
        qy = cy * sp
        qz = sy * cp
    scale = (f"{{x: {length:.3f} y: 0.25 z: 0.25}}" if axis == "x"
             else f"{{x: 0.25 y: 0.25 z: {length:.3f}}}")
    return (f"marker {{ ns: \"{NS}\" id: {mid} action: ADD_MODIFY type: ARROW "
            f"visibility: GUI lifetime {{sec: {lifetime_s}}} "
            f"pose {{position {{x: {x:.3f} y: {y:.3f} z: {z:.3f}}} "
            f"orientation {{x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f}}}}} "
            f"scale {scale} material {{ambient {_pb_color(rgba)} "
            f"diffuse {_pb_color(rgba)}}} }}")


def gz_marker_request(state: SeaState, origin_xy: tuple[float, float],
                      waves_status: str = "active", z: float = 3.0,
                      arrow_axis: str = "z", lifetime_s: int = 6) -> str:
    """``gz.msgs.Marker_V`` protobuf text for ``/marker_array``: an arrow
    per active component above ``origin_xy`` (the spawn / world origin,
    fixed so the operator always finds it) and a text summary."""
    x, y = float(origin_xy[0]), float(origin_xy[1])
    c, w = state.current, state.waves
    parts = []
    if c.speed_mps > 0.0:
        parts.append(_pb_arrow(ID_CURRENT, x, y, z,
                               compass_deg_to_enu_yaw(c.from_deg + 180.0),
                               1.0 + 3.0 * min(c.speed_mps, 1.0), CURRENT_RGBA,
                               arrow_axis, lifetime_s))
    else:
        parts.append(f"marker {{ ns: \"{NS}\" id: {ID_CURRENT} action: DELETE_MARKER }}")
    if w.hs_m > 0.0 and waves_status == "active":
        parts.append(_pb_arrow(ID_WAVES, x, y, z + 0.6,
                               compass_deg_to_enu_yaw(w.from_deg + 180.0),
                               1.0 + 4.0 * min(w.hs_m, 1.0), WAVE_RGBA,
                               arrow_axis, lifetime_s))
    else:
        parts.append(f"marker {{ ns: \"{NS}\" id: {ID_WAVES} action: DELETE_MARKER }}")
    text = summary_text(state, waves_status).replace('"', "'")
    parts.append(
        f"marker {{ ns: \"{NS}\" id: {ID_TEXT} action: ADD_MODIFY type: TEXT "
        f"visibility: GUI lifetime {{sec: {lifetime_s}}} "
        f"pose {{position {{x: {x:.3f} y: {y:.3f} z: {z + 1.6:.3f}}}}} "
        f"scale {{x: 0.5 y: 0.5 z: 0.5}} text: \"{text}\" "
        f"material {{ambient {_pb_color(TEXT_RGBA)} diffuse {_pb_color(TEXT_RGBA)}}} }}")
    return "\n".join(parts)


def gz_delete_all_request() -> str:
    return f"marker {{ ns: \"{NS}\" action: DELETE_ALL }}"
