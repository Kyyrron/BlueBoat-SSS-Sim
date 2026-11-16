"""Which BlueBoat plant Gazebo is integrating, and its vertical (heave / roll / pitch) numbers.

The simulator's hull wrench carries the hydrostatics of the vertical plane and the damping that Control's
Hydrodynamics plugin does not (one owner per term, R-8 of ``BlueBoat-Control/docs/REQUESTS.md``; session 03,
2026-11-15). To size its share it must know what the plugin and the rigid body already carry. Two sources, one
parser each, both ROS-free:

* at run time: the URDF the robot description carries (``/blueboat/robot_description``, latched), parsed by
  :func:`parse_urdf`;
* offline (smoke ``[10]``): Control's xacro files, parsed by :func:`plant_from_xacro` with ElementTree (comments are
  dropped by the parser, so commented-out tags never match). Since 2026-11-15 (session 04b) the files define their
  values through ``xacro:property`` elements inside ``xacro:if`` / ``xacro:unless`` blocks selected by the xacro
  arguments ``plant`` and ``vertical_model``: :func:`plant_from_xacro` evaluates those conditions and the ``${...}``
  expressions for the arguments it is given (a small evaluator over the file's own properties, no xacro).

A plant is told apart by its link yaw inertia: the BlueROV2 legacy plant integrates 3.9235025 kg·m², the identified
BlueBoat plant 7.50 (``PARAMETER_REGISTER.md`` §3). The constants below mirror both, for the offline integrator and
as the fallback when no description arrives; smoke ``[10]`` re-derives them from the xacro.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

VERTICAL_TAGS = ("zDotW", "kDotP", "mDotQ", "zW", "kP", "mQ", "zWabsW", "kPabsP", "mQabsQ")
LEGACY_YAW_INERTIA = 3.9235025
IDENTIFIED_YAW_INERTIA = 7.50


@dataclass(frozen=True)
class PlantVertical:
    """What Gazebo integrates in the vertical plane: rigid mass (with the two propellers), the base link's inertia
    in the LINK frame (roll, pitch, yaw), the CoG height below the link origin, and the Hydrodynamics plugin's
    vertical coefficients (Fossen sign: added mass and damping negative)."""

    name: str
    mass_kg: float
    inertia_link: tuple[float, float, float]
    cog_z_m: float
    added: tuple[float, float, float]          # zDotW, kDotP, mDotQ
    linear: tuple[float, float, float]         # zW, kP, mQ
    quadratic: tuple[float, float, float]      # zWabsW, kPabsP, mQabsQ
    source: str = "mirror"

    @property
    def added_mass_positive(self) -> tuple[float, float, float]:
        """(A33, A44, A55) as positive added inertias."""
        return tuple(float(-a) for a in self.added)


#: The BlueROV2 legacy plant (``plant:=bluerov2_legacy``; every simulation made before 2026-11-13).
LEGACY = PlantVertical("bluerov2_legacy", 16.15, (2.42369, 5.6403125, 3.9235025), -0.12913,
                       (-14.57, -0.12, -0.12), (-17.955, -10.888, -20.761), (-73.37135, -40.0, -40.0))
#: The identified BlueBoat plant as session 04b applied R-8 (2026-11-15; ``vertical_model:=sea_node``, every world
#: run): plugin vertical damping 0 (the simulator owns it); heave added mass capped at 14.5 kg (the plugin's explicit
#: added mass diverges for A >= m); R-8's roll / pitch added inertias carried as RIGID inertia (I + A kept: 2.42369 +
#: 1.79, 5.6403125 + 0.93); the physical CoG, 0.1376 m ABOVE the link origin (a mass budget, bracket +0.097..+0.178).
IDENTIFIED = PlantVertical("identified", 16.15, (4.21369, 6.5703125, 7.50), 0.1376,
                           (-14.5, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
#: The identified plant as committed by session 04 (c189cde), before R-8: the BlueROV2 vertical entries, the
#: third-party roll / pitch inertia and the box-construction CoG.
IDENTIFIED_PRE_R8 = replace(IDENTIFIED, inertia_link=(2.42369, 5.6403125, 7.50), cog_z_m=-0.12913,
                            added=LEGACY.added, linear=LEGACY.linear, quadratic=LEGACY.quadratic)

PLANTS = {"bluerov2_legacy": LEGACY, "identified": IDENTIFIED}


def plant_name_from_yaw_inertia(yaw: float) -> str:
    return "bluerov2_legacy" if abs(float(yaw) - LEGACY_YAW_INERTIA) < 0.05 else "identified"


def _rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def link_frame_diagonal(ixx: float, iyy: float, izz: float, rpy=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    """Diagonal of a diagonal inertia written in an inertial frame rotated by ``rpy`` relative to the link."""
    R = _rpy_matrix(*rpy)
    I = R @ np.diag([ixx, iyy, izz]) @ R.T
    return tuple(float(v) for v in np.round(np.diag(I), 9))


def _local(tag: str) -> str:
    return tag.split("}")[-1]


def _vertical_from(root: ET.Element) -> dict[str, float]:
    found: dict[str, float] = {}
    for el in root.iter():
        t = _local(el.tag)
        if t in VERTICAL_TAGS and el.text is not None:
            try:
                found[t] = float(el.text.strip())
            except ValueError:
                pass
    return found


def parse_urdf(urdf_xml: str, link_suffix: str = "base_link") -> PlantVertical:
    """The vertical numbers of a processed URDF (the robot description). The base link is the link whose name ends
    with ``link_suffix`` (namespaced descriptions name it ``blueboat/base_link``); its mass adds the propellers'
    (every link whose name contains ``prop``). Raises ValueError when the description lacks the base link."""
    root = ET.fromstring(urdf_xml)
    base, props = None, 0.0
    for link in root.iter("link"):
        name = link.get("name", "")
        inert = link.find("inertial")
        if inert is None:
            continue
        m = float(inert.find("mass").get("value"))
        if name.endswith(link_suffix):
            base = (link, inert, m)
        elif "prop" in name.lower():
            props += m
    if base is None:
        raise ValueError(f"no link ending in {link_suffix!r} with an <inertial>")
    _, inert, m = base
    org = inert.find("origin")
    rpy = tuple(float(v) for v in (org.get("rpy", "0 0 0").split() if org is not None else [0, 0, 0]))
    xyz = tuple(float(v) for v in (org.get("xyz", "0 0 0").split() if org is not None else [0, 0, 0]))
    ie = inert.find("inertia")
    diag = link_frame_diagonal(float(ie.get("ixx")), float(ie.get("iyy")), float(ie.get("izz")), rpy)
    v = _vertical_from(root)
    missing = [t for t in VERTICAL_TAGS if t not in v]
    if missing:
        raise ValueError(f"robot description lacks Hydrodynamics tags {missing}")
    name = plant_name_from_yaw_inertia(diag[2])
    return PlantVertical(name, m + props, diag, xyz[2], (v["zDotW"], v["kDotP"], v["mDotQ"]),
                         (v["zW"], v["kP"], v["mQ"]), (v["zWabsW"], v["kPabsP"], v["mQabsQ"]),
                         source="robot_description")


def _xacro_numbers(text: str) -> dict[str, float]:
    props = {m.group(1): m.group(2) for m in re.finditer(
        r'<xacro:property\s+name="([^"]+)"\s+value="([^"$]+)"', text)}
    out = {}
    for k, v in props.items():
        try:
            out[k] = float(v)
        except ValueError:
            pass
    return out


def _xacro_angle(tok: str) -> float:
    """``0``, ``1.57``, ``${pi/2}``, ``${-pi/2}``, ``${pi}`` -> radians (the forms the BlueBoat xacro uses)."""
    s = tok.strip().replace("${", "").replace("}", "").replace(" ", "")
    m = re.fullmatch(r"(-?)pi(?:/([0-9.]+))?", s)
    if m:
        return (-1.0 if m.group(1) else 1.0) * np.pi / (float(m.group(2)) if m.group(2) else 1.0)
    return float(s)


_XNS = "{http://www.ros.org/wiki/xacro}"
#: The Python functions xacro's ``${...}`` offers that Control's files use (no other builtins).
_XACRO_FUNCS = {"pow": pow, "str": str, "abs": abs, "float": float, "int": int, "min": min, "max": max,
                "bool": bool, "round": round}


def _xeval(expr: str, env: dict):
    """Value of a xacro attribute: ``${expr}`` evaluated over the properties seen so far (no builtins), ``$(arg x)``
    from the arguments, anything else as written (a number when it parses)."""
    t = (expr or "").strip()
    t = re.sub(r"\$\(arg\s+([A-Za-z_][A-Za-z0-9_]*)\)", lambda m: repr(env.get(m.group(1), "")), t)
    m = re.fullmatch(r"\$\{(.*)\}", t, flags=re.S)
    if m:
        return eval(m.group(1), {"__builtins__": {}, "pi": np.pi, "True": True, "False": False,  # noqa: S307
                                 **_XACRO_FUNCS}, env)
    try:
        return float(t)
    except ValueError:
        return t.strip("'")


def _xacro_walk(el: ET.Element, env: dict, keep: list) -> None:
    """Depth-first over a xacro tree: record properties into ``env`` in document order, descend into the
    ``xacro:if`` / ``xacro:unless`` blocks whose condition holds, and collect the other elements into ``keep``."""
    for ch in el:
        tag = ch.tag
        if tag in (_XNS + "if", _XNS + "unless"):
            try:
                cond = bool(_xeval(ch.get("value", ""), env))
            except Exception:  # noqa: BLE001 -- a condition we cannot evaluate selects nothing
                continue
            if cond == (tag == _XNS + "if"):
                _xacro_walk(ch, env, keep)
            continue
        if tag == _XNS + "property":
            try:
                env[ch.get("name")] = _xeval(ch.get("value", ""), env)
            except Exception:  # noqa: BLE001 -- a property we cannot evaluate stays undefined
                pass
            continue
        if tag == _XNS + "arg":
            env.setdefault(ch.get("name"), _xeval(ch.get("default", ""), env))
            continue
        keep.append(ch)
        _xacro_walk(ch, env, keep)


def plant_from_xacro(control_root: str | Path, plant: str, vertical_model: str = "sea_node") -> PlantVertical:
    """The vertical numbers of a plant as Control's xacro files state them (offline, no xacro processing): the
    base link's ``<inertial>`` that the arguments ``plant`` / ``vertical_model`` select in ``blueboat.xacro`` and
    the vertical tags of its hydrodynamics file (``hydrodynamics.xacro`` for the legacy plant,
    ``hydrodynamics_identified.xacro`` for the identified one), each ``${...}`` evaluated."""
    urdf = Path(control_root) / "blueboat_description" / "urdf"
    want_legacy = plant == "bluerov2_legacy"
    env: dict = {"plant": plant, "vertical_model": vertical_model, "namespace": "blueboat", "simulation": True,
                 "replay": False, "density": 1000.0, "thrusters": "thrusters_ur"}
    keep: list = []
    _xacro_walk(ET.fromstring((urdf / "blueboat.xacro").read_text(encoding="utf-8")), env, keep)
    inertial = None
    for el in keep:
        if el.tag == "link" and "base_link" in (el.get("name") or "") and el.find("inertial") is not None:
            inertial = el.find("inertial")
            break
    if inertial is None:
        inertial = next(el for el in keep if el.tag == "inertial")
    ie, org = inertial.find("inertia"), inertial.find("origin")
    rpy = [float(_xeval(f"${{{tok}}}" if not tok.startswith("$") else tok, env))
           for tok in ((org.get("rpy") if org is not None else "0 0 0") or "0 0 0").split()]
    xyz = [float(_xeval(f"${{{tok}}}" if not tok.startswith("$") else tok, env))
           for tok in ((org.get("xyz") if org is not None else "0 0 0") or "0 0 0").split()]
    diag = link_frame_diagonal(*(float(_xeval(ie.get(k), env)) for k in ("ixx", "iyy", "izz")), rpy)
    mass = float(env.get("mass", 16.01)) + 2.0 * float(env.get("prop_mass", 0.07))
    hfile = "hydrodynamics.xacro" if want_legacy else "hydrodynamics_identified.xacro"
    henv = dict(env)
    hkeep: list = []
    _xacro_walk(ET.fromstring((urdf / hfile).read_text(encoding="utf-8")), henv, hkeep)
    v: dict[str, float] = {}
    for el in hkeep:
        t = _local(el.tag)
        if t in VERTICAL_TAGS and el.text is not None:
            try:
                v[t] = float(_xeval(el.text, henv))
            except Exception:  # noqa: BLE001
                pass
    missing = [t for t in VERTICAL_TAGS if t not in v]
    if missing:
        raise ValueError(f"{hfile}: vertical tags not resolved {missing}")
    return PlantVertical(plant, mass, diag, xyz[2], (v["zDotW"], v["kDotP"], v["mDotQ"]),
                         (v["zW"], v["kP"], v["mQ"]), (v["zWabsW"], v["kPabsP"], v["mQabsQ"]),
                         source=f"xacro:{hfile}:{vertical_model}")


def presets_file_for(plant: str, config_dir: str | Path) -> Path:
    """The presets file the plant floats on: the identified plant on ``sea_states.yaml``, the legacy plant on the
    frozen ``sea_states_legacy.yaml`` (today's hull and damping split, bit for bit)."""
    d = Path(config_dir)
    return d / ("sea_states_legacy.yaml" if plant == "bluerov2_legacy" else "sea_states.yaml")
