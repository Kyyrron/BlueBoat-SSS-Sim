"""Discovery of MCS GPS-anchored paths (Screen 1's list).

Mirrors the MCS launch dialog's own enumeration
(``mcs/gui/dialogs/launch_dialog.py``): glob the trajectories directory
for ``*.yaml``, exclude ``*.meta.yaml``, and keep only files whose
runtime document carries a usable ``geo_anchor`` -- the same predicate as
``io_yaml.read_geo_anchor``. Deliberately a local reimplementation, not
an import: modules integrate by serving identical interfaces, never by
importing a neighbour's package (CM-3).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

#: Where the MCS Pattern Designer saves trajectories
#: (mcs/config/settings.py: DEFAULT_CONFIG_DIR / "trajectories").
DEFAULT_TRAJECTORIES_DIR = (Path.home() / ".config" / "blueboat_mcs"
                            / "trajectories")


@dataclass(frozen=True)
class AnchoredPathInfo:
    name: str
    file: Path
    lat0: float
    lon0: float


def read_geo_anchor(path: str | Path) -> dict | None:
    """The MCS predicate: the ``geo_anchor`` dict if the runtime YAML has
    one with both lat0 and lon0, else None. Never raises."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        ga = (doc or {}).get("geo_anchor")
        if isinstance(ga, dict) and "lat0" in ga and "lon0" in ga:
            return ga
    except Exception:
        pass
    return None


def list_anchored_paths(directory: str | Path | None = None
                        ) -> list[AnchoredPathInfo]:
    """GPS-anchored MCS paths, sorted by name -- the Screen 1 list."""
    directory = Path(directory) if directory else DEFAULT_TRAJECTORIES_DIR
    if not directory.exists():
        return []
    out: list[AnchoredPathInfo] = []
    for p in sorted(directory.glob("*.yaml")):
        if p.name.endswith(".meta.yaml"):
            continue
        ga = read_geo_anchor(p)
        if ga is not None:
            out.append(AnchoredPathInfo(name=p.stem, file=p,
                                        lat0=float(ga["lat0"]),
                                        lon0=float(ga["lon0"])))
    return out
