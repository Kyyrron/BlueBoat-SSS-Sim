"""Ground-truth contact observations, and the three ways to obtain them.

One record type, :class:`ContactObservation`, and three sources that produce
it. Everything downstream works on the record, so the metric code never knows
which source ran:

``from_replay``
    Walks the bundle's ``trajectory.yaml`` at the bundle's own ping period and
    renders both sides. **Deterministic from the seed alone** -- the renderer
    takes no RNG (noise is applied by its caller, never inside it), so the same
    bundle always yields the same observations. This is the only source that
    can satisfy a "regenerate from the seed and get the same numbers" claim,
    and it is therefore the primary one.

``from_svlog``
    A real run. Recovers the *actually tracked* pose track from a recorded
    ``.svlog`` -- id-150 position, per-ping heading -- and re-derives contacts
    through the same renderer against the same scene. What replay is to the
    intended path, this is to the path the vehicle really took.

``from_jsonl``
    The published ``ground_truth/contacts`` stream dumped a line per message
    (``ros2 topic echo --field data`` or an equivalent bag export). The only
    source that shows what the node *actually emitted*, so it is the
    cross-check on the other two -- but it carries no pose, so aspect is
    unavailable from it.

**Opportunities are counted from the ping cadence, never from message count.**
``sss_sim_node`` publishes a ground-truth message only when a ping saw at least
one object, so silence means "nothing in the swath", not "no ping". A
denominator taken from message count would silently drop every empty ping.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional

import numpy as np

from ..core.geometry import enu_yaw_to_compass_deg
from ..core.types import Pose3D, Side
from ..mission.patterns import WaypointTrajectory
from ..sonar.config import SonarConfig
from ..sonar.renderer import GeometricRenderer
from ..worldgen.scene import SceneModel


@dataclass(frozen=True)
class ContactObservation:
    """One object, seen by one ping, on one side.

    ``look_heading_deg`` is the transducer's compass look direction -- the
    vehicle heading rotated 90 deg toward the side in question. It is what the
    aspect metrics need, and it is ``None`` only for a source that carries no
    pose (``from_jsonl``).
    """

    object_id: int
    object_type: str
    side: Side
    ping_number: int
    t_sim: float
    slant_range_m: float
    extent_bins: float
    shadow_bins: float
    visible: bool
    look_heading_deg: Optional[float] = None
    ghost: bool = False           # a multipath image of the object named by
                                  # object_id, not the object itself
    via: str = ""                 # "" = direct path, else the reflector


@dataclass
class ObservationSet:
    """Observations plus the provenance a report has to state (CM-13)."""

    observations: list[ContactObservation]
    source: str                       # "replay" | "svlog" | "jsonl"
    ping_cycles: int                  # ping cycles walked, both sides counted
    bundle: Optional[str] = None
    seed: Optional[int] = None
    detail: str = ""

    @property
    def has_aspect(self) -> bool:
        return any(o.look_heading_deg is not None for o in self.observations)


# ------------------------------------------------------------------ helpers
def _look_heading_deg(pose: Pose3D, side: Side) -> float:
    """Transducer compass look direction for this pose and side.

    Mirrors ``PingEncoder.encode``: the transducer heading is the vehicle
    heading minus ``side.sign * 90``, so port looks 90 deg counter-clockwise of
    the bow and starboard 90 deg clockwise.
    """
    return float((enu_yaw_to_compass_deg(pose.yaw) - side.sign * 90.0) % 360.0)


def _renderer_for(bundle: Path) -> tuple[SceneModel, SonarConfig,
                                         GeometricRenderer]:
    scene = SceneModel.load(bundle)
    cfg = SonarConfig.from_yaml(bundle / "sonar.yaml")
    return scene, cfg, GeometricRenderer(scene, cfg.acquisition, cfg.model)


def _observations_at(renderer: GeometricRenderer, pose: Pose3D, side: Side,
                     t: float, ping_number: int
                     ) -> Iterator[ContactObservation]:
    """Render one ping and yield its contacts as observations.

    Calls the public ``render()`` rather than reaching for the renderer's
    private geometry: the renderer is an extension seam and this package is
    downstream of it, not inside it. The rendered power is discarded -- only
    the ground truth is wanted -- which costs one full render per ping and is
    what keeps the renderer itself untouched (NC #4).
    """
    look = _look_heading_deg(pose, side)
    for c in renderer.render(side, pose, t).contacts:
        yield ContactObservation(
            object_id=c.object_id, object_type=c.object_type, side=c.side,
            ping_number=ping_number, t_sim=t, slant_range_m=c.slant_range_m,
            extent_bins=c.extent_bins, shadow_bins=c.shadow_bins,
            visible=c.visible, look_heading_deg=look,
            ghost=c.ghost, via=c.via)


# ------------------------------------------------------------------ sources
def from_replay(bundle: str | Path, max_pings: int | None = None
                ) -> ObservationSet:
    """Replay the bundle's own trajectory through the renderer.

    Deterministic: same bundle in, same observations out, byte for byte.
    ``max_pings`` caps the number of ping cycles (both sides per cycle), which
    is what keeps the smoke test's run inside its time budget; ``None`` walks
    the whole mission.
    """
    bundle = Path(bundle)
    scene, cfg, renderer = _renderer_for(bundle)
    traj = WaypointTrajectory.load_yaml(bundle / "trajectory.yaml")
    period = cfg.acquisition.ping_period_s(cfg.model.max_ping_rate_hz)

    n = int(traj.duration / period)
    if max_pings is not None:
        n = min(n, int(max_pings))

    obs: list[ContactObservation] = []
    for k in range(n):
        t = k * period
        x, y, yaw = traj.pose_at(t)
        pose = Pose3D(x, y, 0.0, yaw=yaw)
        for side in Side:
            obs += list(_observations_at(renderer, pose, side, t, k + 1))
    return ObservationSet(
        observations=obs, source="replay", ping_cycles=n,
        bundle=str(bundle), seed=scene.seed,
        detail=f"{n} ping cycles at {period * 1000:.1f} ms, "
               f"{traj.name} {traj.total_length:.0f} m")


def from_svlog(path: str | Path, bundle: str | Path) -> ObservationSet:
    """Re-derive contacts along the pose track recorded in a ``.svlog``.

    The log supplies where the vehicle actually was and where each transducer
    actually pointed; the bundle supplies the scene. Contacts come from the
    same renderer as ``from_replay``, so the two differ only in the path
    walked -- which is exactly the tracking error a real run adds.
    """
    from . import svlog_reader                     # local: keeps import cheap

    path, bundle = Path(path), Path(bundle)
    scene, _cfg, renderer = _renderer_for(bundle)
    profiles, paired = svlog_reader.pose_track(path)

    obs: list[ContactObservation] = []
    for idx, pose in paired:
        prof = profiles[idx]
        obs += list(_observations_at(renderer, pose, prof.side,
                                     prof.timestamp_ms / 1000.0,
                                     prof.ping_number))
    return ObservationSet(
        observations=obs, source="svlog", ping_cycles=len(paired),
        bundle=str(bundle), seed=scene.seed,
        detail=f"{len(paired)} of {len(profiles)} profiles had a pose "
               f"({path.name})")


def from_jsonl(path: str | Path) -> ObservationSet:
    """Read a dump of the published ``ground_truth/contacts`` stream.

    One JSON object per line, each the payload ``sss_sim_node`` publishes.
    Blank lines are skipped. No pose travels on that topic, so
    ``look_heading_deg`` stays ``None`` and the aspect metrics report as
    unavailable rather than being invented.
    """
    path = Path(path)
    obs: list[ContactObservation] = []
    cycles = 0
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            doc = json.loads(line)
            cycles += 1
            t = float(doc.get("t_sim", 0.0))
            for c in doc.get("contacts", []):
                obs.append(ContactObservation(
                    object_id=int(c["object_id"]),
                    object_type=str(c["type"]),
                    side=Side(c["side"]),
                    ping_number=int(c["ping_number"]),
                    t_sim=t,
                    slant_range_m=float(c["slant_range_m"]),
                    extent_bins=float(c["extent_bins"]),
                    shadow_bins=float(c["shadow_bins"]),
                    visible=bool(c["visible"]),
                    look_heading_deg=None,
                    # Defaulted: a dump taken before wall multipath existed
                    # carries neither key, and every contact in it is direct.
                    ghost=bool(c.get("ghost", False)),
                    via=str(c.get("via", ""))))
    return ObservationSet(
        observations=obs, source="jsonl", ping_cycles=cycles,
        detail=f"{cycles} published messages ({path.name}); "
               f"no pose on this topic, so aspect is unavailable")


def relative_aspect_deg(look_heading_deg: float, object_yaw_rad: float
                        ) -> float:
    """Aspect of the object relative to the look direction, in ``[0, 180)``.

    Folded to a half-turn because a rectangular object's acoustic response is
    symmetric under a 180 deg rotation: seeing a pipe end-on from the north and
    from the south is the same aspect, and keeping them apart would halve the
    sample count in every bin for no physical reason.
    """
    obj_heading = enu_yaw_to_compass_deg(object_yaw_rad)
    return float(np.mod(look_heading_deg - obj_heading, 180.0))
