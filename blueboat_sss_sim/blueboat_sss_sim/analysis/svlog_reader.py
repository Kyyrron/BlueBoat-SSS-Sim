"""Minimal read-only ``.svlog`` reader.

A ``.svlog`` is a bare stream of framed Cerulean Ping-Protocol packets, written
by ``sss_processor_node`` in ``BlueBoat-SSS`` from the ``~/raw`` topics. This
module reads the two packet kinds the metrics need and ignores the rest:

* **id 2198** ``OS_MONO_PROFILE`` -- one sonar ping. Carries ``ping_number``,
  ``timestamp_ms`` (the *device* uptime clock) and ``vehicle_heading_deg``.
  Decoded by :func:`blueboat_sss_sim.sonar.encoder.parse_frame`, the same
  function the simulator's own round-trip oracle uses.
* **id 150** mavlink2rest JSON envelope. ``LOCAL_POSITION_NED`` messages carry
  the vehicle position track on the *autopilot* ``time_boot_ms`` clock;
  ``GLOBAL_POSITION_INT`` the GPS fixes (lat/lon degE7, ``hdg`` cdeg) and
  ``ATTITUDE`` roll/pitch/yaw, on the same clock.

Together those give a per-ping pose: position interpolated from id-150,
heading straight off the ping itself.

**Two clock regimes.** A SonarView-native recording carries a live
``time_boot_ms`` (25 Hz in the field corpus); a recording written by
``sss_processor_node`` carries **one frozen value** on every envelope (3
of the 7 corpus files). On a frozen clock no skew can be estimated and no
interpolation is meaningful, so :func:`pose_track` / :func:`gps_track`
pair every profile with the most recent fix **in file order** instead --
the rule the GCS's own loader applies -- and :class:`SvlogStreams`
reports ``clock_frozen`` so a consumer can say which regime it used.

Scope, deliberately: this is **not** a general ``.svlog`` loader. The GCS owns
that (``blueboat_gcs/core/svlog.py``), including per-segment counter offsets,
multi-session files and clock-validity forensics. This module reads a byte
format, imports nothing from ``BlueBoat-SSS`` and modifies nothing there
(CM-3 / NC #8). Files are opened read-only: recordings are primary data and are
never rewritten (CM-7).

**Side identity comes from the packet, never from the ``src`` tag** (CM-5).
``channel_number`` sits at byte 34 of the framed packet; where a device wrote
``255`` there, the sign of ``transducer_heading_deg`` — wrapped to
(-180, 180], the device reports the mounting angle -90 / +90 — is the
fallback, the same rule the GCS and the processor apply. In the field corpus
the ``src`` tag is wrong on up to 29.5 % of packets, so it is not consulted
at all.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator

import numpy as np

from ..core.geometry import compass_deg_to_enu_yaw
from ..core.types import Pose3D, Side
from ..sonar.encoder import OS_MONO_PROFILE_ID, parse_frame

#: mavlink2rest JSON envelope, as ``svlog_helper.MAVLINK_WRAPPER_ID``.
MAVLINK_WRAPPER_ID = 150

#: 'B' 'R' | u16 payload_len | u16 packet_id | u8 src | u8 dst
_HEADER = struct.Struct("<BBHHBB")

#: Value a device writes into ``channel_number`` when it does not know its own
#: channel. Two files in the field corpus carry it on every packet.
_CHANNEL_UNKNOWN = 255


@dataclass(frozen=True)
class SvlogPacket:
    """One framed packet: its id and its payload bytes."""

    packet_id: int
    payload: bytes
    raw: bytes


@dataclass(frozen=True)
class ProfileRecord:
    """The subset of an id-2198 profile the metrics need.

    The acquisition and power fields below are populated only when the file is
    read with ``with_power=True``. They are opt-in because ``pwr_results`` is
    the bulk of the file -- 1.2 kB per ping against ~40 B for the header the
    metrics path uses -- and that path reads whole harbour surveys.
    """

    side: Side
    ping_number: int
    timestamp_ms: int
    vehicle_heading_deg: float

    # --- acquisition in force for this ping (auto-gain moves within a file) ---
    start_mm: int = 0
    length_mm: int = 0
    num_results: int = 0
    gain_index: int = -1
    analog_gain: float = 0.0

    # --- power, as the device represents it -------------------------------
    # ``pwr_results`` is normalised per ping onto ``[min_pwr_db, max_pwr_db]``;
    # use :func:`blueboat_sss_sim.analysis.calibration.scale_to_db` to invert.
    min_pwr_db: float = 0.0
    max_pwr_db: float = 0.0
    pwr_results: np.ndarray | None = field(default=None, compare=False, repr=False)

    # --- always populated (cheap header fields) ----------------------------
    transducer_heading_deg: float = 0.0
    pulse_duration_sec: float = 0.0
    #: Packet index in the file -- the only order a frozen clock leaves.
    order: int = -1


@dataclass(frozen=True)
class PositionRecord:
    """An id-150 ``LOCAL_POSITION_NED`` sample, converted back to ENU."""

    time_boot_ms: int
    x: float          # ENU east
    y: float          # ENU north
    z: float          # ENU up
    order: int = -1   # packet index in the file


@dataclass(frozen=True)
class GpsRecord:
    """An id-150 ``GLOBAL_POSITION_INT`` fix (degrees, metres)."""

    order: int
    time_boot_ms: int
    lat: float
    lon: float
    alt_m: float
    relative_alt_m: float
    hdg_deg: float            # NaN when the autopilot reports 65535 / none
    vx_mps: float
    vy_mps: float


@dataclass(frozen=True)
class AttitudeRecord:
    """An id-150 ``ATTITUDE`` sample (radians, rad/s)."""

    order: int
    time_boot_ms: int
    roll: float
    pitch: float
    yaw: float
    rollspeed: float
    pitchspeed: float
    yawspeed: float


@dataclass
class SvlogStreams:
    """Everything one ordered pass over a recording yields.

    Iterating it gives ``(profiles, positions, skew_ms)`` so the historical
    ``profiles, positions, skew = read_streams(path)`` unpacking keeps
    working.
    """

    profiles: list[ProfileRecord]
    positions: list[PositionRecord]
    gps: list[GpsRecord]
    attitude: list[AttitudeRecord]
    skew_ms: int
    #: ``time_boot_ms`` took a single value on every position / GPS
    #: envelope: the skew is meaningless (reported 0) and consumers pair
    #: by file order.
    clock_frozen: bool
    n_packets: int = 0

    def __iter__(self):
        yield self.profiles
        yield self.positions
        yield self.skew_ms


# --------------------------------------------------------------- framing
def iter_packets(path: str | Path) -> Iterator[SvlogPacket]:
    """Yield every checksum-valid packet in the file, in order.

    A packet whose header, length or checksum does not hold is skipped and
    the scan resynchronises on the next ``'BR'`` -- a truncated tail (the
    normal shape of a recording killed mid-write) therefore costs the last
    packet, not the whole file.
    """
    data = Path(path).read_bytes()
    i, n = 0, len(data)
    while i + _HEADER.size + 2 <= n:
        if data[i] != 0x42 or data[i + 1] != 0x52:        # 'B', 'R'
            i = data.find(b"BR", i + 1)
            if i < 0:
                return
            continue
        _b, _r, plen, pid, _src, _dst = _HEADER.unpack_from(data, i)
        end = i + _HEADER.size + plen + 2
        if end > n:
            return                                        # truncated tail
        body = data[i:i + _HEADER.size + plen]
        (checksum,) = struct.unpack_from("<H", data, i + _HEADER.size + plen)
        if checksum != (sum(body) & 0xFFFF):
            i = data.find(b"BR", i + 1)                   # bad frame; resync
            if i < 0:
                return
            continue
        yield SvlogPacket(pid, bytes(body[_HEADER.size:]), bytes(data[i:end]))
        i = end


# --------------------------------------------------------------- decoding
def _side_from_packet(decoded: dict) -> Side:
    """Side identity from the packet itself (CM-5).

    ``channel_number`` first; where the device wrote ``255``, fall back to the
    sign of ``transducer_heading_deg`` wrapped to (-180, 180]: negative =
    port, positive = starboard. The real device reports the mounting angle
    (-90 / +90) and the encoder has done the same since 2026-09-15; the
    wrap keeps older simulator logs, which carried an absolute bearing,
    readable on the same rule (the GCS's ``core.svlog.resolve_side`` and the
    processor's ``sss_helper.side_from_fields`` apply it verbatim). A
    bearing of exactly 0 or a non-finite one carries no side and raises.
    """
    ch = int(decoded["channel_number"])
    if ch == Side.PORT.channel:
        return Side.PORT
    if ch == Side.STARBOARD.channel:
        return Side.STARBOARD
    if ch != _CHANNEL_UNKNOWN:
        raise ValueError(f"unknown channel_number {ch}")
    h = float(decoded["transducer_heading_deg"])
    if not np.isfinite(h) or h == 0.0:
        raise ValueError("channel_number 255 and no transducer bearing sign")
    rel = (h + 180.0) % 360.0 - 180.0
    if rel == -180.0:
        rel = 180.0
    if rel == 0.0:
        raise ValueError("channel_number 255 and no transducer bearing sign")
    return Side.PORT if rel < 0.0 else Side.STARBOARD


def read_svlog(path: str | Path, with_power: bool = False) -> SvlogStreams:
    """One ordered pass over the file: profiles, positions, GPS fixes,
    attitude samples, the clock skew and the clock regime.

    The three come out of a single pass because the skew can only be estimated
    *in file order*. The sonar devices and the autopilot do not share a clock
    -- ``timestamp_ms`` counts device uptime, ``time_boot_ms`` counts autopilot
    uptime, and on a real log the two differ by a large, per-file arbitrary
    constant. So the two streams cannot be compared numerically to discover the
    offset; that would assume the answer. What *is* known is that a packet
    written next to another was written at nearly the same instant, so each
    mavlink packet votes ``(most recent profile's timestamp_ms) -
    time_boot_ms`` and the median wins. This is the technique the GCS's own
    loader uses on the same files.

    A log with no positions, or none interleaved with a profile, yields skew 0
    -- which is also the correct answer for a simulated log, whose two clocks
    are the same sim clock. A log whose ``time_boot_ms`` never changes (a
    processor-written recording) also yields skew 0, with ``clock_frozen``
    set: the votes would all be one meaningless constant.
    """
    profiles: list[ProfileRecord] = []
    positions: list[PositionRecord] = []
    gps: list[GpsRecord] = []
    attitude: list[AttitudeRecord] = []
    votes: list[int] = []
    boot_values: set[int] = set()
    last_profile_ts: int | None = None
    order = -1

    for pkt in iter_packets(path):
        order += 1
        if pkt.packet_id == OS_MONO_PROFILE_ID:
            d = parse_frame(pkt.raw)
            last_profile_ts = int(d["timestamp_ms"])
            extra = {}
            if with_power:
                extra = dict(
                    start_mm=int(d["start_mm"]),
                    length_mm=int(d["length_mm"]),
                    num_results=int(d["num_results"]),
                    gain_index=int(d["gain_index"]),
                    analog_gain=float(d["analog_gain"]),
                    min_pwr_db=float(d["min_pwr_db"]),
                    max_pwr_db=float(d["max_pwr_db"]),
                    pwr_results=d["pwr_results"],
                )
            profiles.append(ProfileRecord(
                side=_side_from_packet(d),
                ping_number=int(d["ping_number"]),
                timestamp_ms=last_profile_ts,
                vehicle_heading_deg=float(d["vehicle_heading_deg"]),
                transducer_heading_deg=float(d["transducer_heading_deg"]),
                pulse_duration_sec=float(d["pulse_duration_sec"]),
                order=order,
                **extra,
            ))
        elif pkt.packet_id == MAVLINK_WRAPPER_ID:
            try:
                msg = json.loads(pkt.payload.decode("utf-8")).get("message", {})
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue                               # not our envelope shape
            mtype = msg.get("type")
            try:
                if mtype == "LOCAL_POSITION_NED":
                    boot = int(msg["time_boot_ms"])
                    positions.append(PositionRecord(
                        time_boot_ms=boot,
                        x=float(msg["y"]),             # NED east  -> ENU x
                        y=float(msg["x"]),             # NED north -> ENU y
                        z=-float(msg["z"]),            # NED down  -> ENU up
                        order=order,
                    ))
                elif mtype == "GLOBAL_POSITION_INT":
                    boot = int(msg["time_boot_ms"])
                    hdg = msg.get("hdg")
                    hdg_deg = (float("nan") if hdg is None or int(hdg) == 65535
                               else float(hdg) / 100.0)
                    gps.append(GpsRecord(
                        order=order, time_boot_ms=boot,
                        lat=float(msg["lat"]) / 1e7,
                        lon=float(msg["lon"]) / 1e7,
                        alt_m=float(msg.get("alt", 0)) / 1000.0,
                        relative_alt_m=float(msg.get("relative_alt", 0)) / 1000.0,
                        hdg_deg=hdg_deg,
                        vx_mps=float(msg.get("vx", 0)) / 100.0,
                        vy_mps=float(msg.get("vy", 0)) / 100.0,
                    ))
                elif mtype == "ATTITUDE":
                    attitude.append(AttitudeRecord(
                        order=order, time_boot_ms=int(msg["time_boot_ms"]),
                        roll=float(msg["roll"]), pitch=float(msg["pitch"]),
                        yaw=float(msg["yaw"]),
                        rollspeed=float(msg.get("rollspeed", 0.0)),
                        pitchspeed=float(msg.get("pitchspeed", 0.0)),
                        yawspeed=float(msg.get("yawspeed", 0.0))))
                    continue
                else:
                    continue
            except (KeyError, TypeError, ValueError):
                continue                               # malformed message
            boot_values.add(boot)
            if last_profile_ts is not None:
                votes.append(last_profile_ts - boot)

    clock_frozen = bool(boot_values) and len(boot_values) <= 1
    skew = int(np.median(votes)) if votes and not clock_frozen else 0
    return SvlogStreams(profiles, positions, gps, attitude, skew,
                        clock_frozen, n_packets=order + 1)


def read_streams(path: str | Path, with_power: bool = False
                 ) -> tuple[list[ProfileRecord], list[PositionRecord], int]:
    """``(profiles, positions, skew_ms)`` -- see :func:`read_svlog`."""
    s = read_svlog(path, with_power=with_power)
    return s.profiles, s.positions, s.skew_ms


def pair_by_order(profile_order: np.ndarray, item_order: np.ndarray
                  ) -> np.ndarray:
    """Index of the last item written *before* each profile (file order);
    a profile preceding every item takes the first one."""
    item_order = np.asarray(item_order)
    idx = np.searchsorted(item_order, np.asarray(profile_order),
                          side="right") - 1
    return np.clip(idx, 0, max(len(item_order) - 1, 0))


def read_profiles(path: str | Path,
                  with_power: bool = False) -> list[ProfileRecord]:
    """Every id-2198 profile in the file, in recorded order."""
    return read_streams(path, with_power=with_power)[0]


def read_positions(path: str | Path) -> list[PositionRecord]:
    """Every id-150 ``LOCAL_POSITION_NED`` sample, converted NED -> ENU.

    The writer converts ENU -> NED as ``x_n = y_enu``, ``y_e = x_enu``,
    ``z_d = -z_enu``; this inverts exactly that.
    """
    return read_streams(path)[1]


def estimate_boot_skew_ms(path: str | Path) -> int:
    """Constant ``timestamp_ms - time_boot_ms`` offset for this file."""
    return read_streams(path)[2]


def pose_track(path: str | Path) -> tuple[list[ProfileRecord],
                                          list[tuple[int, Pose3D]]]:
    """Read the log and pair every profile with an interpolated vehicle pose.

    Position comes from the id-150 track, linearly interpolated onto the
    profile's own clock after removing the boot skew. Heading comes from the
    autopilot's ATTITUDE yaw (:func:`profile_heading_deg`), NOT from the
    ping's ``vehicle_heading_deg``: on real recordings that field is the
    device's own unreferenced heading (2026-11-01, pass 3: offset to the
    autopilot yaw differs per transducer and per run, -168..+90 deg).

    Returns ``(profiles, [(index into profiles, pose)])``. Profiles falling
    outside the position track's time span are dropped rather than
    extrapolated: a pose guessed past the end of the track would place pings on
    seabed the vehicle never passed over. On a frozen autopilot clock every
    profile takes the most recent position **in file order** instead
    (within one ping period of the truth at the corpus's 25 Hz).
    """
    s = read_svlog(path)
    profiles, positions, skew = s.profiles, s.positions, s.skew_ms
    if not profiles or not positions:
        return profiles, []
    hdg = profile_heading_deg(s)

    if s.clock_frozen:
        idx = pair_by_order(np.array([p.order for p in profiles]),
                            np.array([p.order for p in positions]))
        return profiles, [
            (i, Pose3D(x=positions[j].x, y=positions[j].y, z=positions[j].z,
                       yaw=compass_deg_to_enu_yaw(float(hdg[i]))))
            for i, (prof, j) in enumerate(zip(profiles, idx))]

    pos_t = np.array([p.time_boot_ms + skew for p in positions],
                     dtype=np.float64)
    order = np.argsort(pos_t, kind="stable")
    pos_t = pos_t[order]
    pos_x = np.array([positions[i].x for i in order], dtype=np.float64)
    pos_y = np.array([positions[i].y for i in order], dtype=np.float64)
    pos_z = np.array([positions[i].z for i in order], dtype=np.float64)

    paired: list[tuple[int, Pose3D]] = []
    for i, prof in enumerate(profiles):
        t = float(prof.timestamp_ms)
        if t < pos_t[0] or t > pos_t[-1]:
            continue
        paired.append((i, Pose3D(
            x=float(np.interp(t, pos_t, pos_x)),
            y=float(np.interp(t, pos_t, pos_y)),
            z=float(np.interp(t, pos_t, pos_z)),
            yaw=compass_deg_to_enu_yaw(float(hdg[i])),
        )))
    return profiles, paired


def profile_heading_deg(streams: SvlogStreams,
                        index: np.ndarray | None = None) -> np.ndarray:
    """Vehicle compass heading [deg] at each profile (``index`` into
    ``streams.profiles``, default all), from the autopilot's ATTITUDE yaw
    (MAVLink NED: radians clockwise from north) -- interpolated on the
    profile clock after the boot skew (circularly), or the most recent sample
    in file order on a frozen clock. The ping's own ``vehicle_heading_deg``
    is used only when the file carries no ATTITUDE.

    Why not the ping field: on the real corpus it is the device's own
    unreferenced heading -- its offset to the autopilot yaw differs per
    transducer and per run (pass 3, 2026-11-01: pool -44..-168 deg, beach
    bath3 -77/-56, object_boiteVsDistance -107/-120, harbour
    fullHarbourCleanExample +90/+60), while ATTITUDE yaw and the GPS heading
    agree with the course over ground within ~7 deg. The GCS replay uses
    ATTITUDE yaw too. A simulated recording carries the true heading in both.
    """
    prof = streams.profiles
    idx = (np.arange(len(prof)) if index is None
           else np.asarray(index, dtype=np.int64))
    att = streams.attitude
    if not att:
        return np.array([prof[i].vehicle_heading_deg for i in idx],
                        dtype=np.float64) % 360.0
    if streams.clock_frozen:
        j = pair_by_order(np.array([prof[i].order for i in idx]),
                          np.array([a.order for a in att]))
        return np.degrees(np.array([att[k].yaw for k in j],
                                   dtype=np.float64)) % 360.0
    t = np.array([a.time_boot_ms + streams.skew_ms for a in att],
                 dtype=np.float64)
    o = np.argsort(t, kind="stable")
    t = t[o]
    yaw = np.array([att[i].yaw for i in o], dtype=np.float64)
    pt = np.array([prof[i].timestamp_ms for i in idx], dtype=np.float64)
    c = np.interp(pt, t, np.cos(yaw))
    sn = np.interp(pt, t, np.sin(yaw))
    return np.degrees(np.arctan2(sn, c)) % 360.0


def gps_track(path: str | Path, streams: SvlogStreams | None = None
              ) -> tuple[SvlogStreams, np.ndarray, np.ndarray, np.ndarray]:
    """Every profile paired with a GPS position.

    Returns ``(streams, index, lat, lon)`` -- ``index`` into
    ``streams.profiles`` for the profiles that could be placed. On a live
    clock lat/lon are interpolated onto the profile's own clock after the
    boot skew, and profiles outside the fix track's span are dropped; on a
    frozen clock every profile takes the most recent fix in file order.
    ``streams`` may be passed to avoid a second read.
    """
    s = streams if streams is not None else read_svlog(path)
    if not s.profiles or not s.gps:
        return s, np.empty(0, int), np.empty(0), np.empty(0)
    if s.clock_frozen:
        idx = pair_by_order(np.array([p.order for p in s.profiles]),
                            np.array([g.order for g in s.gps]))
        lat = np.array([s.gps[j].lat for j in idx])
        lon = np.array([s.gps[j].lon for j in idx])
        return s, np.arange(len(s.profiles)), lat, lon
    gt = np.array([g.time_boot_ms + s.skew_ms for g in s.gps], dtype=np.float64)
    order = np.argsort(gt, kind="stable")
    gt = gt[order]
    glat = np.array([s.gps[i].lat for i in order])
    glon = np.array([s.gps[i].lon for i in order])
    pt = np.array([p.timestamp_ms for p in s.profiles], dtype=np.float64)
    inside = (pt >= gt[0]) & (pt <= gt[-1])
    index = np.flatnonzero(inside)
    return (s, index, np.interp(pt[index], gt, glat),
            np.interp(pt[index], gt, glon))
