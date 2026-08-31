"""Minimal read-only ``.svlog`` reader.

A ``.svlog`` is a bare stream of framed Cerulean Ping-Protocol packets, written
by ``sss_processor_node`` in ``BlueBoat-SSS`` from the ``~/raw`` topics. This
module reads the two packet kinds the metrics need and ignores the rest:

* **id 2198** ``OS_MONO_PROFILE`` -- one sonar ping. Carries ``ping_number``,
  ``timestamp_ms`` (the *device* uptime clock) and ``vehicle_heading_deg``.
  Decoded by :func:`blueboat_sss_sim.sonar.encoder.parse_frame`, the same
  function the simulator's own round-trip oracle uses.
* **id 150** mavlink2rest JSON envelope. ``LOCAL_POSITION_NED`` messages carry
  the vehicle position track on the *autopilot* ``time_boot_ms`` clock.

Together those give a per-ping pose: position interpolated from id-150,
heading straight off the ping itself.

Scope, deliberately: this is **not** a general ``.svlog`` loader. The GCS owns
that (``blueboat_gcs/core/svlog.py``), including per-segment counter offsets,
multi-session files and clock-validity forensics. This module reads a byte
format, imports nothing from ``BlueBoat-SSS`` and modifies nothing there
(CM-3 / NC #8). Files are opened read-only: recordings are primary data and are
never rewritten (CM-7).

**Side identity comes from the packet, never from the ``src`` tag** (CM-5).
``channel_number`` sits at byte 34 of the framed packet; where a device wrote
``255`` there, the sign of ``transducer_heading_deg`` relative to the vehicle
heading is the fallback. In the field corpus the ``src`` tag is wrong on up to
29.5 % of packets, so it is not consulted at all.
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


@dataclass(frozen=True)
class PositionRecord:
    """An id-150 ``LOCAL_POSITION_NED`` sample, converted back to ENU."""

    time_boot_ms: int
    x: float          # ENU east
    y: float          # ENU north
    z: float          # ENU up


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
    transducer heading, which sits 90 deg counter-clockwise of the vehicle
    heading to port and 90 deg clockwise to starboard.
    """
    ch = int(decoded["channel_number"])
    if ch == Side.PORT.channel:
        return Side.PORT
    if ch == Side.STARBOARD.channel:
        return Side.STARBOARD
    if ch != _CHANNEL_UNKNOWN:
        raise ValueError(f"unknown channel_number {ch}")
    rel = (float(decoded["transducer_heading_deg"])
           - float(decoded["vehicle_heading_deg"]) + 180.0) % 360.0 - 180.0
    return Side.PORT if rel < 0.0 else Side.STARBOARD


def read_streams(path: str | Path, with_power: bool = False
                 ) -> tuple[list[ProfileRecord], list[PositionRecord], int]:
    """One ordered pass over the file: profiles, positions, and the clock skew.

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
    are the same sim clock.
    """
    profiles: list[ProfileRecord] = []
    positions: list[PositionRecord] = []
    votes: list[int] = []
    last_profile_ts: int | None = None

    for pkt in iter_packets(path):
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
                **extra,
            ))
        elif pkt.packet_id == MAVLINK_WRAPPER_ID:
            try:
                msg = json.loads(pkt.payload.decode("utf-8")).get("message", {})
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue                               # not our envelope shape
            if msg.get("type") != "LOCAL_POSITION_NED":
                continue
            boot = int(msg["time_boot_ms"])
            positions.append(PositionRecord(
                time_boot_ms=boot,
                x=float(msg["y"]),                     # NED east  -> ENU x
                y=float(msg["x"]),                     # NED north -> ENU y
                z=-float(msg["z"]),                    # NED down  -> ENU up
            ))
            if last_profile_ts is not None:
                votes.append(last_profile_ts - boot)

    skew = int(np.median(votes)) if votes else 0
    return profiles, positions, skew


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
    **profile itself** -- every ping carries ``vehicle_heading_deg``, so no
    attitude packet is needed and the heading is exact per ping rather than
    interpolated.

    Returns ``(profiles, [(index into profiles, pose)])``. Profiles falling
    outside the position track's time span are dropped rather than
    extrapolated: a pose guessed past the end of the track would place pings on
    seabed the vehicle never passed over.
    """
    profiles, positions, skew = read_streams(path)
    if not profiles or not positions:
        return profiles, []

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
            yaw=compass_deg_to_enu_yaw(prof.vehicle_heading_deg),
        )))
    return profiles, paired
