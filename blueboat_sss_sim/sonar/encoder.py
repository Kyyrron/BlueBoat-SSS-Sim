"""Ping -> wire formats.

Produces, for every rendered ping, the two artefacts the real
``sss_node.py`` publishes:

1. the field set of ``blueboat_interfaces/OmniscanProfile`` (as a plain
   dict here, so this module stays ROS-free; the ROS node copies fields);
2. the **byte-exact Cerulean Ping-Protocol frame** for
   ``OS_MONO_PROFILE`` (ID 2198) that goes on the ``.../raw`` topic and
   from which the downstream processor rebuilds its ``.svlog``.

The payload layout below was reverse-engineered from a captured frame and
cross-checked against every value visible in the corresponding
``profile`` topic echo (range, num_results, sos, frequency, gains,
headings) -- see docs/topics.md for the annotated byte map.

    'B' 'R' | u16 payload_len | u16 msg_id=2198 | u8 src | u8 dst |
    u32 ping_number | u32 start_mm | u32 length_mm | u32 timestamp_ms |
    u32 ping_hz | u16 gain_index | u16 num_results | u16 sos_dmps |
    u8 channel_number | u8 reserved | f32 pulse_duration_sec |
    f32 analog_gain | f32 max_pwr_db | f32 min_pwr_db |
    f32 transducer_heading_deg | f32 vehicle_heading_deg |
    u16 pwr_results[num_results] | u16 checksum(sum of all prior bytes)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

from ..core.geometry import enu_yaw_to_compass_deg
from ..core.types import Ping, Side
from .config import (OMNISCAN_FREQ_HZ, SPEED_OF_SOUND_MPS, AcquisitionParams,
                     SonarModelConfig)

OS_MONO_PROFILE_ID = 2198
_HEADER = struct.Struct("<BBHHBB")
_PAYLOAD_FIXED = struct.Struct("<IIIIIHHHBBffffff")
ANALOG_GAIN_TABLE = {0: 20.0, 1: 33.0, 2: 46.0, 3: 60.0, 4: 74.55, 5: 88.0}


@dataclass
class EncodedPing:
    """Everything the ROS node needs to publish one ping."""

    side: Side
    frame_id: str
    ping_number: int
    start_mm: int
    length_mm: int
    timestamp_ms: int
    ping_hz: int
    gain_index: int
    num_results: int
    sos_dmps: int
    channel_number: int
    pulse_duration_sec: float
    analog_gain: float
    max_pwr_db: float
    min_pwr_db: float
    transducer_heading_deg: float
    vehicle_heading_deg: float
    pwr_results: np.ndarray          # uint16[num_results]
    raw_frame: bytes                 # framed Ping-Protocol bytes


class PingEncoder:
    """Stateful per-side encoder (ping counter + device uptime clock)."""

    def __init__(self, side: Side, acquisition: AcquisitionParams,
                 model: SonarModelConfig, channel_number: int = 1) -> None:
        self._side = side
        self._acq = acquisition
        self._cfg = model
        self._channel = channel_number
        self._ping_number = 0
        self._frame_id = f"sss_{side.value}_link"

    # ------------------------------------------------------------------
    def encode(self, ping: Ping, gain_multiplier: float = 1.0) -> EncodedPing:
        acq, cfg = self._acq, self._cfg
        self._ping_number += 1

        # Linear power -> u16 counts, gain-index dependent scaling.
        gain_db = (acq.gain_index - 4) * cfg.gain_index_step_db
        scale = cfg.base_scale * (10.0 ** (gain_db / 10.0)) * gain_multiplier
        counts = np.clip(ping.power * scale, 0.0, 65535.0).astype(np.uint16)

        nz = counts[counts > 0]
        max_db = 10.0 * np.log10(max(int(counts.max()), 1)) + cfg.calibration_db_offset
        min_db = (10.0 * np.log10(max(int(nz.min()), 1)) + cfg.calibration_db_offset
                  if nz.size else 0.0)

        vehicle_heading = enu_yaw_to_compass_deg(ping.pose.yaw)
        transducer_heading = (vehicle_heading - self._side.sign * 90.0) % 360.0

        enc = EncodedPing(
            side=self._side,
            frame_id=self._frame_id,
            ping_number=self._ping_number,
            start_mm=ping.start_mm,
            length_mm=ping.length_mm,
            timestamp_ms=int(round(ping.t_sim * 1000.0)),
            ping_hz=OMNISCAN_FREQ_HZ,
            gain_index=acq.gain_index,
            num_results=acq.num_results,
            sos_dmps=int(round(SPEED_OF_SOUND_MPS * 10.0)),
            channel_number=self._channel,
            pulse_duration_sec=acq.pulse_duration_s(cfg.max_ping_rate_hz),
            analog_gain=ANALOG_GAIN_TABLE.get(acq.gain_index, 74.55),
            max_pwr_db=float(max_db),
            min_pwr_db=float(min_db),
            transducer_heading_deg=float(transducer_heading),
            vehicle_heading_deg=float(vehicle_heading),
            pwr_results=counts,
            raw_frame=b"",
        )
        enc.raw_frame = _frame(enc)
        return enc


def _frame(e: EncodedPing) -> bytes:
    payload = _PAYLOAD_FIXED.pack(
        e.ping_number, e.start_mm, e.length_mm, e.timestamp_ms, e.ping_hz,
        e.gain_index, e.num_results, e.sos_dmps, e.channel_number, 0,
        e.pulse_duration_sec, e.analog_gain, e.max_pwr_db, e.min_pwr_db,
        e.transducer_heading_deg, e.vehicle_heading_deg,
    ) + e.pwr_results.astype("<u2").tobytes()
    header = _HEADER.pack(ord("B"), ord("R"), len(payload),
                          OS_MONO_PROFILE_ID, 0, 0)
    body = header + payload
    checksum = sum(body) & 0xFFFF
    return body + struct.pack("<H", checksum)


def parse_frame(raw: bytes) -> dict:
    """Inverse of :func:`_frame` -- used by round-trip tests to guarantee the
    simulator's raw stream is byte-valid Ping Protocol."""
    b, r, plen, mid, src, dst = _HEADER.unpack_from(raw, 0)
    if (b, r) != (ord("B"), ord("R")):
        raise ValueError("bad start bytes")
    if mid != OS_MONO_PROFILE_ID:
        raise ValueError(f"unexpected message id {mid}")
    body = raw[:_HEADER.size + plen]
    (checksum,) = struct.unpack_from("<H", raw, _HEADER.size + plen)
    if checksum != (sum(body) & 0xFFFF):
        raise ValueError("checksum mismatch")
    f = _PAYLOAD_FIXED.unpack_from(raw, _HEADER.size)
    n = f[6]
    pwr = np.frombuffer(raw, dtype="<u2",
                        count=n, offset=_HEADER.size + _PAYLOAD_FIXED.size)
    return {
        "ping_number": f[0], "start_mm": f[1], "length_mm": f[2],
        "timestamp_ms": f[3], "ping_hz": f[4], "gain_index": f[5],
        "num_results": f[6], "sos_dmps": f[7], "channel_number": f[8],
        "pulse_duration_sec": f[10], "analog_gain": f[11],
        "max_pwr_db": f[12], "min_pwr_db": f[13],
        "transducer_heading_deg": f[14], "vehicle_heading_deg": f[15],
        "pwr_results": pwr.copy(),
    }
