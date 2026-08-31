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
    u8 channel_number (0 = port, 1 = starboard) | u8 reserved |
    f32 pulse_duration_sec |
    f32 analog_gain | f32 max_pwr_db | f32 min_pwr_db |
    f32 transducer_heading_deg | f32 vehicle_heading_deg |
    u16 pwr_results[num_results] | u16 checksum(sum of all prior bytes)

``pwr_results`` is **not** absolute counts. The device rescales each ping
onto its own dB axis, so the array spans 0..65535 every ping and the
physical scale rides in ``min_pwr_db`` / ``max_pwr_db``; downstream inverts
it with ``db = min + raw/65535 * (max - min)``. See
:meth:`PingEncoder.encode` and docs/sonar_model.md §6.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

from ..core.geometry import enu_yaw_to_compass_deg
from ..core.types import Ping, Side
from .config import (CALIBRATION_GAIN_INDEX, MAX_GAIN_INDEX, OMNISCAN_FREQ_HZ,
                     SPEED_OF_SOUND_MPS, AcquisitionParams, SonarModelConfig)

OS_MONO_PROFILE_ID = 2198
_HEADER = struct.Struct("<BBHHBB")
_PAYLOAD_FIXED = struct.Struct("<IIIIIHHHBBffffff")
# Indexed by AcquisitionParams.effective_gain_index (0..MAX_GAIN_INDEX), never
# by the raw ``gain_index`` parameter: that one may carry the -1 auto sentinel.
#
# Indices 4-7 are MEASURED -- every profile in the field corpus reports exactly
# one of these four values for its index, with no spread. Indices 0-3 are the
# original estimates: the device's auto-gain never selected them in any
# recording, so nothing constrains them.
ANALOG_GAIN_TABLE = {0: 20.0, 1: 33.0, 2: 46.0, 3: 60.0,
                     4: 74.55, 5: 142.8, 6: 242.025, 7: 464.625}
assert set(ANALOG_GAIN_TABLE) == set(range(MAX_GAIN_INDEX + 1))


def gain_step_db(gain_index: int) -> float:
    """Receive gain at ``gain_index``, in dB relative to the calibration index.

    The ladder is the device's own reported ``analog_gain``, so this replaces
    the flat per-index dB step the model used to assume: the measured ratios
    are +2.82, +2.29 and +2.83 dB across 4->5->6->7, not a constant.
    """
    return 10.0 * np.log10(ANALOG_GAIN_TABLE[gain_index]
                           / ANALOG_GAIN_TABLE[CALIBRATION_GAIN_INDEX])


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
    pwr_results: np.ndarray          # uint16[num_results], per-ping normalised
                                     # onto [min_pwr_db, max_pwr_db]; not counts
    raw_frame: bytes                 # framed Ping-Protocol bytes


class PingEncoder:
    """Stateful per-side encoder (ping counter + device uptime clock)."""

    def __init__(self, side: Side, acquisition: AcquisitionParams,
                 model: SonarModelConfig) -> None:
        self._side = side
        self._acq = acquisition
        self._cfg = model
        # Side identity travels in the packet, never in the topic or the
        # ``src`` device tag, so it is derived from the side rather than
        # passed in -- a caller cannot get it wrong or leave it defaulted.
        self._channel = side.channel
        self._ping_number = 0
        self._frame_id = f"sss_{side.value}_link"

    # ------------------------------------------------------------------
    def encode(self, ping: Ping, gain_multiplier: float = 1.0) -> EncodedPing:
        acq, cfg = self._acq, self._cfg
        self._ping_number += 1

        # Linear power -> the device's wire representation. `pwr_results` is
        # NOT absolute counts: the Omniscan rescales every ping to full scale
        # on a dB axis and reports the axis endpoints in min/max_pwr_db, which
        # is how `sss_helper.scale_to_db` inverts it downstream:
        #
        #     db = min_pwr_db + (raw / 65535) * (max_pwr_db - min_pwr_db)
        #
        # Measured on 68948/68948 field pings: exactly one bin at 65535, the
        # minimum exactly 0, and the span clamped at max_span_db. Emitting
        # absolute counts instead would put a linear-power vector on the wire
        # under a dB label, and every downstream consumer would mis-invert it.
        #
        # The *resolved* gain index is used: a raw -1 (device auto) has no
        # analog-gain figure and cannot be packed into the u16 frame field.
        gain_index = acq.effective_gain_index
        gain_db = gain_step_db(gain_index)

        # Receive-chain gain is a level shift, not an image change -- it moves
        # both dB endpoints together and cancels out of the normalised counts,
        # exactly as it does on the device.
        power = np.maximum(ping.power * gain_multiplier, 1e-30)
        db = 10.0 * np.log10(power) + cfg.calibration_db_offset + gain_db

        max_db = float(db.max())
        min_db = max(float(db.min()), max_db - cfg.max_span_db)
        span = max(max_db - min_db, 1e-9)
        counts = np.rint(
            np.clip((db - min_db) / span, 0.0, 1.0) * 65535.0
        ).astype(np.uint16)

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
            gain_index=gain_index,
            num_results=acq.num_results,
            sos_dmps=int(round(SPEED_OF_SOUND_MPS * 10.0)),
            channel_number=self._channel,
            pulse_duration_sec=acq.pulse_duration_s(cfg.max_ping_rate_hz),
            analog_gain=ANALOG_GAIN_TABLE[gain_index],
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
