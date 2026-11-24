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

    The ladder is the device's own reported ``analog_gain``, an *amplitude*
    gain: the reported-dB axis moves by ``20 log10`` of its ratio. MEASURED
    on the corpus receiver floor (the far decile of the 72-126 m files,
    where nothing but noise is left): the floor steps +5.6 / +4.6 / +5.7 dB
    across 4->5->6->7, against 20 log10 of the table = +5.65 / +4.58 /
    +5.67. (A 10 log10 step, +2.8 dB, was the earlier assumption; it does
    not reproduce the floor shift.)
    """
    return 20.0 * np.log10(ANALOG_GAIN_TABLE[gain_index]
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


def ceiling_at(pulse_s: float, cfg: SonarModelConfig) -> float:
    """The device's reported-level ceiling [dB] for a transmit pulse:
    ``receiver_ceiling_db + 10 k log10(pulse / pulse_level_reference_s)``.

    MEASURED (2026-10-23): ``max_pwr_db`` piles up at one value per
    acquisition -- 67.9 dB on the beach at gains 4-7 -- and over 26 piled
    acquisition groups of the corpus (both sites, both recorders, 600 and
    1200 bins) that value follows 67.81 + 0.91*10log10(tau / 66.5 us),
    residual sd 0.18 dB. It does not depend on the gain index."""
    ratio = max(float(pulse_s), 1e-9) / float(cfg.pulse_level_reference_s)
    return float(cfg.receiver_ceiling_db
                 + 10.0 * float(cfg.receiver_ceiling_pulse_exponent)
                 * np.log10(ratio))


#: Slope left above the ceiling. A pure asymptote squeezes every overloaded
#: bin to within a fraction of a count of full scale, and ties at 65535 break
#: the device's one-full-scale-bin invariant; at 0.02 two bins 1 dB apart stay
#: ~19 counts apart (70 dB span) and a 10 dB overload reads +0.2 dB -- the
#: corpus has gain-4 pings up to 69.4 dB against the 67.9 dB pile-up.
CEILING_RESIDUAL_SLOPE = 0.02


def soft_ceiling_db(db: np.ndarray, ceiling_db: float,
                    knee_db: float) -> np.ndarray:
    """A C1 soft ceiling on reported dB: identity up to ``ceiling - knee``;
    above it, with ``u = x - (ceiling - knee)``,
    ``(ceiling - knee) + (1 - s) * knee * (1 - exp(-u / knee)) + s * u``
    (``s`` = :data:`CEILING_RESIDUAL_SLOPE`): strictly increasing, so the
    per-ping maximum stays a single bin, and every bin below the knee keeps
    its value bit for bit. Near the top it compresses, as the corpus's
    piled-up pings show (more bins within 1 dB of the maximum)."""
    k = max(float(knee_db), 1e-6)
    s = CEILING_RESIDUAL_SLOPE
    x = np.asarray(db, dtype=np.float64)
    start = float(ceiling_db) - k
    over = x > start
    if not over.any():
        return x
    out = x.copy()
    u = x[over] - start
    out[over] = start + (1.0 - s) * k * (1.0 - np.exp(-u / k)) + s * u
    return out


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
        # The two units number their pings independently (corpus: offsets of
        # 0, -1 and +60 measured), so the fault knob offsets one side only.
        self._ping_number = (int(model.fault_counter_offset)
                             if side is Side.STARBOARD else 0)
        self._frame_id = f"sss_{side.value}_link"

    def reconfigure(self, acquisition: AcquisitionParams,
                    model: SonarModelConfig) -> None:
        """Apply new acquisition/model settings without touching the counter.

        The real device numbers pings from power-up; a ping-enable cycle (or
        a range change) never restarts its counter. The simulator's node
        keeps one encoder per side for its whole life and reconfigures it
        here on every enable.
        """
        self._acq = acquisition
        self._cfg = model

    @property
    def ping_number(self) -> int:
        return self._ping_number

    @property
    def _ceiling_db(self) -> float:
        """The reported-level ceiling at this acquisition's pulse (receiver
        ceiling on, see :func:`ceiling_at`)."""
        return ceiling_at(self._acq.pulse_duration_s(self._cfg.max_ping_rate_hz),
                          self._cfg)

    # ------------------------------------------------------------------
    def encode(self, ping: Ping, gain_multiplier: float = 1.0) -> EncodedPing:
        acq, cfg = self._acq, self._cfg
        self._ping_number += 1
        if (cfg.fault_counter_reset_pings > 0
                and self._ping_number % cfg.fault_counter_reset_pings == 0):
            # A device power cycle under a running processor (see the
            # fault-injection block in SonarModelConfig).
            self._ping_number = 1 + (int(cfg.fault_counter_offset)
                                     if self._side is Side.STARBOARD else 0)

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
        if cfg.fault_gain_walk_pings > 0:
            # The device's own auto-gain walks the ladder inside a recording;
            # a constant-gain simulation never makes the processor compensate.
            step = ((self._ping_number - 1) // cfg.fault_gain_walk_pings) % 4
            gain_index = 4 + int(step)
        gain_db = gain_step_db(gain_index)

        # Receive-chain gain is a level shift, not an image change -- it moves
        # both dB endpoints together and cancels out of the normalised counts,
        # exactly as it does on the device.
        power = np.maximum(ping.power * gain_multiplier, 1e-30)
        db = 10.0 * np.log10(power) + cfg.calibration_db_offset + gain_db
        if cfg.receiver_ceiling:
            db = soft_ceiling_db(db, self._ceiling_db, cfg.receiver_ceiling_knee_db)

        max_db = float(db.max())
        min_db = max(float(db.min()), max_db - cfg.max_span_db)
        span = max(max_db - min_db, 1e-9)
        counts = np.rint(
            np.clip((db - min_db) / span, 0.0, 1.0) * 65535.0
        ).astype(np.uint16)
        # Exactly ONE full-scale bin, as on the device (68948/68948 field
        # pings, piled-up ones included): a bin that rounds up to 65535
        # beside the true maximum is reported one count below it. Ties were
        # vanishingly rare until the receiver ceiling's soft knee compressed
        # the top (2026-10-30: 0.1 % of Gazebo profiles carried two); a ping
        # without a tie is unchanged.
        full = counts == 65535
        if np.count_nonzero(full) > 1:
            counts[full] = 65534
            counts[int(np.argmax(db))] = 65535
        # Both endpoints travel as float32 (the frame's `f` fields, the
        # profile's float32). Rounded independently, a clamped pair could
        # read up to ~4e-6 dB WIDER than max_span_db -- which the device's
        # never does (0 pings in the field logs; 26 % of clamped simulated
        # pings, 2026-10-21) and which `normalisation_invariants` reports.
        # Nudge the wire minimum up by float32 ulps until it holds; the
        # counts and every unclamped ping are untouched.
        min_wire = min_db
        hi32, m32 = float(np.float32(max_db)), np.float32(min_db)
        if hi32 - float(m32) > cfg.max_span_db:
            for _ in range(64):
                m32 = np.nextafter(m32, np.float32(np.inf))
                if hi32 - float(m32) <= cfg.max_span_db:
                    break
            min_wire = float(m32)

        vehicle_heading = enu_yaw_to_compass_deg(ping.pose.yaw)
        # `transducer_heading_deg` is the transducer's MOUNTING angle
        # relative to the bow, as the real Omniscan 450 reports it: a
        # constant -90 on every port packet and +90 on every starboard
        # packet across 100 % of the field corpus (project_review.md
        # §3.6), never an absolute bearing. Until 2026-09-15 this encoded
        # (vehicle ∓ 90) mod 360, which SonarView's sign-of-heading side
        # fallback mirrored for every heading in 90..270 (CM-2 violation);
        # the absolute heading rides in `vehicle_heading_deg` alone.
        transducer_heading = -self._side.sign * 90.0

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
            channel_number=255 if cfg.fault_channel_255 else self._channel,
            pulse_duration_sec=acq.pulse_duration_s(cfg.max_ping_rate_hz),
            analog_gain=ANALOG_GAIN_TABLE[gain_index],
            max_pwr_db=float(max_db),
            min_pwr_db=float(min_wire),
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
