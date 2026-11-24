"""The receiver: range envelope, time-to-bin mapping, coherent accumulation.

The response of one point scatterer in the bin stream is the transmit pulse
(a CW tone burst of duration ``tau``, amplitude 1 -- so a longer pulse puts
more energy on the seabed, as the device's does) convolved with the receive
filter (``filter_pulse_ratio`` x the pulse when set, the profiles' 1.4; else
``filter_duration_percent`` of the round-trip window; unit DC gain)
and with the bin's own integration (one bin width, averaging). Three
rectangles convolved: a bell, not the boxcar of the geometric renderer
(pass 4 measured its lag-2/3 correlation too high, hand-off §4). Centred, so a
target is imaged at its true delay. The waveform is not documented by the
maker; a CW burst is assumed (``raycast_design.md`` §12, needed measurement).

Times are true travel times (true sound speed); the device maps time to bins
with its *assumed* speed (``SPEED_OF_SOUND_MPS``, reported as ``sos_dmps``).

Complex baseband: a contributor with amplitude ``A`` and delay ``tau_n``
adds ``A exp(-i omega tau_n) s(t_b - tau_n)`` to the field of every bin ``b``
under its envelope; the mean adds ``E|A|^2 |s|^2`` for a random-phase
(diffuse) contributor.

ROS-free, Qt-free, numpy only.
"""

from __future__ import annotations

import numpy as np

from ..config import SPEED_OF_SOUND_MPS, AcquisitionParams


class Envelope:
    """The point-target range envelope ``s(t)``, centred on t = 0."""

    def __init__(self, pulse_s: float, filter_s: float, bin_s: float,
                 dt: float = 2e-8) -> None:
        self.pulse_s, self.filter_s, self.bin_s = pulse_s, filter_s, bin_s
        def rect(width: float, height: float) -> np.ndarray:
            n = max(int(round(width / dt)), 1)
            return np.full(n, height)
        s = rect(pulse_s, 1.0)
        for w in (filter_s, bin_s):
            if w > 0.0:
                s = np.convolve(s, rect(w, 1.0 / max(int(round(w / dt)), 1)))
        self._s = s
        self._t = (np.arange(s.size) - 0.5 * (s.size - 1)) * dt
        self.half_support_s = 0.5 * s.size * dt

    def __call__(self, t: np.ndarray) -> np.ndarray:
        return np.interp(t, self._t, self._s, left=0.0, right=0.0)


class BinGrid:
    """The device's range bins and the accumulators of one ping-side."""

    def __init__(self, acq: AcquisitionParams, envelope: Envelope,
                 omega: float, device_c: float = SPEED_OF_SOUND_MPS) -> None:
        self.n = acq.num_results
        self.start_m = acq.range_start_mm / 1000.0
        self.bin_m = acq.bin_size_m
        self.device_c = float(device_c)
        self.omega = float(omega)
        self.env = envelope
        # Bin-centre sample times (device convention: r = c_dev t / 2).
        r_b = self.start_m + (np.arange(self.n) + 0.5) * self.bin_m
        self.t_b = 2.0 * r_b / self.device_c
        self.field = np.zeros(self.n, dtype=np.complex128)
        self.mean_diffuse = np.zeros(self.n)
        self.coherent = np.zeros(self.n, dtype=np.complex128)
        dt_bin = 2.0 * self.bin_m / self.device_c
        self._k = int(np.ceil(envelope.half_support_s / dt_bin)) + 1

    def add(self, amp: np.ndarray, tau: np.ndarray, diffuse: bool,
            power: np.ndarray | None = None) -> None:
        """Deposit contributors: complex amplitudes ``amp`` (without the
        carrier phase) at two-way delays ``tau`` [s]. For diffuse
        contributors ``power`` is the EXPECTED ``E|amp|^2`` (without the
        random scatterer factor), so the mean is not itself noisy;
        default ``|amp|^2``."""
        amp = np.asarray(amp, dtype=np.complex128)
        tau = np.asarray(tau, dtype=np.float64)
        if amp.size == 0:
            return
        carrier = amp * np.exp(-1j * np.fmod(self.omega * tau, 2.0 * np.pi))
        centre = np.floor((0.5 * self.device_c * tau - self.start_m)
                          / self.bin_m).astype(np.int64)
        p = np.abs(amp) ** 2 if power is None else np.asarray(power, float)
        for j in range(-self._k, self._k + 1):
            b = centre + j
            ok = (b >= 0) & (b < self.n)
            if not ok.any():
                continue
            bb = b[ok]
            w = self.env(self.t_b[bb] - tau[ok])
            nz = w != 0.0
            if not nz.any():
                continue
            bb, w = bb[nz], w[nz]
            c = carrier[ok][nz] * w
            self.field += (np.bincount(bb, weights=c.real, minlength=self.n)
                           + 1j * np.bincount(bb, weights=c.imag, minlength=self.n))
            if diffuse:
                self.mean_diffuse += np.bincount(
                    bb, weights=p[ok][nz] * w * w, minlength=self.n)
            else:
                self.coherent += (
                    np.bincount(bb, weights=c.real, minlength=self.n)
                    + 1j * np.bincount(bb, weights=c.imag, minlength=self.n))

    def taps(self, tau: np.ndarray):
        """The envelope taps of contributors at delays ``tau``: (contributor
        index, bin, envelope weight w, carrier exp(-i omega tau)) -- what
        :meth:`add` deposits, as sparse triplets (the fit basis)."""
        tau = np.asarray(tau, dtype=np.float64)
        carrier = np.exp(-1j * np.fmod(self.omega * tau, 2.0 * np.pi))
        centre = np.floor((0.5 * self.device_c * tau - self.start_m)
                          / self.bin_m).astype(np.int64)
        ii, bb, ww = [], [], []
        idx = np.arange(tau.size)
        for j in range(-self._k, self._k + 1):
            b = centre + j
            ok = (b >= 0) & (b < self.n)
            if not ok.any():
                continue
            w = self.env(self.t_b[b[ok]] - tau[ok])
            nz = w != 0.0
            ii.append(idx[ok][nz])
            bb.append(b[ok][nz])
            ww.append(w[nz])
        if not ii:
            z = np.zeros(0)
            return z.astype(np.int64), z.astype(np.int64), z, carrier
        return np.concatenate(ii), np.concatenate(bb), np.concatenate(ww), carrier

    def add_group(self, det: np.ndarray, f: np.ndarray, tau: np.ndarray,
                  group: np.ndarray, chunk: int = 400_000) -> None:
        """Deposit contributors that come in coherent groups: every path
        pair of one scatterer shares its random factor ``f`` (Brown17), so
        the paths interfere with each other (Lloyd's mirror, the image comb)
        while different scatterers stay incoherent. Field: sum of
        ``det * f``; mean: sum over groups of ``|sum over the group of det|^2``
        per bin (``E|f|^2 = 1``)."""
        det = np.asarray(det, dtype=np.complex128)
        if det.size == 0:
            return
        f = np.asarray(f, dtype=np.complex128)
        tau = np.asarray(tau, dtype=np.float64)
        group = np.asarray(group, dtype=np.int64)
        order = np.argsort(group, kind="stable")
        det, f, tau, group = det[order], f[order], tau[order], group[order]
        # chunk on group boundaries so a group is never split
        starts = np.flatnonzero(np.r_[True, group[1:] != group[:-1]])
        cuts = list(starts[::max(1, len(starts) * chunk // max(det.size, 1))]) + [det.size]
        cuts = sorted(set(int(c) for c in cuts))
        for a, b in zip(cuts[:-1], cuts[1:]):
            d, ff, t, g = det[a:b], f[a:b], tau[a:b], group[a:b]
            carrier = d * np.exp(-1j * np.fmod(self.omega * t, 2.0 * np.pi))
            centre = np.floor((0.5 * self.device_c * t - self.start_m)
                              / self.bin_m).astype(np.int64)
            keys, vals = [], []
            for j in range(-self._k, self._k + 1):
                bb = centre + j
                ok = (bb >= 0) & (bb < self.n)
                if not ok.any():
                    continue
                w = self.env(self.t_b[bb[ok]] - t[ok])
                nz = w != 0.0
                if not nz.any():
                    continue
                c = carrier[ok][nz] * w[nz]
                bsel = bb[ok][nz]
                fc = c * ff[ok][nz]
                self.field += (np.bincount(bsel, weights=fc.real, minlength=self.n)
                               + 1j * np.bincount(bsel, weights=fc.imag,
                                                  minlength=self.n))
                keys.append(g[ok][nz] * self.n + bsel)
                vals.append(c)
            if not keys:
                continue
            k_all = np.concatenate(keys)
            v_all = np.concatenate(vals)
            uk, inv = np.unique(k_all, return_inverse=True)
            sre = np.bincount(inv, weights=v_all.real)
            sim = np.bincount(inv, weights=v_all.imag)
            self.mean_diffuse += np.bincount(uk % self.n,
                                             weights=sre * sre + sim * sim,
                                             minlength=self.n)

    @property
    def mean(self) -> np.ndarray:
        """``E|field|^2``: diffuse mean + the coherent part's own power."""
        return self.mean_diffuse + np.abs(self.coherent) ** 2

    @property
    def intensity(self) -> np.ndarray:
        """The realisation ``|field|^2``."""
        return np.abs(self.field) ** 2


def envelope_for(acq: AcquisitionParams, filter_duration_percent: float,
                 bin_integration: bool = True,
                 max_ping_rate_hz: float = 0.0,
                 filter_pulse_ratio: float = 0.0) -> Envelope:
    """The envelope of an acquisition: the device's pulse law
    (``AcquisitionParams.pulse_duration_s``), the receive filter -- a
    multiple of the pulse (measured, ``ReceiverConfig.filter_pulse_ratio``)
    or, when that is 0, a share of the round-trip window -- and one bin of
    integration."""
    two_way = 2.0 * acq.range_max_m / SPEED_OF_SOUND_MPS
    bin_s = 2.0 * acq.bin_size_m / SPEED_OF_SOUND_MPS
    tau = acq.pulse_duration_s(max_ping_rate_hz)
    filt = (float(filter_pulse_ratio) * tau if filter_pulse_ratio > 0.0
            else float(filter_duration_percent) * two_way)
    return Envelope(tau, filt, bin_s if bin_integration else 0.0)
