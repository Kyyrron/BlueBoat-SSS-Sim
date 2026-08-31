"""Waterfall image assembly from ping streams.

A waterfall is the standard SSS raster: one image row per ping, columns =
slant-range bins, port mirrored so range increases away from the centre.
The builder accumulates pings per side and cuts fixed-height tiles suited
to detector training (YOLO expects reasonably square images).

Display mapping: per-tile percentile normalisation of the **dB** samples, the
same treatment typical SSS viewers apply, so synthetic tiles have the
grey-level statistics detectors will meet on real data.

The samples are also range-equalised first: the profile stream the device
reports is pre-TVG, so the raw across-track gradient is the full two-way
spreading loss -- ~24 dB across the swath at the 15 m / 4 m-altitude default --
and a single percentile stretch across that crushes the outer third of the
swath to black, taking the objects out there with it. Only the smooth
``log10(range)`` trend is removed (see :func:`_range_trend`), which is the
display-side TVG a viewer applies. It does not touch the wire stream, which
stays exactly what the device would have sent.

``pwr_results`` arrives already on a dB axis -- the device normalises every
ping onto ``[min_pwr_db, max_pwr_db]`` and downstream inverts that with
``db = min + raw/65535 * (max - min)``. The compression the log used to supply
is therefore already in the samples, and applying ``log1p`` on top of them
compresses twice. ``log_compress`` stays as an escape hatch for a caller that
hands over genuinely linear power, and defaults off.

Because that axis is **per ping**, two neighbouring pings map the same physical
dB to different counts. Callers pass ``min_pwr_db`` / ``max_pwr_db`` alongside
each row so the builder stacks *absolute* dB and the rows are commensurable --
the same conversion the GCS applies before it draws a waterfall. Omitting them
stacks the raw array and leaves that per-row gain jitter in the image.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class WaterfallTileConfig:
    tile_pings: int = 512               # rows per exported tile
    overlap_pings: int = 64             # row overlap between tiles
    log_compress: bool = False          # samples are already dB; see module doc
    range_equalise: bool = True         # remove the across-track range trend
    p_low: float = 1.0                  # percentile black point
    p_high: float = 99.5                # percentile white point


@dataclass
class PingRow:
    """One buffered ping row plus the metadata labeling needs."""

    power: np.ndarray                   # absolute dB per bin when the caller
                                        # supplied the endpoints, else the raw
                                        # per-ping normalised uint16
    ping_index: int                     # row index in the side's stream
    contacts: list = field(default_factory=list)  # GroundTruthContact-likes


def _range_trend(img: np.ndarray) -> np.ndarray:
    """The tile's smooth ``a + b*log10(r)`` across-track trend, as a row.

    Deliberately a two-parameter fit rather than a per-column median. The
    gradient that has to go is the two-way spreading loss, which is exactly
    ``log10`` in range; subtracting the column medians instead would also
    remove everything else that varies with range -- the water column would
    come out as bright as the seabed, and the bottom return would stop being
    the brightest thing in the ping. Bin index stands in for range: they are
    proportional, so the difference lands in ``a`` and cancels.
    """
    n = img.shape[1]
    if n < 4:
        return np.zeros((1, n))
    x = np.log10(np.arange(n) + 1.0)
    y = np.median(img, axis=0)
    b, a = np.polyfit(x, y, 1)
    return (a + b * x)[None, :]


class WaterfallBuilder:
    """Per-side ping accumulator producing normalised uint8 tiles."""

    def __init__(self, num_results: int, cfg: WaterfallTileConfig | None = None) -> None:
        self._n = num_results
        self._cfg = cfg or WaterfallTileConfig()
        self._rows: list[PingRow] = []
        self._next_index = 0
        self._emitted_upto = 0

    def add_ping(self, power: np.ndarray, contacts: list | None = None,
                 min_pwr_db: float | None = None,
                 max_pwr_db: float | None = None) -> None:
        """Buffer one ping row.

        Pass the profile's ``min_pwr_db`` / ``max_pwr_db`` to have the row
        converted to absolute dB first (see the module docstring); without
        them the raw per-ping normalised array is stacked as-is.
        """
        if len(power) != self._n:
            raise ValueError(f"expected {self._n} bins, got {len(power)}")
        row = np.asarray(power, dtype=np.float64)
        if min_pwr_db is not None and max_pwr_db is not None:
            row = min_pwr_db + (row / 65535.0) * (max_pwr_db - min_pwr_db)
        self._rows.append(PingRow(row, self._next_index, list(contacts or [])))
        self._next_index += 1

    # ------------------------------------------------------------------
    def ready_tiles(self, flush: bool = False) -> list[tuple[np.ndarray, list[PingRow]]]:
        """Return completed (image, rows) tiles; call with ``flush=True`` at
        end of mission to emit the trailing partial tile."""
        cfg = self._cfg
        tiles: list[tuple[np.ndarray, list[PingRow]]] = []
        step = cfg.tile_pings - cfg.overlap_pings
        while len(self._rows) - self._emitted_upto >= cfg.tile_pings:
            rows = self._rows[self._emitted_upto:self._emitted_upto + cfg.tile_pings]
            tiles.append((self._render(rows), rows))
            self._emitted_upto += step
        if flush and len(self._rows) - self._emitted_upto >= max(32, cfg.overlap_pings):
            rows = self._rows[self._emitted_upto:]
            tiles.append((self._render(rows), rows))
            self._emitted_upto = len(self._rows)
        return tiles

    def _render(self, rows: list[PingRow]) -> np.ndarray:
        cfg = self._cfg
        img = np.stack([r.power for r in rows]).astype(np.float64)
        if cfg.log_compress:
            img = np.log1p(img)
        if cfg.range_equalise:
            img = img - _range_trend(img)
        lo, hi = np.percentile(img, [cfg.p_low, cfg.p_high])
        if hi <= lo:
            hi = lo + 1.0
        img = np.clip((img - lo) / (hi - lo), 0.0, 1.0)
        return (img * 255.0).astype(np.uint8)
