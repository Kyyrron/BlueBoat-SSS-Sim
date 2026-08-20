"""Waterfall image assembly from ping streams.

A waterfall is the standard SSS raster: one image row per ping, columns =
slant-range bins, port mirrored so range increases away from the centre.
The builder accumulates pings per side and cuts fixed-height tiles suited
to detector training (YOLO expects reasonably square images).

Display mapping: log compression + per-tile percentile normalisation, the
same treatment typical SSS viewers apply, so synthetic tiles have the
grey-level statistics detectors will meet on real data.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class WaterfallTileConfig:
    tile_pings: int = 512               # rows per exported tile
    overlap_pings: int = 64             # row overlap between tiles
    log_compress: bool = True
    p_low: float = 1.0                  # percentile black point
    p_high: float = 99.5                # percentile white point


@dataclass
class PingRow:
    """One buffered ping row plus the metadata labeling needs."""

    power: np.ndarray                   # uint16[num_results]
    ping_index: int                     # row index in the side's stream
    contacts: list = field(default_factory=list)  # GroundTruthContact-likes


class WaterfallBuilder:
    """Per-side ping accumulator producing normalised uint8 tiles."""

    def __init__(self, num_results: int, cfg: WaterfallTileConfig | None = None) -> None:
        self._n = num_results
        self._cfg = cfg or WaterfallTileConfig()
        self._rows: list[PingRow] = []
        self._next_index = 0
        self._emitted_upto = 0

    def add_ping(self, power: np.ndarray, contacts: list | None = None) -> None:
        if len(power) != self._n:
            raise ValueError(f"expected {self._n} bins, got {len(power)}")
        self._rows.append(PingRow(np.asarray(power, dtype=np.uint16),
                                  self._next_index, list(contacts or [])))
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
        lo, hi = np.percentile(img, [cfg.p_low, cfg.p_high])
        if hi <= lo:
            hi = lo + 1.0
        img = np.clip((img - lo) / (hi - lo), 0.0, 1.0)
        return (img * 255.0).astype(np.uint8)
