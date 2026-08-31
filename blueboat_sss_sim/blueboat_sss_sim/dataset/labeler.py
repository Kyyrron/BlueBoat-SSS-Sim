"""Automatic YOLO labeling from renderer ground truth.

The renderer emits, per ping, the exact slant-range position, extent and
shadow length of every insonified object
(:class:`~blueboat_sss_sim.core.types.GroundTruthContact`). The labeler
aggregates these per-object across the pings of a waterfall tile into one
bounding box.

Box convention (configurable): ``highlight`` boxes cover the bright echo
only; ``highlight_shadow`` extends the box down-range over the acoustic
shadow, which many SSS detection works include because the shadow carries
most of the shape information.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

import numpy as np

from ..core.types import GroundTruthContact
from .waterfall import PingRow


#: Class name carried by every multipath ghost under ``ghost_mode="class"``.
GHOST_CLASS = "ghost"


@dataclass
class LabelConfig:
    box_mode: str = "highlight_shadow"       # "highlight" | "highlight_shadow"
    min_rows: int = 2                        # discard contacts thinner than this
    pad_bins: float = 3.0
    pad_rows: float = 2.0
    class_names: list[str] | None = None     # fixed class order; None = discover
    ghost_mode: str = "class"                # what a multipath ghost is
                                             # labelled as:
                                             #   "class"    -> one extra
                                             #     "ghost" class (default):
                                             #     the detector is taught to
                                             #     reject it, which is the
                                             #     whole value of a ghost as
                                             #     a hard negative
                                             #   "skip"     -> no box at all
                                             #   "per_type" -> "ghost_<type>"
                                             #   "as_real"  -> the mirrored
                                             #     object's own class. Teaches
                                             #     that a ghost IS the object;
                                             #     present for completeness,
                                             #     not recommended

    def ghost_class_for(self, object_type: str) -> str | None:
        """Class name a ghost of ``object_type`` carries, or None to drop it."""
        if self.ghost_mode == "skip":
            return None
        if self.ghost_mode == "per_type":
            return f"ghost_{object_type}"
        if self.ghost_mode == "as_real":
            return object_type
        if self.ghost_mode == "class":
            return GHOST_CLASS
        raise ValueError(f"unknown ghost_mode {self.ghost_mode!r}")


@dataclass
class YoloBox:
    class_id: int
    x_center: float                          # all normalised 0..1
    y_center: float
    width: float
    height: float
    object_id: int
    object_type: str

    def to_line(self) -> str:
        return (f"{self.class_id} {self.x_center:.6f} {self.y_center:.6f} "
                f"{self.width:.6f} {self.height:.6f}")


class TileLabeler:
    """Turns one tile's rows into YOLO boxes."""

    def __init__(self, num_results: int, bin_size_m: float,
                 cfg: LabelConfig | None = None) -> None:
        self._n = num_results
        self._bin_m = bin_size_m
        self._cfg = cfg or LabelConfig()
        self._classes: list[str] = list(self._cfg.class_names or [])

    @property
    def class_names(self) -> list[str]:
        return list(self._classes)

    def _class_id(self, name: str) -> int:
        if name not in self._classes:
            if self._cfg.class_names is not None:
                raise KeyError(f"object type '{name}' not in fixed class list")
            self._classes.append(name)
        return self._classes.index(name)

    def label_tile(self, rows: list[PingRow]) -> list[YoloBox]:
        cfg = self._cfg
        h = len(rows)
        # Keyed on (object_id, via), never object_id alone: an object and its
        # own multipath ghost are two features at two ranges, and merging them
        # would draw one box spanning the empty water between.
        per_object: dict[tuple[int, str],
                         list[tuple[int, GroundTruthContact]]] = defaultdict(list)
        for r_i, row in enumerate(rows):
            for c in row.contacts:
                if c.visible:
                    per_object[c.group_key].append((r_i, c))

        boxes: list[YoloBox] = []
        for _, hits in per_object.items():
            if len(hits) < cfg.min_rows:
                continue
            c0 = hits[0][1]
            class_name = (cfg.ghost_class_for(c0.object_type) if c0.ghost
                          else c0.object_type)
            if class_name is None:
                continue
            rows_hit = np.array([r for r, _ in hits], dtype=float)
            bins = np.array([c.slant_range_m / self._bin_m for _, c in hits])
            ext = np.array([c.extent_bins for _, c in hits])
            shad = np.array([c.shadow_bins for _, c in hits])

            x0 = float(np.min(bins - ext / 2) - cfg.pad_bins)
            x1 = float(np.max(bins + ext / 2) + cfg.pad_bins)
            if cfg.box_mode == "highlight_shadow":
                x1 = float(np.max(bins + ext / 2 + shad) + cfg.pad_bins)
            y0 = float(rows_hit.min() - cfg.pad_rows)
            y1 = float(rows_hit.max() + cfg.pad_rows)

            x0, x1 = np.clip([x0, x1], 0, self._n - 1)
            y0, y1 = np.clip([y0, y1], 0, h - 1)
            if x1 - x0 < 1 or y1 - y0 < 1:
                continue
            boxes.append(YoloBox(
                class_id=self._class_id(class_name),
                x_center=(x0 + x1) / 2 / self._n,
                y_center=(y0 + y1) / 2 / h,
                width=(x1 - x0) / self._n,
                height=(y1 - y0) / h,
                object_id=c0.object_id,
                object_type=class_name))
        return boxes
