"""YOLO dataset writer.

Standard Ultralytics layout::

    dataset_root/
        images/train/  images/val/
        labels/train/  labels/val/
        dataset.yaml

Tiles are assigned to val with probability ``val_fraction`` (deterministic
per tile via a hash, so re-exports are stable).
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from .labeler import YoloBox
from .waterfall import WaterfallTileConfig


@dataclass
class ExportConfig:
    val_fraction: float = 0.15
    write_empty_labels: bool = True     # keep background tiles (recommended)


class YoloDatasetWriter:
    def __init__(self, root: str | Path, cfg: ExportConfig | None = None,
                 tile_cfg: WaterfallTileConfig | None = None) -> None:
        self._root = Path(root)
        self._cfg = cfg or ExportConfig()
        # Recorded so dataset.yaml can state how the pixels were made; pass the
        # builder's own config when it is not the default.
        tc = tile_cfg or WaterfallTileConfig()
        self._range_equalised = tc.range_equalise
        self._p_low, self._p_high = tc.p_low, tc.p_high
        for split in ("train", "val"):
            (self._root / "images" / split).mkdir(parents=True, exist_ok=True)
            (self._root / "labels" / split).mkdir(parents=True, exist_ok=True)
        self._count = 0

    def add_tile(self, image: np.ndarray, boxes: list[YoloBox],
                 name: str) -> Path:
        split = self._split_for(name)
        img_path = self._root / "images" / split / f"{name}.png"
        lbl_path = self._root / "labels" / split / f"{name}.txt"
        Image.fromarray(image, mode="L").save(img_path)
        if boxes or self._cfg.write_empty_labels:
            lbl_path.write_text("\n".join(b.to_line() for b in boxes),
                                encoding="utf-8")
        self._count += 1
        return img_path

    def finalize(self, class_names: list[str]) -> Path:
        doc = {
            "path": str(self._root.resolve()),
            "train": "images/train",
            "val": "images/val",
            "names": {i: n for i, n in enumerate(class_names)},
            # How the pixels were made. A consumer that inverts the display
            # mapping (the augmentation stage does) has to be told, because
            # assuming the wrong one mis-scales every intensity it computes.
            # `db` is not `log`: the samples reaching the tile are already on
            # the device's per-ping dB axis, so no log was applied on top.
            "layout": "waterfall",
            "intensity_mapping": "db",
            "shadow_included": True,
            "meta": {
                "intensity_mapping": "db",
                "range_equalised": bool(self._range_equalised),
                "normalisation": "per-tile percentile",
                "percentiles": [self._p_low, self._p_high],
            },
        }
        p = self._root / "dataset.yaml"
        with open(p, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, sort_keys=False)
        return p

    def _split_for(self, name: str) -> str:
        h = int(hashlib.sha1(name.encode()).hexdigest(), 16) % 1000
        return "val" if h < self._cfg.val_fraction * 1000 else "train"

    @property
    def tile_count(self) -> int:
        return self._count
