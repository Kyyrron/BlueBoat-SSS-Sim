#!/usr/bin/env python3
"""Dataset recorder node.

Subscribes to the *standard* sonar interface (``.../profile``) plus the
simulation-only ``~/ground_truth/contacts`` stream, assembles per-side
waterfall tiles and writes a ready-to-train YOLO dataset.

Because it only consumes published topics, it runs happily in parallel
with the existing processing pipeline and never interferes with it. On
real data (no ground-truth topic) it can still export *unlabeled* tiles
for review or pseudo-labeling.

Parameters
----------
output_dir            dataset root (required)
tile_pings            rows per exported tile              (default 512)
overlap_pings         row overlap between tiles           (default 64)
box_mode              highlight | highlight_shadow        (default highlight_shadow)
val_fraction          validation split fraction           (default 0.15)
autosave_period_s     flush completed tiles every N sec   (default 5.0)
run_name              filename prefix                     (default "run")
"""

from __future__ import annotations

import json
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from blueboat_interfaces.msg import OmniscanProfile

from ..core.types import GroundTruthContact, Side
from ..dataset.exporter import ExportConfig, YoloDatasetWriter
from ..dataset.labeler import LabelConfig, TileLabeler
from ..dataset.waterfall import WaterfallBuilder, WaterfallTileConfig


class DatasetRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("sss_dataset_recorder")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("tile_pings", 512)
        self.declare_parameter("overlap_pings", 64)
        self.declare_parameter("box_mode", "highlight_shadow")
        self.declare_parameter("val_fraction", 0.15)
        self.declare_parameter("autosave_period_s", 5.0)
        self.declare_parameter("run_name", "run")

        out = self.get_parameter("output_dir").value
        if not out:
            raise RuntimeError("parameter 'output_dir' is required")
        self._writer = YoloDatasetWriter(out, ExportConfig(
            val_fraction=float(self.get_parameter("val_fraction").value)))
        self._tile_cfg = WaterfallTileConfig(
            tile_pings=int(self.get_parameter("tile_pings").value),
            overlap_pings=int(self.get_parameter("overlap_pings").value))
        self._label_cfg = LabelConfig(
            box_mode=str(self.get_parameter("box_mode").value))
        self._run = str(self.get_parameter("run_name").value)

        self._builders: dict[Side, WaterfallBuilder] = {}
        self._labelers: dict[Side, TileLabeler] = {}
        self._tile_counter: dict[Side, int] = defaultdict(int)
        # ground truth arrives keyed by (side, ping_number)
        self._pending_contacts: dict[tuple[str, int], list[GroundTruthContact]] = {}

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        for side in ("port", "starboard"):
            self.create_subscription(
                OmniscanProfile, f"/side_scan_sonar/{side}/profile",
                self._on_profile, qos)
        self.create_subscription(
            String, "/side_scan_sonar/ground_truth/contacts",
            self._on_ground_truth, 50)

        self.create_timer(
            float(self.get_parameter("autosave_period_s").value), self._flush)
        self.get_logger().info(f"recording dataset to {out}")

    # ----------------------------------------------------------------- inputs
    def _on_ground_truth(self, msg: String) -> None:
        doc = json.loads(msg.data)
        for c in doc.get("contacts", []):
            key = (c["side"], int(c["ping_number"]))
            self._pending_contacts.setdefault(key, []).append(GroundTruthContact(
                object_id=int(c["object_id"]), object_type=str(c["type"]),
                side=Side(c["side"]), slant_range_m=float(c["slant_range_m"]),
                extent_bins=float(c["extent_bins"]),
                shadow_bins=float(c["shadow_bins"]),
                visible=bool(c["visible"])))

    def _on_profile(self, msg: OmniscanProfile) -> None:
        side = Side(msg.side)
        if side not in self._builders:
            self._builders[side] = WaterfallBuilder(msg.num_results, self._tile_cfg)
            self._labelers[side] = TileLabeler(
                msg.num_results, msg.length_mm / 1000.0 / msg.num_results,
                self._label_cfg)
        contacts = self._pending_contacts.pop((msg.side, msg.ping_number), [])
        self._builders[side].add_ping(msg.pwr_results, contacts)

    # ------------------------------------------------------------------ output
    def _flush(self, final: bool = False) -> None:
        for side, builder in self._builders.items():
            for image, rows in builder.ready_tiles(flush=final):
                boxes = self._labelers[side].label_tile(rows)
                i = self._tile_counter[side]
                self._tile_counter[side] += 1
                name = f"{self._run}_{side.value}_{i:05d}"
                self._writer.add_tile(image, boxes, name)
                self.get_logger().info(
                    f"tile {name}: {image.shape[0]}x{image.shape[1]}, "
                    f"{len(boxes)} boxes")

    def finalize(self) -> None:
        self._flush(final=True)
        classes: list[str] = []
        for lab in self._labelers.values():
            for n in lab.class_names:
                if n not in classes:
                    classes.append(n)
        if self._writer.tile_count:
            p = self._writer.finalize(classes or ["object"])
            self.get_logger().info(
                f"dataset finalised: {self._writer.tile_count} tiles -> {p}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
