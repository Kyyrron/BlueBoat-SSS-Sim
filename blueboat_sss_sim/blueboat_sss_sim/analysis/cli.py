"""``mission_metrics`` CLI: a mission bundle -> detection metrics.

Three sources, one metric path::

    # The bundle's own trajectory, deterministic from the seed
    python3 -m blueboat_sss_sim.analysis.cli --bundle ~/runs/r3 --out ~/metrics/r3

    # The path a real run actually tracked
    python3 -m blueboat_sss_sim.analysis.cli --bundle ~/runs/r3 \\
        --source svlog --input run.svlog --out ~/metrics/r3_real

    # What the node actually published (no pose, so no aspect)
    python3 -m blueboat_sss_sim.analysis.cli --bundle ~/runs/r3 \\
        --source jsonl --input contacts.jsonl --out ~/metrics/r3_pub

Reads the bundle and the recording; writes only into ``--out``. Bundles and
recordings are write-once (CM-7 / NC #10).
"""

from __future__ import annotations

import argparse
from pathlib import Path

from ..worldgen.scene import SceneModel
from .contacts import from_jsonl, from_replay, from_svlog
from .metrics import CRITERIA, DEFAULT_CRITERION, compute_metrics
from .report import content_digest, to_dict, write_report


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bundle", required=True,
                    help="mission bundle directory (scene.npz + manifest)")
    ap.add_argument("--out", required=True,
                    help="output directory for metrics.json / metrics.md")
    ap.add_argument("--source", default="replay",
                    choices=("replay", "svlog", "jsonl"),
                    help="where observations come from (default: replay)")
    ap.add_argument("--input", default=None,
                    help="recording path; required for svlog and jsonl")
    ap.add_argument("--criterion", default=DEFAULT_CRITERION,
                    choices=sorted(CRITERIA),
                    help=f"detection definition (default: {DEFAULT_CRITERION})")
    ap.add_argument("--range-bin-m", type=float, default=1.0)
    ap.add_argument("--aspect-bin-deg", type=float, default=15.0)
    ap.add_argument("--max-pings", type=int, default=None,
                    help="cap replay at this many ping cycles")
    args = ap.parse_args()

    bundle = Path(args.bundle)
    if args.source == "replay":
        obs = from_replay(bundle, max_pings=args.max_pings)
    else:
        if not args.input:
            ap.error(f"--input is required for --source {args.source}")
        obs = (from_svlog(args.input, bundle) if args.source == "svlog"
               else from_jsonl(args.input))

    scene = SceneModel.load(bundle)
    m = compute_metrics(obs, scene, criterion=args.criterion,
                        range_bin_m=args.range_bin_m,
                        aspect_bin_deg=args.aspect_bin_deg)
    js, md = write_report(m, args.out)

    print(f"criterion '{m.criterion}': {m.criterion_description}")
    print(f"  {m.n_observations} observations over {m.ping_cycles} ping cycles")
    print(f"  {len(m.detected)} detected / "
          f"{len(m.observed_not_detected)} below criterion / "
          f"{len(m.never_ensonified)} never ensonified "
          f"= {m.n_objects} in the manifest")
    if not m.aspect_available:
        print("  aspect metrics unavailable: this source carries no pose")
    print(f"  content digest {content_digest(to_dict(m))[:16]} "
          f"(provenance excluded)")
    print(f"  wrote {js}")
    print(f"         {md}")


if __name__ == "__main__":
    main()
