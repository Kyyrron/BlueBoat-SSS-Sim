"""Output guard: no tool writes into a field recording's tree (CM-7).

Recorded ``.svlog`` files, run bags and position logs are write-once primary
field record. Every command that reads recordings and writes a report calls
:func:`refuse_inside` on its output directory first. Until 2026-10-23 the
check (then private to ``calibration/__main__``) looked only at each input
file's own folder, so ``--out SSS_data/x`` passed for an input in
``SSS_data/beach_sept/``; ``sss_svlog_compare`` and
``sss_calibration_report`` did not check at all.

Protected, together with everything below them:

* each input's own directory;
* the deepest common ancestor of the inputs (when there are several);
* every explicit root (a manifest's ``--root``);
* the field corpus, ``$BLUEBOAT_SSS_CORPUS`` or ``~/ros2_ws/data/SSS_data``
  (the smoke suite's convention), when it exists.

ROS-free; stdlib only.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable

#: Default location of the real ``.svlog`` corpus (smoke ``[2f]`` uses the same).
DEFAULT_CORPUS = Path.home() / "ros2_ws" / "data" / "SSS_data"


def corpus_root() -> Path | None:
    """The field corpus directory if it exists, else ``None``."""
    p = Path(os.environ.get("BLUEBOAT_SSS_CORPUS", DEFAULT_CORPUS)).expanduser()
    return p.resolve() if p.is_dir() else None


def protected_dirs(inputs: Iterable[str | Path],
                   roots: Iterable[str | Path] = (),
                   include_corpus: bool = True) -> list[Path]:
    """Every directory an output must stay out of (see module doc)."""
    files = [Path(p).expanduser().resolve() for p in inputs]
    dirs = {f.parent if not f.is_dir() else f for f in files}
    if len(files) > 1:
        common = Path(os.path.commonpath([str(d) for d in dirs]))
        if common != Path(common.anchor):        # never protect "/" itself
            dirs.add(common)
    dirs.update(Path(r).expanduser().resolve() for r in roots)
    if include_corpus:
        c = corpus_root()
        if c is not None:
            dirs.add(c)
    return sorted(dirs)


def refuse_inside(out: str | Path, inputs: Iterable[str | Path] = (),
                  roots: Iterable[str | Path] = (),
                  include_corpus: bool = True) -> None:
    """Raise ``SystemExit`` if ``out`` is, or lies inside, a protected
    directory (field data are write-once, CM-7)."""
    o = Path(out).expanduser().resolve()
    for d in protected_dirs(inputs, roots, include_corpus):
        if o == d or d in o.parents:
            raise SystemExit(f"--out {o} lies inside the recordings' tree "
                             f"({d}); field data are write-once (CM-7)")
