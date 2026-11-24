"""The split guard: seeds and harbours cannot leak between dataset, tuning and
evaluation worlds.

``data/splits.yaml`` gives each split a seed range and each real harbour a
split. :func:`check_split` refuses a placement seed outside the range of the
split the block names, and a harbour that belongs to another split. An
override (``placement.split_override: "<reason>"``) lets it through; the
reason is returned and written into the world's metadata.

Pure Python.
"""

from __future__ import annotations

from .data import default_splits


class SplitError(ValueError):
    """A seed or a harbour of another split."""


def split_of_seed(seed: int) -> str | None:
    for name, rec in default_splits()["splits"].items():
        lo, hi = rec["seeds"]
        if int(lo) <= int(seed) <= int(hi):
            return str(name)
    return None


def seed_range(split: str) -> tuple[int, int]:
    rec = default_splits()["splits"].get(split)
    if rec is None:
        raise SplitError(f"placement.split {split!r}: one of {sorted(default_splits()['splits'])}")
    return int(rec["seeds"][0]), int(rec["seeds"][1])


def split_of_harbour(harbour: str) -> str | None:
    return default_splits().get("harbours", {}).get(str(harbour))


def check_split(seed: int, split: str, harbour: str = "", override: str = "") -> dict:
    """Raise :class:`SplitError` unless the seed lies in the split's range
    and the harbour (when given) belongs to the split. Returns the record
    for the metadata: ``{split, seed_range, harbour, overridden, reason,
    violations}``."""
    lo, hi = seed_range(split)
    bad = []
    if not lo <= int(seed) <= hi:
        other = split_of_seed(seed)
        bad.append(f"seed {int(seed)} is not in the {split} range {lo}..{hi}"
                   + (f" (it belongs to {other})" if other else " (it belongs to no split)"))
    if harbour:
        hs = split_of_harbour(harbour)
        if hs is None:
            bad.append(f"harbour {harbour!r} is not in the harbour split registry")
        elif hs != split:
            bad.append(f"harbour {harbour!r} belongs to the {hs} split, not to {split}")
    if bad and not override:
        raise SplitError("split guard: " + "; ".join(bad)
                         + ". Use another seed or harbour, or set placement.split_override to a reason "
                           "(it is recorded with the world).")
    return {"split": split, "seed_range": [lo, hi], "harbour": harbour or None,
            "overridden": bool(bad), "reason": override if bad else "", "violations": bad}
