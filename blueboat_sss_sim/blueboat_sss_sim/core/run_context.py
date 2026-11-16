"""Run context: the metadata a launch hands its nodes (C3 of the completion round, 2026-11-15).

A JSON object (sim_config, plant, odometry profile, tuning file and sea-schedule SHA-256 ...) travels as ONE string
parameter. launch_ros evaluates a string parameter as YAML, which would turn a JSON object into a mapping, so the
launch passes it base64-encoded with a ``b64:`` prefix; :func:`decode` takes either form.
"""

from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path

PREFIX = "b64:"


def encode(ctx: dict) -> str:
    return PREFIX + base64.b64encode(json.dumps(ctx, sort_keys=True).encode("utf-8")).decode("ascii")


def decode(raw: str) -> dict:
    raw = str(raw or "")
    if not raw:
        return {}
    try:
        if raw.startswith(PREFIX):
            return json.loads(base64.b64decode(raw[len(PREFIX):]).decode("utf-8"))
        return json.loads(raw)
    except (ValueError, TypeError):
        return {"raw": raw}


def file_sha256(path) -> str | None:
    """SHA-256 of a file, or None when the path is empty or missing."""
    if not path:
        return None
    p = Path(str(path)).expanduser()
    if not p.is_file():
        return None
    return hashlib.sha256(p.read_bytes()).hexdigest()
