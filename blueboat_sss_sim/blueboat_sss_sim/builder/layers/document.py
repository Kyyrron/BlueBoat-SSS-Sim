"""The layer document of Screen 3 (Qt-free): the ordered stack of
authored layers above the depth source, exactly what
``DepthState.layers`` stores.

Ids are ``L<n>`` and every new layer takes ``seed = n``, so its texture
stream follows it through reorders (``worldgen.depth.compose`` draws
``[seed, 2, layer.seed]``).
"""

from __future__ import annotations

import copy
import re
from typing import Any, Mapping, Sequence

from ...worldgen.depth.layers import LAYER_KINDS, coerce_params

_ID_RE = re.compile(r"^L(\d+)$")


class LayerDocument:
    def __init__(self, layers: Sequence[Mapping[str, Any]] | None = None) -> None:
        self.layers: list[dict[str, Any]] = [copy.deepcopy(dict(L))
                                             for L in (layers or [])]

    # ----------------------------------------------------------- queries
    def ids(self) -> list[str]:
        return [str(L["id"]) for L in self.layers]

    def index(self, layer_id: str) -> int:
        for i, L in enumerate(self.layers):
            if str(L["id"]) == layer_id:
                return i
        raise KeyError(f"no layer {layer_id!r}")

    def get(self, layer_id: str) -> dict[str, Any]:
        return self.layers[self.index(layer_id)]

    def next_number(self) -> int:
        n = 0
        for L in self.layers:
            m = _ID_RE.match(str(L.get("id", "")))
            if m:
                n = max(n, int(m.group(1)))
            n = max(n, int(L.get("seed") or 0))
        return n + 1

    # ------------------------------------------------------------ edits
    def add(self, kind: str, geometry: dict | None = None,
            params: Mapping[str, Any] | None = None, name: str | None = None,
            mask: Any = None, after: str | None = None,
            layer_id: str | None = None) -> str:
        """Append (or insert after ``after``) a layer of ``kind``; a
        ``layer_id`` names it instead of the next ``L<n>`` (the reserved
        ids the pages use to find their own rows again)."""
        spec = LAYER_KINDS[kind]
        n = self.next_number()
        L = {"id": layer_id or f"L{n}", "kind": kind, "name": name or spec.label,
             "enabled": True, "opacity": 1.0, "blend": spec.blend,
             "mask": mask, "geometry": geometry,
             "params": coerce_params(spec.schema, params, f"{kind}.params")
             if spec.validate_params is None else dict(params or {}),
             "targets": {c: True for c in spec.channels}, "seed": n}
        pos = len(self.layers) if after is None else self.index(after) + 1
        self.layers.insert(pos, L)
        return L["id"]

    def add_many(self, layers: Sequence[Mapping[str, Any]]) -> list[str]:
        """Append ready layer dicts (a recipe), re-numbering their ids and
        seeds after the document's own."""
        out = []
        for L in layers:
            L = copy.deepcopy(dict(L))
            n = self.next_number()
            L["id"], L["seed"] = f"L{n}", n
            self.layers.append(L)
            out.append(L["id"])
        return out

    def remove(self, layer_id: str) -> None:
        del self.layers[self.index(layer_id)]

    def move(self, layer_id: str, delta: int) -> None:
        i = self.index(layer_id)
        j = max(0, min(len(self.layers) - 1, i + delta))
        if i != j:
            L = self.layers.pop(i)
            self.layers.insert(j, L)

    def duplicate(self, layer_id: str) -> str:
        L = copy.deepcopy(self.get(layer_id))
        n = self.next_number()
        L["id"], L["seed"] = f"L{n}", n
        L["name"] = f"{L.get('name', L['kind'])} copy"
        self.layers.insert(self.index(layer_id) + 1, L)
        return L["id"]

    # -------------------------------------------------------- snapshots
    def snapshot(self) -> list[dict[str, Any]]:
        return copy.deepcopy(self.layers)

    def restore(self, layers: Sequence[Mapping[str, Any]]) -> None:
        self.layers = [copy.deepcopy(dict(L)) for L in layers]

    def to_list(self) -> list[dict[str, Any]]:
        return self.snapshot()
