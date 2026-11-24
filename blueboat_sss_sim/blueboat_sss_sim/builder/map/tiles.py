"""Background satellite/OSM tiles without extra dependencies.

Copied from the BlueBoat GCS's ``mapping/tiles.py`` (the project's best
tile engine -- bounded-concurrency ``QNetworkAccessManager`` downloads
with a persistent disk cache), adapted only in cache location and in
taking its slippy math from :mod:`blueboat_sss_sim.core.geo`. Copied,
never imported: modules integrate by identical interfaces (CM-3).

Offline the fetcher degrades to cache-only: any area browsed once stays
available, an unbrowsed area simply shows the grid.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Set, Tuple

from PySide6.QtCore import QObject, QUrl, Signal
from PySide6.QtGui import QImage
from PySide6.QtNetwork import (QNetworkAccessManager, QNetworkReply,
                               QNetworkRequest)

from ...core.geo import metres_per_pixel

TILE_SIZE_PX = 256
MIN_ZOOM, MAX_ZOOM = 3, 19

#: Esri World Imagery (note the {z}/{y}/{x} order) -- the same source the
#: MCS map and the GCS satellite layer use.
ESRI_SATELLITE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
                      "World_Imagery/MapServer/tile/{z}/{y}/{x}")
OSM_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
DEFAULT_CACHE_DIR = Path.home() / ".cache" / "blueboat_sss_sim" / "tiles"

TileKey = Tuple[int, int, int]  # (z, x, y)


def zoom_for_resolution(lat: float, metres_per_px: float) -> int:
    """Smallest zoom whose tiles are at least as sharp as the view."""
    for z in range(MAX_ZOOM, MIN_ZOOM - 1, -1):
        if metres_per_pixel(lat, z) >= metres_per_px * 0.75:
            return z
    return MAX_ZOOM


class TileFetcher(QObject):
    """Async tile downloads with a persistent disk cache.

    Emits ``tile_ready(z, x, y, QImage)``. Failed downloads are dropped
    silently (the layer simply shows nothing there); a tile is
    re-requested on the next viewport change.
    """

    tile_ready = Signal(int, int, int, QImage)

    def __init__(self, url_template: str = ESRI_SATELLITE_URL,
                 cache_dir: Path = DEFAULT_CACHE_DIR,
                 max_concurrent: int = 6,
                 parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._template = url_template
        self._cache_dir = Path(cache_dir)
        self._cache_dir.mkdir(parents=True, exist_ok=True)
        self._nam = QNetworkAccessManager(self)
        self._pending: Set[TileKey] = set()
        self._queue: list[TileKey] = []
        self._max_concurrent = max_concurrent
        self._inflight = 0

    # -- public ---------------------------------------------------------------
    def request(self, key: TileKey) -> Optional[QImage]:
        """Return the tile immediately if cached, else schedule a download."""
        img = self._load_cached(key)
        if img is not None:
            return img
        if key not in self._pending:
            self._pending.add(key)
            self._queue.append(key)
            self._pump()
        return None

    # -- internals ----------------------------------------------------------
    def _cache_path(self, key: TileKey) -> Path:
        z, x, y = key
        return self._cache_dir / str(z) / str(x) / f"{y}.png"

    def _load_cached(self, key: TileKey) -> Optional[QImage]:
        p = self._cache_path(key)
        if p.is_file():
            img = QImage(str(p))
            if not img.isNull():
                return img
        return None

    def _pump(self) -> None:
        while self._queue and self._inflight < self._max_concurrent:
            key = self._queue.pop(0)
            z, x, y = key
            url = self._template.format(z=z, x=x, y=y)
            req = QNetworkRequest(QUrl(url))
            # OSM usage policy requires an identifying user agent.
            req.setHeader(QNetworkRequest.UserAgentHeader,
                          "BlueBoatWorldBuilder/1.0 (research USV survey)")
            reply = self._nam.get(req)
            self._inflight += 1
            reply.finished.connect(lambda r=reply, k=key: self._on_reply(r, k))

    def _on_reply(self, reply: QNetworkReply, key: TileKey) -> None:
        self._inflight -= 1
        self._pending.discard(key)
        try:
            if reply.error() == QNetworkReply.NetworkError.NoError:
                data = bytes(reply.readAll())
                img = QImage.fromData(data)
                if not img.isNull():
                    path = self._cache_path(key)
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes(data)
                    self.tile_ready.emit(key[0], key[1], key[2], img)
        finally:
            reply.deleteLater()
            self._pump()
