"""
map_widget.py — OpenStreetMap tile widget
PyQt6 QWidget using QPainter + threaded urllib for tile fetching.
Dynamically fetches and tiles the viewport around the ROV position,
drawing the path trail and ROV heading indicator.
Supports mouse-wheel zooming and larger high-DPI markers.
"""

import math
import threading
import urllib.request
from collections import deque

from PyQt6.QtCore import Qt, QPointF, pyqtSignal, QObject
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QPixmap, QPolygonF, QFont
from PyQt6.QtWidgets import QWidget

DEFAULT_ZOOM = 16  # Closer, more detailed street/water view
TILE_SIZE = 256
MAX_TRAIL = 800
USER_AGENT = "ROVGroundControlStation/1.0 (https://github.com/coratia/rov-gcs; telemetry-intern@coratia.com)"


def _latlon_to_world_px(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Return world pixel coordinates at the given zoom level."""
    n = 2.0 ** zoom
    wx = ((lon + 180.0) / 360.0 * n) * TILE_SIZE
    lat_r = math.radians(lat)
    lat_r = max(min(lat_r, 1.4844), -1.4844)
    wy = ((1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n) * TILE_SIZE
    return wx, wy


class _TileFetcher(QObject):
    tile_ready = pyqtSignal(tuple, bytes)

    def fetch(self, key: tuple):
        zoom, tx, ty = key
        url = f"https://tile.openstreetmap.org/{zoom}/{tx}/{ty}.png"
        try:
            req = urllib.request.Request(
                url,
                headers={
                    "User-Agent": USER_AGENT,
                    "Accept": "image/png,image/*;q=0.8",
                }
            )
            with urllib.request.urlopen(req, timeout=6) as resp:
                if resp.status == 200:
                    data = resp.read()
                    self.tile_ready.emit(key, data)
        except Exception:
            pass


class MapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(450, 320)
        self.setStyleSheet("background: #181818;")

        self.zoom: int = DEFAULT_ZOOM
        self._lat: float = 12.9716
        self._lon: float = 77.5946
        self._yaw: float = 0.0

        self._trail: deque[tuple[float, float]] = deque(maxlen=MAX_TRAIL)
        self._tile_cache: dict[tuple, QPixmap] = {}
        self._pending: set[tuple] = set()

        self._fetcher = _TileFetcher()
        self._fetcher.tile_ready.connect(self._on_tile_ready)

    def set_zoom(self, new_zoom: int):
        new_zoom = max(12, min(18, new_zoom))
        if new_zoom != self.zoom:
            self.zoom = new_zoom
            self._fetch_visible_tiles()
            self.update()

    def wheelEvent(self, event):
        """Allow zooming in/out with the mouse scroll wheel."""
        delta = event.angleDelta().y()
        if delta > 0:
            self.set_zoom(self.zoom + 1)
        elif delta < 0:
            self.set_zoom(self.zoom - 1)
        event.accept()

    def update_position(self, lat: float, lon: float, yaw: float):
        self._lat = lat
        self._lon = lon
        self._yaw = yaw
        self._trail.append((lat, lon))

        self._fetch_visible_tiles()
        self.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._fetch_visible_tiles()

    def showEvent(self, event):
        super().showEvent(event)
        self._fetch_visible_tiles()

    def _fetch_visible_tiles(self):
        w = max(self.width(), 400)
        h = max(self.height(), 300)

        rov_wx, rov_wy = _latlon_to_world_px(self._lat, self._lon, self.zoom)
        view_left = rov_wx - w / 2.0
        view_top  = rov_wy - h / 2.0

        min_tx = int(math.floor(view_left / TILE_SIZE))
        max_tx = int(math.floor((view_left + w) / TILE_SIZE))
        min_ty = int(math.floor(view_top / TILE_SIZE))
        max_ty = int(math.floor((view_top + h) / TILE_SIZE))

        for tx in range(min_tx, max_tx + 1):
            for ty in range(min_ty, max_ty + 1):
                key = (self.zoom, tx, ty)
                if key not in self._tile_cache and key not in self._pending:
                    self._pending.add(key)
                    t = threading.Thread(target=self._fetcher.fetch, args=(key,), daemon=True)
                    t.start()

    def _on_tile_ready(self, key: tuple, data: bytes):
        pix = QPixmap()
        if pix.loadFromData(data):
            self._tile_cache[key] = pix
            self._pending.discard(key)
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        rov_wx, rov_wy = _latlon_to_world_px(self._lat, self._lon, self.zoom)
        view_left = rov_wx - w / 2.0
        view_top  = rov_wy - h / 2.0

        min_tx = int(math.floor(view_left / TILE_SIZE))
        max_tx = int(math.floor((view_left + w) / TILE_SIZE))
        min_ty = int(math.floor(view_top / TILE_SIZE))
        max_ty = int(math.floor((view_top + h) / TILE_SIZE))

        # 1. Draw map tiles
        for tx in range(min_tx, max_tx + 1):
            for ty in range(min_ty, max_ty + 1):
                key = (self.zoom, tx, ty)
                screen_x = int(tx * TILE_SIZE - view_left)
                screen_y = int(ty * TILE_SIZE - view_top)

                if key in self._tile_cache:
                    painter.drawPixmap(screen_x, screen_y, self._tile_cache[key])
                else:
                    painter.fillRect(screen_x, screen_y, TILE_SIZE, TILE_SIZE, QColor(32, 35, 40))
                    painter.setPen(QColor(60, 65, 72))
                    painter.drawRect(screen_x, screen_y, TILE_SIZE, TILE_SIZE)

        # 2. Draw vehicle breadcrumb trail
        if len(self._trail) >= 2:
            trail_pts = []
            for lat, lon in self._trail:
                pt_wx, pt_wy = _latlon_to_world_px(lat, lon, self.zoom)
                trail_pts.append(QPointF(pt_wx - view_left, pt_wy - view_top))

            pen = QPen(QColor(0, 215, 255, 220), 3.0)
            painter.setPen(pen)
            for i in range(1, len(trail_pts)):
                painter.drawLine(trail_pts[i - 1], trail_pts[i])

        # 3. Draw ROV marker at center
        cx = w / 2.0
        cy = h / 2.0
        self._draw_rov_marker(painter, cx, cy, self._yaw)

        # 4. Viewport info badge (top left)
        font = QFont("sans-serif", 11, QFont.Weight.Medium)
        painter.setFont(font)
        painter.fillRect(10, 10, 310, 28, QColor(18, 20, 24, 210))
        painter.setPen(QColor(56, 189, 248))
        painter.drawText(18, 29, f"OSM Zoom {self.zoom} (Scroll to zoom) • {self._lat:.5f}, {self._lon:.5f}")

        painter.end()

    def _draw_rov_marker(self, painter: QPainter, cx: float, cy: float, yaw: float):
        # Outer pulsing radar ring
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor(0, 215, 255, 140), 2.5))
        painter.drawEllipse(QPointF(cx, cy), 18, 18)

        # Vehicle body circle
        painter.setBrush(QBrush(QColor(239, 68, 68)))
        painter.setPen(QPen(QColor(255, 255, 255), 2.2))
        painter.drawEllipse(QPointF(cx, cy), 10, 10)

        # Heading vector line
        arrow_len = 32
        angle_r = math.radians(yaw - 90.0)  # 0 deg = North (upward)
        tip_x = cx + arrow_len * math.cos(angle_r)
        tip_y = cy + arrow_len * math.sin(angle_r)

        painter.setPen(QPen(QColor(250, 204, 21), 2.8))
        painter.drawLine(QPointF(cx, cy), QPointF(tip_x, tip_y))

        # Heading triangle arrowhead
        left_x = tip_x + 9 * math.cos(angle_r + math.radians(145))
        left_y = tip_y + 9 * math.sin(angle_r + math.radians(145))
        right_x = tip_x + 9 * math.cos(angle_r - math.radians(145))
        right_y = tip_y + 9 * math.sin(angle_r - math.radians(145))
        poly = QPolygonF([QPointF(tip_x, tip_y), QPointF(left_x, left_y), QPointF(right_x, right_y)])

        painter.setBrush(QBrush(QColor(250, 204, 21)))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawPolygon(poly)
