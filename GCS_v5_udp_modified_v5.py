import sys
import os
import math
import csv
import threading
import time
import socket
import struct
from datetime import datetime, timedelta
import pyqtgraph as pg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion
from PyQt5.QtCore import Qt, QTimer, QUrl, QThread, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QPainterPath, QPalette, QFont, QImage, QPixmap
import numpy as np
import cv2
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QTextEdit,
    QFrame,
    QStatusBar,
    QMenuBar,
    QStackedWidget,
    QFileDialog,
    QLineEdit,
    QComboBox,
)
from PyQt5.QtWebEngineWidgets import QWebEngineView


# ========== Leaflet 地圖 HTML ==========

MAP_HTML_CONTENT = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<title>UAV Live Map</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0">

<link rel="stylesheet"
 href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>

<style>
html, body {
    height: 100%;
    margin: 0;
}
#map {
    width: 100%;
    height: 100%;
}
</style>
</head>
<body>

<div id="map"></div>

<script>
// 初始中心點
var map = L.map('map').setView([25.03, 121.56], 17);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19
}).addTo(map);

// 紅色 UAV icon
var uavIcon = L.icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
    iconSize: [25, 41],
    iconAnchor: [12, 20]
});

var marker = L.marker([25.03, 121.56], {icon: uavIcon}).addTo(map);
var path = L.polyline([], { color: 'lime' }).addTo(map);

function updateMarker(lat, lon) {
    var ll = [lat, lon];
    marker.setLatLng(ll);
    path.addLatLng(ll);
    map.setView(ll);
}

function clearTrack() {
    path.setLatLngs([]);
}
window.updateMarker = updateMarker;
window.clearTrack = clearTrack;
</script>

</body>
</html>
"""


def ensure_map_html(filename: str = "map.html"):
    """若 map.html 不存在就寫入預設內容"""
    if not os.path.exists(filename):
        with open(filename, "w", encoding="utf-8") as f:
            f.write(MAP_HTML_CONTENT)


# ========== Telemetry & Link ==========

class Telemetry:
    """共用飛行資料結構"""

    def __init__(self):
        self._lock = threading.Lock()
        self.timestamp = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.altitude = 0.0
        self.airspeed = 0.0
        self.lat = 25.03
        self.lon = 121.56
        self.temperature = 20.0
        self.battery = 100.0
        self.gps_sats = 10
        self.flight_mode = "MANUAL"

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                setattr(self, k, v)

    def snapshot(self):
        with self._lock:
            import copy
            return copy.copy(self)


class FakeLink:
    """假資料來源（視覺測試 / Demo 用）"""

    def __init__(self, telemetry: Telemetry, update_hz: float = 20.0):
        self.telemetry = telemetry
        self.update_hz = update_hz
        self._running = False
        self._t0 = time.time()
        self._thread = None
        self.center_lat = telemetry.lat
        self.center_lon = telemetry.lon

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._running = True
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False

    def _run(self):
        modes = ["MANUAL", "STAB", "LOITER", "AUTO"]
        while self._running:
            try:
                t = time.time() - self._t0

                # 姿態 / 高度 / 速度 / 溫度
                radius_lat = 0.0008
                radius_lon = 0.0008
                self.telemetry.update(
                    timestamp=t,
                    roll=30.0 * math.sin(t * 0.5),
                    pitch=15.0 * math.sin(t * 0.3),
                    yaw=(t * 15.0) % 360.0,
                    altitude=100.0 + 10.0 * math.sin(t * 0.2),
                    airspeed=25.0 + 3.0 * math.sin(t * 0.7),
                    temperature=20.0 + 10.0 * math.sin(t * 0.4),
                    battery=max(0.0, 100.0 - t * 0.2),
                    gps_sats=8 + int(2 * math.sin(t * 0.1)),
                    flight_mode=modes[int(t / 15) % len(modes)],
                    lat=self.center_lat + radius_lat * math.cos(t * 0.1),
                    lon=self.center_lon + radius_lon * math.sin(t * 0.1),
                )

            except Exception as e:
                print("FakeLink run error:", e)
                break

            time.sleep(1.0 / self.update_hz)

    def send_arm(self, arm): pass
    def send_takeoff(self, altitude_m=10.0): pass
    def send_land(self): pass
    def send_set_mode(self, mode_name): pass
    def send_waypoints(self, waypoints): pass


class MavlinkLink:
    """
    MAVLink 連線（pymavlink）
    - 支援 serial: COMx (Windows) / /dev/ttyUSB0 (Linux)
    - 支援 udp: udp:0.0.0.0:14550
    - 讀取 ATTITUDE / GLOBAL_POSITION_INT / VFR_HUD / SYS_STATUS / GPS_RAW_INT / HEARTBEAT
    """

    def __init__(self, telemetry: Telemetry, connection_string: str = "COM5", baudrate: int = 115200, update_hz: float = 50.0):
        self.telemetry = telemetry
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.update_hz = update_hz

        self._running = False
        self._thread = None
        self._master = None
        self._t0 = None
        self._mavutil = None
        self._message_debug = False
        self._link_scheme = self._detect_scheme(connection_string)

    @staticmethod
    def _detect_scheme(connection_string: str) -> str:
        cs = (connection_string or "").strip().lower()
        if cs.startswith(("udp:", "udpin:", "udpout:", "tcp:", "tcpin:", "tcpout:")):
            return "NETWORK"
        return "SERIAL"

    @property
    def link_scheme(self):
        return self._link_scheme

    @staticmethod
    def build_udp_connection(host: str, port: int) -> str:
        host = (host or "0.0.0.0").strip() or "0.0.0.0"
        return f"udp:{host}:{int(port)}"

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._running = True
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False

    @staticmethod
    def _safe_float(x, default=0.0):
        try:
            return float(x)
        except Exception:
            return default

    def _request_message_interval(self, message_id, frequency_hz):
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            self._mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            1000000 / frequency_hz,
            0, 0, 0, 0, 0,
        )

    def _open_connection(self, mavutil):
        if self._link_scheme == "SERIAL":
            return mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate,
                autoreconnect=False,
            )
        return mavutil.mavlink_connection(self.connection_string, autoreconnect=False)

    def _heartbeat_fields(self, msg):
        mavlink = self._mavutil.mavlink
        vehicle_map = {
            mavlink.MAV_TYPE_GENERIC: "GENERIC",
            mavlink.MAV_TYPE_FIXED_WING: "FIXED_WING",
            mavlink.MAV_TYPE_QUADROTOR: "QUADROTOR",
            mavlink.MAV_TYPE_COAXIAL: "COAXIAL",
            mavlink.MAV_TYPE_HELICOPTER: "HELICOPTER",
            mavlink.MAV_TYPE_ANTENNA_TRACKER: "TRACKER",
            mavlink.MAV_TYPE_GCS: "GCS",
            mavlink.MAV_TYPE_AIRSHIP: "AIRSHIP",
            mavlink.MAV_TYPE_FREE_BALLOON: "BALLOON",
            mavlink.MAV_TYPE_ROCKET: "ROCKET",
            mavlink.MAV_TYPE_GROUND_ROVER: "ROVER",
            mavlink.MAV_TYPE_SURFACE_BOAT: "BOAT",
            mavlink.MAV_TYPE_SUBMARINE: "SUBMARINE",
            mavlink.MAV_TYPE_HEXAROTOR: "HEXAROTOR",
            mavlink.MAV_TYPE_OCTOROTOR: "OCTOROTOR",
            mavlink.MAV_TYPE_TRICOPTER: "TRICOPTER",
            mavlink.MAV_TYPE_FLAPPING_WING: "FLAPPING_WING",
            mavlink.MAV_TYPE_KITE: "KITE",
            mavlink.MAV_TYPE_ONBOARD_CONTROLLER: "ONBOARD_CONTROLLER",
            mavlink.MAV_TYPE_VTOL_DUOROTOR: "VTOL_DUOROTOR",
            mavlink.MAV_TYPE_VTOL_QUADROTOR: "VTOL_QUADROTOR",
            mavlink.MAV_TYPE_VTOL_TILTROTOR: "VTOL_TILTROTOR",
            mavlink.MAV_TYPE_VTOL_RESERVED2: "VTOL_RESERVED2",
            mavlink.MAV_TYPE_VTOL_RESERVED3: "VTOL_RESERVED3",
            mavlink.MAV_TYPE_VTOL_RESERVED4: "VTOL_RESERVED4",
            mavlink.MAV_TYPE_VTOL_RESERVED5: "VTOL_RESERVED5",
        }
        vtype = vehicle_map.get(getattr(msg, 'type', None), f"TYPE_{getattr(msg, 'type', 'NA')}")
        return {
            'system_id': getattr(msg, 'get_srcSystem', lambda: 0)(),
            'component_id': getattr(msg, 'get_srcComponent', lambda: 0)(),
            'vehicle_type': vtype,
            'last_heartbeat_monotonic': time.monotonic(),
            'last_heartbeat_text': datetime.now().strftime('%H:%M:%S'),
        }

    def _run(self):
        try:
            from pymavlink import mavutil
            self._mavutil = mavutil
        except ImportError:
            print("請先安裝 pymavlink：pip install pymavlink")
            self._running = False
            return

        try:
            self._master = self._open_connection(mavutil)
        except Exception as e:
            print("MAVLink 連線失敗:", e)
            self._running = False
            return

        try:
            hb = self._master.wait_heartbeat(timeout=8)
            self.telemetry.update(**self._heartbeat_fields(hb))
        except Exception as e:
            print("等待 Heartbeat 失敗:", e)
            self._running = False
            try:
                self._master.close()
            except Exception:
                pass
            return

        try:
            if self._link_scheme == "SERIAL":
                self._master.mav.request_data_stream_send(
                    self._master.target_system,
                    self._master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    10, 1,
                )
                self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)
                self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
                self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)
                self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)
                self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2)
        except Exception as e:
            print("設定資料流頻率失敗:", e)

        self._t0 = time.time()
        while self._running:
            try:
                msg = self._master.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                mtype = msg.get_type()
                now = time.time()
                if mtype == 'BAD_DATA':
                    continue
                if mtype == 'ATTITUDE':
                    self.telemetry.update(
                        timestamp=now - self._t0,
                        roll=math.degrees(msg.roll),
                        pitch=math.degrees(msg.pitch),
                        yaw=(math.degrees(msg.yaw) + 360.0) % 360.0,
                    )
                elif mtype == 'GLOBAL_POSITION_INT':
                    self.telemetry.update(
                        timestamp=now - self._t0,
                        lat=msg.lat / 1e7,
                        lon=msg.lon / 1e7,
                        altitude=msg.relative_alt / 1000.0,
                    )
                elif mtype == 'VFR_HUD':
                    if hasattr(msg, 'airspeed'):
                        self.telemetry.update(timestamp=now - self._t0, airspeed=self._safe_float(msg.airspeed, self.telemetry.airspeed))
                elif mtype == 'SYS_STATUS':
                    if hasattr(msg, 'battery_remaining') and msg.battery_remaining != -1:
                        self.telemetry.update(timestamp=now - self._t0, battery=float(msg.battery_remaining))
                elif mtype == 'GPS_RAW_INT':
                    if hasattr(msg, 'satellites_visible'):
                        self.telemetry.update(timestamp=now - self._t0, gps_sats=int(msg.satellites_visible))
                elif mtype == 'HEARTBEAT':
                    try:
                        mode_str = mavutil.mode_string_v10(msg)
                    except Exception:
                        mode_str = 'UNKNOWN'
                    fields = self._heartbeat_fields(msg)
                    fields.update(timestamp=now - self._t0, flight_mode=mode_str)
                    self.telemetry.update(**fields)
            except Exception as e:
                print('MAVLink 接收錯誤:', e)
                time.sleep(0.1)

        try:
            if self._master:
                self._master.close()
        except Exception:
            pass

    def send_heartbeat(self):
        if self._master is None or self._mavutil is None:
            return
        try:
            self._master.mav.heartbeat_send(
                self._mavutil.mavlink.MAV_TYPE_GCS,
                self._mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
        except Exception:
            pass

    def send_arm(self, arm: bool):
        if self._master is None:
            return
        try:
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1 if arm else 0,
                0, 0, 0, 0, 0, 0
            )
        except Exception as e:
            print(f'send_arm error: {e}')

    def send_takeoff(self, altitude_m: float = 10.0):
        if self._master is None:
            return
        try:
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                self._mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0, altitude_m
            )
        except Exception as e:
            print(f'send_takeoff error: {e}')

    def send_land(self):
        if self._master is None:
            return
        try:
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                self._mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
        except Exception as e:
            print(f'send_land error: {e}')

    def send_set_mode(self, mode_name: str):
        if self._master is None or self._mavutil is None:
            return
        try:
            mode_id = self._master.mode_mapping().get(mode_name.upper())
            if mode_id is None:
                print(f'Unknown mode: {mode_name}')
                return
            self._master.mav.set_mode_send(
                self._master.target_system,
                self._mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
        except Exception as e:
            print(f'send_set_mode error: {e}')

    def send_waypoints(self, waypoints: list):
        if self._master is None or self._mavutil is None:
            return
        try:
            mavlink = self._mavutil.mavlink
            count = len(waypoints)
            self._master.mav.mission_count_send(
                self._master.target_system, self._master.target_component, count
            )
            for i, wp in enumerate(waypoints):
                self._master.mav.mission_item_send(
                    self._master.target_system,
                    self._master.target_component,
                    i,
                    mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1,
                    0, 0, 0, 0,
                    float(wp['lat']), float(wp['lon']), float(wp['alt'])
                )
        except Exception as e:
            print(f'send_waypoints error: {e}')

class AttitudeWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.setMinimumSize(220, 220)

    def set_attitude(self, roll_deg: float, pitch_deg: float):
        self.roll = roll_deg
        self.pitch = pitch_deg
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2
        cy = h / 2
        size = min(w, h)
        radius = size * 0.45  # 儀表半徑

        # 1. 畫背景（儀表外框）
        painter.setBrush(QColor("#020617"))
        painter.setPen(QPen(QColor("#4b5563"), 3))
        painter.drawEllipse(int(cx - radius), int(cy - radius),
                            int(2 * radius), int(2 * radius))

        # 設定裁剪區域 (Clip)，讓天地線只顯示在圓圈內
        clip_path = QPainterPath()
        clip_path.addEllipse(cx - radius, cy - radius, radius * 2, radius * 2)
        painter.setClipPath(clip_path)

        # --- 開始繪製旋轉層 (天地線 + 俯仰刻度) ---
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.roll)  # 旋轉整層

        # 計算俯仰位移 (Pixel per Degree)
        # 假設半徑涵蓋約 30 度視角
        pitch_scale = radius / 30.0  
        dy = self.pitch * pitch_scale

        # 2. 畫天地線 (Sky & Ground)
        # 畫天空 (藍色)
        sky_height = radius * 4 
        sky_top = -sky_height + dy
        painter.fillRect(int(-radius * 4), int(sky_top),
                         int(radius * 8), int(sky_height), QColor("#0ea5e9")) # Sky Blue

        # 畫地面 (棕色)
        ground_top = dy
        painter.fillRect(int(-radius * 4), int(ground_top),
                         int(radius * 8), int(sky_height), QColor("#854d0e")) # Earth Brown

        # 畫地平線 (白色實線)
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawLine(int(-radius * 4), int(dy), int(radius * 4), int(dy))

        # 3. 畫俯仰刻度 (Pitch Ladder)
        font = painter.font()
        font.setPixelSize(10)
        font.setBold(True)
        painter.setFont(font)

        # 畫出 +/- 90 度的刻度
        for i in range(-9, 10): # -90 到 90 度，每 10 度一階
            angle = i * 10
            if angle == 0: continue
            
            y = dy - angle * pitch_scale
            
            # 如果刻度超出顯示範圍太遠就不畫，節省效能
            if y < -radius or y > radius:
                continue

            # 長度：每10度較長
            width = radius * 0.5
            
            painter.setPen(QPen(QColor("white"), 1))
            painter.drawLine(int(-width / 2), int(y), int(width / 2), int(y))
            
            # 畫數字
            text = f"{abs(angle)}"
            fm = painter.fontMetrics()
            tw = fm.width(text)
            th = fm.height()
            # 數字畫在線的左右兩側
            painter.drawText(int(-width/2 - tw - 4), int(y + th/3), text)
            painter.drawText(int(width/2 + 4), int(y + th/3), text)
            
            # 在中間畫個 5 度的短線 (例如 15, 25)
            # 這裡簡單處理：畫 i*10 + 5 度的線
            y5 = dy - (angle + 5) * pitch_scale
            if -radius < y5 < radius and angle != 90:
                painter.drawLine(int(-width / 4), int(y5), int(width / 4), int(y5))

        painter.restore() 
        # --- 結束旋轉層 ---

        painter.setClipping(False) # 取消裁剪，因為接下來的指針要畫在最上層

        # 4. 畫固定參考飛機符號 (The "W" / Symbolic Aircraft)
        # 這是固定在儀表中心的
        painter.setPen(QPen(QColor("#facc15"), 3)) # 黃色
        painter.setBrush(Qt.NoBrush)
        
        # 左翼
        painter.drawLine(int(cx - radius * 0.4), int(cy), int(cx - radius * 0.15), int(cy))
        painter.drawLine(int(cx - radius * 0.15), int(cy), int(cx - radius * 0.15), int(cy + radius * 0.15))
        # 右翼
        painter.drawLine(int(cx + radius * 0.4), int(cy), int(cx + radius * 0.15), int(cy))
        painter.drawLine(int(cx + radius * 0.15), int(cy), int(cx + radius * 0.15), int(cy + radius * 0.15))
        # 中心點 (小點)
        painter.setBrush(QColor("#facc15"))
        painter.drawEllipse(int(cx)-2, int(cy)-2, 4, 4)

        # 5. 畫頂部滾轉刻度 (Roll Scale)
        painter.setPen(QPen(QColor("white"), 2))
        
        # 刻度位置：10, 20, 30, 45, 60 度
        scale_angles = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]
        roll_radius = radius * 0.95
        
        for a in scale_angles:
            rad = math.radians(a - 90) # -90 是因為 0 度在 12 點鐘方向
            x1 = cx + roll_radius * math.cos(rad)
            y1 = cy + roll_radius * math.sin(rad)
            
            # 長刻度與短刻度
            len_scale = radius * 0.1 if abs(a) % 30 == 0 else radius * 0.05
            x2 = cx + (roll_radius - len_scale) * math.cos(rad)
            y2 = cy + (roll_radius - len_scale) * math.sin(rad)
            
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # 6. 畫滾轉指針 (Sky Pointer) —— 這就是「姿態三角」
        # 固定在 12 點鐘方向的橘色倒三角形
        triangle_path = QPainterPath()
        top_y = cy - roll_radius + 5
        # 畫一個倒三角
        triangle_path.moveTo(cx, top_y + 14)     # 下頂點
        triangle_path.lineTo(cx - 7, top_y)      # 左上
        triangle_path.lineTo(cx + 7, top_y)      # 右上
        triangle_path.closeSubpath()
        
        painter.setBrush(QColor("#f97316")) # 橘色
        painter.setPen(Qt.NoPen)
        painter.drawPath(triangle_path)

        # 7. 側滑儀 (Slip/Skid Indicator / The Ball)
        # 在底部畫一個小長條
        slip_rect_w = radius * 0.8
        slip_rect_h = radius * 0.12
        slip_rect_y = cy + radius * 0.65
        
        painter.setPen(QPen(QColor("white"), 1))
        painter.setBrush(Qt.NoBrush)
        # 畫框框 (略帶弧度)
        painter.drawRoundedRect(int(cx - slip_rect_w/2), int(slip_rect_y), 
                                int(slip_rect_w), int(slip_rect_h), 5, 5)
        
        # 畫中間的兩條參考線
        painter.drawLine(int(cx - 10), int(slip_rect_y), int(cx - 10), int(slip_rect_y + slip_rect_h))
        painter.drawLine(int(cx + 10), int(slip_rect_y), int(cx + 10), int(slip_rect_y + slip_rect_h))

        # 畫球 (The Ball) - 假設沒有側滑數據，先畫在中間
        # 未來若有 telemetry.side_slip，可修改 ball_center_x
        ball_center_x = cx 
        ball_radius = slip_rect_h * 0.4
        painter.setBrush(QColor("#e5e7eb")) # 銀白色球
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(int(ball_center_x - ball_radius), int(slip_rect_y + slip_rect_h/2 - ball_radius),
                            int(ball_radius*2), int(ball_radius*2))


class GaugeWidget(QWidget):
    """圓形儀表"""

    def __init__(self, title: str, unit: str,
                 min_value: float, max_value: float, parent=None):
        super().__init__(parent)
        self.title = title
        self.unit = unit
        self.min_value = min_value
        self.max_value = max_value
        self.value = 0.0
        self.setMinimumSize(200, 200)

    def set_value(self, v: float):
        self.value = max(self.min_value, min(self.max_value, v))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        size = min(w, h)
        cx = w / 2
        cy = h / 2 + 20
        radius = size * 0.4

        painter.fillRect(self.rect(), QColor("#020617"))

        painter.setPen(QColor("#e5e7eb"))
        painter.drawText(0, 0, w, 30, Qt.AlignCenter,
                         f"{self.title}: {self.value: .1f} {self.unit}")

        painter.setPen(QPen(QColor("#4b5563"), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(int(cx - radius), int(cy - radius),
                            int(2 * radius), int(2 * radius))

        painter.save()
        painter.translate(cx, cy)
        painter.setPen(QPen(QColor("#9ca3af"), 1))

        start_angle = -210
        end_angle = 30
        steps = 10
        for i in range(steps + 1):
            a = math.radians(start_angle + (end_angle - start_angle) * i / steps)
            x1 = (radius - 10) * math.cos(a)
            y1 = (radius - 10) * math.sin(a)
            x2 = radius * math.cos(a)
            y2 = radius * math.sin(a)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        norm = (self.value - self.min_value) / (self.max_value - self.min_value or 1)
        angle = math.radians(start_angle + (end_angle - start_angle) * norm)
        painter.setPen(QPen(QColor("red"), 3))
        x = (radius - 20) * math.cos(angle)
        y = (radius - 20) * math.sin(angle)
        painter.drawLine(0, 0, int(x), int(y))

        painter.restore()

# ========== 地圖 Widget ==========

class WebEngineMapWidget(QWebEngineView):
    """Leaflet + OSM 地圖"""

    def __init__(self, html_path: str = "map.html", parent=None):
        super().__init__(parent)
        path = os.path.abspath(html_path)
        url = QUrl.fromLocalFile(path)
        self.load(url)

    def update_marker(self, lat: float, lon: float):
        js = f"updateMarker({lat}, {lon});"
        self.page().runJavaScript(js)

    def clear_track(self):
        self.page().runJavaScript("clearTrack();")


# ========== 飛行監控儀表 + 線圖視窗 ==========

class FlightChartsWindow(QMainWindow):
    """即時儀表 + 曲線圖（讀同一個 Telemetry）"""

    def __init__(self, telemetry: Telemetry, parent=None):
        super().__init__(parent)
        self.telemetry = telemetry
        self.setWindowTitle("Flight monitoring")
        self.resize(1200, 700)

        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # 上：三個圓形儀表
        gauges_layout = QHBoxLayout()
        self.alt_gauge = GaugeWidget("Altitude", "m", 0, 1000)
        self.vel_gauge = GaugeWidget("Velocity", "m/s", 0, 100)
        self.temp_gauge = GaugeWidget("Temperature", "°C", -20, 60)
        gauges_layout.addWidget(self.alt_gauge)
        gauges_layout.addWidget(self.vel_gauge)
        gauges_layout.addWidget(self.temp_gauge)
        layout.addLayout(gauges_layout)

        # 下：三個曲線
        plots_layout = QHBoxLayout()

        pg.setConfigOptions(background="#020617", foreground="w")

        self.alt_plot = pg.PlotWidget(title="Altitude")
        self.alt_plot.setLabel("left", "Altitude", units="m")
        self.alt_plot.setLabel("bottom", "Time", units="s")
        self.alt_curve = self.alt_plot.plot(pen=pg.mkPen(width=2))
        plots_layout.addWidget(self.alt_plot)

        self.vel_plot = pg.PlotWidget(title="Velocity")
        self.vel_plot.setLabel("left", "Velocity", units="m/s")
        self.vel_plot.setLabel("bottom", "Time", units="s")
        self.vel_curve = self.vel_plot.plot(pen=pg.mkPen(width=2))
        plots_layout.addWidget(self.vel_plot)

        self.temp_plot = pg.PlotWidget(title="Temperature")
        self.temp_plot.setLabel("left", "Temperature", units="°C")
        self.temp_plot.setLabel("bottom", "Time", units="s")
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen(width=2))
        plots_layout.addWidget(self.temp_plot)

        layout.addLayout(plots_layout)

        self.times = []
        self.alt_hist = []
        self.vel_hist = []
        self.temp_hist = []

        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_view)
        self.timer.start()

    def update_view(self):
        t = self.telemetry.timestamp
        alt = self.telemetry.altitude
        vel = self.telemetry.airspeed
        temp = self.telemetry.temperature

        self.alt_gauge.set_value(alt)
        self.vel_gauge.set_value(vel)
        self.temp_gauge.set_value(temp)

        self.times.append(t)
        self.alt_hist.append(alt)
        self.vel_hist.append(vel)
        self.temp_hist.append(temp)
        if len(self.times) > 500:
            self.times = self.times[-500:]
            self.alt_hist = self.alt_hist[-500:]
            self.vel_hist = self.vel_hist[-500:]
            self.temp_hist = self.temp_hist[-500:]

        self.alt_curve.setData(self.times, self.alt_hist)
        self.vel_curve.setData(self.times, self.vel_hist)
        self.temp_curve.setData(self.times, self.temp_hist)


# ========== 回放視窗（Replay） ==========

class ReplayWindow(QMainWindow):
    """讀取 CSV 記錄檔，播放 Alt / Vel / Temp 曲線"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Flight log replay")
        self.resize(1000, 600)

        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.info_label = QLabel("尚未載入檔案")
        layout.addWidget(self.info_label)

        plots_layout = QHBoxLayout()
        pg.setConfigOptions(background="#020617", foreground="w")

        self.alt_plot = pg.PlotWidget(title="Altitude (replay)")
        self.alt_plot.setLabel("left", "Altitude", units="m")
        self.alt_plot.setLabel("bottom", "Time", units="s")
        self.alt_curve = self.alt_plot.plot(pen=pg.mkPen(width=2))
        plots_layout.addWidget(self.alt_plot)

        self.vel_plot = pg.PlotWidget(title="Velocity (replay)")
        self.vel_plot.setLabel("left", "Velocity", units="m/s")
        self.vel_plot.setLabel("bottom", "Time", units="s")
        self.vel_curve = self.vel_plot.plot(pen=pg.mkPen(width=2))
        plots_layout.addWidget(self.vel_plot)

        self.temp_plot = pg.PlotWidget(title="Temperature (replay)")
        self.temp_plot.setLabel("left", "Temperature", units="°C")
        self.temp_plot.setLabel("bottom", "Time", units="s")
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen(width=2))
        plots_layout.addWidget(self.temp_plot)

        layout.addLayout(plots_layout)

        ctl_layout = QHBoxLayout()
        self.play_button = QPushButton("播放")
        self.play_button.setEnabled(False)
        self.play_button.clicked.connect(self.toggle_play)
        ctl_layout.addWidget(self.play_button)
        ctl_layout.addStretch()
        layout.addLayout(ctl_layout)

        self.times = []
        self.alt = []
        self.vel = []
        self.temp = []
        self._idx = 0
        self._playing = False

        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.step)

    def load_csv(self, filename: str):
        self.times.clear()
        self.alt.clear()
        self.vel.clear()
        self.temp.clear()
        self._idx = 0
        self._playing = False
        self.timer.stop()
        try:
            with open(filename, "r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.times.append(float(row.get("time_s", 0.0)))
                    self.alt.append(float(row.get("altitude_m", 0.0)))
                    self.vel.append(float(row.get("airspeed_mps", 0.0)))
                    temp_str = row.get("temperature_C") or row.get("temperature", "0.0")
                    self.temp.append(float(temp_str))
            if self.times:
                self.info_label.setText(f"已載入：{os.path.basename(filename)}，共 {len(self.times)} 筆")
                self.play_button.setEnabled(True)
                self.update_curves(full=True)
            else:
                self.info_label.setText("檔案無資料")
        except Exception as e:
            self.info_label.setText(f"讀取失敗：{e}")

    def update_curves(self, full=False):
        if full:
            self.alt_curve.setData(self.times, self.alt)
            self.vel_curve.setData(self.times, self.vel)
            self.temp_curve.setData(self.times, self.temp)
        else:
            t = self.times[: self._idx]
            a = self.alt[: self._idx]
            v = self.vel[: self._idx]
            tm = self.temp[: self._idx]
            self.alt_curve.setData(t, a)
            self.vel_curve.setData(t, v)
            self.temp_curve.setData(t, tm)

    def toggle_play(self):
        if not self.times:
            return
        self._playing = not self._playing
        if self._playing:
            self.play_button.setText("暫停")
            self._idx = 0
            self.timer.start()
        else:
            self.play_button.setText("播放")
            self.timer.stop()

    def step(self):
        if not self._playing:
            return
        self._idx += 1
        if self._idx >= len(self.times):
            self._playing = False
            self.play_button.setText("播放")
            self.timer.stop()
            return
        self.update_curves(full=False)


# ========== 任務規劃視窗 ==========

class VideoReceiver(QThread):
    """UDP 影像接收執行緒，接收樹莓派傳來的已畫框 JPEG 影像"""
    frame_received = pyqtSignal(bytes)
    status_changed = pyqtSignal(str)

    def __init__(self, host="0.0.0.0", port=5600):
        super().__init__()
        self.host = host
        self.port = port
        self._running = False

    def run(self):
        self._running = True
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(2.0)
        try:
            sock.bind((self.host, self.port))
            self.status_changed.emit(f"監聽 UDP {self.host}:{self.port} ...")
        except Exception as e:
            self.status_changed.emit(f"❌ 綁定失敗：{e}")
            return

        buf = b""
        while self._running:
            try:
                data, _ = sock.recvfrom(65535)
                if len(data) < 4:
                    continue
                # 封包格式：4 bytes 總長度 + N bytes JPEG 資料（分片重組）
                header = data[:4]
                total_size = struct.unpack(">I", header)[0]
                buf += data[4:]
                if len(buf) >= total_size:
                    self.frame_received.emit(buf[:total_size])
                    buf = b""
            except socket.timeout:
                continue
            except Exception as e:
                self.status_changed.emit(f"接收錯誤：{e}")
                buf = b""

        sock.close()
        self.status_changed.emit("影像串流已停止。")

    def stop(self):
        self._running = False
        self.wait()


class VideoWindow(QMainWindow):
    """影像顯示視窗：顯示從樹莓派 UDP 接收到的 YOLOv8 已辨識影像"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("影像辨識顯示 Video Feed")
        self.resize(860, 580)
        self._receiver = None

        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # 頂部控制列
        ctrl_row = QHBoxLayout()
        port_label = QLabel("UDP Port:")
        port_label.setStyleSheet("color: #9ca3af;")
        self.port_input = QLineEdit("5600")
        self.port_input.setFixedWidth(80)
        self.start_btn = QPushButton("開始接收")
        self.start_btn.setStyleSheet("background-color: #15803d; color: white;")
        self.start_btn.clicked.connect(self.toggle_stream)
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setStyleSheet("background-color: #b91c1c; color: white;")
        self.stop_btn.clicked.connect(self.stop_stream)
        self.stop_btn.setEnabled(False)
        ctrl_row.addWidget(port_label)
        ctrl_row.addWidget(self.port_input)
        ctrl_row.addWidget(self.start_btn)
        ctrl_row.addWidget(self.stop_btn)
        ctrl_row.addStretch()
        layout.addLayout(ctrl_row)

        # 影像顯示區
        self.image_label = QLabel("尚未接收到影像\n請確認樹莓派已啟動 pi_inference.py")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: #111827; color: #6b7280; font-size: 14px;")
        self.image_label.setMinimumSize(800, 480)
        layout.addWidget(self.image_label)

        # 狀態列
        self.status_label = QLabel("就緒。")
        self.status_label.setStyleSheet("color: #9ca3af; font-size: 11px;")
        layout.addWidget(self.status_label)

    def toggle_stream(self):
        if self._receiver and self._receiver.isRunning():
            return
        try:
            port = int(self.port_input.text().strip())
        except ValueError:
            port = 5600
        self._receiver = VideoReceiver(host="0.0.0.0", port=port)
        self._receiver.frame_received.connect(self._on_frame)
        self._receiver.status_changed.connect(self.status_label.setText)
        self._receiver.start()
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_stream(self):
        if self._receiver:
            self._receiver.stop()
            self._receiver = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def _on_frame(self, jpeg_bytes: bytes):
        try:
            img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if frame is None:
                return
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            qt_image = QImage(frame_rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(
                pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        except Exception as e:
            self.status_label.setText(f"影像解碼錯誤：{e}")

    def closeEvent(self, event):
        self.stop_stream()
        super().closeEvent(event)


class MissionPlannerWindow(QMainWindow):
    """任務規劃視窗：航點清單 + 地圖點選 + Waypoint 上傳"""

    def __init__(self, link, parent=None):
        super().__init__(parent)
        self.link = link
        self.waypoints = []  # list of {'lat': float, 'lon': float, 'alt': float}
        self.setWindowTitle("任務規劃 Mission Planner")
        self.resize(900, 650)

        central = QWidget(self)
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(8, 8, 8, 8)

        # 左側：航點清單 + 控制
        left_panel = QVBoxLayout()
        left_panel.setSpacing(6)

        title = QLabel("航點清單 Waypoints")
        title.setStyleSheet("color: #9ca3af; font-weight: bold;")
        left_panel.addWidget(title)

        self.wp_list = QTextEdit()
        self.wp_list.setReadOnly(True)
        self.wp_list.setMinimumWidth(260)
        left_panel.addWidget(self.wp_list)

        # 手動加入航點
        add_title = QLabel("手動加入航點")
        add_title.setStyleSheet("color: #9ca3af;")
        left_panel.addWidget(add_title)

        form_layout = QGridLayout()
        form_layout.addWidget(QLabel("緯度 Lat:"), 0, 0)
        self.lat_input = QLineEdit("25.0300")
        form_layout.addWidget(self.lat_input, 0, 1)
        form_layout.addWidget(QLabel("經度 Lon:"), 1, 0)
        self.lon_input = QLineEdit("121.5600")
        form_layout.addWidget(self.lon_input, 1, 1)
        form_layout.addWidget(QLabel("高度 Alt (m):"), 2, 0)
        self.alt_input = QLineEdit("30")
        form_layout.addWidget(self.alt_input, 2, 1)
        left_panel.addLayout(form_layout)

        btn_row1 = QHBoxLayout()
        add_wp_btn = QPushButton("加入航點")
        add_wp_btn.clicked.connect(self.add_waypoint)
        del_wp_btn = QPushButton("刪除最後一點")
        del_wp_btn.clicked.connect(self.delete_last_waypoint)
        btn_row1.addWidget(add_wp_btn)
        btn_row1.addWidget(del_wp_btn)
        left_panel.addLayout(btn_row1)

        btn_row2 = QHBoxLayout()
        clear_btn = QPushButton("清除所有航點")
        clear_btn.clicked.connect(self.clear_waypoints)
        btn_row2.addWidget(clear_btn)
        left_panel.addLayout(btn_row2)

        upload_btn = QPushButton("上傳任務到飛控")
        upload_btn.setStyleSheet("background-color: #1d4ed8; color: white; padding: 6px;")
        upload_btn.clicked.connect(self.upload_mission)
        left_panel.addWidget(upload_btn)

        self.status_label = QLabel("就緒。")
        self.status_label.setWordWrap(True)
        left_panel.addWidget(self.status_label)
        left_panel.addStretch()

        main_layout.addLayout(left_panel, 1)

        # 右側：地圖（支援點擊加入航點）
        right_panel = QVBoxLayout()
        map_title = QLabel("地圖（點擊可加入航點）")
        map_title.setStyleSheet("color: #9ca3af; font-weight: bold;")
        right_panel.addWidget(map_title)

        self.map_view = QWebEngineView()
        self.map_view.setHtml(self._mission_map_html())
        # 使用 QWebChannel 讓地圖點擊傳回座標
        right_panel.addWidget(self.map_view)

        main_layout.addLayout(right_panel, 2)

    def _mission_map_html(self):
        return """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
<style>html,body{height:100%;margin:0;}#map{width:100%;height:100%;}</style>
</head>
<body>
<div id="map"></div>
<script>
var map = L.map('map').setView([25.03, 121.56], 15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);
var markers = [];
var polyline = L.polyline([], {color:'cyan'}).addTo(map);

map.on('click', function(e){
    var lat = e.latlng.lat.toFixed(6);
    var lon = e.latlng.lng.toFixed(6);
    // 傳給 Python：用 title 模擬（因沒有 QWebChannel，用 window.location 觸發）
    window.location.hash = 'wp:' + lat + ',' + lon;
});

function addMarker(lat, lon, idx) {
    var m = L.marker([lat, lon]).addTo(map);
    m.bindTooltip('WP' + idx, {permanent:true, direction:'right'});
    markers.push(m);
    var lls = markers.map(function(mk){ return mk.getLatLng(); });
    polyline.setLatLngs(lls);
}
function clearMarkers() {
    markers.forEach(function(m){ map.removeLayer(m); });
    markers = [];
    polyline.setLatLngs([]);
}
window.addMarker = addMarker;
window.clearMarkers = clearMarkers;
</script>
</body>
</html>"""

    def _refresh_list(self):
        lines = []
        for i, wp in enumerate(self.waypoints):
            lines.append(f"WP{i+1}: Lat={wp['lat']:.6f}, Lon={wp['lon']:.6f}, Alt={wp['alt']:.1f}m")
        self.wp_list.setPlainText("\n".join(lines) if lines else "（尚無航點）")
        # 同步更新地圖
        self.map_view.page().runJavaScript("clearMarkers();")
        for i, wp in enumerate(self.waypoints):
            js = f"addMarker({wp['lat']}, {wp['lon']}, {i+1});"
            self.map_view.page().runJavaScript(js)

    def add_waypoint(self):
        try:
            lat = float(self.lat_input.text().strip())
            lon = float(self.lon_input.text().strip())
            alt = float(self.alt_input.text().strip())
        except ValueError:
            self.status_label.setText("⚠ 請輸入有效的數字座標。")
            return
        self.waypoints.append({'lat': lat, 'lon': lon, 'alt': alt})
        self._refresh_list()
        self.status_label.setText(f"已加入 WP{len(self.waypoints)}：({lat:.5f}, {lon:.5f}, {alt}m)")

    def delete_last_waypoint(self):
        if self.waypoints:
            self.waypoints.pop()
            self._refresh_list()
            self.status_label.setText(f"已刪除最後一個航點，剩餘 {len(self.waypoints)} 點。")

    def clear_waypoints(self):
        self.waypoints.clear()
        self._refresh_list()
        self.status_label.setText("已清除所有航點。")

    def upload_mission(self):
        if not self.waypoints:
            self.status_label.setText("⚠ 航點清單為空，無法上傳。")
            return
        try:
            self.link.send_waypoints(self.waypoints)
            self.status_label.setText(f"✅ 已上傳 {len(self.waypoints)} 個航點到飛控。")
        except Exception as e:
            self.status_label.setText(f"❌ 上傳失敗：{e}")


# ========== 主視窗 Ground Station ==========

class GroundStationWindow(QMainWindow):
    """地面站主視窗（FakeLink / SerialLink + 回放 + 儀表 + 地圖）"""

    def __init__(self):
        super().__init__()

        self.setWindowTitle("PyQt5 UAV Ground Station - Advanced")
        self.resize(1400, 800)

        self.telemetry = Telemetry()
        self.link_mode = "FAKE"  # FAKE / MAVLINK_SERIAL / MAVLINK_UDP
        self.link = FakeLink(self.telemetry)
        self.connected = False

        self.logging = False
        self.log_file = None
        self.csv_writer = None
        self.sample_count = 0

        # UI 物件指標
        self.status_label = None
        self.conn_led_label = None
        self.connect_button = None
        self.footer_label = None
        self.system_id_label = None
        self.component_id_label = None
        self.vehicle_type_label = None
        self.heartbeat_label = None
        self.link_health_label = None
        self.udp_host_input = None
        self.udp_port_input = None
        self.timeout_input = None
        self.auto_reconnect_check = None
        self.port_input = None
        self.baud_input = None
        self.video_status_label = None
        self.video_start_btn = None
        self.video_stop_btn = None
        self.video_port_input = None
        self.video_label = None

        self.time_label = None
        self.roll_label = None
        self.pitch_label = None
        self.yaw_label = None
        self.alt_label = None
        self.airspeed_label = None
        self.lat_label = None
        self.lon_label = None

        self.att_widget = None
        self.att_text_label = None

        self.map_widget = None
        self.log_text = None

        self.logging_status_label = None
        self.logging_button = None
        self.log_led_label = None

        self.reset_map_button = None

        self.telem_stack = None
        self.alt_icon_value_label = None
        self.airspeed_icon_value_label = None
        self.att_icon_value_label = None
        self.pos_icon_value_label = None
        self.batt_icon_value_label = None
        self.gps_icon_value_label = None
        self.mode_icon_value_label = None
        self.telem_toggle_button = None

        self.charts_window = None
        self.replay_window = None
        self.mission_window = None
        self.video_window = None
        self.ui_tick = 0
        self._video_receiver = None
        self.heartbeat_timeout_sec = 5.0
        self.heartbeat_emit_enabled = True
        self.last_gcs_heartbeat_sent = 0.0
        self.last_link_health = "Unknown"
        self.link_lost_popup_active = False
        self.auto_reconnect_enabled = True
        self.reconnect_attempting = False
        self.reconnect_count = 0

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start()

    # ----- UI 佈局 -----

    def _build_ui(self):
        menu_bar = QMenuBar(self)
        self.setMenuBar(menu_bar)

        file_menu = menu_bar.addMenu("檔案")
        file_menu.addAction("離開").triggered.connect(self.close)

        view_menu = menu_bar.addMenu("檢視")
        view_menu.addAction("開啟飛行監控圖表").triggered.connect(self.open_charts_window)

        tools_menu = menu_bar.addMenu("工具")
        tools_menu.addAction("切換為 FakeLink").triggered.connect(self.set_fake_link_mode)
        tools_menu.addAction("切換為 MAVLink").triggered.connect(self.set_serial_link_mode)
        tools_menu.addAction("開啟飛行記錄回放").triggered.connect(self.open_replay)
        tools_menu.addAction("任務規劃").triggered.connect(self.open_mission_planner)
        tools_menu.addAction("影像辨識顯示").triggered.connect(self.open_video_window)

        status_bar = QStatusBar(self)
        self.setStatusBar(status_bar)
        self.footer_label = QLabel("就緒。")
        status_bar.addPermanentWidget(self.footer_label)

        central = QWidget(self)
        self.setCentralWidget(central)

        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # 左：連線 + 地平儀
        left = QVBoxLayout()
        left.setSpacing(8)

        conn_box = QFrame()
        conn_box.setFrameShape(QFrame.StyledPanel)
        conn_layout = QHBoxLayout(conn_box)
        conn_layout.setContentsMargins(8, 8, 8, 8)
        conn_layout.setSpacing(8)

        self.conn_led_label = QLabel("●")
        self.conn_led_label.setStyleSheet("color: #ef4444; font-size: 14px;")
        conn_layout.addWidget(self.conn_led_label)

        self.status_label = QLabel("未連線 (模式: FakeLink)")
        conn_layout.addWidget(self.status_label)
        conn_layout.addStretch()

        # Port 輸入框
        port_label = QLabel("Port:")
        conn_layout.addWidget(port_label)
        self.port_input = QLineEdit("COM5")
        self.port_input.setFixedWidth(80)
        conn_layout.addWidget(self.port_input)

        # Baud 輸入框
        baud_label = QLabel("Baud:")
        conn_layout.addWidget(baud_label)
        self.baud_input = QLineEdit("115200")
        self.baud_input.setFixedWidth(80)
        conn_layout.addWidget(self.baud_input)

        self.connect_button = QPushButton("連線")
        self.connect_button.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_button)

        left.addWidget(conn_box)

        att_box = QFrame()
        att_box.setFrameShape(QFrame.StyledPanel)
        att_layout = QVBoxLayout(att_box)
        att_layout.setContentsMargins(8, 8, 8, 8)
        att_layout.setSpacing(4)

        att_title = QLabel("人工地平儀（Attitude）")
        att_title.setStyleSheet("color: #9ca3af; font-weight: bold;")
        att_layout.addWidget(att_title)

        self.att_widget = AttitudeWidget()
        att_layout.addWidget(self.att_widget)

        self.att_text_label = QLabel("Roll: 0.0°  |  Pitch: 0.0°")
        self.att_text_label.setAlignment(Qt.AlignCenter)
        att_layout.addWidget(self.att_text_label)

        left.addWidget(att_box)
        left.addStretch()

        # 飛行指令控制面板
        ctrl_box = QFrame()
        ctrl_box.setFrameShape(QFrame.StyledPanel)
        ctrl_layout = QVBoxLayout(ctrl_box)
        ctrl_layout.setContentsMargins(8, 8, 8, 8)
        ctrl_layout.setSpacing(4)

        ctrl_title = QLabel("飛行指令控制")
        ctrl_title.setStyleSheet("color: #9ca3af; font-weight: bold;")
        ctrl_layout.addWidget(ctrl_title)

        arm_row = QHBoxLayout()
        arm_btn = QPushButton("ARM")
        arm_btn.setStyleSheet("background-color: #15803d; color: white;")
        arm_btn.clicked.connect(lambda: self._send_cmd(lambda: self.link.send_arm(True), "ARM 指令已送出"))
        disarm_btn = QPushButton("DISARM")
        disarm_btn.setStyleSheet("background-color: #b91c1c; color: white;")
        disarm_btn.clicked.connect(lambda: self._send_cmd(lambda: self.link.send_arm(False), "DISARM 指令已送出"))
        arm_row.addWidget(arm_btn)
        arm_row.addWidget(disarm_btn)
        ctrl_layout.addLayout(arm_row)

        takeoff_row = QHBoxLayout()
        takeoff_btn = QPushButton("起飛 10m")
        takeoff_btn.clicked.connect(lambda: self._send_cmd(lambda: self.link.send_takeoff(10.0), "起飛指令已送出 (10m)"))
        land_btn = QPushButton("降落")
        land_btn.clicked.connect(lambda: self._send_cmd(lambda: self.link.send_land(), "降落指令已送出"))
        takeoff_row.addWidget(takeoff_btn)
        takeoff_row.addWidget(land_btn)
        ctrl_layout.addLayout(takeoff_row)

        mode_row = QHBoxLayout()
        mode_label = QLabel("模式:")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["MANUAL", "STABILIZE", "LOITER", "AUTO", "RTL"])
        mode_switch_btn = QPushButton("切換")
        mode_switch_btn.clicked.connect(self._switch_mode)
        mode_row.addWidget(mode_label)
        mode_row.addWidget(self.mode_combo)
        mode_row.addWidget(mode_switch_btn)
        ctrl_layout.addLayout(mode_row)

        left.addWidget(ctrl_box)

        # 影像辨識顯示區塊
        video_box = QFrame()
        video_box.setFrameShape(QFrame.StyledPanel)
        video_layout = QVBoxLayout(video_box)
        video_layout.setContentsMargins(8, 8, 8, 8)
        video_layout.setSpacing(4)

        video_title = QLabel("影像辨識顯示")
        video_title.setStyleSheet("color: #9ca3af; font-weight: bold;")
        video_layout.addWidget(video_title)

        # Port 輸入與控制按鈕
        video_ctrl_row = QHBoxLayout()
        video_port_label = QLabel("Port:")
        video_port_label.setStyleSheet("color: #9ca3af;")
        self.video_port_input = QLineEdit("5600")
        self.video_port_input.setFixedWidth(60)
        self.video_start_btn = QPushButton("接收")
        self.video_start_btn.setStyleSheet("background-color: #15803d; color: white;")
        self.video_start_btn.clicked.connect(self._start_video_stream)
        self.video_stop_btn = QPushButton("停止")
        self.video_stop_btn.setStyleSheet("background-color: #b91c1c; color: white;")
        self.video_stop_btn.clicked.connect(self._stop_video_stream)
        self.video_stop_btn.setEnabled(False)
        video_ctrl_row.addWidget(video_port_label)
        video_ctrl_row.addWidget(self.video_port_input)
        video_ctrl_row.addWidget(self.video_start_btn)
        video_ctrl_row.addWidget(self.video_stop_btn)
        video_layout.addLayout(video_ctrl_row)

        # 影像顯示 Label
        self.video_label = QLabel("尚未接收到影像")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #111827; color: #6b7280; font-size: 11px;")
        self.video_label.setFixedHeight(200)
        video_layout.addWidget(self.video_label)

        # 狀態文字
        self.video_status_label = QLabel("就緒。")
        self.video_status_label.setStyleSheet("color: #9ca3af; font-size: 10px;")
        video_layout.addWidget(self.video_status_label)

        left.addWidget(video_box)

        left.addStretch()

        # 右側主區
        right = QVBoxLayout()
        right.setSpacing(8)

        # 上：Telemetry + Logging
        upper = QHBoxLayout()
        upper.setSpacing(8)

        # Telemetry 卡片（圖示 / 詳細）
        telem_box = QFrame()
        telem_box.setFrameShape(QFrame.StyledPanel)
        telem_layout = QVBoxLayout(telem_box)
        telem_layout.setContentsMargins(8, 8, 8, 8)
        telem_layout.setSpacing(4)

        telem_title = QLabel("即時飛行資料（Telemetry）")
        telem_title.setStyleSheet("color: #9ca3af; font-weight: bold;")
        telem_layout.addWidget(telem_title)

        page_icon = QWidget()
        icon_layout = QGridLayout(page_icon)
        icon_layout.setContentsMargins(4, 4, 4, 4)
        icon_layout.setSpacing(6)

        def make_icon_card(icon_text: str, title: str):
            card = QFrame()
            card.setFrameShape(QFrame.StyledPanel)
            v = QVBoxLayout(card)
            v.setContentsMargins(6, 6, 6, 6)
            v.setSpacing(2)

            icon_label = QLabel(icon_text)
            icon_label.setAlignment(Qt.AlignCenter)
            icon_label.setStyleSheet("font-size: 22px;")
            v.addWidget(icon_label)

            title_label = QLabel(title)
            title_label.setAlignment(Qt.AlignCenter)
            title_label.setStyleSheet("color: #9ca3af;")
            v.addWidget(title_label)

            value_label = QLabel("--")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setStyleSheet("font-size: 16px; font-weight: bold;")
            v.addWidget(value_label)

            return card, value_label

        alt_card, self.alt_icon_value_label = make_icon_card("⛰", "高度 Altitude")
        airspeed_card, self.airspeed_icon_value_label = make_icon_card("💨", "空速 Airspeed")
        att_card, self.att_icon_value_label = make_icon_card("🛫", "姿態 Attitude")
        pos_card, self.pos_icon_value_label = make_icon_card("📍", "位置 Position")
        batt_card, self.batt_icon_value_label = make_icon_card("🔋", "電池 Battery")
        gps_card, self.gps_icon_value_label = make_icon_card("📡", "GPS Sats")
        mode_card, self.mode_icon_value_label = make_icon_card("🎛", "模式 Mode")

        icon_layout.addWidget(alt_card, 0, 0)
        icon_layout.addWidget(airspeed_card, 0, 1)
        icon_layout.addWidget(att_card, 1, 0)
        icon_layout.addWidget(pos_card, 1, 1)
        icon_layout.addWidget(batt_card, 2, 0)
        icon_layout.addWidget(gps_card, 2, 1)
        icon_layout.addWidget(mode_card, 3, 0, 1, 2)

        page_detail = QWidget()
        grid = QGridLayout(page_detail)
        grid.setContentsMargins(4, 4, 4, 4)
        grid.setSpacing(2)

        def add_row(row, text):
            label = QLabel(text)
            value = QLabel("0.00")
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(label, row, 0)
            grid.addWidget(value, row, 1)
            return value

        self.time_label = add_row(0, "時間 t (s)")
        self.roll_label = add_row(1, "滾轉角 Roll (deg)")
        self.pitch_label = add_row(2, "俯仰角 Pitch (deg)")
        self.yaw_label = add_row(3, "偏航角 Yaw (deg)")
        self.alt_label = add_row(4, "高度 Altitude (m)")
        self.airspeed_label = add_row(5, "空速 Airspeed (m/s)")
        self.lat_label = add_row(6, "緯度 Lat (deg)")
        self.lon_label = add_row(7, "經度 Lon (deg)")

        self.telem_stack = QStackedWidget()
        self.telem_stack.addWidget(page_icon)
        self.telem_stack.addWidget(page_detail)

        telem_layout.addWidget(self.telem_stack)

        self.telem_toggle_button = QPushButton("切換為詳細列表")
        self.telem_toggle_button.clicked.connect(self.toggle_telem_page)
        telem_layout.addWidget(self.telem_toggle_button)

        upper.addWidget(telem_box, 2)

        # Logging 卡片
        log_ctrl_box = QFrame()
        log_ctrl_box.setFrameShape(QFrame.StyledPanel)
        log_ctrl_layout = QVBoxLayout(log_ctrl_box)
        log_ctrl_layout.setContentsMargins(8, 8, 8, 8)
        log_ctrl_layout.setSpacing(4)

        log_ctrl_title = QLabel("資料記錄控制")
        log_ctrl_title.setStyleSheet("color: #9ca3af; font-weight: bold;")
        log_ctrl_layout.addWidget(log_ctrl_title)

        row_status = QHBoxLayout()
        self.log_led_label = QLabel("●")
        self.log_led_label.setStyleSheet("color: #6b7280; font-size: 14px;")
        row_status.addWidget(self.log_led_label)

        self.logging_status_label = QLabel("目前未記錄")
        self.logging_status_label.setWordWrap(True)
        row_status.addWidget(self.logging_status_label)
        log_ctrl_layout.addLayout(row_status)

        self.logging_button = QPushButton("開始記錄")
        self.logging_button.clicked.connect(self.toggle_logging)
        log_ctrl_layout.addWidget(self.logging_button)

        upper.addWidget(log_ctrl_box, 1)

        right.addLayout(upper)

        # 中：地圖
        map_box = QFrame()
        map_box.setFrameShape(QFrame.StyledPanel)
        map_layout = QVBoxLayout(map_box)
        map_layout.setContentsMargins(8, 8, 8, 8)
        map_layout.setSpacing(4)

        map_title = QLabel("地圖 / 飛行軌跡（Leaflet OSM）")
        map_title.setStyleSheet("color: #9ca3af; font-weight: bold;")
        map_layout.addWidget(map_title)

        self.map_widget = WebEngineMapWidget("map.html")
        self.map_widget.setMinimumHeight(260)
        map_layout.addWidget(self.map_widget)

        btn_row_map = QHBoxLayout()
        btn_row_map.addStretch()
        self.reset_map_button = QPushButton("重置軌跡")
        self.reset_map_button.clicked.connect(self.on_reset_map_clicked)
        btn_row_map.addWidget(self.reset_map_button)
        map_layout.addLayout(btn_row_map)

        right.addWidget(map_box)

        # 下：Log 視窗
        log_box = QFrame()
        log_box.setFrameShape(QFrame.StyledPanel)
        log_layout = QVBoxLayout(log_box)
        log_layout.setContentsMargins(8, 8, 8, 8)
        log_layout.setSpacing(4)

        log_title = QLabel("系統訊息 / Log")
        log_title.setStyleSheet("color: #9ca3af; font-weight: bold;")
        log_layout.addWidget(log_title)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)

        btn_row_log = QHBoxLayout()
        btn_row_log.addStretch()
        clear_log_button = QPushButton("清除訊息")
        clear_log_button.clicked.connect(self.log_text.clear)
        btn_row_log.addWidget(clear_log_button)
        log_layout.addLayout(btn_row_log)

        right.addWidget(log_box)

        root.addLayout(left, 1)
        root.addLayout(right, 2)

    # ----- 工具函數 -----

    def log(self, text: str):
        self.log_text.append(text)

    def set_footer(self, text: str):
        self.footer_label.setText(text)

    def on_reset_map_clicked(self):
        if self.map_widget:
            self.map_widget.clear_track()
            self.log("🗺️ 地圖軌跡已清除。")
            self.set_footer("地圖軌跡已重置。")

    # ----- 飛行指令輔助方法 -----

    def _send_cmd(self, cmd_func, log_msg: str):
        if not self.connected:
            self.log("⚠ 尚未連線，無法送出指令。")
            return
        try:
            cmd_func()
            self.log(f"✈ {log_msg}")
            self.set_footer(log_msg)
        except Exception as e:
            self.log(f"❌ 指令送出失敗：{e}")

    def _switch_mode(self):
        mode = self.mode_combo.currentText()
        self._send_cmd(lambda: self.link.send_set_mode(mode), f"模式切換指令已送出：{mode}")

    # ----- Link 模式切換 -----

    def set_fake_link_mode(self):
        if self.connected:
            self.log("請先中斷連線再切換模式。")
            return
        self.link_mode = "FAKE"
        self.link = FakeLink(self.telemetry)
        self.status_label.setText("未連線 (模式: FakeLink)")
        self.set_footer("模式已切換為 FakeLink。")

    def set_serial_link_mode(self):
        if self.connected:
           self.log("請先中斷連線再切換模式。")
           return

        self.link_mode = "MAVLINK"
        port = self.port_input.text().strip() or "COM5"
        try:
            baud = int(self.baud_input.text().strip())
        except ValueError:
            baud = 115200
        self.link = MavlinkLink(self.telemetry, connection_string=port, baudrate=baud)
        self.status_label.setText("未連線 (模式: MAVLink)")
        self.set_footer(f"模式已切換為 MAVLink ({port}, {baud})。")

    # ----- 連線控制 -----

    def toggle_connection(self):
        if not self.connected:
            # 若為 MAVLink 模式，重建 link 以確保使用最新的 port/baud 設定
            if self.link_mode == "MAVLINK":
                port = self.port_input.text().strip() or "COM5"
                try:
                    baud = int(self.baud_input.text().strip())
                except ValueError:
                    baud = 115200
                self.link = MavlinkLink(self.telemetry, connection_string=port, baudrate=baud)

            self.connected = True
            self.status_label.setText(f"已連線（模式: {self.link_mode}）")
            self.connect_button.setText("中斷連線")
            if self.conn_led_label:
                self.conn_led_label.setStyleSheet("color: #22c55e; font-size: 14px;")
            print("目前 link 類型是：", type(self.link))
            self.link.start()
            self.log(f"✅ 已連線，開始接收 {self.link_mode} 資料。")
            self.set_footer("已連線。")
        else:
            self.connected = False
            self.status_label.setText(f"未連線 (模式: {self.link_mode})")
            self.connect_button.setText("連線")
            if self.conn_led_label:
                self.conn_led_label.setStyleSheet("color: #ef4444; font-size: 14px;")
            self.link.stop()
            self.log("🔌 已中斷連線。")
            self.set_footer("已中斷連線。")
            if self.logging:
                self.stop_logging()

    # ----- Logging -----

    def toggle_logging(self):
        if not self.logging:
            self.start_logging()
        else:
            self.stop_logging()

    def start_logging(self):
        if not self.connected:
            self.log("⚠ 無法開始記錄：尚未連線。")
            self.set_footer("無法開始記錄：尚未連線。")
            return

        if self.logging:
            return

        filename = datetime.now().strftime("telemetry_%Y%m%d_%H%M%S.csv")
        start_time_str = datetime.now().strftime("%H:%M:%S")

        try:
            self.log_file = open(filename, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow([
                "time_s",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "altitude_m",
                "airspeed_mps",
                "lat_deg",
                "lon_deg",
                "temperature_C",
                "battery_pct",
                "gps_sats",
                "flight_mode",
            ])
            self.log_file.flush()
        except Exception as e:
            self.log(f"❌ 開啟記錄檔失敗：{e}")
            self.set_footer("開啟記錄檔失敗。")
            self.log_file = None
            self.csv_writer = None
            return

        self.sample_count = 0
        self.logging = True
        self.logging_button.setText("停止記錄")
        self.logging_status_label.setText(
            f"正在記錄：{filename}\n已寫入樣本：0 筆"
        )
        if self.log_led_label:
            self.log_led_label.setStyleSheet("color: #f97316; font-size: 14px;")
        self.log(f"💾 {start_time_str} 開始記錄到檔案：{filename}")
        self.set_footer(f"記錄中：{filename}")

    def stop_logging(self):
        if not self.logging:
            return

        end_time_str = datetime.now().strftime("%H:%M:%S")
        total_samples = self.sample_count

        self.logging = False
        self.logging_button.setText("開始記錄")
        self.logging_status_label.setText(
            f"目前未記錄（上次共寫入 {total_samples} 筆）"
        )
        if self.log_led_label:
            self.log_led_label.setStyleSheet("color: #6b7280; font-size: 14px;")

        if self.log_file:
            try:
                self.log_file.flush()
                self.log_file.close()
                self.log(
                    f"💾 {end_time_str} 已停止記錄並關閉檔案，總樣本數：{total_samples} 筆。"
                )
                self.set_footer("已停止記錄。")
            except Exception as e:
                self.log(f"⚠ 關閉記錄檔時發生錯誤：{e}")

        self.log_file = None
        self.csv_writer = None

    # ----- Telemetry 頁面切換 -----

    def toggle_telem_page(self):
        index = self.telem_stack.currentIndex()
        if index == 0:
            self.telem_stack.setCurrentIndex(1)
            self.telem_toggle_button.setText("切換為圖示視圖")
        else:
            self.telem_stack.setCurrentIndex(0)
            self.telem_toggle_button.setText("切換為詳細列表")

    # ----- 子視窗 -----

    def open_charts_window(self):
        if self.charts_window is None:
            self.charts_window = FlightChartsWindow(self.telemetry, self)
        self.charts_window.show()
        self.charts_window.raise_()
        self.charts_window.activateWindow()

    def open_replay(self):
        if self.replay_window is None:
            self.replay_window = ReplayWindow(self)
        fname, _ = QFileDialog.getOpenFileName(
            self, "開啟記錄檔", "", "CSV Files (*.csv);;All Files (*)"
        )
        if fname:
            self.replay_window.load_csv(fname)
            self.replay_window.show()
            self.replay_window.raise_()
            self.replay_window.activateWindow()

    def open_mission_planner(self):
        if self.mission_window is None:
            self.mission_window = MissionPlannerWindow(self.link, self)
        self.mission_window.show()
        self.mission_window.raise_()
        self.mission_window.activateWindow()

    def open_video_window(self):
        if self.video_window is None:
            self.video_window = VideoWindow(self)
        self.video_window.show()
        self.video_window.raise_()
        self.video_window.activateWindow()

    # ----- 嵌入影像串流方法 -----

    def _start_video_stream(self):
        if self._video_receiver and self._video_receiver.isRunning():
            return
        try:
            port = int(self.video_port_input.text().strip())
        except ValueError:
            port = 5600
        self._video_receiver = VideoReceiver(host="0.0.0.0", port=port)
        self._video_receiver.frame_received.connect(self._on_video_frame)
        self._video_receiver.status_changed.connect(self.video_status_label.setText)
        self._video_receiver.start()
        self.video_start_btn.setEnabled(False)
        self.video_stop_btn.setEnabled(True)
        self.video_status_label.setText(f"監聴 UDP port {port} ...")

    def _stop_video_stream(self):
        if self._video_receiver:
            self._video_receiver.stop()
            self._video_receiver = None
        self.heartbeat_timeout_sec = 5.0
        self.heartbeat_emit_enabled = True
        self.last_gcs_heartbeat_sent = 0.0
        self.last_link_health = "Unknown"
        self.link_lost_popup_active = False
        self.auto_reconnect_enabled = True
        self.reconnect_attempting = False
        self.reconnect_count = 0
        self.video_start_btn.setEnabled(True)
        self.video_stop_btn.setEnabled(False)
        self.video_status_label.setText("已停止。")

    def _on_video_frame(self, jpeg_bytes: bytes):
        try:
            img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if frame is None:
                return
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            qt_image = QImage(frame_rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(
                pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        except Exception as e:
            self.video_status_label.setText(f"解碼錯誤：{e}")

    def closeEvent(self, event):
        self._stop_video_stream()
        super().closeEvent(event)

    # ----- 主更新 -----


    def _show_link_lost_popup(self, message: str):
        if self.link_lost_popup_active:
            return
        self.link_lost_popup_active = True
        QMessageBox.warning(self, "Link Lost", message)
        self.link_lost_popup_active = False

    def _restart_current_link(self):
        try:
            self.link.stop()
        except Exception:
            pass
        if self.link_mode == "MAVLINK_SERIAL":
            port = self.port_input.text().strip() or "COM5"
            try:
                baud = int(self.baud_input.text().strip())
            except ValueError:
                baud = 115200
            self.link = MavlinkLink(self.telemetry, connection_string=port, baudrate=baud)
        elif self.link_mode == "MAVLINK_UDP":
            host = self.udp_host_input.text().strip() or "0.0.0.0"
            try:
                port = int(self.udp_port_input.text().strip())
            except ValueError:
                port = 14550
            self.link = MavlinkLink(self.telemetry, connection_string=MavlinkLink.build_udp_connection(host, port))
        else:
            self.link = FakeLink(self.telemetry)
        self.link.start()
        self.reconnect_attempting = True
        self.reconnect_count += 1
        self.set_footer(f"嘗試重連第 {self.reconnect_count} 次")

    def update_ui(self):
        if not self.connected:
            return

        telem = self.telemetry.snapshot()

        # 這裡加上 tick 計數器，每次執行都 +1
        if not hasattr(self, 'ui_tick'):
            self.ui_tick = 0
        self.ui_tick += 1

        t = telem.timestamp
        if not all([
            self.time_label, self.roll_label, self.pitch_label, self.yaw_label,
            self.alt_label, self.airspeed_label, self.lat_label, self.lon_label,
            self.att_widget, self.att_text_label, self.status_label
        ]):
            return

        self.time_label.setText(f"{t:6.2f}")

        heartbeat_age = None
        if telem.last_heartbeat_monotonic > 0:
            heartbeat_age = time.monotonic() - telem.last_heartbeat_monotonic
        if heartbeat_age is not None and heartbeat_age <= self.heartbeat_timeout_sec:
            if self.conn_led_label:
                self.conn_led_label.setStyleSheet("color: #22c55e; font-size: 14px")
            link_health = "Healthy"
        elif heartbeat_age is not None and heartbeat_age <= self.heartbeat_timeout_sec * 2:
            if self.conn_led_label:
                self.conn_led_label.setStyleSheet("color: #f59e0b; font-size: 14px")
            link_health = "Heartbeat delayed"
        else:
            if self.conn_led_label:
                self.conn_led_label.setStyleSheet("color: #ef4444; font-size: 14px")
            link_health = "Heartbeat timeout"
        self.roll_label.setText(f"{telem.roll:6.2f}")
        self.pitch_label.setText(f"{telem.pitch:6.2f}")
        self.yaw_label.setText(f"{telem.yaw:6.2f}")
        self.alt_label.setText(f"{telem.altitude:6.2f}")
        self.airspeed_label.setText(f"{telem.airspeed:6.2f}")
        self.lat_label.setText(f"{telem.lat:.6f}")
        self.lon_label.setText(f"{telem.lon:.6f}")
        (self.system_id_label.setText if self.system_id_label else (lambda *a, **k: None))(str(telem.system_id))
        (self.component_id_label.setText if self.component_id_label else (lambda *a, **k: None))(str(telem.component_id))
        (self.vehicle_type_label.setText if self.vehicle_type_label else (lambda *a, **k: None))(telem.vehicle_type)
        hb_text = telem.last_heartbeat_text
        if telem.last_heartbeat_monotonic > 0:
            age = time.monotonic() - telem.last_heartbeat_monotonic
            hb_text = f"{telem.last_heartbeat_text} ({age:.1f}s ago)"
        (self.heartbeat_label.setText if self.heartbeat_label else (lambda *a, **k: None))(hb_text)
        (self.link_health_label.setText if self.link_health_label else (lambda *a, **k: None))(link_health)

        if isinstance(self.link, MavlinkLink) and self.heartbeat_emit_enabled:
            now_monotonic = time.monotonic()
            if now_monotonic - self.last_gcs_heartbeat_sent >= 1.0:
                self.link.send_heartbeat()
                self.last_gcs_heartbeat_sent = now_monotonic

        if link_health == "Heartbeat timeout":
            self.status_label.setText(f"⚠️ {self.link_mode} 連線逾時")
            if self.last_link_health != "Heartbeat timeout":
                self.log("偵測到 heartbeat timeout")
                self._show_link_lost_popup(f"{self.link_mode} 已失去心跳，請檢查 Doodle Labs / companion computer / 飛控資料流。")
                if self.auto_reconnect_enabled and self.link_mode in ("MAVLINK_SERIAL", "MAVLINK_UDP"):
                    self._restart_current_link()
        elif link_health == "Heartbeat delayed":
            self.status_label.setText(f"🟠 {self.link_mode} 心跳延遲")
        else:
            if self.reconnect_attempting:
                self.set_footer(f"重連成功，共嘗試 {self.reconnect_count} 次")
                self.reconnect_attempting = False
            self.status_label.setText(f"🟢 {self.link_mode} 連線正常")
        self.last_link_health = link_health

        # 圖示卡片
        if self.alt_icon_value_label:
            self.alt_icon_value_label.setText(f"{telem.altitude:5.1f} m")
        if self.airspeed_icon_value_label:
            self.airspeed_icon_value_label.setText(f"{telem.airspeed:5.1f} m/s")
        if self.att_icon_value_label:
            self.att_icon_value_label.setText(
                f"R {telem.roll:4.0f}° / P {telem.pitch:4.0f}°"
            )
        if self.pos_icon_value_label:
            self.pos_icon_value_label.setText(
                f"{telem.lat:.4f}, {telem.lon:.4f}"
            )
        if self.batt_icon_value_label:
            self.batt_icon_value_label.setText(f"{telem.battery:4.0f}%")
        if self.gps_icon_value_label:
            self.gps_icon_value_label.setText(f"{telem.gps_sats} sats")
        if self.mode_icon_value_label:
            self.mode_icon_value_label.setText(telem.flight_mode)

        # 2D 地平儀 (每次都更新，保持滑順)
        if self.att_widget:
            self.att_widget.set_attitude(telem.roll, telem.pitch)
        if self.att_text_label:
            self.att_text_label.setText(
            f"Roll: {telem.roll:5.1f}°  |  Pitch: {telem.pitch:5.1f}°"
        )
        
        # === 以下是降頻優化的部分 ===

        # 地圖：每 10 次 tick 才更新一次，避免 JavaScript 卡死 UI 執行緒
        if self.ui_tick % 10 == 0:
            try:
                if self.map_widget:
                    self.map_widget.update_marker(telem.lat, telem.lon)
            except Exception as e:
                self.log(f"🗺️ 地圖更新錯誤：{e}")

        # Logging 寫檔：每 3 次 tick 才寫入一次，且移除 flush() 減少 IO 阻塞
        if self.logging and self.csv_writer is not None and self.ui_tick % 3 == 0:
            try:
                self.csv_writer.writerow([
                    f"{t:.3f}",
                    f"{telem.roll:.3f}",
                    f"{telem.pitch:.3f}",
                    f"{telem.yaw:.3f}",
                    f"{telem.altitude:.3f}",
                    f"{telem.airspeed:.3f}",
                    f"{telem.lat:.6f}",
                    f"{telem.lon:.6f}",
                    f"{telem.temperature:.3f}",
                    f"{telem.battery:.1f}",
                    f"{telem.gps_sats:d}",
                    telem.flight_mode,
                ])
                
                self.sample_count += 1
                self.logging_status_label.setText(
                    f"正在記錄：樣本數 {self.sample_count} 筆"
                )
            except Exception as e:
                self.log(f"⚠ 寫入 CSV 發生錯誤：{e}")
                self.stop_logging()


# ========== Dark Theme & main() ==========

def setup_dark_palette(app: QApplication):
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor("#0f172a"))
    palette.setColor(QPalette.Base, QColor("#020617"))
    palette.setColor(QPalette.AlternateBase, QColor("#111827"))
    palette.setColor(QPalette.Text, QColor("#e5e7eb"))
    palette.setColor(QPalette.WindowText, QColor("#e5e7eb"))
    palette.setColor(QPalette.Button, QColor("#1f2937"))
    palette.setColor(QPalette.ButtonText, QColor("#e5e7eb"))
    palette.setColor(QPalette.Highlight, QColor("#22c55e"))
    palette.setColor(QPalette.HighlightedText, QColor("#020617"))
    app.setPalette(palette)


def main():
    ensure_map_html("map.html")

    app = QApplication(sys.argv)
    setup_dark_palette(app)

    app.setStyleSheet("""
        QMainWindow {
            background-color: #020617;
        }
        QFrame {
            background-color: #020617;
            border: 1px solid #1f2937;
            border-radius: 6px;
        }
        QLabel {
            color: #e5e7eb;
        }
        QTextEdit {
            background-color: #020617;
            color: #e5e7eb;
            border: 1px solid #1f2937;
            border-radius: 6px;
        }
        QPushButton {
            background-color: #1f2937;
            color: #e5e7eb;
            border: 1px solid #4b5563;
            border-radius: 4px;
            padding: 4px 10px;
        }
        QPushButton:hover {
            background-color: #374151;
        }
        QPushButton:pressed {
            background-color: #111827;
        }
        QStatusBar {
            background-color: #020617;
            color: #e5e7eb;
        }
        QMenuBar {
            background-color: #020617;
            color: #e5e7eb;
        }
        QMenuBar::item:selected {
            background-color: #111827;
        }
        QMenu {
            background-color: #020617;
            color: #e5e7eb;
        }
        QMenu::item:selected {
            background-color: #111827;
        }
    """)

    win = GroundStationWindow()
    win.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
