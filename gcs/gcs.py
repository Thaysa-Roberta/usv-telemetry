"""
gcs.py – Estação de Controle em Solo (Ground Control Station)
--------------------------------------------------------------
Interface gráfica para monitoramento e controle do USV via MAVLink/TCP.

Dados exibidos:
  • Posição GPS (latitude, longitude, altitude)
  • Orientação IMU (roll, pitch, yaw em tempo real)
  • Status da conexão e heartbeat
  • Log de eventos

Controles:
  • Sliders de velocidade linear e angular → /cmd_vel via RC_CHANNELS_OVERRIDE
  • Botão de parada de emergência

Uso:
    python gcs.py
    python gcs.py --host 192.168.1.50 --port 14550
"""

import sys
import time
import math
import argparse
import threading
from collections import deque

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox, QGridLayout,
    QTextEdit, QSplitter, QLCDNumber, QLineEdit
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject, QUrl
from PyQt6.QtGui import QFont, QColor, QPalette
from PyQt6.QtWebEngineWidgets import QWebEngineView

import pyqtgraph as pg

try:
    from pymavlink import mavutil
    MAVLINK_OK = True
except ImportError:
    MAVLINK_OK = False

# ──────────────────────────────────────────────────────────────────────────────
# Worker de comunicação MAVLink (thread separada)
# ──────────────────────────────────────────────────────────────────────────────

class MavlinkWorker(QObject):
    """Recebe mensagens MAVLink e emite sinais Qt."""

    sig_gps      = pyqtSignal(float, float, float)   # lat, lon, alt
    sig_attitude = pyqtSignal(float, float, float)   # roll, pitch, yaw (rad)
    sig_status   = pyqtSignal(str)                   # texto de status
    sig_log      = pyqtSignal(str)                   # linha de log
    sig_connected = pyqtSignal(bool)

    def __init__(self, host: str, port: int):
        super().__init__()
        self.host = host
        self.port = port
        self._mav = None
        self._running = False
        self._connected = False

    def start(self):
        self._running = True
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def stop(self):
        self._running = False

    def _run(self):
        try:
            while self._running:
                if not self._connected:
                    self._connect()
                else:
                    self._receive()
        except RuntimeError:
            pass  # Qt object deleted — thread encerrada normalmente

    def _connect(self):
        if not MAVLINK_OK:
            self.sig_log.emit('[AVISO] pymavlink não instalado — modo demo.')
            self._connected = True
            self.sig_connected.emit(False)
            time.sleep(2)
            return
        try:
            self.sig_log.emit(f'Conectando a {self.host}:{self.port}...')
            self._mav = mavutil.mavlink_connection(
                f'tcp:{self.host}:{self.port}',
                source_system=255,
                source_component=0
            )
            self._mav.wait_heartbeat(timeout=5)
            self._connected = True
            self.sig_connected.emit(True)
            self.sig_log.emit('GCS conectada ao USV ✓')
        except Exception as e:
            self.sig_log.emit(f'Falha na conexão: {e}. Tentando novamente em 3 s...')
            time.sleep(3)

    def _receive(self):
        if not MAVLINK_OK or self._mav is None:
            time.sleep(1)
            return
        try:
            msg = self._mav.recv_match(
                type=['GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT', 'STATUSTEXT'],
                blocking=True, timeout=2.0
            )
            if msg is None:
                return
            t = msg.get_type()
            if t == 'GLOBAL_POSITION_INT':
                self.sig_gps.emit(
                    msg.lat / 1e7,
                    msg.lon / 1e7,
                    msg.alt / 1000.0
                )
            elif t == 'ATTITUDE':
                self.sig_attitude.emit(msg.roll, msg.pitch, msg.yaw)
            elif t == 'STATUSTEXT':
                self.sig_log.emit(f'[USV] {msg.text}')
        except RuntimeError:
            self._running = False  # Qt object deleted, encerra thread
        except Exception as e:
            self._connected = False
            self._mav = None
            try:
                self.sig_log.emit(f'Erro na recepção: {e}')
                self.sig_connected.emit(False)
            except RuntimeError:
                self._running = False

    def send_rc(self, throttle: int, yaw: int):
        """Envia RC_CHANNELS_OVERRIDE: canal 3=throttle, canal 4=yaw."""
        if not MAVLINK_OK or self._mav is None or not self._connected:
            return
        try:
            self._mav.mav.rc_channels_override_send(
                self._mav.target_system,
                self._mav.target_component,
                0, 0,           # ch1, ch2
                throttle,       # ch3 – velocidade linear
                yaw,            # ch4 – velocidade angular
                0, 0, 0, 0      # ch5–ch8
            )
        except Exception as e:
            self.sig_log.emit(f'Erro ao enviar RC: {e}')


# ──────────────────────────────────────────────────────────────────────────────
# Janela principal
# ──────────────────────────────────────────────────────────────────────────────

class GCSWindow(QMainWindow):

    def __init__(self, host: str, port: int):
        super().__init__()
        self.setWindowTitle('USV – Estação de Controle em Solo (GCS)')
        self.resize(1100, 720)

        self._worker = MavlinkWorker(host, port)
        self._worker.sig_gps.connect(self._on_gps)
        self._worker.sig_attitude.connect(self._on_attitude)
        self._worker.sig_log.connect(self._on_log)
        self._worker.sig_connected.connect(self._on_connection_change)

        # Buffers para gráficos (últimos 200 pontos)
        n = 200
        self._t_buf   = deque([0.0] * n, maxlen=n)
        self._roll_buf  = deque([0.0] * n, maxlen=n)
        self._pitch_buf = deque([0.0] * n, maxlen=n)
        self._yaw_buf   = deque([0.0] * n, maxlen=n)
        self._t0 = time.time()

        self._build_ui()
        self._worker.start()

        # Timer de atualização dos gráficos (20 Hz)
        self._plot_timer = QTimer()
        self._plot_timer.timeout.connect(self._update_plots)
        self._plot_timer.start(50)

    # ── Construção da UI ──────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(splitter)

        # Painel esquerdo: telemetria + controles
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.addWidget(self._build_status_group())
        left_layout.addWidget(self._build_gps_group())
        left_layout.addWidget(self._build_control_group())
        left_layout.addWidget(self._build_log_group())
        splitter.addWidget(left)

        # Painel direito: gráficos de atitude + trilha GPS
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(self._build_attitude_group())
        right_layout.addWidget(self._build_gps_track_group())
        splitter.addWidget(right)

        splitter.setSizes([420, 660])

    def _build_status_group(self):
        grp = QGroupBox('Conexão')
        layout = QHBoxLayout(grp)
        self._lbl_status = QLabel('● Desconectado')
        self._lbl_status.setStyleSheet('color: red; font-weight: bold;')
        layout.addWidget(self._lbl_status)
        return grp

    def _build_gps_group(self):
        grp = QGroupBox('GPS / Posição')
        grid = QGridLayout(grp)
        font_val = QFont('Courier', 12)
        style = ('QLineEdit { background: white; color: #1a1a1a; '
                 'border: 1px solid #aaa; border-radius: 3px; padding: 2px 6px; }')

        def _make_field():
            f = QLineEdit('---')
            f.setReadOnly(True)
            f.setAlignment(Qt.AlignmentFlag.AlignRight)
            f.setFont(font_val)
            f.setStyleSheet(style)
            return f

        self._lcd_lat = _make_field()
        self._lcd_lon = _make_field()
        self._lcd_alt = _make_field()

        grid.addWidget(QLabel('Latitude:'),     0, 0)
        grid.addWidget(self._lcd_lat,           0, 1)
        grid.addWidget(QLabel('Longitude:'),    1, 0)
        grid.addWidget(self._lcd_lon,           1, 1)
        grid.addWidget(QLabel('Altitude (m):'), 2, 0)
        grid.addWidget(self._lcd_alt,           2, 1)
        return grp

    def _build_gps_track_group(self):
        grp = QGroupBox('Mapa de Posição GPS')
        layout = QVBoxLayout(grp)
        grp.setMaximumHeight(260)

        self._map_view = QWebEngineView()
        self._map_ready = False
        self._pending_pos: tuple | None = None
        self._map_view.loadFinished.connect(self._on_map_loaded)
        self._map_view.setHtml(self._leaflet_html(-30.0595, -51.1732))
        layout.addWidget(self._map_view)
        return grp

    def _on_map_loaded(self, ok: bool):
        self._map_ready = ok
        if ok and self._pending_pos:
            lat, lon = self._pending_pos
            self._map_view.page().runJavaScript(f'updatePos({lat}, {lon});')
            self._pending_pos = None

    @staticmethod
    def _leaflet_html(lat: float, lon: float) -> str:
        return f"""<!DOCTYPE html>
<html><head>
<meta charset='utf-8'/>
<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>
<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>
<style>html,body,#map{{margin:0;padding:0;height:100%;}}</style>
</head><body>
<div id='map'></div>
<script>
  var map = L.map('map').setView([{lat},{lon}], 16);
  L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
    attribution: '&copy; OpenStreetMap contributors'
  }}).addTo(map);
  var marker = L.circleMarker([{lat},{lon}], {{
    radius: 10, color: '#e74c3c', fillColor: '#e74c3c', fillOpacity: 0.9
  }}).addTo(map);
  function updatePos(lat, lon) {{
    marker.setLatLng([lat, lon]);
    map.panTo([lat, lon]);
  }}
</script>
</body></html>"""

    def _build_attitude_group(self):
        grp = QGroupBox('Atitude IMU (Roll / Pitch / Yaw) – graus')
        layout = QVBoxLayout(grp)

        pg.setConfigOptions(antialias=True)
        self._plot_widget = pg.GraphicsLayoutWidget()

        colors = {'Roll': '#e74c3c', 'Pitch': '#2ecc71', 'Yaw': '#3498db'}
        self._curves = {}
        self._plots  = {}

        for i, (name, color) in enumerate(colors.items()):
            p = self._plot_widget.addPlot(row=i, col=0, title=name)
            p.setLabel('left', 'graus')
            p.setLabel('bottom', 's')
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setYRange(-180, 180)
            curve = p.plot(pen=pg.mkPen(color=color, width=2))
            self._curves[name] = curve
            self._plots[name]  = p

        layout.addWidget(self._plot_widget)
        return grp

    def _build_control_group(self):
        grp = QGroupBox('Controle de Movimento')
        layout = QVBoxLayout(grp)

        # Throttle (velocidade linear)
        h1 = QHBoxLayout()
        h1.addWidget(QLabel('Velocidade:'))
        self._sld_throttle = QSlider(Qt.Orientation.Horizontal)
        self._sld_throttle.setRange(1000, 2000)
        self._sld_throttle.setValue(1500)
        self._sld_throttle.setTickInterval(100)
        self._sld_throttle.valueChanged.connect(self._send_command)
        h1.addWidget(self._sld_throttle)
        self._lbl_throttle = QLabel('1500')
        h1.addWidget(self._lbl_throttle)
        layout.addLayout(h1)

        # Yaw (velocidade angular)
        h2 = QHBoxLayout()
        h2.addWidget(QLabel('Direção:  '))
        self._sld_yaw = QSlider(Qt.Orientation.Horizontal)
        self._sld_yaw.setRange(1000, 2000)
        self._sld_yaw.setValue(1500)
        self._sld_yaw.setTickInterval(100)
        self._sld_yaw.valueChanged.connect(self._send_command)
        h2.addWidget(self._sld_yaw)
        self._lbl_yaw = QLabel('1500')
        h2.addWidget(self._lbl_yaw)
        layout.addLayout(h2)

        # Botão de emergência
        btn_stop = QPushButton('⛔  PARADA DE EMERGÊNCIA')
        btn_stop.setStyleSheet(
            'background-color: #c0392b; color: white; '
            'font-weight: bold; font-size: 14px; padding: 8px;'
        )
        btn_stop.clicked.connect(self._emergency_stop)
        layout.addWidget(btn_stop)

        return grp

    def _build_log_group(self):
        grp = QGroupBox('Log de Eventos')
        layout = QVBoxLayout(grp)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumHeight(120)
        self._log.setFont(QFont('Courier', 9))
        layout.addWidget(self._log)
        return grp

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_gps(self, lat: float, lon: float, alt: float):
        self._lcd_lat.setText(f'{lat:.6f}')
        self._lcd_lon.setText(f'{lon:.6f}')
        self._lcd_alt.setText(f'{alt:.1f}')
        if self._map_ready:
            self._map_view.page().runJavaScript(f'updatePos({lat}, {lon});')
        else:
            self._pending_pos = (lat, lon)

    def _on_attitude(self, roll: float, pitch: float, yaw: float):
        t = time.time() - self._t0
        self._t_buf.append(t)
        self._roll_buf.append(math.degrees(roll))
        self._pitch_buf.append(math.degrees(pitch))
        self._yaw_buf.append(math.degrees(yaw))

    def _update_plots(self):
        t = list(self._t_buf)
        self._curves['Roll'].setData(t, list(self._roll_buf))
        self._curves['Pitch'].setData(t, list(self._pitch_buf))
        self._curves['Yaw'].setData(t, list(self._yaw_buf))

    def _on_log(self, msg: str):
        self._log.append(f'[{time.strftime("%H:%M:%S")}] {msg}')

    def _on_connection_change(self, connected: bool):
        if connected:
            self._lbl_status.setText('● Conectado')
            self._lbl_status.setStyleSheet('color: #27ae60; font-weight: bold;')
        else:
            self._lbl_status.setText('● Desconectado')
            self._lbl_status.setStyleSheet('color: red; font-weight: bold;')

    def _send_command(self):
        throttle = self._sld_throttle.value()
        yaw      = self._sld_yaw.value()
        self._lbl_throttle.setText(str(throttle))
        self._lbl_yaw.setText(str(yaw))
        self._worker.send_rc(throttle, yaw)

    def _emergency_stop(self):
        self._sld_throttle.setValue(1500)
        self._sld_yaw.setValue(1500)
        self._worker.send_rc(1500, 1500)
        self._on_log('⛔ PARADA DE EMERGÊNCIA ativada')

    def closeEvent(self, event):
        self._worker.stop()
        super().closeEvent(event)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='USV Ground Control Station')
    parser.add_argument('--host', default='127.0.0.1',
                        help='IP do USV / container Docker (padrão: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=14550,
                        help='Porta MAVLink TCP (padrão: 14550)')
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = GCSWindow(args.host, args.port)
    window.show()
    sys.exit(app.exec())
