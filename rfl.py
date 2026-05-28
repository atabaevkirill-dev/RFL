#!/usr/bin/env python3
"""
3Km Eye-Safe Laser Ranging Module — Desktop Control Application
Real hardware via serial port, auto-detect ports, UART TTL 3.3V
"""

import sys
import math
import struct
import threading
import socket
from datetime import datetime
from collections import deque

import serial
import serial.tools.list_ports

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QLabel, QComboBox, QSpinBox,
    QTextEdit, QGroupBox, QFrame, QSlider, QTabWidget, QLineEdit, QStackedWidget, QRadioButton, QButtonGroup
)
from PyQt6.QtCore import Qt, QTimer, QPointF, pyqtSignal, QObject, QUrl
from PyQt6.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QPixmap, QImage,
    QLinearGradient, QConicalGradient, QPainterPath
)

# Попытка импортировать QtMultimedia для камеры
try:
    from PyQt6.QtMultimedia import QCamera, QMediaDevice, QCaptureSession
    from PyQt6.QtMultimediaWidgets import QGraphicsVideoItem
    HAS_MULTIMEDIA = True
except ImportError:
    HAS_MULTIMEDIA = False

# Для HTTP-потока камеры
import urllib.request

# ── Colours ──────────────────────────────────────────────────────────────────
BG_DARK      = QColor("#0A0E17")
BG_PANEL     = QColor("#131C2E")
ACCENT_CYAN  = QColor("#00D4FF")
ACCENT_GREEN = QColor("#00FF88")
ACCENT_RED   = QColor("#FF3B5C")
ACCENT_AMBER = QColor("#FFB830")
TEXT_BRIGHT  = QColor("#E8F4FD")
TEXT_DIM     = QColor("#4A6080")
GRID_LINE    = QColor("#1A2840")

# ── LRF Protocol helpers ─────────────────────────────────────────────────────
FRAME_HEAD = bytes([0xEE, 0x16])

def build_frame(cmd: int, params: list[int] = []) -> bytes:
    body = [0x03, cmd] + params
    length = len(body)  # Теперь длина включает Device code, Command code и Command parameters
    chk = sum(body) & 0xFF
    return bytes([0xEE, 0x16, length] + body + [chk])

CMD_SELF_CHECK  = build_frame(0x01)
CMD_SINGLE      = build_frame(0x02)
CMD_CONT_START  = build_frame(0x04)
CMD_STOP        = build_frame(0x05)

def cmd_set_mode(mode: int) -> bytes:        # 1=first 2=last 3=multi
    return build_frame(0x03, [mode])

def cmd_set_freq(hz: int) -> bytes:
    return build_frame(0xA1, [hz & 0xFF, 0x00])

def cmd_set_min_gate(dist: int) -> bytes:
    return build_frame(0xA2, [(dist >> 8) & 0xFF, dist & 0xFF])

def cmd_set_max_gate(dist: int) -> bytes:
    return build_frame(0xA4, [(dist >> 8) & 0xFF, dist & 0xFF])

def cmd_query_min_gate() -> bytes:
    return build_frame(0xA3)

def cmd_query_max_gate() -> bytes:
    return build_frame(0xA5)

def cmd_query_fpga_version() -> bytes:
    return build_frame(0xA6)

def cmd_query_mcu_version() -> bytes:
    return build_frame(0xA7)

def cmd_query_hw_version() -> bytes:
    return build_frame(0xA8)

def cmd_query_sn() -> bytes:
    return build_frame(0xA9)

def cmd_query_total_light_count() -> bytes:
    return build_frame(0x90)

def cmd_query_power_on_light_count() -> bytes:
    return build_frame(0x91)

def cmd_set_baud_rate(baud_code: int) -> bytes:
    # Преобразуем код скорости передачи в 4-байтовое значение
    baud_bytes = [
        (baud_code >> 24) & 0xFF,
        (baud_code >> 16) & 0xFF,
        (baud_code >> 8) & 0xFF,
        baud_code & 0xFF
    ]
    return build_frame(0xA0, baud_bytes)

def parse_response_frame(data: bytes) -> dict | None:
    """Parse a response frame according to the protocol specification."""
    if len(data) < 6:  # Минимальный размер заголовка
        return None
    if data[0] != 0xEE or data[1] != 0x16:
        return None
    
    length = data[2]
    if len(data) < length + 4:  # Заголовок (2) + длина (1) + тело + чек-сумма (1)
        return None

    cmd = data[4]
    params = data[5:2+length+1]  # Параметры после команды до чек-суммы
    received_checksum = data[2+length+1]

    # Проверяем чек-сумму
    calculated_checksum = sum(data[3:2+length+1]) & 0xFF
    if received_checksum != calculated_checksum:
        return None

    result = {
        'cmd': cmd,
        'params': params,
        'raw_data': data
    }

    # Обработка различных команд ответа
    if cmd == 0x01:  # Self-check response
        if len(params) >= 4:
            result['status3'] = params[0]
            result['status2'] = params[1]  # Эхо интенсивсность
            result['status1'] = params[2]  # Битовые флаги статуса
            result['status0'] = params[3]  # Статус питания
            
            # Расшифровка битовых флагов
            status1 = params[2]
            result['fpga_system_status'] = bool(status1 & 0x01)
            result['laser_light_output'] = bool(status1 & 0x02)
            result['main_wave_detection'] = bool(status1 & 0x04)
            result['echo_detection'] = bool(status1 & 0x08)
            result['bias_switch'] = bool(status1 & 0x10)
            result['bias_output'] = bool(status1 & 0x20)
            result['temperature_state'] = bool(status1 & 0x40)
            result['light_output_off'] = bool(status1 & 0x80)
            result['power_5v6_status'] = bool(params[3] & 0x01)
            
    elif cmd in (0x02, 0x04):  # Single or continuous ranging response
        if len(params) >= 4:
            result['status'] = params[0]
            hi, lo, dec = params[1], params[2], params[3]
            dist = hi * 256 + lo + dec * 0.1
            result['distance'] = dist

            # Определяем статус в зависимости от режима
            status_val = params[0]
            if status_val == 0x04:
                result['out_of_range'] = True
            else:
                result['out_of_range'] = False

    elif cmd == 0x03:  # Set mode response
        result['success'] = True

    elif cmd == 0x05:  # Stop ranging response
        result['stopped'] = True

    elif cmd == 0x06:  # Ranging anomaly
        if len(params) >= 4:
            result['reserved1'] = params[0]
            result['reserved2'] = params[1]
            result['reserved3'] = params[2]
            result['status1'] = params[3]  # Битовые флаги статуса

    elif cmd == 0xA1:  # Set frequency response
        result['frequency_set'] = True

    elif cmd in (0xA2, 0xA4):  # Set min/max gate response
        if len(params) >= 2:
            gate_distance = (params[0] << 8) | params[1]
            result['gate_distance'] = gate_distance

    elif cmd in (0xA3, 0xA5):  # Query min/max gate response
        if len(params) >= 2:
            gate_distance = (params[0] << 8) | params[1]
            result['gate_distance'] = gate_distance

    elif cmd in (0xA6, 0xA7):  # Query FPGA/MCU version response
        if len(params) >= 4:
            version_byte, date, month_year, author = params
            
            major = (version_byte >> 4) & 0x0F
            minor = version_byte & 0x0F
            month = (month_year >> 4) & 0x0F
            year = 2020 + (month_year & 0x0F)
            
            authors_fpga = {0x6C: "cliu", 0x5D: "dwu", 0xCC: "cycheng"}
            authors_mcu = {0x00: "jyang", 0xF1: "llfu", 0x01: "zqxiong"}
            authors = {**authors_fpga, **authors_mcu}
            
            result['version'] = f"V{major}.{minor}"
            result['date'] = date
            result['month'] = month
            result['year'] = year
            result['author'] = authors.get(author, f"Unknown(0x{author:02X})")

    elif cmd == 0xA8:  # Query HW version response
        if len(params) >= 4:
            mbvs, ctvs, apdvs, ldvs = params
            
            def decode_version(v):
                major = (v >> 4) & 0x0F
                minor = v & 0x0F
                return f"V{major}.{minor}"
            
            result['motherboard'] = decode_version(mbvs)
            result['control_board'] = decode_version(ctvs)
            result['detection_board'] = decode_version(apdvs)
            result['driver_board'] = decode_version(ldvs)

    elif cmd == 0xA9:  # Query SN response
        if len(params) >= 3:
            month_year, num_high, num_low = params
            
            month = (month_year >> 4) & 0x0F
            year = 2020 + (month_year & 0x0F)
            sn = (num_high << 8) | num_low
            
            result['month'] = month
            result['year'] = year
            result['serial_number'] = f"{year:04d}{month:02d}{sn:04d}"

    elif cmd in (0x90, 0x91):  # Query light count response
        if len(params) >= 3:
            result['light_count'] = (params[0] << 16) | (params[1] << 8) | params[2]

    return result

def parse_range_response(data: bytes) -> float | None:
    """Parse a ranging response, return distance in metres or None."""
    parsed = parse_response_frame(data)
    if parsed and 'distance' in parsed and not parsed.get('out_of_range', False):
        return parsed['distance']
    return None

# ── Serial/TCP worker (runs in thread) ───────────────────────────────────────
class CommWorker(QObject):
    distance_received = pyqtSignal(float)
    raw_frame         = pyqtSignal(bytes, bytes)   # tx, rx
    error             = pyqtSignal(str)
    connected         = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self._ser: serial.Serial | None = None
        self._sock: socket.socket | None = None
        self._is_tcp = False
        self._lock = threading.Lock()
        self._buf = bytearray()

    def connect_serial(self, port: str, baud: int) -> bool:
        try:
            self._ser = serial.Serial(port, baud, timeout=1.0)
            self._is_tcp = False
            self.connected.emit(True)
            return True
        except Exception as e:
            self.error.emit(f"Cannot open {port}: {e}")
            return False

    def connect_tcp(self, host: str, port: int) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(1.0)
            self._sock.connect((host, port))
            self._is_tcp = True
            self.connected.emit(True)
            return True
        except Exception as e:
            self.error.emit(f"Cannot connect to {host}:{port}: {e}")
            return False

    def disconnect(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        if self._sock:
            try:
                self._sock.close()
            except:
                pass
        self._ser = None
        self._sock = None
        self.connected.emit(False)

    def send(self, frame: bytes):
        if self._is_tcp:
            if not self._sock:
                self.error.emit("Not connected (TCP)")
                return
            with self._lock:
                self._sock.sendall(frame)
        else:
            if not self._ser or not self._ser.is_open:
                self.error.emit("Not connected (Serial)")
                return
            with self._lock:
                self._ser.write(frame)
                self._ser.flush()

    def send_and_recv(self, frame: bytes):
        """Send a command and collect the response frame."""
        if self._is_tcp:
            self._send_and_recv_tcp(frame)
        else:
            self._send_and_recv_serial(frame)

    def _send_and_recv_serial(self, frame: bytes):
        if not self._ser or not self._ser.is_open:
            self.error.emit("Not connected")
            return
        with self._lock:
            self._ser.write(frame)
            self._ser.flush()
            # Определяем ожидаемую длину ответа в зависимости от команды
            cmd = frame[3]  # Команда находится в 4-м байте (после заголовка и длины)
            
            # Для разных команд может потребоваться разная длина ответа
            if cmd in [0x01, 0x02, 0x04]:  # Self-check, single, continuous ranging
                expected_len = 10
            elif cmd in [0x03, 0x05]:  # Set mode, stop ranging
                expected_len = 6
            elif cmd in [0xA1]:  # Set frequency
                expected_len = 6
            elif cmd in [0xA2, 0xA3, 0xA4, 0xA5]:  # Gate commands
                expected_len = 8
            elif cmd in [0xA6, 0xA7, 0xA8]:  # Version queries
                expected_len = 10
            elif cmd == 0xA9:  # SN query
                expected_len = 9
            elif cmd in [0x90, 0x91]:  # Light count queries
                expected_len = 9
            else:
                expected_len = 10  # По умолчанию
                
            # Ждем ответ от устройства
            import time
            time.sleep(0.05)  # Небольшая задержка для стабильности
            raw = self._ser.read(expected_len)
            self.raw_frame.emit(frame, raw)
            
            # Парсим ответ и отправляем сигнал о расстоянии если это измерение
            parsed = parse_response_frame(raw)
            if parsed:
                if parsed['cmd'] in (0x02, 0x04) and 'distance' in parsed and not parsed.get('out_of_range', False):
                    self.distance_received.emit(parsed['distance'])
                elif parsed['cmd'] == 0x06:  # Anomaly
                    self.error.emit(f"Ranging anomaly detected: status=0x{parsed['status1']:02X}")

    def _send_and_recv_tcp(self, frame: bytes):
        if not self._sock:
            self.error.emit("Not connected (TCP)")
            return
        with self._lock:
            self._sock.sendall(frame)
            # Определяем ожидаемую длину ответа
            cmd = frame[3]
            
            if cmd in [0x01, 0x02, 0x04]:
                expected_len = 10
            elif cmd in [0x03, 0x05]:
                expected_len = 6
            elif cmd in [0xA1]:
                expected_len = 6
            elif cmd in [0xA2, 0xA3, 0xA4, 0xA5]:
                expected_len = 8
            elif cmd in [0xA6, 0xA7, 0xA8]:
                expected_len = 10
            elif cmd == 0xA9:
                expected_len = 9
            elif cmd in [0x90, 0x91]:
                expected_len = 9
            else:
                expected_len = 10
            
            import time
            time.sleep(0.05)
            
            # Читаем заголовок и длину
            header = bytearray()
            while len(header) < 3:
                try:
                    chunk = self._sock.recv(3 - len(header))
                    if not chunk:
                        self.error.emit("TCP connection closed")
                        return
                    header.extend(chunk)
                except socket.timeout:
                    self.error.emit("TCP read timeout")
                    return
            
            if header[0] != 0xEE or header[1] != 0x16:
                self.error.emit(f"Invalid TCP header: {header[:2].hex()}")
                return
            
            length = header[2]
            remaining = length + 1  # тело + чексумма
            
            raw = bytearray(header)
            while len(raw) < 3 + length + 1:
                try:
                    chunk = self._sock.recv(3 + length + 1 - len(raw))
                    if not chunk:
                        break
                    raw.extend(chunk)
                except socket.timeout:
                    break
            
            self.raw_frame.emit(frame, bytes(raw))
            
            parsed = parse_response_frame(bytes(raw))
            if parsed:
                if parsed['cmd'] in (0x02, 0x04) and 'distance' in parsed and not parsed.get('out_of_range', False):
                    self.distance_received.emit(parsed['distance'])
                elif parsed['cmd'] == 0x06:
                    self.error.emit(f"Ranging anomaly detected: status=0x{parsed['status1']:02X}")

    def read_pending(self):
        """Called by timer — drain incoming bytes and parse frames."""
        if self._is_tcp:
            self._read_pending_tcp()
        else:
            self._read_pending_serial()

    def _read_pending_serial(self):
        if not self._ser or not self._ser.is_open:
            return
        try:
            n = self._ser.in_waiting
            if n:
                self._buf += self._ser.read(n)
                
                # Обработка буфера по полным фреймам
                while len(self._buf) >= 6:  # Минимальная длина фрейма
                    # Ищем начало фрейма
                    idx = self._buf.find(b'\xEE\x16')
                    if idx < 0:
                        # Если начало фрейма не найдено, очищаем буфер до следующего возможного начала
                        if len(self._buf) > 100:  # Если буфер большой, очищаем его полностью
                            self._buf.clear()
                        break
                    if idx > 0:
                        # Удаляем байты до начала фрейма
                        self._buf = self._buf[idx:]
                    
                    # Проверяем, достаточно ли байтов для чтения длины
                    if len(self._buf) < 3:
                        break
                    
                    # Читаем длину пакета
                    length = self._buf[2]
                    expected_length = 3 + length + 1  # заголовок(2) + длина(1) + тело(length) + чек-сумма(1)
                    
                    # Проверяем, есть ли полный фрейм в буфере
                    if len(self._buf) < expected_length:
                        break
                    
                    # Извлекаем полный фрейм
                    frame = bytes(self._buf[:expected_length])
                    self._buf = self._buf[expected_length:]
                    
                    # Обработка ответа
                    parsed = parse_response_frame(frame)
                    if parsed:
                        # Обработка ответов на измерения
                        if parsed['cmd'] in (0x02, 0x04) and 'distance' in parsed and not parsed.get('out_of_range', False):
                            self.distance_received.emit(parsed['distance'])
                        
                        # Обработка аномальных ситуаций
                        elif parsed['cmd'] == 0x06:
                            self.error.emit(f"Ranging anomaly detected: status=0x{parsed['status1']:02X}")
                        
                        # Отправка фрейма в GUI для отображения
                        self.raw_frame.emit(b'', frame)
                    else:
                        # Если не удалось распарсить, отправляем необработанный фрейм для диагностики
                        self.raw_frame.emit(b'', frame)
        except Exception as e:
            self.error.emit(str(e))

    def _read_pending_tcp(self):
        if not self._sock:
            return
        try:
            self._sock.setblocking(False)
            try:
                data = self._sock.recv(4096)
                if not data:
                    self.error.emit("TCP connection closed")
                    self.disconnect()
                    return
                self._buf += data
                
                # Обработка буфера по полным фреймам
                while len(self._buf) >= 6:
                    idx = self._buf.find(b'\xEE\x16')
                    if idx < 0:
                        if len(self._buf) > 100:
                            self._buf.clear()
                        break
                    if idx > 0:
                        self._buf = self._buf[idx:]
                    
                    if len(self._buf) < 3:
                        break
                    
                    length = self._buf[2]
                    expected_length = 3 + length + 1
                    
                    if len(self._buf) < expected_length:
                        break
                    
                    frame = bytes(self._buf[:expected_length])
                    self._buf = self._buf[expected_length:]
                    
                    parsed = parse_response_frame(frame)
                    if parsed:
                        if parsed['cmd'] in (0x02, 0x04) and 'distance' in parsed and not parsed.get('out_of_range', False):
                            self.distance_received.emit(parsed['distance'])
                        elif parsed['cmd'] == 0x06:
                            self.error.emit(f"Ranging anomaly detected: status=0x{parsed['status1']:02X}")
                        self.raw_frame.emit(b'', frame)
                    else:
                        self.raw_frame.emit(b'', frame)
            except BlockingIOError:
                pass  # Нет данных
        except Exception as e:
            self.error.emit(str(e))

# ── Custom widgets ────────────────────────────────────────────────────────────
class CameraWidget(QWidget):
    """Виджет для отображения потока IP-камеры с центральной меткой цели"""
    
    frame_updated = pyqtSignal(QPixmap)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._camera_ip = "192.168.1.168"
        self._camera_user = "admin"
        self._camera_pass = "123456"
        self._current_pixmap = QPixmap()
        self._target_distance = 0.0
        self._max_dist = 4200.0
        
        # Таймер для обновления кадра
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._fetch_frame)
        self._update_timer.start(500)  # Обновление каждые 500мс
        
        # Метка для отображения расстояния
        self._distance_label = QLabel("0.0 m", self)
        self._distance_label.setStyleSheet(
            "color: #00FF88; font-size: 24px; font-weight: bold; "
            "background: rgba(10, 14, 23, 0.7); padding: 8px 16px; "
            "border-radius: 4px; border: 1px solid #00FF88;"
        )
        self._distance_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._distance_label.hide()
        
        # Метка цели в центре
        self._target_label = QLabel("⊕", self)
        self._target_label.setStyleSheet(
            "color: #FF3B5C; font-size: 32px; font-weight: bold;"
        )
        self._target_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
    def set_camera_credentials(self, ip: str, user: str, password: str):
        """Установить учетные данные камеры"""
        self._camera_ip = ip
        self._camera_user = user
        self._camera_pass = password
        self._log(f"Camera credentials updated: {ip}")
        
    def _fetch_frame(self):
        """Получить кадр с IP-камеры через HTTP"""
        try:
            # Формируем URL для MJPEG потока (типичный для многих IP-камер)
            # Для Hikvision/Dahua может потребоваться другой формат
            url = f"http://{self._camera_user}:{self._camera_pass}@{self._camera_ip}/Streaming/Channels/101"
            
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=2) as response:
                # Читаем данные изображения
                image_data = response.read()
                
                if image_data:
                    image = QImage()
                    if image.loadFromData(image_data):
                        pixmap = QPixmap.fromImage(image)
                        self._current_pixmap = pixmap.scaled(
                            self.size(), 
                            Qt.AspectRatioMode.KeepAspectRatio,
                            Qt.TransformationMode.SmoothTransformation
                        )
                        self.frame_updated.emit(self._current_pixmap)
                        self.update()
                        
        except Exception as e:
            # При ошибке показываем заглушку
            self._current_pixmap = QPixmap()
            self.update()
    
    def set_target_distance(self, distance: float):
        """Установить расстояние до цели"""
        self._target_distance = distance
        self._distance_label.setText(f"{distance:.1f} m")
        self._distance_label.show()
        self.update()
    
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Заполняем фон
        p.fillRect(self.rect(), BG_DARK)
        
        # Рисуем кадр камеры если есть
        if not self._current_pixmap.isNull():
            # Центрируем изображение
            px_rect = self._current_pixmap.rect()
            px_rect.moveCenter(self.rect().center())
            p.drawPixmap(px_rect, self._current_pixmap)
        else:
            # Показываем заглушку если нет изображения
            p.setPen(QPen(TEXT_DIM, 2))
            p.setFont(QFont("Courier New", 14))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, 
                      f"IP CAMERA\n{self._camera_ip}\nNo Signal")
        
        # Рисуем прицельную метку в центре
        cx, cy = self.width() // 2, self.height() // 2
        
        # Внешний круг
        p.setPen(QPen(ACCENT_RED, 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), 25, 25)
        
        # Внутренний круг
        p.setPen(QPen(ACCENT_RED, 1))
        p.drawEllipse(QPointF(cx, cy), 15, 15)
        
        # Перекрестие
        p.drawLine(cx - 30, cy, cx + 30, cy)
        p.drawLine(cx, cy - 30, cx, cy + 30)
        
        # Точка в центре
        p.setBrush(QBrush(ACCENT_RED))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), 3, 3)
        
        # Метка расстояния внизу
        if self._target_distance > 0:
            label_x = cx - 60
            label_y = cy + 50
            self._distance_label.setGeometry(label_x, label_y, 120, 40)
            self._distance_label.raise_()
        
        p.end()


class ChartWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(150)
        self._data: deque[float] = deque(maxlen=300)

    def push(self, v: float):
        self._data.append(v); self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        PL, PR, PT, PB = 52, 10, 10, 26
        p.fillRect(self.rect(), BG_PANEL)

        # grid
        p.setFont(QFont("Courier New", 7))
        max_val = 4200.0
        for i in range(5):
            y = PT + (h - PT - PB) * i / 4
            p.setPen(QPen(GRID_LINE, 1, Qt.PenStyle.DashLine))
            p.drawLine(PL, int(y), w - PR, int(y))
            p.setPen(QPen(TEXT_DIM))
            p.drawText(2, int(y) + 4, f"{int(max_val * (1 - i/4))}")

        if len(self._data) < 2:
            p.end(); return

        data = list(self._data)
        cw = w - PL - PR; ch = h - PT - PB

        def xy(i, v):
            return QPointF(PL + cw * i / (len(data) - 1),
                           PT + ch * (1 - v / max_val))

        path = QPainterPath()
        fill = QPainterPath()
        p0 = xy(0, data[0])
        path.moveTo(p0)
        fill.moveTo(QPointF(p0.x(), h - PB))
        fill.lineTo(p0)
        for i in range(1, len(data)):
            pt = xy(i, data[i])
            path.lineTo(pt); fill.lineTo(pt)
        fill.lineTo(QPointF(xy(len(data)-1, data[-1]).x(), h - PB))
        fill.closeSubpath()

        grad = QLinearGradient(0, PT, 0, h - PB)
        c1 = QColor(ACCENT_CYAN); c1.setAlpha(45)
        c2 = QColor(ACCENT_CYAN); c2.setAlpha(0)
        grad.setColorAt(0, c1); grad.setColorAt(1, c2)
        p.setBrush(QBrush(grad)); p.setPen(Qt.PenStyle.NoPen)
        p.drawPath(fill)

        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(ACCENT_CYAN, 2))
        p.drawPath(path)

        lp = xy(len(data)-1, data[-1])
        p.setBrush(QBrush(ACCENT_CYAN))
        p.drawEllipse(lp, 4, 4)
        p.end()


class BigDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(96)
        self._value = "----.-"

    def set_value(self, v: float):
        self._value = f"{v:.1f}"; self.update()

    def clear(self):
        self._value = "----.-"; self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), BG_PANEL)

        lf = QFont("Courier New", 9)
        lf.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 3)
        p.setFont(lf); p.setPen(QPen(TEXT_DIM))
        p.drawText(12, 20, "DISTANCE")

        vf = QFont("Courier New", 38, QFont.Weight.Bold)
        p.setFont(vf); p.setPen(QPen(ACCENT_CYAN))
        tw = p.fontMetrics().horizontalAdvance(self._value)
        p.drawText(12, h - 18, self._value)

        uf = QFont("Courier New", 15)
        p.setFont(uf); p.setPen(QPen(TEXT_DIM))
        p.drawText(14 + tw, h - 18, "m")

        grad = QLinearGradient(0, h-2, w, h-2)
        grad.setColorAt(0.0, QColor(0,0,0,0))
        grad.setColorAt(0.5, ACCENT_CYAN)
        grad.setColorAt(1.0, QColor(0,0,0,0))
        p.setPen(QPen(QBrush(grad), 2))
        p.drawLine(0, h-1, w, h-1)
        p.end()


class LEDLabel(QWidget):
    def __init__(self, text, color=None, parent=None):
        super().__init__(parent)
        self._text = text
        self._color = color or ACCENT_GREEN
        self._on = False
        self.setFixedHeight(24)

    def set_on(self, s: bool): self._on = s; self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        col = self._color if self._on else TEXT_DIM
        p.setBrush(QBrush(col)); p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(4, 6, 12, 12)
        p.setFont(QFont("Courier New", 8))
        p.setPen(QPen(TEXT_BRIGHT if self._on else TEXT_DIM))
        p.drawText(22, 17, self._text)
        p.end()


# ── Style ─────────────────────────────────────────────────────────────────────
STYLE = """
QWidget { background:#0A0E17; color:#E8F4FD; font-family:"Courier New",monospace; }
QGroupBox { border:1px solid #1A2840; border-radius:6px; margin-top:16px;
            padding-top:8px; font-size:9px; letter-spacing:2px; color:#4A6080; }
QGroupBox::title { subcontrol-origin:margin; left:10px; }
QPushButton { background:#111827; color:#00D4FF; border:1px solid #00D4FF;
              border-radius:4px; padding:7px 14px; font-size:10px; letter-spacing:1px; }
QPushButton:hover  { background:#00D4FF; color:#0A0E17; }
QPushButton:pressed{ background:#0099BB; color:#0A0E17; }
QPushButton:disabled { color:#2A3A50; border-color:#2A3A50; }
QPushButton#red  { color:#FF3B5C; border-color:#FF3B5C; }
QPushButton#red:hover  { background:#FF3B5C; color:#0A0E17; }
QPushButton#green{ color:#00FF88; border-color:#00FF88; }
QPushButton#green:hover{ background:#00FF88; color:#0A0E17; }
QComboBox { background:#111827; color:#E8F4FD; border:1px solid #1A2840;
            border-radius:4px; padding:4px 8px; }
QComboBox::drop-down { border:none; width:20px; }
QComboBox QAbstractItemView { background:#111827;
    selection-background-color:#00D4FF; selection-color:#0A0E17; }
QSpinBox { background:#111827; color:#E8F4FD;
           border:1px solid #1A2840; border-radius:4px; padding:4px 8px; }
QTextEdit { background:#070B12; color:#00D4FF; border:1px solid #1A2840;
            border-radius:4px; font-size:10px; }
QScrollBar:vertical { background:#0A0E17; width:8px; }
QScrollBar::handle:vertical { background:#1A2840; border-radius:4px; }
QStatusBar { color:#4A6080; font-size:9px; }
QSlider::groove:horizontal { background:#1A2840; height:4px; border-radius:2px; }
QSlider::handle:horizontal { background:#00D4FF; width:14px; height:14px;
                              margin:-5px 0; border-radius:7px; }
QSlider::sub-page:horizontal { background:#00D4FF; border-radius:2px; }
QTabWidget::pane { border:1px solid #1A2840; }
QTabBar::tab { background:#111827; color:#4A6080; padding:6px 14px;
               font-size:9px; letter-spacing:1px; }
QTabBar::tab:selected { color:#00D4FF; border-bottom:2px solid #00D4FF; }
"""


# ── Main Window ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("3KM EYE-SAFE LRF  ·  CONTROL PANEL")
        self.resize(1200, 820)
        self.setStyleSheet(STYLE)

        self._worker = CommWorker()
        self._worker.distance_received.connect(self._on_distance)
        self._worker.raw_frame.connect(self._on_raw_frame)
        self._worker.error.connect(self._on_error)
        self._worker.connected.connect(self._on_connected)

        self._connected = False
        self._continuous = False
        self._history: list[float] = []
        self._shot_count = 0
        self._connection_mode = "serial"  # "serial" или "tcp"

        # Timer to drain serial RX in continuous mode
        self._rx_timer = QTimer(self)
        self._rx_timer.setInterval(50)
        self._rx_timer.timeout.connect(self._worker.read_pending)

        self._build_ui()
        self._refresh_ports()
        self._log("Application started — select port and click CONNECT")

    # ── Build UI ──────────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        # ── LEFT ──────────────────────────────────────────────────────────────
        left = QVBoxLayout(); left.setSpacing(8)
        root.addLayout(left, 0)

        title = QLabel("◈ LRF-3KM")
        title.setFont(QFont("Courier New", 18, QFont.Weight.Bold))
        title.setStyleSheet("color:#00D4FF; letter-spacing:4px;")
        left.addWidget(title)
        sub = QLabel("EYE-SAFE · 1535 nm · CLASS I")
        sub.setStyleSheet("color:#4A6080; font-size:8px; letter-spacing:3px;")
        left.addWidget(sub)

        # Connection
        cb = QGroupBox("CONNECTION"); cl = QVBoxLayout(cb); cl.setSpacing(6)

        # Mode selection (Serial / TCP)
        mode_row = QHBoxLayout()
        self._serial_radio = QRadioButton("SERIAL")
        self._tcp_radio = QRadioButton("TCP/IP")
        self._serial_radio.setChecked(True)
        self._mode_group = QButtonGroup(self)
        self._mode_group.addButton(self._serial_radio)
        self._mode_group.addButton(self._tcp_radio)
        self._serial_radio.toggled.connect(self._on_mode_changed)
        mode_row.addWidget(self._serial_radio)
        mode_row.addWidget(self._tcp_radio)
        cl.addLayout(mode_row)

        # Serial settings
        self._serial_widget = QWidget()
        serial_layout = QVBoxLayout(self._serial_widget)
        serial_layout.setContentsMargins(0, 4, 0, 0)
        serial_layout.setSpacing(6)
        
        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("PORT"))
        self._port_cb = QComboBox(); self._port_cb.setMinimumWidth(130)
        port_row.addWidget(self._port_cb, 1)
        refresh_btn = QPushButton("↺")
        refresh_btn.setFixedWidth(30); refresh_btn.setToolTip("Refresh ports")
        refresh_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(refresh_btn)
        serial_layout.addLayout(port_row)

        baud_row = QHBoxLayout()
        baud_row.addWidget(QLabel("BAUD"))
        self._baud_cb = QComboBox()
        self._baud_cb.addItems(["115200", "57600", "9600"])
        baud_row.addWidget(self._baud_cb, 1)
        serial_layout.addLayout(baud_row)
        
        cl.addWidget(self._serial_widget)

        # TCP settings
        self._tcp_widget = QWidget()
        tcp_layout = QVBoxLayout(self._tcp_widget)
        tcp_layout.setContentsMargins(0, 4, 0, 0)
        tcp_layout.setSpacing(6)
        self._tcp_widget.setVisible(False)
        
        host_row = QHBoxLayout()
        host_row.addWidget(QLabel("HOST"))
        self._host_edit = QLineEdit("192.168.1.100")
        self._host_edit.setPlaceholderText("IP address")
        host_row.addWidget(self._host_edit, 1)
        tcp_layout.addLayout(host_row)
        
        tcpport_row = QHBoxLayout()
        tcpport_row.addWidget(QLabel("PORT"))
        self._tcp_port_spin = QSpinBox()
        self._tcp_port_spin.setRange(1, 65535)
        self._tcp_port_spin.setValue(8888)
        tcpport_row.addWidget(self._tcp_port_spin, 1)
        tcp_layout.addLayout(tcpport_row)
        
        cl.addWidget(self._tcp_widget)

        self._conn_btn = QPushButton("⚡  CONNECT")
        self._conn_btn.setObjectName("green")
        self._conn_btn.clicked.connect(self._toggle_connect)
        cl.addWidget(self._conn_btn)

        self._led_conn  = LEDLabel("CONNECTED",    ACCENT_GREEN)
        self._led_laser = LEDLabel("LASER ACTIVE", ACCENT_CYAN)
        self._led_echo  = LEDLabel("ECHO DETECTED",ACCENT_AMBER)
        cl.addWidget(self._led_conn)
        cl.addWidget(self._led_laser)
        cl.addWidget(self._led_echo)
        left.addWidget(cb)

        # Target mode
        mb = QGroupBox("TARGET MODE"); ml = QVBoxLayout(mb)
        self._btn_first = QPushButton("⊳  FIRST TARGET")
        self._btn_last  = QPushButton("⊲  LAST TARGET")
        self._btn_multi = QPushButton("⊳⊲ MULTI-TARGET")
        for btn, m in [(self._btn_first,1),(self._btn_last,2),(self._btn_multi,3)]:
            btn.clicked.connect(lambda _, mm=m: self._set_mode(mm))
            ml.addWidget(btn)
        self._active_mode = 1
        self._btn_first.setStyleSheet("background:#00D4FF;color:#0A0E17;border-radius:4px;padding:7px 14px;")
        left.addWidget(mb)

        # Gate
        gb = QGroupBox("RANGE GATE"); gl = QGridLayout(gb)
        gl.addWidget(QLabel("MIN (m)"), 0, 0)
        self._min_gate = QSpinBox(); self._min_gate.setRange(10,20000); self._min_gate.setValue(15)
        gl.addWidget(self._min_gate, 0, 1)
        gl.addWidget(QLabel("MAX (m)"), 1, 0)
        self._max_gate = QSpinBox(); self._max_gate.setRange(10,20000); self._max_gate.setValue(4200)
        gl.addWidget(self._max_gate, 1, 1)
        apply_g = QPushButton("APPLY GATE"); apply_g.clicked.connect(self._apply_gate)
        gl.addWidget(apply_g, 2, 0, 1, 2)
        
        # Query gate buttons
        query_row = QHBoxLayout()
        query_min_btn = QPushButton("QUERY MIN"); query_min_btn.clicked.connect(self._query_min_gate)
        query_max_btn = QPushButton("QUERY MAX"); query_max_btn.clicked.connect(self._query_max_gate)
        query_row.addWidget(query_min_btn)
        query_row.addWidget(query_max_btn)
        gl.addLayout(query_row, 3, 0, 1, 2)
        left.addWidget(gb)

        # Frequency
        fb = QGroupBox("FREQUENCY"); fl = QVBoxLayout(fb)
        self._freq_lbl = QLabel("1 Hz")
        self._freq_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._freq_lbl.setStyleSheet("color:#00FF88;font-size:18px;font-weight:bold;")
        self._freq_slider = QSlider(Qt.Orientation.Horizontal)
        self._freq_slider.setRange(1, 10); self._freq_slider.setValue(1)
        self._freq_slider.valueChanged.connect(self._freq_changed)
        fl.addWidget(self._freq_lbl); fl.addWidget(self._freq_slider)
        left.addWidget(fb)

        # Device info queries
        info_gb = QGroupBox("DEVICE INFO"); info_layout = QGridLayout(info_gb)
        fpga_btn = QPushButton("FPGA Version"); fpga_btn.clicked.connect(self._query_fpga_version)
        mcu_btn = QPushButton("MCU Version"); mcu_btn.clicked.connect(self._query_mcu_version)
        hw_btn = QPushButton("HW Version"); hw_btn.clicked.connect(self._query_hw_version)
        sn_btn = QPushButton("SN Number"); sn_btn.clicked.connect(self._query_sn)
        info_layout.addWidget(fpga_btn, 0, 0); info_layout.addWidget(mcu_btn, 0, 1)
        info_layout.addWidget(hw_btn, 1, 0); info_layout.addWidget(sn_btn, 1, 1)
        left.addWidget(info_gb)

        left.addStretch(1)
        self_chk = QPushButton("🔍  SELF CHECK"); self_chk.clicked.connect(self._self_check)
        left.addWidget(self_chk)
        clr = QPushButton("⎚  CLEAR LOG"); clr.clicked.connect(lambda: self._log_box.clear())
        left.addWidget(clr)

        # ── CENTRE ────────────────────────────────────────────────────────────
        centre = QVBoxLayout(); centre.setSpacing(8)
        root.addLayout(centre, 1)

        self._big_disp = BigDisplay(); centre.addWidget(self._big_disp)

        ctrl = QHBoxLayout(); ctrl.setSpacing(8)
        self._single_btn = QPushButton("◎  SINGLE SHOT")
        self._single_btn.setMinimumHeight(44)
        self._single_btn.clicked.connect(self._single_shot)
        self._single_btn.setEnabled(False)

        self._cont_btn = QPushButton("▶  CONTINUOUS")
        self._cont_btn.setObjectName("green")
        self._cont_btn.setMinimumHeight(44)
        self._cont_btn.clicked.connect(self._toggle_continuous)
        self._cont_btn.setEnabled(False)

        self._stop_btn = QPushButton("■  STOP")
        self._stop_btn.setObjectName("red")
        self._stop_btn.setMinimumHeight(44)
        self._stop_btn.clicked.connect(self._stop_ranging)
        self._stop_btn.setEnabled(False)

        for b in [self._single_btn, self._cont_btn, self._stop_btn]:
            ctrl.addWidget(b, 1)
        centre.addLayout(ctrl)

        # Stats
        stats = QHBoxLayout(); stats.setSpacing(8)
        self._stat_min = self._stat_box("MIN",   "---- m")
        self._stat_max = self._stat_box("MAX",   "---- m")
        self._stat_avg = self._stat_box("AVG",   "---- m")
        self._stat_cnt = self._stat_box("SHOTS", "0")
        for w in [self._stat_min, self._stat_max, self._stat_avg, self._stat_cnt]:
            stats.addWidget(w, 1)
        centre.addLayout(stats)

        chart_box = QGroupBox("DISTANCE HISTORY"); cbl = QVBoxLayout(chart_box)
        self._chart = ChartWidget(); cbl.addWidget(self._chart)
        centre.addWidget(chart_box, 1)

        tabs = QTabWidget()
        self._log_box = QTextEdit(); self._log_box.setReadOnly(True); self._log_box.setMaximumHeight(130)
        tabs.addTab(self._log_box, "SYSTEM LOG")
        self._proto_box = QTextEdit(); self._proto_box.setReadOnly(True); self._proto_box.setMaximumHeight(130)
        tabs.addTab(self._proto_box, "FRAME INSPECTOR")
        centre.addWidget(tabs)

        # ── RIGHT ─────────────────────────────────────────────────────────────
        right = QVBoxLayout(); right.setSpacing(8)
        root.addLayout(right, 0)

        rl = QLabel("IP CAMERA VIEW")
        rl.setStyleSheet("color:#4A6080;font-size:9px;letter-spacing:3px;")
        right.addWidget(rl)
        
        # Camera settings group
        cam_settings = QGroupBox("CAMERA SETTINGS")
        cam_layout = QGridLayout(cam_settings)
        
        cam_layout.addWidget(QLabel("IP Address"), 0, 0)
        self._cam_ip_edit = QLineEdit("192.168.1.168")
        cam_layout.addWidget(self._cam_ip_edit, 0, 1)
        
        cam_layout.addWidget(QLabel("Username"), 1, 0)
        self._cam_user_edit = QLineEdit("admin")
        cam_layout.addWidget(self._cam_user_edit, 1, 1)
        
        cam_layout.addWidget(QLabel("Password"), 2, 0)
        self._cam_pass_edit = QLineEdit("123456")
        self._cam_pass_edit.setEchoMode(QLineEdit.EchoMode.Password)
        cam_layout.addWidget(self._cam_pass_edit, 2, 1)
        
        apply_cam_btn = QPushButton("APPLY CAMERA")
        apply_cam_btn.clicked.connect(self._apply_camera_settings)
        cam_layout.addWidget(apply_cam_btn, 3, 0, 1, 2)
        
        right.addWidget(cam_settings)
        
        # Camera widget with crosshair
        self._camera = CameraWidget()
        self._camera.setFixedWidth(295)
        self._camera.setFixedHeight(295)
        right.addWidget(self._camera)

        info_box = QGroupBox("DEVICE SPECS"); il = QGridLayout(info_box)
        specs = [("WAVELENGTH","1535 ± 5 nm"),("MAX RANGE","4200 m"),
                 ("MIN RANGE","15 m"),("ACCURACY","± 1 m"),
                 ("DIVERGENCE","0.7 mrad"),("INTERFACE","UART TTL 3.3V"),
                 ("SUPPLY","4.5 – 16 V DC"),("WEIGHT","32 ± 1 g")]
        for i,(k,v) in enumerate(specs):
            kl = QLabel(k); kl.setStyleSheet("color:#4A6080;font-size:8px;")
            vl = QLabel(v); vl.setStyleSheet("color:#E8F4FD;font-size:9px;")
            il.addWidget(kl,i,0); il.addWidget(vl,i,1)
        right.addWidget(info_box)
        right.addStretch(1)

        self.statusBar().showMessage("STATUS: OFFLINE  |  SELECT PORT AND CONNECT")

    # ── Mode change handler ───────────────────────────────────────────────────
    def _on_mode_changed(self):
        if self._serial_radio.isChecked():
            self._connection_mode = "serial"
            self._serial_widget.setVisible(True)
            self._tcp_widget.setVisible(False)
            self._refresh_ports()
            self._log("Connection mode: SERIAL")
        else:
            self._connection_mode = "tcp"
            self._serial_widget.setVisible(False)
            self._tcp_widget.setVisible(True)
            self._log("Connection mode: TCP/IP")

    # ── Port refresh ──────────────────────────────────────────────────────────
    def _refresh_ports(self):
        if self._connection_mode != "serial":
            return
        self._port_cb.clear()
        ports = serial.tools.list_ports.comports()
        if not ports:
            self._port_cb.addItem("(no ports found)")
            self._log("No serial ports detected")
        else:
            for p in sorted(ports, key=lambda x: x.device):
                desc = p.description or ""
                label = f"{p.device}  —  {desc}" if desc and desc != p.device else p.device
                self._port_cb.addItem(label, p.device)   # userData = raw device path
            self._log(f"{len(ports)} port(s) found: " +
                      ", ".join(p.device for p in ports))

    # ── Connection ────────────────────────────────────────────────────────────
    def _toggle_connect(self):
        if not self._connected:
            if self._connection_mode == "serial":
                idx = self._port_cb.currentIndex()
                port = self._port_cb.itemData(idx) or self._port_cb.currentText().split()[0]
                baud = int(self._baud_cb.currentText())
                self._log(f"Connecting to {port} @ {baud} bps …")
                self._worker.connect_serial(port, baud)
            else:
                host = self._host_edit.text().strip()
                port = self._tcp_port_spin.value()
                self._log(f"Connecting to {host}:{port} …")
                self._worker.connect_tcp(host, port)
        else:
            self._stop_ranging()
            self._worker.disconnect()

    def _on_connected(self, state: bool):
        self._connected = state
        self._led_conn.set_on(state)
        if state:
            self._conn_btn.setText("✕  DISCONNECT")
            self._conn_btn.setObjectName("red"); self._conn_btn.setStyle(self._conn_btn.style())
            self._single_btn.setEnabled(True)
            self._cont_btn.setEnabled(True)
            if self._connection_mode == "serial":
                port = self._port_cb.itemData(self._port_cb.currentIndex()) or ""
                baud_str = f" | {self._baud_cb.currentText()} bps"
            else:
                port = f"{self._host_edit.text()}:{self._tcp_port_spin.value()}"
                baud_str = ""
            self._log(f"Connected → {port}")
            self.statusBar().showMessage(f"STATUS: ONLINE  |  {port}{baud_str}")
        else:
            self._conn_btn.setText("⚡  CONNECT")
            self._conn_btn.setObjectName("green"); self._conn_btn.setStyle(self._conn_btn.style())
            self._single_btn.setEnabled(False)
            self._cont_btn.setEnabled(False)
            self._stop_btn.setEnabled(False)
            self._led_laser.set_on(False); self._led_echo.set_on(False)
            self._log("Disconnected")
            self.statusBar().showMessage("STATUS: OFFLINE")

    # ── Ranging commands ──────────────────────────────────────────────────────
    def _single_shot(self):
        if not self._connected:
            self._log("⚠  Not connected"); return
        frame = cmd_set_mode(self._active_mode)
        self._worker.send(frame)
        self._proto_box.append("[TX] SET_MODE: " + frame.hex(" ").upper())
        frame = CMD_SINGLE
        self._worker.send(frame)
        self._proto_box.append("[TX] SINGLE:   " + frame.hex(" ").upper())
        self._led_laser.set_on(True)
        self._log("Single shot initiated")
        # use thread so UI stays responsive
        threading.Thread(target=self._worker.send_and_recv,
                         args=(CMD_SINGLE,), daemon=True).start()

    def _toggle_continuous(self):
        if not self._continuous:
            self._continuous = True
            self._cont_btn.setText("⏸  PAUSE")
            self._stop_btn.setEnabled(True)
            self._single_btn.setEnabled(False)
            hz = self._freq_slider.value()
            self._worker.send(cmd_set_freq(hz))
            self._worker.send(cmd_set_mode(self._active_mode))
            frame = CMD_CONT_START
            self._worker.send(frame)
            self._proto_box.append("[TX] CONT_START: " + frame.hex(" ").upper())
            self._rx_timer.start()
            self._led_laser.set_on(True)
            self._log("Continuous ranging started")
        else:
            self._pause_continuous()

    def _pause_continuous(self):
        self._continuous = False
        self._rx_timer.stop()
        self._worker.send(CMD_STOP)
        self._cont_btn.setText("▶  CONTINUOUS")
        self._stop_btn.setEnabled(False)
        self._single_btn.setEnabled(True)
        self._led_laser.set_on(False)
        self._log("Ranging paused")

    def _stop_ranging(self):
        self._continuous = False
        self._rx_timer.stop()
        if self._connected:
            self._worker.send(CMD_STOP)
            self._proto_box.append("[TX] STOP: " + CMD_STOP.hex(" ").upper())
        self._cont_btn.setText("▶  CONTINUOUS")
        self._stop_btn.setEnabled(False)
        self._single_btn.setEnabled(self._connected)
        self._led_laser.set_on(False); self._led_echo.set_on(False)
        self._log("Ranging stopped")

    def _self_check(self):
        if not self._connected:
            self._log("⚠  Not connected"); return
        frame = CMD_SELF_CHECK
        self._proto_box.append("[TX] SELF_CHECK: " + frame.hex(" ").upper())
        self._log("🔍 Self-check initiated...")
        threading.Thread(target=self._do_self_check, daemon=True).start()

    def _do_self_check(self):
        self._worker.send_and_recv(CMD_SELF_CHECK)

    # ── Data reception ────────────────────────────────────────────────────────
    def _on_distance(self, dist: float):
        self._shot_count += 1
        self._history.append(dist)
        if len(self._history) > 1000: self._history = self._history[-1000:]
        self._big_disp.set_value(dist)
        self._chart.push(dist)
        # Обновляем камеру вместо радара
        self._camera.set_target_distance(dist)
        
        # Update LEDs - we have a valid distance so both laser and echo should be active
        self._led_laser.set_on(True)
        self._led_echo.set_on(True)
        
        mn = min(self._history); mx = max(self._history)
        avg = sum(self._history) / len(self._history)
        self._update_stat(self._stat_min, f"{mn:.1f} m")
        self._update_stat(self._stat_max, f"{mx:.1f} m")
        self._update_stat(self._stat_avg, f"{avg:.1f} m")
        self._update_stat(self._stat_cnt, str(self._shot_count))
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log_box.append(f"[{ts}]  ► {dist:.1f} m")
        self._log_box.verticalScrollBar().setValue(
            self._log_box.verticalScrollBar().maximum())

    def _on_raw_frame(self, tx: bytes, rx: bytes):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if rx:
            parsed = parse_response_frame(rx)
            if parsed:
                cmd = parsed['cmd']
                log_msg = None
                if cmd == 0x01:  # Self-check response
                    status1 = parsed.get('status1', 0)
                    status2 = parsed.get('status2', 0)
                    fpga_ok = parsed.get('fpga_system_status', False)
                    echo_intensity = status2
                    
                    status_str = (
                        f"FPGA={'OK' if fpga_ok else 'FAIL'}, "
                        f"Echo={echo_intensity}, "
                        f"Laser={'ON' if parsed.get('laser_light_output') else 'OFF'}, "
                        f"5V6={'OK' if parsed.get('power_5v6_status') else 'FAIL'}"
                    )
                    self._proto_box.append(f"[{ts}] [RX] SELF_CHECK: {status_str}")
                    log_msg = f"✓ SELF_CHECK: {status_str}"
                    
                elif cmd in (0x02, 0x04):  # Range response
                    dist = parsed.get('distance', 'N/A')
                    status = parsed.get('status', 'N/A')
                    self._proto_box.append(f"[{ts}] [RX] RANGE: {dist}m (status: 0x{status:02X})")
                    
                elif cmd == 0x06:  # Anomaly
                    self._proto_box.append(f"[{ts}] [RX] ANOMALY: status1=0x{parsed['status1']:02X}")
                    log_msg = f"⚠ ANOMALY: status=0x{parsed['status1']:02X}"
                    
                elif cmd == 0xA1:  # Set frequency response
                    self._proto_box.append(f"[{ts}] [RX] FREQ_SET: OK")
                    log_msg = "✓ Frequency set successfully"
                    
                elif cmd == 0xA2:  # Set MIN gate response
                    dist = parsed.get('gate_distance', 'N/A')
                    self._proto_box.append(f"[{ts}] [RX] GATE_MIN_SET: {dist}m")
                    log_msg = f"✓ MIN gate set to {dist}m"
                    
                elif cmd == 0xA4:  # Set MAX gate response
                    dist = parsed.get('gate_distance', 'N/A')
                    self._proto_box.append(f"[{ts}] [RX] GATE_MAX_SET: {dist}m")
                    log_msg = f"✓ MAX gate set to {dist}m"
                    
                elif cmd == 0xA3:  # Query MIN gate response
                    dist = parsed.get('gate_distance', 'N/A')
                    self._proto_box.append(f"[{ts}] [RX] GATE_MIN_QUERY: {dist}m")
                    log_msg = f"✓ MIN gate: {dist}m"
                    
                elif cmd == 0xA5:  # Query MAX gate response
                    dist = parsed.get('gate_distance', 'N/A')
                    self._proto_box.append(f"[{ts}] [RX] GATE_MAX_QUERY: {dist}m")
                    log_msg = f"✓ MAX gate: {dist}m"
                    
                elif cmd == 0xA6:  # FPGA version
                    ver = parsed.get('version', 'N/A')
                    author = parsed.get('author', '')
                    date = parsed.get('date', 0)
                    month = parsed.get('month', 0)
                    year = parsed.get('year', 0)
                    self._proto_box.append(f"[{ts}] [RX] FPGA_VERSION: {ver} by {author} ({year}/{month:02d}/{date})")
                    log_msg = f"✓ FPGA: {ver} by {author}"
                    
                elif cmd == 0xA7:  # MCU version
                    ver = parsed.get('version', 'N/A')
                    author = parsed.get('author', '')
                    date = parsed.get('date', 0)
                    month = parsed.get('month', 0)
                    year = parsed.get('year', 0)
                    self._proto_box.append(f"[{ts}] [RX] MCU_VERSION: {ver} by {author} ({year}/{month:02d}/{date})")
                    log_msg = f"✓ MCU: {ver} by {author}"
                    
                elif cmd == 0xA8:  # HW version
                    mb = parsed.get('motherboard', 'N/A')
                    ct = parsed.get('control_board', 'N/A')
                    apd = parsed.get('detection_board', 'N/A')
                    ld = parsed.get('driver_board', 'N/A')
                    self._proto_box.append(f"[{ts}] [RX] HW_VERSION: MB={mb}, CT={ct}, APD={apd}, LD={ld}")
                    log_msg = f"✓ HW: MB={mb}, CT={ct}, APD={apd}, LD={ld}"
                    
                elif cmd == 0xA9:  # SN response
                    sn = parsed.get('serial_number', 'N/A')
                    month = parsed.get('month', 0)
                    year = parsed.get('year', 0)
                    self._proto_box.append(f"[{ts}] [RX] SN: {sn} (MFG: {year}/{month:02d})")
                    log_msg = f"✓ Serial Number: {sn}"
                    
                elif cmd in (0x90, 0x91):  # Light count responses
                    count = parsed.get('light_count', 'N/A')
                    count_type = "TOTAL" if cmd == 0x90 else "POWER_ON"
                    self._proto_box.append(f"[{ts}] [RX] LIGHT_COUNT_{count_type}: {count}")
                    log_msg = f"✓ {count_type} light count: {count}"
                    
                else:
                    self._proto_box.append(f"[{ts}] [RX] UNPARSED: {rx.hex(' ').upper()}")
                
                # Log to main log box if we have a message
                if log_msg:
                    self._log(log_msg)
            else:
                self._proto_box.append(f"[{ts}] [RX] INVALID_FRAME: {rx.hex(' ').upper()}")
                self._log(f"⚠ Invalid frame received: {rx.hex(' ').upper()}")
            
            sb = self._proto_box.verticalScrollBar()
            sb.setValue(sb.maximum())

    def _on_error(self, msg: str):
        self._log(f"⚠  ERROR: {msg}")

    # ── Settings ───────────────────────────────────────────────────────────────
    def _set_mode(self, mode: int):
        self._active_mode = mode
        names = {1:"FIRST TARGET",2:"LAST TARGET",3:"MULTI-TARGET"}
        for btn, m in [(self._btn_first,1),(self._btn_last,2),(self._btn_multi,3)]:
            if m == mode:
                btn.setStyleSheet("background:#00D4FF;color:#0A0E17;border-radius:4px;padding:7px 14px;")
            else:
                btn.setStyleSheet("")
        self._log(f"Mode → {names[mode]}")
        if self._connected:
            self._worker.send(cmd_set_mode(mode))

    def _freq_changed(self, val: int):
        self._freq_lbl.setText(f"{val} Hz")
        if self._connected and self._continuous:
            self._worker.send(cmd_set_freq(val))

    def _apply_gate(self):
        mn, mx = self._min_gate.value(), self._max_gate.value()
        if mn >= mx: self._log("⚠  MIN must be < MAX"); return
        if not self._connected:
            self._log("⚠  Not connected"); return
        frame_min = cmd_set_min_gate(mn)
        frame_max = cmd_set_max_gate(mx)
        self._proto_box.append("[TX] SET_MIN_GATE: " + frame_min.hex(" ").upper())
        self._proto_box.append("[TX] SET_MAX_GATE: " + frame_max.hex(" ").upper())
        self._log(f"Gate → {mn} m … {mx} m")
        threading.Thread(target=lambda: (self._worker.send_and_recv(frame_min), self._worker.send_and_recv(frame_max)), daemon=True).start()

    # ── Device info queries ────────────────────────────────────────────────────
    def _query_min_gate(self):
        if not self._connected:
            self._log("⚠  Not connected"); return
        frame = cmd_query_min_gate()
        self._worker.send_and_recv(frame)
        self._proto_box.append("[TX] QUERY_MIN_GATE: " + frame.hex(" ").upper())
        self._log("Querying MIN gate...")

    def _query_max_gate(self):
        if not self._connected:
            self._log("⚠  Not connected"); return
        frame = cmd_query_max_gate()
        self._worker.send_and_recv(frame)
        self._proto_box.append("[TX] QUERY_MAX_GATE: " + frame.hex(" ").upper())
        self._log("Querying MAX gate...")

    def _query_fpga_version(self):
        if not self._connected:
            self._log("⚠  Not connected"); return
        frame = cmd_query_fpga_version()
        self._worker.send_and_recv(frame)
        self._proto_box.append("[TX] QUERY_FPGA_VERSION: " + frame.hex(" ").upper())
        self._log("Querying FPGA version...")

    def _query_mcu_version(self):
        if not self._connected:
            self._log("⚠  Not connected"); return
        frame = cmd_query_mcu_version()
        self._worker.send_and_recv(frame)
        self._proto_box.append("[TX] QUERY_MCU_VERSION: " + frame.hex(" ").upper())
        self._log("Querying MCU version...")

    def _query_hw_version(self):
        if not self._connected:
            self._log("⚠  Not connected"); return
        frame = cmd_query_hw_version()
        self._worker.send_and_recv(frame)
        self._proto_box.append("[TX] QUERY_HW_VERSION: " + frame.hex(" ").upper())
        self._log("Querying HW version...")

    def _query_sn(self):
        if not self._connected:
            self._log("⚠  Not connected"); return
        frame = cmd_query_sn()
        self._worker.send_and_recv(frame)
        self._proto_box.append("[TX] QUERY_SN: " + frame.hex(" ").upper())
        self._log("Querying Serial Number...")

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _stat_box(self, label: str, value: str) -> QFrame:
        f = QFrame()
        f.setStyleSheet("background:#131C2E;border:1px solid #1A2840;border-radius:4px;")
        lay = QVBoxLayout(f); lay.setContentsMargins(8,4,8,4); lay.setSpacing(2)
        kl = QLabel(label); kl.setStyleSheet("color:#4A6080;font-size:8px;letter-spacing:2px;")
        vl = QLabel(value); vl.setStyleSheet("color:#00D4FF;font-size:13px;font-weight:bold;")
        lay.addWidget(kl); lay.addWidget(vl)
        f._vl = vl
        return f

    def _update_stat(self, f: QFrame, text: str):
        f._vl.setText(text)

    def _apply_camera_settings(self):
        """Применить настройки IP-камеры"""
        ip = self._cam_ip_edit.text().strip()
        user = self._cam_user_edit.text().strip()
        password = self._cam_pass_edit.text().strip()
        
        if not ip:
            self._log("⚠  Camera IP address is required")
            return
            
        self._camera.set_camera_credentials(ip, user, password)
        self._log(f"✓ Camera settings applied: {ip}")

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self._log_box.append(f"[{ts}]  {msg}")
        self._log_box.verticalScrollBar().setValue(
            self._log_box.verticalScrollBar().maximum())

    def closeEvent(self, event):
        self._stop_ranging()
        self._worker.disconnect()
        super().closeEvent(event)


# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("LRF-3KM Control")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())