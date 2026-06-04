import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

import serial
from serial.tools import list_ports
from PySide6.QtCore import QThread, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QPlainTextEdit,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

BAUD_RATE = 921600

CAN_ID_CONTROL = 0x200
CAN_ID_TELEM_SUMMARY = 0x700
CAN_ID_TELEM_EXTRA = 0x701
CAN_ID_ECU_RAW_DEBUG = 0x702
CAN_ID_RETRY_MODE_STAT = 0x790
CAN_ID_ADC_REF_STAT = 0x791
CAN_ID_EFUSE_BASE = 0x710
CAN_ID_EFUSE_LAST = CAN_ID_EFUSE_BASE + 7
CAN_ID_ADC_BASE = 0x720
CAN_ID_ADC_LAST = CAN_ID_ADC_BASE + 7
CAN_ID_TEMP_BASE = 0x730
CAN_ID_TEMP_LAST = CAN_ID_TEMP_BASE + 7
CAN_ID_MCU = 0x740

CAN_BUS_BITRATE_BPS = 500_000
CAN_UTIL_WINDOW_S = 1.0
SERIAL_BITS_PER_BYTE = 10
SERIAL_UTIL_WINDOW_S = 1.0
SD_STATE_POLL_MS = 2000

CONTROL_MAGIC = 0xA5
CONTROL_OP_NOP = 0x00
CONTROL_OP_OUTPUT = 0x01
CONTROL_OP_RETRY_MODE = 0x02
CONTROL_OP_CLEAR_FAULTS = 0x03
CONTROL_OP_ADC_REF = 0x04

CONTROL_FLAG_DEBUG = 0x04
OVERRIDE_FLAG_ARMED = 0x01
OVERRIDE_FLAG_START_ON = 0x02

CONTROL_STATE_FLAG_DEBUG_ACTIVE = 0x01
CONTROL_STATE_FLAG_ECU_FRESH = 0x02
CONTROL_STATE_FLAG_OVERRIDE_ARMED = 0x08
CONTROL_STATE_FLAG_OVERRIDE_ACTIVE = 0x10
CONTROL_STATE_FLAG_SAFETY_BLOCKED = 0x20
CONTROL_STATE_FLAG_HYBRID_LATCHED = 0x40
CONTROL_STATE_FLAG_START_ON = 0x80

ECU_RAW_FLAG_SEEN = 0x01
ECU_RAW_FLAG_FRESH = 0x02
ECU_RAW_FLAG_BYTE1 = 0x04
ECU_RAW_FLAG_DLC_OK = 0x08

PDU_OUTPUTS = ["Hybrid", "Vent1", "Vent2", "IGN", "Fuel", "WP1", "WP2", "12V"]
SYSTEM_FLAG_NAMES = [
    "ENABLE_FAIL",
    "READ_FAIL",
    "PEC_FAIL",
    "TIMEOUT",
    "NACK_OR_BUS",
    "ZERO_TELEM",
    "CAN_TX_FAIL",
    "SCAN_NO_ACK",
]
FLT_FLAG_NAMES = ["FLT", "FLTM", "SDC", "BSPD", "BOTS", "INERTIA", "F6", "F7"]
TPS_STATUS_CML_BITS = [
    (7, "INV_CMD"),
    (6, "INV_DATA"),
    (5, "INV_PEC"),
    (4, "MEM_FLT"),
    (0, "OTHER"),
]

HISTORY_LEN = 240
CAN_COMPARE_TIMEOUT_MS = 350


@dataclass(frozen=True)
class SignalDef:
    name: str
    can_id: int
    offset: int
    size: int
    scale: float
    unit: str
    signed: bool = False


MAXXECU_SIGNALS = [
    SignalDef("RPM", 0x520, 0, 2, 1.0, "rpm"),
    SignalDef("Throttle", 0x520, 2, 2, 0.1, "%"),
    SignalDef("Boost", 0x520, 4, 2, 0.1, "kPa"),
    SignalDef("Lambda", 0x520, 6, 2, 0.001, "lambda"),
    SignalDef("Vehicle speed", 0x522, 6, 2, 1.0, "km/h"),
    SignalDef("Battery voltage", 0x530, 0, 2, 0.01, "V"),
    SignalDef("Intake temp", 0x530, 4, 2, 0.1, "C", signed=True),
    SignalDef("Coolant temp", 0x530, 6, 2, 0.1, "C", signed=True),
    SignalDef("EGT1", 0x531, 6, 2, 1.0, "C"),
    SignalDef("User analog 1", 0x535, 0, 2, 0.1, "user"),
    SignalDef("User analog 2", 0x535, 2, 2, 0.1, "user"),
    SignalDef("User analog 3", 0x535, 4, 2, 0.1, "user"),
    SignalDef("User analog 4", 0x535, 6, 2, 0.1, "user"),
    SignalDef("Gear", 0x536, 0, 2, 1.0, ""),
    SignalDef("Oil pressure", 0x536, 4, 2, 0.1, "kPa"),
    SignalDef("Oil temp", 0x536, 6, 2, 0.1, "C", signed=True),
]

KEY_CARD_ORDER = [
    "RPM",
    "Throttle",
    "Boost",
    "Lambda",
    "Vehicle speed",
    "Battery voltage",
    "Coolant temp",
    "Oil pressure",
    "Oil temp",
    "Gear",
]


def parse_hex_payload(payload_text: str, dlc: int) -> list[int]:
    if dlc == 0:
        return []
    payload = []
    for token in payload_text.split():
        try:
            payload.append(int(token, 16) & 0xFF)
        except ValueError:
            return []
    return payload[:dlc]


def read_le(data: list[int], offset: int, size: int, signed: bool) -> int | None:
    end = offset + size
    if end > len(data):
        return None
    raw = 0
    for i in range(size):
        raw |= data[offset + i] << (8 * i)
    if signed:
        sign_bit = 1 << (size * 8 - 1)
        if raw & sign_bit:
            raw -= 1 << (size * 8)
    return raw


class TrendWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._samples: list[float] = []
        self.setMinimumHeight(90)

    def set_samples(self, samples: list[float]):
        self._samples = samples
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#111417"))

        w = max(1, self.width())
        h = max(1, self.height())

        painter.setPen(QPen(QColor("#2d353f"), 1))
        for y in (h // 4, h // 2, (3 * h) // 4):
            painter.drawLine(0, y, w, y)

        if len(self._samples) < 2:
            return

        min_v = min(self._samples)
        max_v = max(self._samples)
        span = max(0.001, max_v - min_v)

        painter.setPen(QPen(QColor("#2fd47b"), 2))
        points = []
        n = len(self._samples)
        for i, value in enumerate(self._samples):
            x = int((i / (n - 1)) * (w - 1))
            y_norm = (value - min_v) / span
            y = int((h - 4) - (y_norm * (h - 8)))
            points.append((x, y))

        for i in range(1, len(points)):
            painter.drawLine(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1])


class SerialWorker(QThread):
    line_received = Signal(str)
    status_changed = Signal(bool, str)

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._target_port = ""
        self._running = True
        self._tx_queue: queue.Queue[str] = queue.Queue()

    def set_target_port(self, port_name: str):
        with self._lock:
            self._target_port = port_name.strip()

    def clear_target_port(self):
        with self._lock:
            self._target_port = ""

    def send_line(self, line: str):
        self._tx_queue.put(line)

    def stop(self):
        self._running = False

    def run(self):
        ser = None
        active_port = ""

        while self._running:
            with self._lock:
                target = self._target_port

            if not target:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                if active_port:
                    active_port = ""
                    self.status_changed.emit(False, "Disconnected")
                time.sleep(0.1)
                continue

            if ser is None or not ser.is_open or active_port != target:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                try:
                    ser = serial.Serial(target, BAUD_RATE, timeout=0.05)
                    active_port = target
                    self.status_changed.emit(True, f"Connected: {active_port}")
                except Exception as exc:
                    ser = None
                    active_port = ""
                    self.status_changed.emit(False, f"Connect failed: {exc}")
                    time.sleep(0.7)
                    continue

            try:
                for _ in range(20):
                    line = self._tx_queue.get_nowait()
                    ser.write((line + "\n").encode("ascii", errors="ignore"))
            except queue.Empty:
                pass
            except Exception as exc:
                self.status_changed.emit(False, f"Write error: {exc}")

            try:
                raw = ser.readline()
                if raw:
                    line = raw.decode(errors="replace").strip()
                    if line:
                        self.line_received.emit(line)
            except Exception as exc:
                self.status_changed.emit(False, f"Read error: {exc}")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                active_port = ""

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


class TelemetryWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Telemetry Dashboard")
        self.resize(1680, 980)

        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.latest_values: dict[str, float | int | None] = {sig.name: None for sig in MAXXECU_SIGNALS}
        self.latest_update_ms: dict[str, int] = {sig.name: 0 for sig in MAXXECU_SIGNALS}

        self.rx_frames_can1 = 0
        self.rx_frames_can2 = 0
        self.rx_frames_total = 0
        self.can_compare_pending_can1: dict[tuple[int, int, tuple[int, ...]], deque[int]] = {}
        self.can_compare_pending_can2: dict[tuple[int, int, tuple[int, ...]], deque[int]] = {}
        self.can_compare_match_count = 0
        self.can_compare_miss_count = 0
        self.last_can_timestamp_ms = 0
        self.last_fps_reset = time.monotonic()
        self.fps_counter = 0
        self.can_bits_window_can1: deque[tuple[float, int]] = deque()
        self.can_bits_window_can2: deque[tuple[float, int]] = deque()
        self.can_utilization_can1 = 0.0
        self.can_utilization_can2 = 0.0
        self.serial_bits_window: deque[tuple[float, int]] = deque()
        self.serial_utilization = 0.0
        self.serial_rx_bytes_total = 0
        self.serial_tx_bytes_total = 0

        self.pdu_control_state_flags = 0
        self.pdu_requested_mask = 0
        self.pdu_applied_mask = 0
        self.pdu_ecu_mask = 0
        self.pdu_dash_mask = 0
        self.pdu_ecu_raw_flags = 0
        self.pdu_ecu_raw_decoded = 0
        self.pdu_ecu_raw_requested = 0
        self.pdu_ecu_raw_b0 = 0
        self.pdu_ecu_raw_b1 = 0
        self.pdu_ecu_raw_dlc = 0
        self.pdu_ecu_raw_count = 0
        self.pdu_retry_mode = 0
        self.pdu_retry_applied = 0
        self.pdu_adc_ref = 0
        self.pdu_adc_ref_applied = 0

        self.mcu_shunt = 0
        self.flt_bits = 0
        self.system_flags = 0
        self.vin_avg_mv = 0
        self.sum_current_da = 0
        self.temp_avg_c = 0.0
        self.temp_peak_c = 0.0
        self.err_flags = 0

        self.efuse_voltage = [0] * 8
        self.efuse_current = [0] * 8
        self.efuse_power = [0] * 8
        self.efuse_status_word = [0] * 8
        self.efuse_adc_current = [0] * 8
        self.efuse_adc_voltage = [0] * 8
        self.efuse_adc_diff = [0] * 8
        self.efuse_adc_flags = [0] * 8
        self.efuse_cml_status = [0] * 8
        self.efuse_temp_c: list[float | None] = [None] * 8
        self.efuse_current_history: list[deque[float]] = [deque(maxlen=HISTORY_LEN) for _ in range(8)]
        self.shunt_history: deque[float] = deque(maxlen=HISTORY_LEN)

        self.asm_fields: dict[str, str] = {}
        self.asm_init_event_text = "No manual init command sent"
        self.last_imu_text = "No IMU samples yet"
        self.sd_fields: dict[str, str] = {}
        self.sd_last_event_text = "No SD events yet"
        self.csv_log_summary_text = "No CSV log loaded"
        self.modem_fields: dict[str, str] = {}
        self.mqtt_fields: dict[str, str] = {}
        self.gnss_fields: dict[str, str] = {}
        self.ntrip_fields: dict[str, str] = {}
        self.modem_last_event_text = "No modem events yet"
        self.mqtt_last_event_text = "No MQTT events yet"
        self.gnss_last_event_text = "No GNSS events yet"
        self.ntrip_last_event_text = "No NTRIP events yet"

        self._build_ui()

        self.serial_worker = SerialWorker()
        self.serial_worker.line_received.connect(self._on_serial_line)
        self.serial_worker.status_changed.connect(self._on_serial_status)
        self.serial_worker.start()

        self.process_timer = QTimer(self)
        self.process_timer.timeout.connect(self._process_incoming_lines)
        self.process_timer.start(30)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_ui)
        self.refresh_timer.start(80)

        self.keepalive_timer = QTimer(self)
        self.keepalive_timer.timeout.connect(self._send_pdu_keepalive)
        self.keepalive_timer.start(120)

        self.sd_poll_timer = QTimer(self)
        self.sd_poll_timer.timeout.connect(self._poll_sd_state)
        self.sd_poll_timer.start(SD_STATE_POLL_MS)

        self._refresh_ports()

    def closeEvent(self, event):
        self.serial_worker.stop()
        self.serial_worker.wait(1500)
        super().closeEvent(event)

    def _build_ui(self):
        root = QWidget(self)
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)

        serial_row = QHBoxLayout()
        serial_row.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(130)
        serial_row.addWidget(self.port_combo)

        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_ports)
        serial_row.addWidget(refresh_btn)

        connect_btn = QPushButton("Connect")
        connect_btn.clicked.connect(self._connect_selected_port)
        serial_row.addWidget(connect_btn)

        disconnect_btn = QPushButton("Disconnect")
        disconnect_btn.clicked.connect(self._disconnect_port)
        serial_row.addWidget(disconnect_btn)

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("font-weight:700; color:#b00020;")
        serial_row.addWidget(self.status_label)
        serial_row.addStretch()

        main_layout.addLayout(serial_row)

        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.overview_tab = QWidget()
        self.pdu_tab = QWidget()
        self.asm_tab = QWidget()
        self.sd_tab = QWidget()
        self.modem_tab = QWidget()
        self.gnss_tab = QWidget()
        self.raw_tab = QWidget()

        self.tabs.addTab(self.overview_tab, "Overview")
        self.tabs.addTab(self.pdu_tab, "PDU Control + Diagnostics")
        self.tabs.addTab(self.asm_tab, "ASM330 Debug")
        self.tabs.addTab(self.sd_tab, "SD Logging")
        self.tabs.addTab(self.modem_tab, "4G Modem")
        self.tabs.addTab(self.gnss_tab, "GNSS RTK")
        self.tabs.addTab(self.raw_tab, "Raw")

        self._build_overview_tab()
        self._build_pdu_tab()
        self._build_asm_tab()
        self._build_sd_tab()
        self._build_modem_tab()
        self._build_gnss_tab()
        self._build_raw_tab()

    def _build_overview_tab(self):
        layout = QVBoxLayout(self.overview_tab)

        cards_box = QGroupBox("Live Vehicle Data")
        cards_layout = QGridLayout(cards_box)
        self.card_value_labels: dict[str, QLabel] = {}

        for idx, key in enumerate(KEY_CARD_ORDER):
            signal = next((sig for sig in MAXXECU_SIGNALS if sig.name == key), None)
            unit = signal.unit if signal else ""
            card = QGroupBox(key)
            card_layout = QVBoxLayout(card)
            value_lbl = QLabel("--")
            value_lbl.setAlignment(Qt.AlignCenter)
            value_lbl.setStyleSheet("font-size: 30px; font-weight: 700;")
            unit_lbl = QLabel(unit)
            unit_lbl.setAlignment(Qt.AlignCenter)
            unit_lbl.setStyleSheet("font-size: 14px; color:#555;")
            card_layout.addWidget(value_lbl)
            card_layout.addWidget(unit_lbl)
            cards_layout.addWidget(card, idx // 5, idx % 5)
            self.card_value_labels[key] = value_lbl

        layout.addWidget(cards_box)

        self.overview_stats_label = QLabel("CAN1: 0  CAN2: 0  FPS: 0")
        self.overview_stats_label.setStyleSheet("font-weight: 600;")
        layout.addWidget(self.overview_stats_label)

        util_row = QHBoxLayout()
        util_row.addWidget(QLabel("CAN1 utilization:"))
        self.can1_util_bar = QProgressBar()
        self.can1_util_bar.setRange(0, 100)
        self.can1_util_bar.setFormat("%p%")
        util_row.addWidget(self.can1_util_bar)
        self.can1_util_label = QLabel("0.0%")
        util_row.addWidget(self.can1_util_label)

        util_row.addSpacing(12)
        util_row.addWidget(QLabel("CAN2 utilization:"))
        self.can2_util_bar = QProgressBar()
        self.can2_util_bar.setRange(0, 100)
        self.can2_util_bar.setFormat("%p%")
        util_row.addWidget(self.can2_util_bar)
        self.can2_util_label = QLabel("0.0%")
        util_row.addWidget(self.can2_util_label)

        util_row.addSpacing(12)
        util_row.addWidget(QLabel("Serial utilization:"))
        self.serial_util_bar = QProgressBar()
        self.serial_util_bar.setRange(0, 100)
        self.serial_util_bar.setFormat("%p%")
        util_row.addWidget(self.serial_util_bar)
        self.serial_util_label = QLabel("0.0%")
        util_row.addWidget(self.serial_util_label)
        util_row.addStretch()
        layout.addLayout(util_row)

        compare_row = QHBoxLayout()
        self.can_compare_label = QLabel("CAN1/CAN2 compare: waiting")
        self.can_compare_label.setStyleSheet("font-weight: 700; color:#666;")
        compare_row.addWidget(self.can_compare_label)

        self.can_compare_counts_label = QLabel("matched: 0   missed: 0   pending: 0")
        self.can_compare_counts_label.setStyleSheet("color:#555;")
        compare_row.addWidget(self.can_compare_counts_label)

        reset_compare_btn = QPushButton("Reset CAN Compare")
        reset_compare_btn.clicked.connect(self._reset_can_compare)
        compare_row.addWidget(reset_compare_btn)
        compare_row.addStretch()
        layout.addLayout(compare_row)

        self.overview_imu_label = QLabel("IMU: waiting for data")
        self.overview_imu_label.setStyleSheet("font-weight: 600;")
        layout.addWidget(self.overview_imu_label)

        self.signal_table = QTableWidget(len(MAXXECU_SIGNALS), 3)
        self.signal_table.setHorizontalHeaderLabels(["Signal", "Value", "Last CAN ms"])
        self.signal_table.verticalHeader().setVisible(False)
        for row, sig in enumerate(MAXXECU_SIGNALS):
            self.signal_table.setItem(row, 0, QTableWidgetItem(f"{sig.name} [{sig.unit}]"))
            self.signal_table.setItem(row, 1, QTableWidgetItem("--"))
            self.signal_table.setItem(row, 2, QTableWidgetItem("0"))
        self.signal_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.signal_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.signal_table)

    def _build_pdu_tab(self):
        layout = QVBoxLayout(self.pdu_tab)

        cfg_box = QGroupBox("Command")
        cfg_layout = QHBoxLayout(cfg_box)

        self.override_check = QCheckBox("Manual override")
        self.override_check.stateChanged.connect(self._on_override_toggled)
        cfg_layout.addWidget(self.override_check)

        self.debug_check = QCheckBox("Debug hold")
        self.debug_check.stateChanged.connect(self._send_pdu_keepalive)
        cfg_layout.addWidget(self.debug_check)

        self.start_on_check = QCheckBox("Start output ON")
        self.start_on_check.setChecked(True)
        self.start_on_check.stateChanged.connect(self._send_pdu_keepalive)
        cfg_layout.addWidget(self.start_on_check)

        cfg_layout.addStretch()
        layout.addWidget(cfg_box)

        outputs_box = QGroupBox("PDU Outputs")
        outputs_layout = QGridLayout(outputs_box)
        self.output_checks: list[QCheckBox] = []
        for i, name in enumerate(PDU_OUTPUTS):
            cb = QCheckBox(name)
            cb.stateChanged.connect(self._on_output_changed)
            self.output_checks.append(cb)
            outputs_layout.addWidget(cb, i // 4, i % 4)
        layout.addWidget(outputs_box)

        buttons_box = QGroupBox("Actions")
        buttons_layout = QHBoxLayout(buttons_box)
        for text, op, p0 in [
            ("Retry Fast", CONTROL_OP_RETRY_MODE, 1),
            ("Retry Race", CONTROL_OP_RETRY_MODE, 2),
            ("Retry Test", CONTROL_OP_RETRY_MODE, 3),
            ("ADC External", CONTROL_OP_ADC_REF, 0),
            ("ADC Internal", CONTROL_OP_ADC_REF, 1),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, opcode=op, param0=p0: self._send_pdu_command(opcode, param0))
            buttons_layout.addWidget(btn)

        clear_btn = QPushButton("Clear Faults")
        clear_btn.clicked.connect(lambda: self._send_pdu_command(CONTROL_OP_CLEAR_FAULTS, 0xFF))
        buttons_layout.addWidget(clear_btn)
        buttons_layout.addStretch()
        layout.addWidget(buttons_box)

        status_box = QGroupBox("PDU State")
        status_layout = QFormLayout(status_box)
        self.pdu_owner_label = QLabel("No data")
        self.pdu_mask_label = QLabel("Req 0x00 App 0x00 ECU 0x00 Dash 0x00")
        self.pdu_raw_label = QLabel("0x080 dbg: --")
        self.pdu_retry_label = QLabel("Retry mode: --")
        self.pdu_adc_label = QLabel("ADC ref: --")
        status_layout.addRow("Source:", self.pdu_owner_label)
        status_layout.addRow("Masks:", self.pdu_mask_label)
        status_layout.addRow("Raw ECU:", self.pdu_raw_label)
        status_layout.addRow("Retry:", self.pdu_retry_label)
        status_layout.addRow("ADC Ref:", self.pdu_adc_label)
        layout.addWidget(status_box)

        summary_box = QGroupBox("PDU Electrical Summary")
        summary_layout = QFormLayout(summary_box)
        self.summary_vin_label = QLabel("0 mV")
        self.summary_current_label = QLabel("0.0 A")
        self.summary_power_label = QLabel("0.0 W")
        self.summary_temp_label = QLabel("avg 0.0 C, peak 0.0 C")
        self.summary_shunt_label = QLabel("0 mA")
        self.summary_err_label = QLabel("Err flags 0x00")
        summary_layout.addRow("VIN avg:", self.summary_vin_label)
        summary_layout.addRow("Current sum:", self.summary_current_label)
        summary_layout.addRow("Combined power:", self.summary_power_label)
        summary_layout.addRow("Temperature:", self.summary_temp_label)
        summary_layout.addRow("Shunt current:", self.summary_shunt_label)
        summary_layout.addRow("Errors:", self.summary_err_label)
        layout.addWidget(summary_box)

        flags_box = QGroupBox("System Flags")
        flags_layout = QGridLayout(flags_box)
        self.flt_flag_labels: list[QLabel] = []
        self.sys_flag_labels: list[QLabel] = []

        for i, name in enumerate(FLT_FLAG_NAMES):
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("padding:4px; background:#5d1f1f; color:white;")
            flags_layout.addWidget(lbl, 0, i)
            self.flt_flag_labels.append(lbl)

        for i, name in enumerate(SYSTEM_FLAG_NAMES):
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("padding:4px; background:#1d3b2d; color:white;")
            flags_layout.addWidget(lbl, 1, i)
            self.sys_flag_labels.append(lbl)

        layout.addWidget(flags_box)

        self.efuse_table = QTableWidget(8, 12)
        self.efuse_table.setHorizontalHeaderLabels(
            [
                "eFuse",
                "V [mV]",
                "I [mA]",
                "P [10mW]",
                "Temp [C]",
                "ADC I [mA]",
                "ADC V [mV]",
                "ADC diff [mA]",
                "CML",
                "StatusWord",
                "Desired/Applied",
                "RawECU",
            ]
        )
        self.efuse_table.verticalHeader().setVisible(False)
        self.efuse_table.setEditTriggers(QTableWidget.NoEditTriggers)
        for row, name in enumerate(PDU_OUTPUTS):
            self.efuse_table.setItem(row, 0, QTableWidgetItem(name))
            for col in range(1, 12):
                self.efuse_table.setItem(row, col, QTableWidgetItem("--"))
        self.efuse_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.efuse_table)

        graph_box = QGroupBox("Current Trends")
        graph_layout = QGridLayout(graph_box)

        shunt_card = QGroupBox("Shunt [mA]")
        shunt_layout = QVBoxLayout(shunt_card)
        self.shunt_graph = TrendWidget()
        shunt_layout.addWidget(self.shunt_graph)
        graph_layout.addWidget(shunt_card, 0, 0, 1, 2)

        self.efuse_graphs: list[TrendWidget] = []
        for i, name in enumerate(PDU_OUTPUTS):
            card = QGroupBox(f"{name} current [mA]")
            card_layout = QVBoxLayout(card)
            trend = TrendWidget()
            card_layout.addWidget(trend)
            graph_layout.addWidget(card, 1 + (i // 4), i % 4)
            self.efuse_graphs.append(trend)

        layout.addWidget(graph_box)

        self._set_override_widgets_enabled(False)

    def _build_asm_tab(self):
        layout = QVBoxLayout(self.asm_tab)

        btn_row = QHBoxLayout()
        for cmd in ["ASMDBG", "ASMDUMP", "ASMRAW", "ASMSTATE"]:
            btn = QPushButton(cmd)
            btn.clicked.connect(lambda _, c=cmd: self._send_serial_line(c))
            btn_row.addWidget(btn)

        init_btn = QPushButton("Init Retry Until Success")
        init_btn.clicked.connect(lambda: self._send_serial_line("ASMINIT"))
        btn_row.addWidget(init_btn)

        stop_btn = QPushButton("Stop Init Retry")
        stop_btn.clicked.connect(lambda: self._send_serial_line("ASMINITSTOP"))
        btn_row.addWidget(stop_btn)

        self.asmreg_input = QLineEdit()
        self.asmreg_input.setPlaceholderText("Register hex, e.g. 0F")
        btn_row.addWidget(self.asmreg_input)

        reg_btn = QPushButton("Read Reg")
        reg_btn.clicked.connect(self._send_asmreg_command)
        btn_row.addWidget(reg_btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        fields_box = QGroupBox("Parsed ASM330 State")
        fields_layout = QFormLayout(fields_box)
        self.asm_ready_label = QLabel("--")
        self.asm_whoami_label = QLabel("--")
        self.asm_irq_label = QLabel("--")
        self.asm_samples_label = QLabel("--")
        self.asm_status_label = QLabel("--")
        self.asm_nosample_label = QLabel("--")
        self.asm_drdy_label = QLabel("--")
        self.asm_raw_label = QLabel("--")
        self.asm_retry_label = QLabel("--")
        self.asm_last_ok_label = QLabel("--")
        self.asm_init_cycles_label = QLabel("--")
        self.asm_init_ok_label = QLabel("--")
        self.asm_init_fail_label = QLabel("--")
        self.asm_addr_label = QLabel("--")
        self.asm_ctrl3_label = QLabel("--")
        self.asm_ctrl4_label = QLabel("--")
        self.asm_ctrl9_label = QLabel("--")
        self.asm_init_event_label = QLabel(self.asm_init_event_text)
        fields_layout.addRow("Ready:", self.asm_ready_label)
        fields_layout.addRow("Retry enabled:", self.asm_retry_label)
        fields_layout.addRow("Last init OK:", self.asm_last_ok_label)
        fields_layout.addRow("Init cycles:", self.asm_init_cycles_label)
        fields_layout.addRow("Init OK count:", self.asm_init_ok_label)
        fields_layout.addRow("Init fail count:", self.asm_init_fail_label)
        fields_layout.addRow("I2C address:", self.asm_addr_label)
        fields_layout.addRow("WHOAMI:", self.asm_whoami_label)
        fields_layout.addRow("CTRL3_C:", self.asm_ctrl3_label)
        fields_layout.addRow("CTRL4_C:", self.asm_ctrl4_label)
        fields_layout.addRow("CTRL9_XL:", self.asm_ctrl9_label)
        fields_layout.addRow("IRQ count:", self.asm_irq_label)
        fields_layout.addRow("Samples:", self.asm_samples_label)
        fields_layout.addRow("Status:", self.asm_status_label)
        fields_layout.addRow("No sample count:", self.asm_nosample_label)
        fields_layout.addRow("DRDY pin:", self.asm_drdy_label)
        fields_layout.addRow("Last RAW:", self.asm_raw_label)
        fields_layout.addRow("Last init event:", self.asm_init_event_label)
        layout.addWidget(fields_box)

        self.asm_imu_label = QLabel("IMU: waiting")
        self.asm_imu_label.setStyleSheet("font-weight: 600;")
        layout.addWidget(self.asm_imu_label)

        self.asm_log = QPlainTextEdit()
        self.asm_log.setReadOnly(True)
        self.asm_log.setMaximumBlockCount(1200)
        layout.addWidget(self.asm_log)

    def _build_sd_tab(self):
        layout = QVBoxLayout(self.sd_tab)

        cmd_row = QHBoxLayout()

        sd_on_btn = QPushButton("Start Recording")
        sd_on_btn.clicked.connect(lambda: self._send_serial_line("SDREC ON"))
        cmd_row.addWidget(sd_on_btn)

        sd_off_btn = QPushButton("Stop Recording")
        sd_off_btn.clicked.connect(lambda: self._send_serial_line("SDREC OFF"))
        cmd_row.addWidget(sd_off_btn)

        sd_toggle_btn = QPushButton("Toggle")
        sd_toggle_btn.clicked.connect(lambda: self._send_serial_line("SDREC TOGGLE"))
        cmd_row.addWidget(sd_toggle_btn)

        sd_state_btn = QPushButton("Read SD State")
        sd_state_btn.clicked.connect(lambda: self._send_serial_line("SDSTATE"))
        cmd_row.addWidget(sd_state_btn)

        clear_btn = QPushButton("Clear SD Log")
        clear_btn.clicked.connect(lambda: self.sd_log.clear())
        cmd_row.addWidget(clear_btn)

        load_csv_btn = QPushButton("Load CSV Log")
        load_csv_btn.clicked.connect(self._load_csv_log)
        cmd_row.addWidget(load_csv_btn)
        cmd_row.addStretch()
        layout.addLayout(cmd_row)

        status_box = QGroupBox("SD Status")
        status_layout = QFormLayout(status_box)
        self.sd_record_state_label = QLabel("--")
        self.sd_card_present_label = QLabel("--")
        self.sd_health_label = QLabel("--")
        self.sd_ready_label = QLabel("--")
        self.sd_error_label = QLabel("--")
        self.sd_read_ok_label = QLabel("--")
        self.sd_write_ok_label = QLabel("--")
        self.sd_req_label = QLabel("--")
        self.sd_active_label = QLabel("--")
        self.sd_open_label = QLabel("--")
        self.sd_next_index_label = QLabel("--")
        self.sd_last_file_label = QLabel("--")
        self.sd_last_event_label = QLabel(self.sd_last_event_text)
        self.sd_last_event_label.setWordWrap(True)

        status_layout.addRow("Recording:", self.sd_record_state_label)
        status_layout.addRow("Card present:", self.sd_card_present_label)
        status_layout.addRow("Health:", self.sd_health_label)
        status_layout.addRow("Card ready:", self.sd_ready_label)
        status_layout.addRow("Error state:", self.sd_error_label)
        status_layout.addRow("Readable:", self.sd_read_ok_label)
        status_layout.addRow("Writable:", self.sd_write_ok_label)
        status_layout.addRow("Request flag:", self.sd_req_label)
        status_layout.addRow("Active flag:", self.sd_active_label)
        status_layout.addRow("File open:", self.sd_open_label)
        status_layout.addRow("Next LOG index:", self.sd_next_index_label)
        status_layout.addRow("Last opened file:", self.sd_last_file_label)
        status_layout.addRow("Last event:", self.sd_last_event_label)
        layout.addWidget(status_box)

        self.sd_log = QPlainTextEdit()
        self.sd_log.setReadOnly(True)
        self.sd_log.setMaximumBlockCount(3000)
        layout.addWidget(self.sd_log)

        self.csv_log_summary_label = QLabel(self.csv_log_summary_text)
        self.csv_log_summary_label.setWordWrap(True)
        layout.addWidget(self.csv_log_summary_label)

        self.csv_log_preview = QPlainTextEdit()
        self.csv_log_preview.setReadOnly(True)
        self.csv_log_preview.setMaximumBlockCount(1200)
        layout.addWidget(self.csv_log_preview)

    def _build_raw_tab(self):
        layout = QVBoxLayout(self.raw_tab)

        cmd_row = QHBoxLayout()
        self.raw_cmd_input = QLineEdit()
        self.raw_cmd_input.setPlaceholderText("Raw command to telemetry board (e.g. TX,B,S,200,6,01 03 00 00 00 A5)")
        cmd_row.addWidget(self.raw_cmd_input)

        send_btn = QPushButton("Send")
        send_btn.clicked.connect(self._send_raw_command)
        cmd_row.addWidget(send_btn)
        layout.addLayout(cmd_row)

        self.raw_log = QPlainTextEdit()
        self.raw_log.setReadOnly(True)
        self.raw_log.setMaximumBlockCount(5000)
        layout.addWidget(self.raw_log)

    def _build_modem_tab(self):
        layout = QVBoxLayout(self.modem_tab)

        cmd_row = QHBoxLayout()
        for text, cmd in [
            ("Read Modem State", "MODEMSTATE"),
            ("Connect PDP", "MODEMCONNECT"),
            ("Disconnect PDP", "MODEMDISCONNECT"),
            ("Pulse PWRKEY", "MODEMPWRKEY"),
            ("Reset Modem", "MODEMRESET"),
            ("Read MQTT State", "MQTTSTATE"),
            ("MQTT On", "MQTTON"),
            ("MQTT Off", "MQTTOFF"),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, c=cmd: self._send_serial_line(c))
            cmd_row.addWidget(btn)

        clear_btn = QPushButton("Clear Modem Log")
        clear_btn.clicked.connect(lambda: self.modem_log.clear())
        cmd_row.addWidget(clear_btn)
        cmd_row.addStretch()
        layout.addLayout(cmd_row)

        apn_box = QGroupBox("APN Configuration")
        apn_layout = QGridLayout(apn_box)
        self.apn_input = QLineEdit()
        self.apn_user_input = QLineEdit()
        self.apn_pass_input = QLineEdit()
        self.apn_pass_input.setEchoMode(QLineEdit.Password)
        apn_layout.addWidget(QLabel("APN:"), 0, 0)
        apn_layout.addWidget(self.apn_input, 0, 1)
        apn_layout.addWidget(QLabel("User:"), 0, 2)
        apn_layout.addWidget(self.apn_user_input, 0, 3)
        apn_layout.addWidget(QLabel("Pass:"), 0, 4)
        apn_layout.addWidget(self.apn_pass_input, 0, 5)
        apn_btn = QPushButton("Apply APN")
        apn_btn.clicked.connect(self._send_modem_apn)
        apn_layout.addWidget(apn_btn, 0, 6)
        layout.addWidget(apn_box)

        http_box = QGroupBox("HTTP Connectivity Test")
        http_layout = QGridLayout(http_box)
        self.http_host_input = QLineEdit("example.com")
        self.http_path_input = QLineEdit("/")
        http_layout.addWidget(QLabel("Host:"), 0, 0)
        http_layout.addWidget(self.http_host_input, 0, 1)
        http_layout.addWidget(QLabel("Path:"), 0, 2)
        http_layout.addWidget(self.http_path_input, 0, 3)
        http_btn = QPushButton("Run HTTP Test")
        http_btn.clicked.connect(self._send_modem_http_test)
        http_layout.addWidget(http_btn, 0, 4)
        layout.addWidget(http_box)

        mqtt_cfg_box = QGroupBox("MQTT Broker Configuration")
        mqtt_cfg_layout = QGridLayout(mqtt_cfg_box)
        self.mqtt_host_input = QLineEdit("9ddfaf6f481045449c7efc293f3a389f.s1.eu.hivemq.cloud")
        self.mqtt_port_input = QLineEdit("8883")
        self.mqtt_client_input = QLineEdit("telemetry-node")
        self.mqtt_prefix_input = QLineEdit("szen/telemetry/node")
        mqtt_cfg_layout.addWidget(QLabel("Host:"), 0, 0)
        mqtt_cfg_layout.addWidget(self.mqtt_host_input, 0, 1)
        mqtt_cfg_layout.addWidget(QLabel("Port:"), 0, 2)
        mqtt_cfg_layout.addWidget(self.mqtt_port_input, 0, 3)
        mqtt_cfg_layout.addWidget(QLabel("Client ID:"), 1, 0)
        mqtt_cfg_layout.addWidget(self.mqtt_client_input, 1, 1)
        mqtt_cfg_layout.addWidget(QLabel("Topic prefix:"), 1, 2)
        mqtt_cfg_layout.addWidget(self.mqtt_prefix_input, 1, 3)
        mqtt_cfg_btn = QPushButton("Apply MQTT Endpoint")
        mqtt_cfg_btn.clicked.connect(self._send_mqtt_config)
        mqtt_cfg_layout.addWidget(mqtt_cfg_btn, 0, 4, 2, 1)
        layout.addWidget(mqtt_cfg_box)

        mqtt_auth_box = QGroupBox("MQTT Authentication")
        mqtt_auth_layout = QGridLayout(mqtt_auth_box)
        self.mqtt_user_input = QLineEdit()
        self.mqtt_pass_input = QLineEdit()
        self.mqtt_pass_input.setEchoMode(QLineEdit.Password)
        mqtt_auth_layout.addWidget(QLabel("User:"), 0, 0)
        mqtt_auth_layout.addWidget(self.mqtt_user_input, 0, 1)
        mqtt_auth_layout.addWidget(QLabel("Pass:"), 0, 2)
        mqtt_auth_layout.addWidget(self.mqtt_pass_input, 0, 3)
        mqtt_auth_btn = QPushButton("Apply MQTT Auth")
        mqtt_auth_btn.clicked.connect(self._send_mqtt_auth)
        mqtt_auth_layout.addWidget(mqtt_auth_btn, 0, 4)
        layout.addWidget(mqtt_auth_box)

        status_box = QGroupBox("Modem State")
        status_layout = QFormLayout(status_box)
        self.modem_uart_label = QLabel("--")
        self.modem_uart_baud_label = QLabel("--")
        self.modem_uart_target_label = QLabel("--")
        self.modem_uart_probe_label = QLabel("--")
        self.modem_uart_flow_label = QLabel("--")
        self.modem_uart_high_label = QLabel("--")
        self.modem_ready_label = QLabel("--")
        self.modem_sim_label = QLabel("--")
        self.modem_network_label = QLabel("--")
        self.modem_gprs_label = QLabel("--")
        self.modem_internet_label = QLabel("--")
        self.modem_signal_label = QLabel("--")
        self.modem_http_code_label = QLabel("--")
        self.modem_apn_label = QLabel("--")
        self.modem_ip_label = QLabel("--")
        self.modem_operator_label = QLabel("--")
        self.modem_info_label = QLabel("--")
        self.modem_boot_timer_label = QLabel("possibly not booted modem")
        self.modem_event_label = QLabel(self.modem_last_event_text)
        self.modem_event_label.setWordWrap(True)
        status_layout.addRow("UART ready:", self.modem_uart_label)
        status_layout.addRow("UART baud:", self.modem_uart_baud_label)
        status_layout.addRow("UART target:", self.modem_uart_target_label)
        status_layout.addRow("Last UART probe:", self.modem_uart_probe_label)
        status_layout.addRow("Flow control:", self.modem_uart_flow_label)
        status_layout.addRow("High-speed UART:", self.modem_uart_high_label)
        status_layout.addRow("Modem ready:", self.modem_ready_label)
        status_layout.addRow("SIM ready:", self.modem_sim_label)
        status_layout.addRow("Network attached:", self.modem_network_label)
        status_layout.addRow("PDP / GPRS:", self.modem_gprs_label)
        status_layout.addRow("Internet test:", self.modem_internet_label)
        status_layout.addRow("Signal quality:", self.modem_signal_label)
        status_layout.addRow("HTTP status:", self.modem_http_code_label)
        status_layout.addRow("APN:", self.modem_apn_label)
        status_layout.addRow("Local IP:", self.modem_ip_label)
        status_layout.addRow("Operator:", self.modem_operator_label)
        status_layout.addRow("Modem info:", self.modem_info_label)
        status_layout.addRow("12s boot timer:", self.modem_boot_timer_label)
        status_layout.addRow("Last event:", self.modem_event_label)
        layout.addWidget(status_box)

        mqtt_state_box = QGroupBox("MQTT State")
        mqtt_state_layout = QFormLayout(mqtt_state_box)
        self.mqtt_cfg_label = QLabel("--")
        self.mqtt_enabled_label = QLabel("--")
        self.mqtt_socket_label = QLabel("--")
        self.mqtt_endpoint_label = QLabel("--")
        self.mqtt_prefix_label = QLabel("--")
        self.mqtt_tx_label = QLabel("--")
        self.mqtt_rx_label = QLabel("--")
        self.mqtt_reconnect_label = QLabel("--")
        self.mqtt_dropped_label = QLabel("--")
        self.mqtt_event_label = QLabel(self.mqtt_last_event_text)
        self.mqtt_event_label.setWordWrap(True)
        mqtt_state_layout.addRow("Configured:", self.mqtt_cfg_label)
        mqtt_state_layout.addRow("Enabled:", self.mqtt_enabled_label)
        mqtt_state_layout.addRow("Socket:", self.mqtt_socket_label)
        mqtt_state_layout.addRow("Endpoint:", self.mqtt_endpoint_label)
        mqtt_state_layout.addRow("Topic prefix:", self.mqtt_prefix_label)
        mqtt_state_layout.addRow("TX count:", self.mqtt_tx_label)
        mqtt_state_layout.addRow("RX count:", self.mqtt_rx_label)
        mqtt_state_layout.addRow("Reconnects:", self.mqtt_reconnect_label)
        mqtt_state_layout.addRow("Dropped inbound:", self.mqtt_dropped_label)
        mqtt_state_layout.addRow("Last MQTT event:", self.mqtt_event_label)
        layout.addWidget(mqtt_state_box)

        self.modem_log = QPlainTextEdit()
        self.modem_log.setReadOnly(True)
        self.modem_log.setMaximumBlockCount(2000)
        layout.addWidget(self.modem_log)

    def _build_gnss_tab(self):
        layout = QVBoxLayout(self.gnss_tab)

        cmd_row = QHBoxLayout()
        for text, cmd in [
            ("Read GNSS State", "GNSSSTATE"),
            ("Reset GNSS", "GNSSRESET"),
            ("Read NTRIP State", "NTRIPSTATE"),
            ("Start NTRIP", "NTRIPON"),
            ("Stop NTRIP", "NTRIPOFF"),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, c=cmd: self._send_serial_line(c))
            cmd_row.addWidget(btn)

        clear_btn = QPushButton("Clear GNSS Log")
        clear_btn.clicked.connect(lambda: self.gnss_log.clear())
        cmd_row.addWidget(clear_btn)
        cmd_row.addStretch()
        layout.addLayout(cmd_row)

        ntrip_box = QGroupBox("NTRIP Configuration")
        ntrip_layout = QGridLayout(ntrip_box)
        self.ntrip_host_input = QLineEdit()
        self.ntrip_port_input = QLineEdit("2101")
        self.ntrip_mount_input = QLineEdit()
        self.ntrip_user_input = QLineEdit()
        self.ntrip_pass_input = QLineEdit()
        self.ntrip_pass_input.setEchoMode(QLineEdit.Password)
        ntrip_layout.addWidget(QLabel("Host:"), 0, 0)
        ntrip_layout.addWidget(self.ntrip_host_input, 0, 1)
        ntrip_layout.addWidget(QLabel("Port:"), 0, 2)
        ntrip_layout.addWidget(self.ntrip_port_input, 0, 3)
        ntrip_layout.addWidget(QLabel("Mount:"), 0, 4)
        ntrip_layout.addWidget(self.ntrip_mount_input, 0, 5)
        ntrip_layout.addWidget(QLabel("User:"), 1, 0)
        ntrip_layout.addWidget(self.ntrip_user_input, 1, 1)
        ntrip_layout.addWidget(QLabel("Pass:"), 1, 2)
        ntrip_layout.addWidget(self.ntrip_pass_input, 1, 3)
        ntrip_btn = QPushButton("Apply NTRIP Config")
        ntrip_btn.clicked.connect(self._send_ntrip_config)
        ntrip_layout.addWidget(ntrip_btn, 1, 5)
        layout.addWidget(ntrip_box)

        gnss_box = QGroupBox("GNSS State")
        gnss_layout = QFormLayout(gnss_box)
        self.gnss_uart_label = QLabel("--")
        self.gnss_ready_label = QLabel("--")
        self.gnss_cfg_label = QLabel("--")
        self.gnss_baud_label = QLabel("--")
        self.gnss_fix_label = QLabel("--")
        self.gnss_carrier_label = QLabel("--")
        self.gnss_siv_label = QLabel("--")
        self.gnss_position_label = QLabel("--")
        self.gnss_altitude_label = QLabel("--")
        self.gnss_accuracy_label = QLabel("--")
        self.gnss_tow_label = QLabel("--")
        self.gnss_pvt_label = QLabel("--")
        self.gnss_timepulse_label = QLabel("--")
        self.gnss_event_label = QLabel(self.gnss_last_event_text)
        self.gnss_event_label.setWordWrap(True)
        gnss_layout.addRow("UART ready:", self.gnss_uart_label)
        gnss_layout.addRow("GNSS ready:", self.gnss_ready_label)
        gnss_layout.addRow("Configured:", self.gnss_cfg_label)
        gnss_layout.addRow("UART baud:", self.gnss_baud_label)
        gnss_layout.addRow("Fix:", self.gnss_fix_label)
        gnss_layout.addRow("Carrier solution:", self.gnss_carrier_label)
        gnss_layout.addRow("Satellites:", self.gnss_siv_label)
        gnss_layout.addRow("Position:", self.gnss_position_label)
        gnss_layout.addRow("Altitude:", self.gnss_altitude_label)
        gnss_layout.addRow("Accuracy:", self.gnss_accuracy_label)
        gnss_layout.addRow("Time of week:", self.gnss_tow_label)
        gnss_layout.addRow("PVT count:", self.gnss_pvt_label)
        gnss_layout.addRow("Timepulse:", self.gnss_timepulse_label)
        gnss_layout.addRow("Last GNSS event:", self.gnss_event_label)
        layout.addWidget(gnss_box)

        ntrip_state_box = QGroupBox("NTRIP State")
        ntrip_state_layout = QFormLayout(ntrip_state_box)
        self.ntrip_cfg_label = QLabel("--")
        self.ntrip_enabled_label = QLabel("--")
        self.ntrip_socket_label = QLabel("--")
        self.ntrip_endpoint_label = QLabel("--")
        self.ntrip_rtcm_label = QLabel("--")
        self.ntrip_gga_label = QLabel("--")
        self.ntrip_reconnect_label = QLabel("--")
        self.ntrip_event_label = QLabel(self.ntrip_last_event_text)
        self.ntrip_event_label.setWordWrap(True)
        ntrip_state_layout.addRow("Configured:", self.ntrip_cfg_label)
        ntrip_state_layout.addRow("Enabled:", self.ntrip_enabled_label)
        ntrip_state_layout.addRow("Socket:", self.ntrip_socket_label)
        ntrip_state_layout.addRow("Endpoint:", self.ntrip_endpoint_label)
        ntrip_state_layout.addRow("RTCM bytes:", self.ntrip_rtcm_label)
        ntrip_state_layout.addRow("GGA messages:", self.ntrip_gga_label)
        ntrip_state_layout.addRow("Reconnects:", self.ntrip_reconnect_label)
        ntrip_state_layout.addRow("Last NTRIP event:", self.ntrip_event_label)
        layout.addWidget(ntrip_state_box)

        self.gnss_log = QPlainTextEdit()
        self.gnss_log.setReadOnly(True)
        self.gnss_log.setMaximumBlockCount(2500)
        layout.addWidget(self.gnss_log)

    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def _connect_selected_port(self):
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "No port", "Select a COM port first.")
            return
        self.serial_worker.set_target_port(port)

    def _disconnect_port(self):
        self.serial_worker.clear_target_port()

    def _on_serial_status(self, ok: bool, text: str):
        self.status_label.setText(text)
        self.status_label.setStyleSheet("font-weight:700; color:#0a7f2e;" if ok else "font-weight:700; color:#b00020;")
        if ok:
            self._send_serial_line("SDSTATE")
            self._send_serial_line("MODEMSTATE")
            self._send_serial_line("MQTTSTATE")
            self._send_serial_line("GNSSSTATE")
            self._send_serial_line("NTRIPSTATE")

    def _on_serial_line(self, line: str):
        self.serial_rx_bytes_total += len(line) + 1
        self._update_serial_utilization(len(line) + 1)
        self.rx_queue.put(line)

    def _send_serial_line(self, line: str):
        self.serial_tx_bytes_total += len(line) + 1
        self._update_serial_utilization(len(line) + 1)
        self.serial_worker.send_line(line)

    def _poll_sd_state(self):
        if self.status_label.text().startswith("Connected:"):
            for cmd in ("SDSTATE", "MODEMSTATE", "MQTTSTATE", "GNSSSTATE", "NTRIPSTATE"):
                self._send_serial_line(cmd)

    def _load_csv_log(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open Telemetry CSV Log",
            "",
            "CSV files (*.csv);;All files (*)",
        )
        if not file_path:
            return

        try:
            with open(file_path, "r", encoding="utf-8", errors="replace") as handle:
                lines = [line.rstrip("\r\n") for line in handle]
        except Exception as exc:
            QMessageBox.warning(self, "CSV load failed", f"Could not read file:\n{exc}")
            return

        if not lines:
            self.csv_log_summary_text = f"{file_path}: empty file"
            self.csv_log_summary_label.setText(self.csv_log_summary_text)
            self.csv_log_preview.setPlainText("")
            return

        data_lines = [line for line in lines if line and not line.startswith("TYPE,")]
        can1_count = sum(1 for line in data_lines if line.startswith("CAN1,"))
        can2_count = sum(1 for line in data_lines if line.startswith("CAN2,"))
        first_ms = None
        last_ms = None
        for line in data_lines:
            parts = line.split(",", 2)
            if len(parts) < 2:
                continue
            try:
                ms = int(parts[1])
            except ValueError:
                continue
            if first_ms is None:
                first_ms = ms
            last_ms = ms

        duration_ms = 0 if (first_ms is None or last_ms is None) else max(0, last_ms - first_ms)
        ratio_text = "n/a" if can1_count == 0 else f"{(can2_count / can1_count) * 100.0:.1f}%"
        self.csv_log_summary_text = (
            f"{file_path} | rows={len(data_lines)} | CAN1={can1_count} | CAN2={can2_count} "
            f"| CAN2/CAN1={ratio_text} | duration={duration_ms} ms"
        )
        self.csv_log_summary_label.setText(self.csv_log_summary_text)

        preview_head = data_lines[:30]
        preview_tail = data_lines[-30:] if len(data_lines) > 30 else []
        preview_lines = [f"Loaded: {file_path}", ""]
        preview_lines.extend(preview_head)
        if preview_tail:
            preview_lines.append("...")
            preview_lines.extend(preview_tail)
        self.csv_log_preview.setPlainText("\n".join(preview_lines))

    def _send_raw_command(self):
        cmd = self.raw_cmd_input.text().strip()
        if not cmd:
            return
        self._send_serial_line(cmd)
        self.raw_cmd_input.clear()

    def _send_asmreg_command(self):
        text = self.asmreg_input.text().strip().upper()
        if not text:
            return
        self._send_serial_line(f"ASMREG {text}")

    def _send_modem_apn(self):
        apn = self.apn_input.text().strip()
        user = self.apn_user_input.text().strip()
        password = self.apn_pass_input.text().strip()
        self._send_serial_line(f"MODEMAPN,{apn},{user},{password}")

    def _send_modem_http_test(self):
        host = self.http_host_input.text().strip()
        path = self.http_path_input.text().strip() or "/"
        if not host:
            QMessageBox.warning(self, "Missing host", "Enter an HTTP host first.")
            return
        self._send_serial_line(f"MODEMHTTP,{host},{path}")

    def _send_mqtt_config(self):
        host = self.mqtt_host_input.text().strip()
        port = self.mqtt_port_input.text().strip()
        client_id = self.mqtt_client_input.text().strip()
        topic_prefix = self.mqtt_prefix_input.text().strip()
        if not host or not port or not client_id or not topic_prefix:
            QMessageBox.warning(self, "Missing MQTT config", "Host, port, client ID, and topic prefix are required.")
            return
        self._send_serial_line(f"MQTTCFG,{host},{port},{client_id},{topic_prefix}")

    def _send_mqtt_auth(self):
        user = self.mqtt_user_input.text().strip() or "-"
        password = self.mqtt_pass_input.text().strip() or "-"
        self._send_serial_line(f"MQTTAUTH,{user},{password}")

    def _send_ntrip_config(self):
        host = self.ntrip_host_input.text().strip()
        port = self.ntrip_port_input.text().strip()
        mount = self.ntrip_mount_input.text().strip()
        user = self.ntrip_user_input.text().strip()
        password = self.ntrip_pass_input.text().strip()
        if not host or not port or not mount:
            QMessageBox.warning(self, "Missing NTRIP config", "Host, port, and mount point are required.")
            return
        self._send_serial_line(f"NTRIPCFG,{host},{port},{mount},{user},{password}")

    def _send_can_frame(self, can_id: int, data: list[int], bus: str = "1", extended: bool = False):
        payload = " ".join(f"{b & 0xFF:02X}" for b in data)
        frame_type = "E" if extended else "S"
        self._send_serial_line(f"TX,{bus},{frame_type},{can_id:X},{len(data)},{payload}")

    def _reset_can_compare(self):
        self.can_compare_pending_can1.clear()
        self.can_compare_pending_can2.clear()
        self.can_compare_match_count = 0
        self.can_compare_miss_count = 0

    def _prune_can_compare(self, now_ms: int):
        for pending_map in (self.can_compare_pending_can1, self.can_compare_pending_can2):
            empty_keys = []
            for signature, timestamps in pending_map.items():
                while timestamps and (now_ms - timestamps[0]) > CAN_COMPARE_TIMEOUT_MS:
                    timestamps.popleft()
                    self.can_compare_miss_count += 1
                if not timestamps:
                    empty_keys.append(signature)
            for signature in empty_keys:
                pending_map.pop(signature, None)

    def _track_can_compare(self, bus_name: str, can_id: int, dlc: int, payload: list[int], timestamp_ms: int):
        self._prune_can_compare(timestamp_ms)

        signature = (can_id, dlc, tuple(payload[:dlc]))
        if bus_name == "CAN1":
            own_map = self.can_compare_pending_can1
            other_map = self.can_compare_pending_can2
        elif bus_name == "CAN2":
            own_map = self.can_compare_pending_can2
            other_map = self.can_compare_pending_can1
        else:
            return

        other_timestamps = other_map.get(signature)
        if other_timestamps:
            other_timestamps.popleft()
            if not other_timestamps:
                other_map.pop(signature, None)
            self.can_compare_match_count += 1
            return

        own_map.setdefault(signature, deque()).append(timestamp_ms)

    def _estimate_can_frame_bits(self, extended: bool, dlc: int) -> int:
        payload_bits = max(0, min(8, dlc)) * 8
        base_bits = (67 if extended else 47) + payload_bits
        return int(base_bits * 1.2)

    def _prune_util_window(self, now: float):
        for window in (self.can_bits_window_can1, self.can_bits_window_can2):
            while window and (now - window[0][0]) > CAN_UTIL_WINDOW_S:
                window.popleft()

    def _prune_serial_window(self, now: float):
        while self.serial_bits_window and (now - self.serial_bits_window[0][0]) > SERIAL_UTIL_WINDOW_S:
            self.serial_bits_window.popleft()

    def _update_serial_utilization(self, byte_count: int):
        now = time.monotonic()
        bits = max(0, byte_count) * SERIAL_BITS_PER_BYTE
        self.serial_bits_window.append((now, bits))
        self._prune_serial_window(now)
        bits_total = sum(value for _, value in self.serial_bits_window)
        self.serial_utilization = min(100.0, (bits_total / BAUD_RATE) * 100.0)

    def _update_can_utilization(self, bus_name: str, extended: bool, dlc: int):
        now = time.monotonic()
        frame_bits = self._estimate_can_frame_bits(extended, dlc)
        if bus_name == "CAN1":
            self.can_bits_window_can1.append((now, frame_bits))
        elif bus_name == "CAN2":
            self.can_bits_window_can2.append((now, frame_bits))

        self._prune_util_window(now)
        bits_can1 = sum(bits for _, bits in self.can_bits_window_can1)
        bits_can2 = sum(bits for _, bits in self.can_bits_window_can2)
        self.can_utilization_can1 = min(100.0, (bits_can1 / CAN_BUS_BITRATE_BPS) * 100.0)
        self.can_utilization_can2 = min(100.0, (bits_can2 / CAN_BUS_BITRATE_BPS) * 100.0)

    def _build_pdu_mask(self) -> int:
        mask = 0
        for i, cb in enumerate(self.output_checks):
            if cb.isChecked():
                mask |= 1 << i
        return mask

    def _set_override_widgets_enabled(self, enabled: bool):
        for cb in self.output_checks:
            cb.setEnabled(enabled)
        self.start_on_check.setEnabled(enabled)

    def _on_override_toggled(self):
        enabled = self.override_check.isChecked()
        self._set_override_widgets_enabled(enabled)
        if enabled:
            for i, cb in enumerate(self.output_checks):
                cb.blockSignals(True)
                cb.setChecked(((self.pdu_requested_mask >> i) & 1) != 0)
                cb.blockSignals(False)
            self.start_on_check.setChecked((self.pdu_control_state_flags & CONTROL_STATE_FLAG_START_ON) != 0)
        self._send_pdu_keepalive()

    def _on_output_changed(self):
        if self.override_check.isChecked():
            self._send_pdu_command(CONTROL_OP_OUTPUT, 0)

    def _send_pdu_command(self, opcode: int, param0: int, param1: int = 0):
        manual = self.override_check.isChecked()
        mask = self._build_pdu_mask() if manual else self.pdu_requested_mask
        flags = 0
        if manual:
            flags |= OVERRIDE_FLAG_ARMED
        if self.start_on_check.isChecked():
            flags |= OVERRIDE_FLAG_START_ON
        if self.debug_check.isChecked():
            flags |= CONTROL_FLAG_DEBUG

        payload = [mask & 0xFF, flags & 0xFF, opcode & 0xFF, param0 & 0xFF, param1 & 0xFF, CONTROL_MAGIC]
        self._send_can_frame(CAN_ID_CONTROL, payload, bus="1", extended=False)

    def _send_pdu_keepalive(self):
        if self.override_check.isChecked() or self.debug_check.isChecked():
            self._send_pdu_command(CONTROL_OP_NOP, 0)

    def _process_incoming_lines(self):
        drained = 0
        while drained < 500:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            drained += 1
            self._handle_line(line)

    def _handle_line(self, line: str):
        self.raw_log.appendPlainText(line)

        if line.startswith("CAN1,") or line.startswith("CAN2,"):
            self._handle_prefixed_can_line(line)
            return

        if line.startswith("SD,") or line.startswith("BOOT,RECORD,"):
            self._handle_sd_line(line)
            return

        if line.startswith("MODEM,") or line.startswith("MQTT,"):
            self._handle_modem_line(line)
            return

        if line.startswith("GNSS,") or line.startswith("NTRIP,"):
            self._handle_gnss_line(line)
            return

        if line.startswith("IMU,"):
            self._handle_imu_line(line)
            return

        if line.startswith("ASMDBG") or line.startswith("BOOT,ASM330") or line.startswith("ASM330,"):
            self._handle_asm_line(line)
            return

        if line.startswith("CMD,"):
            self.asm_log.appendPlainText(line)

    def _handle_modem_line(self, line: str):
        self.modem_log.appendPlainText(line)
        if line.startswith("MQTT,"):
            self.mqtt_last_event_text = line
        else:
            self.modem_last_event_text = line

        if line.startswith("MODEM,STATE,"):
            for part in line.split(",")[2:]:
                if "=" not in part:
                    continue
                key, value = part.split("=", 1)
                self.modem_fields[key.strip()] = value.strip()
            return

        if line.startswith("MQTT,STATE,"):
            for part in line.split(",")[2:]:
                if "=" not in part:
                    continue
                key, value = part.split("=", 1)
                self.mqtt_fields[key.strip()] = value.strip()

    def _handle_gnss_line(self, line: str):
        self.gnss_log.appendPlainText(line)

        if line.startswith("GNSS,"):
            self.gnss_last_event_text = line
        if line.startswith("NTRIP,"):
            self.ntrip_last_event_text = line

        if line.startswith("GNSS,STATE,"):
            for part in line.split(",")[2:]:
                if "=" not in part:
                    continue
                key, value = part.split("=", 1)
                self.gnss_fields[key.strip()] = value.strip()
            return

        if line.startswith("NTRIP,STATE,"):
            for part in line.split(",")[2:]:
                if "=" not in part:
                    continue
                key, value = part.split("=", 1)
                self.ntrip_fields[key.strip()] = value.strip()

    def _handle_sd_line(self, line: str):
        self.sd_log.appendPlainText(line)
        self.sd_last_event_text = line

        if line.startswith("SD,CARD,INSERTED"):
            self.sd_fields["CARD"] = "1"
            return

        if line.startswith("SD,CARD,REMOVED"):
            self.sd_fields["CARD"] = "0"
            self.sd_fields["READY"] = "0"
            self.sd_fields["ACTIVE"] = "0"
            self.sd_fields["OPEN"] = "0"
            return

        if line.startswith("SD,OK"):
            self.sd_fields["CARD"] = "1"
            self.sd_fields["READY"] = "1"
            self.sd_fields["ERR"] = "0"
            return

        if line.startswith("SD,ERR,"):
            self.sd_fields["ERR"] = "1"
            return

        if line.startswith("SD,RECORD,ON"):
            self.sd_fields["REQ"] = "1"
            return

        if line.startswith("SD,RECORD,OFF"):
            self.sd_fields["REQ"] = "0"
            self.sd_fields["ACTIVE"] = "0"
            return

        if line.startswith("SD,LOG_OPEN,"):
            self.sd_fields["OPEN"] = "1"
            self.sd_fields["ACTIVE"] = "1"
            self.sd_fields["LAST_FILE"] = line.split(",", 2)[2].strip()
            return

        if line.startswith("SD,LOG_CLOSED"):
            self.sd_fields["OPEN"] = "0"
            self.sd_fields["ACTIVE"] = "0"
            return

        if line.startswith("BOOT,RECORD,"):
            req_on = line.endswith(",ON")
            self.sd_fields["REQ"] = "1" if req_on else "0"
            return

        if line.startswith("SD,STATE,"):
            for part in line.split(",")[2:]:
                if "=" not in part:
                    continue
                key, value = part.split("=", 1)
                self.sd_fields[key.strip()] = value.strip()

    def _handle_prefixed_can_line(self, line: str):
        parts = line.split(",", 5)
        if len(parts) < 6:
            return

        bus_name = parts[0].strip()
        try:
            timestamp_ms = int(parts[1].strip())
            extended = parts[2].strip().upper() == "E"
            can_id = int(parts[3].strip(), 16)
            dlc = int(parts[4].strip())
        except ValueError:
            return

        payload = parse_hex_payload(parts[5].strip(), dlc)
        if len(payload) < dlc:
            return

        self.rx_frames_total += 1
        self.fps_counter += 1
        if bus_name == "CAN1":
            self.rx_frames_can1 += 1
        elif bus_name == "CAN2":
            self.rx_frames_can2 += 1

        self._update_can_utilization(bus_name, extended, dlc)
        self.last_can_timestamp_ms = max(self.last_can_timestamp_ms, timestamp_ms)
        self._track_can_compare(bus_name, can_id, dlc, payload, timestamp_ms)

        self._decode_maxxecu(can_id, payload, timestamp_ms)
        self._decode_pdu_feedback(can_id, payload)

    def _decode_maxxecu(self, can_id: int, payload: list[int], timestamp_ms: int):
        for sig in MAXXECU_SIGNALS:
            if sig.can_id != can_id:
                continue
            raw = read_le(payload, sig.offset, sig.size, sig.signed)
            if raw is None:
                continue
            self.latest_values[sig.name] = raw * sig.scale
            self.latest_update_ms[sig.name] = timestamp_ms

    def _decode_pdu_feedback(self, can_id: int, payload: list[int]):
        if can_id == CAN_ID_TELEM_SUMMARY and len(payload) >= 8:
            self.vin_avg_mv = payload[0] | (payload[1] << 8)
            self.sum_current_da = payload[2] | (payload[3] << 8)
            temp_avg_raw = payload[4] | (payload[5] << 8)
            if temp_avg_raw & 0x8000:
                temp_avg_raw -= 1 << 16
            self.temp_avg_c = temp_avg_raw / 10.0
            self.err_flags = payload[6]
            self.flt_bits = payload[7]
            return

        if can_id == CAN_ID_TELEM_EXTRA and len(payload) >= 8:
            temp_peak_raw = payload[0] | (payload[1] << 8)
            if temp_peak_raw & 0x8000:
                temp_peak_raw -= 1 << 16
            self.temp_peak_c = temp_peak_raw / 10.0
            self.system_flags = payload[2]
            self.pdu_control_state_flags = payload[3]
            self.pdu_requested_mask = payload[4]
            self.pdu_applied_mask = payload[5]
            self.pdu_ecu_mask = payload[6]
            self.pdu_dash_mask = payload[7]
            return

        if can_id == CAN_ID_ECU_RAW_DEBUG and len(payload) >= 8:
            self.pdu_ecu_raw_flags = payload[0]
            self.pdu_ecu_raw_decoded = payload[1]
            self.pdu_ecu_raw_b0 = payload[2]
            self.pdu_ecu_raw_b1 = payload[3]
            self.pdu_ecu_raw_dlc = payload[4]
            self.pdu_ecu_raw_count = payload[5] | (payload[6] << 8)
            self.pdu_ecu_raw_requested = payload[7]
            return

        if CAN_ID_EFUSE_BASE <= can_id <= CAN_ID_EFUSE_LAST and len(payload) >= 8:
            idx = can_id - CAN_ID_EFUSE_BASE
            self.efuse_voltage[idx] = payload[0] | (payload[1] << 8)
            self.efuse_current[idx] = payload[2] | (payload[3] << 8)
            self.efuse_power[idx] = payload[4] | (payload[5] << 8)
            self.efuse_status_word[idx] = payload[6] | (payload[7] << 8)
            self.efuse_current_history[idx].append(float(self.efuse_current[idx]))
            return

        if CAN_ID_ADC_BASE <= can_id <= CAN_ID_ADC_LAST and len(payload) >= 7:
            idx = can_id - CAN_ID_ADC_BASE
            self.efuse_adc_current[idx] = payload[0] | (payload[1] << 8)
            self.efuse_adc_voltage[idx] = payload[2] | (payload[3] << 8)
            self.efuse_adc_diff[idx] = payload[4] | (payload[5] << 8)
            self.efuse_adc_flags[idx] = payload[6]
            self.efuse_cml_status[idx] = payload[7] if len(payload) >= 8 else 0
            return

        if CAN_ID_TEMP_BASE <= can_id <= CAN_ID_TEMP_LAST and len(payload) >= 3:
            idx = can_id - CAN_ID_TEMP_BASE
            temp_raw = payload[0] | (payload[1] << 8)
            if temp_raw & 0x8000:
                temp_raw -= 1 << 16
            self.efuse_temp_c[idx] = temp_raw / 10.0 if payload[2] != 0 else None
            return

        if can_id == CAN_ID_MCU and len(payload) >= 4:
            self.flt_bits = payload[0]
            self.system_flags = payload[1]
            self.mcu_shunt = payload[2] | (payload[3] << 8)
            self.shunt_history.append(float(self.mcu_shunt))
            return

        if can_id == CAN_ID_RETRY_MODE_STAT and len(payload) >= 2:
            self.pdu_retry_mode = payload[0]
            self.pdu_retry_applied = payload[1]
            return

        if can_id == CAN_ID_ADC_REF_STAT and len(payload) >= 2:
            self.pdu_adc_ref = payload[0]
            self.pdu_adc_ref_applied = payload[1]

    def _handle_imu_line(self, line: str):
        parts = line.split(",")
        if len(parts) < 10:
            return
        self.last_imu_text = (
            f"t={parts[1]}ms  Acc[g]=({parts[2]}, {parts[3]}, {parts[4]})  "
            f"Gyro[dps]=({parts[5]}, {parts[6]}, {parts[7]})  |a|={parts[8]}  HardStop={parts[9]}"
        )

    def _handle_asm_line(self, line: str):
        self.asm_log.appendPlainText(line)

        if line.startswith("ASM330,INIT,START"):
            self.asm_init_event_text = "Retry loop started"
        elif line.startswith("ASM330,INIT,STOP"):
            self.asm_init_event_text = "Retry loop stopped"
        elif line.startswith("ASM330,INIT,ATTEMPT,"):
            self.asm_init_event_text = line.replace("ASM330,INIT,ATTEMPT,", "Attempt #")

        if ",LAST_RAW," in line:
            self.asm_raw_label.setText(line.split(",LAST_RAW,", 1)[1].strip())
            return

        if "=" not in line:
            return

        for part in line.split(","):
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            self.asm_fields[key.strip()] = value.strip()

    def _refresh_ui(self):
        now = time.monotonic()
        elapsed = max(0.001, now - self.last_fps_reset)
        fps = int(self.fps_counter / elapsed)
        if elapsed >= 1.0:
            self.last_fps_reset = now
            self.fps_counter = 0

        self._prune_util_window(now)
        bits_can1 = sum(bits for _, bits in self.can_bits_window_can1)
        bits_can2 = sum(bits for _, bits in self.can_bits_window_can2)
        self.can_utilization_can1 = min(100.0, (bits_can1 / CAN_BUS_BITRATE_BPS) * 100.0)
        self.can_utilization_can2 = min(100.0, (bits_can2 / CAN_BUS_BITRATE_BPS) * 100.0)
        self._prune_serial_window(now)
        serial_bits_total = sum(bits for _, bits in self.serial_bits_window)
        self.serial_utilization = min(100.0, (serial_bits_total / BAUD_RATE) * 100.0)

        self._prune_can_compare(self.last_can_timestamp_ms)
        pending_total = sum(len(items) for items in self.can_compare_pending_can1.values()) + sum(
            len(items) for items in self.can_compare_pending_can2.values()
        )

        self.overview_stats_label.setText(
            f"CAN1 RX: {self.rx_frames_can1}   CAN2 RX: {self.rx_frames_can2}   Total: {self.rx_frames_total}   Serial FPS: {fps}"
        )
        self.can1_util_bar.setValue(int(self.can_utilization_can1))
        self.can2_util_bar.setValue(int(self.can_utilization_can2))
        self.serial_util_bar.setValue(int(self.serial_utilization))
        self.can1_util_label.setText(f"{self.can_utilization_can1:.1f}%")
        self.can2_util_label.setText(f"{self.can_utilization_can2:.1f}%")
        self.serial_util_label.setText(f"{self.serial_utilization:.1f}%")
        self.can_compare_counts_label.setText(
            f"matched: {self.can_compare_match_count}   missed: {self.can_compare_miss_count}   pending: {pending_total}"
        )

        if self.can_compare_match_count == 0 and self.can_compare_miss_count == 0 and pending_total == 0:
            self.can_compare_label.setText("CAN1/CAN2 compare: waiting")
            self.can_compare_label.setStyleSheet("font-weight: 700; color:#666;")
        elif self.can_compare_miss_count == 0 and pending_total == 0:
            self.can_compare_label.setText("CAN1/CAN2 compare: MATCH")
            self.can_compare_label.setStyleSheet("font-weight: 700; color:#0a7f2e;")
        elif self.can_compare_miss_count == 0:
            self.can_compare_label.setText("CAN1/CAN2 compare: pending")
            self.can_compare_label.setStyleSheet("font-weight: 700; color:#9a6a00;")
        else:
            self.can_compare_label.setText("CAN1/CAN2 compare: MISMATCH")
            self.can_compare_label.setStyleSheet("font-weight: 700; color:#b00020;")

        self.overview_imu_label.setText(self.last_imu_text)
        self.asm_imu_label.setText(self.last_imu_text)

        for key, label in self.card_value_labels.items():
            value = self.latest_values.get(key)
            if value is None:
                label.setText("--")
            elif isinstance(value, float):
                label.setText(f"{value:.2f}")
            else:
                label.setText(str(value))

        for row, sig in enumerate(MAXXECU_SIGNALS):
            value = self.latest_values.get(sig.name)
            if value is None:
                text = "--"
            elif isinstance(value, float):
                text = f"{value:.3f}"
            else:
                text = str(value)
            self.signal_table.item(row, 1).setText(text)
            self.signal_table.item(row, 2).setText(str(self.latest_update_ms.get(sig.name, 0)))

        if self.pdu_control_state_flags & CONTROL_STATE_FLAG_OVERRIDE_ACTIVE:
            owner = "Manual override"
        elif self.pdu_control_state_flags & CONTROL_STATE_FLAG_ECU_FRESH:
            owner = "MaxxECU"
        else:
            owner = "No fresh command"

        details = []
        if self.pdu_control_state_flags & CONTROL_STATE_FLAG_OVERRIDE_ARMED:
            details.append("armed")
        if self.pdu_control_state_flags & CONTROL_STATE_FLAG_SAFETY_BLOCKED:
            details.append("safety blocked")
        if self.pdu_control_state_flags & CONTROL_STATE_FLAG_HYBRID_LATCHED:
            details.append("hybrid latched")
        if self.pdu_control_state_flags & CONTROL_STATE_FLAG_DEBUG_ACTIVE:
            details.append("debug")
        if details:
            owner += " (" + ", ".join(details) + ")"

        self.pdu_owner_label.setText(owner)
        self.pdu_mask_label.setText(
            f"Req 0x{self.pdu_requested_mask:02X}  App 0x{self.pdu_applied_mask:02X}  ECU 0x{self.pdu_ecu_mask:02X}  Dash 0x{self.pdu_dash_mask:02X}"
        )

        raw_seen = 1 if (self.pdu_ecu_raw_flags & ECU_RAW_FLAG_SEEN) else 0
        raw_fresh = 1 if (self.pdu_ecu_raw_flags & ECU_RAW_FLAG_FRESH) else 0
        raw_byte1 = 1 if (self.pdu_ecu_raw_flags & ECU_RAW_FLAG_BYTE1) else 0
        raw_dlc_ok = 1 if (self.pdu_ecu_raw_flags & ECU_RAW_FLAG_DLC_OK) else 0
        self.pdu_raw_label.setText(
            (
                f"seen={raw_seen} fresh={raw_fresh} byte1={raw_byte1} dlc_ok={raw_dlc_ok} "
                f"dec=0x{self.pdu_ecu_raw_decoded:02X} req=0x{self.pdu_ecu_raw_requested:02X} "
                f"raw=0x{self.pdu_ecu_raw_b0:02X}/0x{self.pdu_ecu_raw_b1:02X} dlc={self.pdu_ecu_raw_dlc} count={self.pdu_ecu_raw_count}"
            )
        )

        self.pdu_retry_label.setText(f"mode={self.pdu_retry_mode} ({'applied' if self.pdu_retry_applied else 'pending'})")
        self.pdu_adc_label.setText(f"ref={self.pdu_adc_ref} ({'applied' if self.pdu_adc_ref_applied else 'pending'})")

        total_current_a = self.sum_current_da / 10.0
        vin_v = self.vin_avg_mv / 1000.0
        power_w = total_current_a * vin_v

        self.summary_vin_label.setText(f"{self.vin_avg_mv} mV")
        self.summary_current_label.setText(f"{total_current_a:.2f} A")
        self.summary_power_label.setText(f"{power_w:.2f} W")
        self.summary_temp_label.setText(f"avg {self.temp_avg_c:.1f} C, peak {self.temp_peak_c:.1f} C")
        self.summary_shunt_label.setText(f"{self.mcu_shunt} mA")
        self.summary_err_label.setText(f"Err flags 0x{self.err_flags:02X}")

        self._refresh_output_check_colors()
        self._refresh_flag_labels()
        self._refresh_efuse_table()
        self._refresh_graphs()

        self.asm_ready_label.setText(self.asm_fields.get("READY", "--"))
        self.asm_retry_label.setText(self.asm_fields.get("RETRY", "--"))
        self.asm_last_ok_label.setText(self.asm_fields.get("LAST_OK", "--"))
        self.asm_init_cycles_label.setText(self.asm_fields.get("INIT_CYCLES", "--"))
        self.asm_init_ok_label.setText(self.asm_fields.get("INIT_OK", "--"))
        self.asm_init_fail_label.setText(self.asm_fields.get("INIT_FAIL", "--"))
        self.asm_addr_label.setText(self.asm_fields.get("ADDR", "--"))
        self.asm_whoami_label.setText(self.asm_fields.get("WHOAMI", "--"))
        self.asm_ctrl3_label.setText(self.asm_fields.get("CTRL3_C", "--"))
        self.asm_ctrl4_label.setText(self.asm_fields.get("CTRL4_C", "--"))
        self.asm_ctrl9_label.setText(self.asm_fields.get("CTRL9_XL", "--"))
        self.asm_irq_label.setText(self.asm_fields.get("IRQ", "--"))
        self.asm_samples_label.setText(self.asm_fields.get("SAMPLES", "--"))
        self.asm_status_label.setText(self.asm_fields.get("STATUS", "--"))
        self.asm_nosample_label.setText(self.asm_fields.get("NOSAMPLE", "--"))
        self.asm_drdy_label.setText(self.asm_fields.get("DRDY_PIN", "--"))
        self.asm_init_event_label.setText(self.asm_init_event_text)

        sd_ready = self.sd_fields.get("READY", "--")
        sd_card = self.sd_fields.get("CARD", "--")
        sd_err = self.sd_fields.get("ERR", "--")
        sd_read_ok = self.sd_fields.get("READ_OK", "--")
        sd_write_ok = self.sd_fields.get("WRITE_OK", "--")
        sd_req = self.sd_fields.get("REQ", "--")
        sd_active = self.sd_fields.get("ACTIVE", "--")
        sd_open = self.sd_fields.get("OPEN", "--")
        sd_next_index = self.sd_fields.get("NEXT_INDEX", "--")
        sd_last_file = self.sd_fields.get("LAST_FILE", "--")

        self.sd_card_present_label.setText(sd_card)
        self.sd_ready_label.setText(sd_ready)
        self.sd_error_label.setText(sd_err)
        self.sd_read_ok_label.setText(sd_read_ok)
        self.sd_write_ok_label.setText(sd_write_ok)
        self.sd_req_label.setText(sd_req)
        self.sd_active_label.setText(sd_active)
        self.sd_open_label.setText(sd_open)
        self.sd_next_index_label.setText(sd_next_index)
        self.sd_last_file_label.setText(sd_last_file)
        self.sd_last_event_label.setText(self.sd_last_event_text)

        recording_active = sd_active == "1"
        self.sd_record_state_label.setText("RECORDING" if recording_active else "IDLE")
        self.sd_record_state_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if recording_active else "font-weight:700; color:#666;"
        )

        healthy = sd_card == "1" and sd_ready == "1" and sd_err == "0" and sd_read_ok == "1" and sd_write_ok == "1"
        self.sd_health_label.setText("OK" if healthy else "FAIL")
        self.sd_health_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if healthy else "font-weight:700; color:#b00020;"
        )

        self.sd_card_present_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if sd_card == "1" else "font-weight:700; color:#b00020;"
        )
        self.sd_ready_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if sd_ready == "1" else "font-weight:700; color:#b00020;"
        )

        modem_ready = self.modem_fields.get("READY", "--")
        modem_sim = self.modem_fields.get("SIM", "--")
        modem_net = self.modem_fields.get("NET", "--")
        modem_gprs = self.modem_fields.get("GPRS", "--")
        modem_internet = self.modem_fields.get("INTERNET", "--")
        uart_ready = self.modem_fields.get("UART", "--")
        uart_baud = self.modem_fields.get("UART_BAUD", "--")
        uart_target = self.modem_fields.get("UART_TARGET", "--")
        uart_probe = self.modem_fields.get("UART_PROBE", "--")
        uart_flow = self.modem_fields.get("FLOW", "--")
        uart_high = self.modem_fields.get("HIGH", "--")
        self.modem_uart_label.setText(uart_ready)
        self.modem_uart_baud_label.setText(f"{uart_baud} bps" if uart_baud not in {"--", "0"} else uart_baud)
        self.modem_uart_target_label.setText(f"{uart_target} bps" if uart_target not in {"--", "0"} else uart_target)
        self.modem_uart_probe_label.setText(f"{uart_probe} bps" if uart_probe not in {"--", "0"} else uart_probe)
        self.modem_uart_flow_label.setText("RTS/CTS enabled" if uart_flow == "1" else "off" if uart_flow == "0" else uart_flow)
        self.modem_uart_high_label.setText("active" if uart_high == "1" else "standard" if uart_high == "0" else uart_high)
        self.modem_ready_label.setText(modem_ready)
        self.modem_sim_label.setText(modem_sim)
        self.modem_network_label.setText(modem_net)
        self.modem_gprs_label.setText(modem_gprs)
        self.modem_internet_label.setText(modem_internet)
        self.modem_signal_label.setText(self.modem_fields.get("CSQ", "--"))
        self.modem_http_code_label.setText(self.modem_fields.get("HTTP_CODE", "--"))
        self.modem_apn_label.setText(self.modem_fields.get("APN", "--"))
        self.modem_ip_label.setText(self.modem_fields.get("IP", "--"))
        self.modem_operator_label.setText(self.modem_fields.get("OP", "--"))
        self.modem_info_label.setText(self.modem_fields.get("INFO", "--"))
        boot12 = self.modem_fields.get("BOOT12", "0")
        uptime_ms_text = self.modem_fields.get("UP_MS", "0")
        boot_timer_text = "possibly not booted modem"
        try:
            uptime_s = int(uptime_ms_text) / 1000.0
            boot_timer_text = f"{'modem booted' if boot12 == '1' else 'possibly not booted modem'} ({uptime_s:.1f}s since boot)"
        except ValueError:
            if boot12 == "1":
                boot_timer_text = "modem booted"
        self.modem_boot_timer_label.setText(boot_timer_text)
        self.modem_event_label.setText(self.modem_last_event_text)
        self.modem_uart_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if uart_ready == "1" else "font-weight:700; color:#b00020;"
        )
        self.modem_uart_high_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if uart_high == "1" else "font-weight:700; color:#8a6d00;"
        )
        self.modem_ready_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if modem_ready == "1" else "font-weight:700; color:#b00020;"
        )
        self.modem_network_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if modem_net == "1" else "font-weight:700; color:#b00020;"
        )
        self.modem_gprs_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if modem_gprs == "1" else "font-weight:700; color:#b00020;"
        )
        self.modem_internet_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if modem_internet == "1" else "font-weight:700; color:#b00020;"
        )
        self.modem_boot_timer_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if boot12 == "1" else "font-weight:700; color:#9a6a00;"
        )

        mqtt_cfg = self.mqtt_fields.get("CFG", "--")
        mqtt_enabled = self.mqtt_fields.get("EN", "--")
        mqtt_socket = self.mqtt_fields.get("SOCK", "--")
        self.mqtt_cfg_label.setText(mqtt_cfg)
        self.mqtt_enabled_label.setText(mqtt_enabled)
        self.mqtt_socket_label.setText(mqtt_socket)
        self.mqtt_endpoint_label.setText(
            f"{self.mqtt_fields.get('HOST', '--')}:{self.mqtt_fields.get('PORT', '--')} / {self.mqtt_fields.get('CLIENT', '--')}"
        )
        self.mqtt_prefix_label.setText(self.mqtt_fields.get("PREFIX", "--"))
        self.mqtt_tx_label.setText(self.mqtt_fields.get("TX", "--"))
        self.mqtt_rx_label.setText(self.mqtt_fields.get("RX", "--"))
        self.mqtt_reconnect_label.setText(self.mqtt_fields.get("RECONNECTS", "--"))
        self.mqtt_dropped_label.setText(self.mqtt_fields.get("DROPPED", "--"))
        self.mqtt_event_label.setText(self.mqtt_last_event_text)
        self.mqtt_cfg_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if mqtt_cfg == "1" else "font-weight:700; color:#b00020;"
        )
        self.mqtt_enabled_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if mqtt_enabled == "1" else "font-weight:700; color:#666;"
        )
        self.mqtt_socket_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if mqtt_socket == "1" else "font-weight:700; color:#b00020;"
        )

        gnss_ready = self.gnss_fields.get("READY", "--")
        gnss_cfg = self.gnss_fields.get("CFG", "--")
        gnss_fix_valid = self.gnss_fields.get("FIX_VALID", "--")
        gnss_carrier = self.gnss_fields.get("CARR", "--")
        self.gnss_uart_label.setText(self.gnss_fields.get("UART", "--"))
        self.gnss_ready_label.setText(gnss_ready)
        self.gnss_cfg_label.setText(gnss_cfg)
        self.gnss_baud_label.setText(self.gnss_fields.get("UART_BAUD", "--"))
        fix_type = self.gnss_fields.get("FIX", "--")
        self.gnss_fix_label.setText(f"{fix_type} ({'valid' if gnss_fix_valid == '1' else 'no fix'})")
        carrier_text = {"0": "none", "1": "RTK float", "2": "RTK fixed"}.get(gnss_carrier, gnss_carrier)
        self.gnss_carrier_label.setText(carrier_text)
        self.gnss_siv_label.setText(self.gnss_fields.get("SIV", "--"))

        lat_text = self.gnss_fields.get("LAT_E7", "0")
        lon_text = self.gnss_fields.get("LON_E7", "0")
        try:
            lat = int(lat_text) / 10_000_000.0
            lon = int(lon_text) / 10_000_000.0
            self.gnss_position_label.setText(f"{lat:.7f}, {lon:.7f}")
        except ValueError:
            self.gnss_position_label.setText("--")

        alt_text = self.gnss_fields.get("ALT_MM", "0")
        try:
            self.gnss_altitude_label.setText(f"{int(alt_text) / 1000.0:.3f} m")
        except ValueError:
            self.gnss_altitude_label.setText("--")

        hacc_text = self.gnss_fields.get("HACC_MM", "0")
        vacc_text = self.gnss_fields.get("VACC_MM", "0")
        try:
            self.gnss_accuracy_label.setText(f"H {int(hacc_text) / 1000.0:.3f} m / V {int(vacc_text) / 1000.0:.3f} m")
        except ValueError:
            self.gnss_accuracy_label.setText("--")

        self.gnss_tow_label.setText(self.gnss_fields.get("TOW", "--"))
        self.gnss_pvt_label.setText(self.gnss_fields.get("PVT", "--"))
        self.gnss_timepulse_label.setText(self.gnss_fields.get("TIMEPULSE", "--"))
        self.gnss_event_label.setText(self.gnss_last_event_text)
        self.gnss_ready_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if gnss_ready == "1" else "font-weight:700; color:#b00020;"
        )
        self.gnss_cfg_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if gnss_cfg == "1" else "font-weight:700; color:#b00020;"
        )
        self.gnss_carrier_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if gnss_carrier == "2" else "font-weight:700; color:#9a6a00;" if gnss_carrier == "1" else "font-weight:700; color:#666;"
        )

        ntrip_cfg = self.ntrip_fields.get("CFG", "--")
        ntrip_enabled = self.ntrip_fields.get("EN", "--")
        ntrip_socket = self.ntrip_fields.get("SOCK", "--")
        self.ntrip_cfg_label.setText(ntrip_cfg)
        self.ntrip_enabled_label.setText(ntrip_enabled)
        self.ntrip_socket_label.setText(ntrip_socket)
        self.ntrip_endpoint_label.setText(
            f"{self.ntrip_fields.get('HOST', '--')}:{self.ntrip_fields.get('PORT', '--')} / {self.ntrip_fields.get('MOUNT', '--')}"
        )
        self.ntrip_rtcm_label.setText(self.ntrip_fields.get("RTCM_BYTES", "--"))
        self.ntrip_gga_label.setText(self.ntrip_fields.get("GGA_TX", "--"))
        self.ntrip_reconnect_label.setText(self.ntrip_fields.get("RECONNECTS", "--"))
        self.ntrip_event_label.setText(self.ntrip_last_event_text)
        self.ntrip_cfg_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if ntrip_cfg == "1" else "font-weight:700; color:#b00020;"
        )
        self.ntrip_enabled_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if ntrip_enabled == "1" else "font-weight:700; color:#666;"
        )
        self.ntrip_socket_label.setStyleSheet(
            "font-weight:700; color:#0a7f2e;" if ntrip_socket == "1" else "font-weight:700; color:#b00020;"
        )

    def _refresh_output_check_colors(self):
        for i, cb in enumerate(self.output_checks):
            requested = ((self.pdu_requested_mask >> i) & 1) != 0
            applied = ((self.pdu_applied_mask >> i) & 1) != 0

            if requested != applied:
                color = "#f6c343"
            elif applied:
                color = "#7ad17a"
            else:
                color = "#e17373"

            pal = cb.palette()
            pal.setColor(cb.backgroundRole(), QColor(color))
            cb.setAutoFillBackground(True)
            cb.setPalette(pal)

    def _refresh_flag_labels(self):
        for i, lbl in enumerate(self.flt_flag_labels):
            active = ((self.flt_bits >> i) & 1) != 0
            lbl.setStyleSheet(
                "padding:4px; color:white; background:#ba2d2d;" if active else "padding:4px; color:white; background:#2b6b3b;"
            )

        for i, lbl in enumerate(self.sys_flag_labels):
            active = ((self.system_flags >> i) & 1) != 0
            lbl.setStyleSheet(
                "padding:4px; color:white; background:#ba2d2d;" if active else "padding:4px; color:white; background:#2b6b3b;"
            )

    def _refresh_efuse_table(self):
        for i in range(8):
            temp_text = "--" if self.efuse_temp_c[i] is None else f"{self.efuse_temp_c[i]:.1f}"
            cml_flags = self.efuse_cml_status[i]
            cml_text = "|".join(name for bit, name in TPS_STATUS_CML_BITS if (cml_flags >> bit) & 1)
            if not cml_text:
                cml_text = "OK"

            desired = (self.pdu_requested_mask >> i) & 1
            applied = (self.pdu_applied_mask >> i) & 1
            raw = (self.pdu_ecu_raw_requested >> i) & 1

            self.efuse_table.item(i, 1).setText(str(self.efuse_voltage[i]))
            self.efuse_table.item(i, 2).setText(str(self.efuse_current[i]))
            self.efuse_table.item(i, 3).setText(str(self.efuse_power[i]))
            self.efuse_table.item(i, 4).setText(temp_text)
            self.efuse_table.item(i, 5).setText(str(self.efuse_adc_current[i]))
            self.efuse_table.item(i, 6).setText(str(self.efuse_adc_voltage[i]))
            self.efuse_table.item(i, 7).setText(str(self.efuse_adc_diff[i]))
            self.efuse_table.item(i, 8).setText(cml_text)
            self.efuse_table.item(i, 9).setText(f"0x{self.efuse_status_word[i]:04X}")
            self.efuse_table.item(i, 10).setText(f"{desired}/{applied}")
            self.efuse_table.item(i, 11).setText(str(raw))

    def _refresh_graphs(self):
        self.shunt_graph.set_samples(list(self.shunt_history))
        for i, graph in enumerate(self.efuse_graphs):
            graph.set_samples(list(self.efuse_current_history[i]))


def main():
    app = QApplication(sys.argv)
    window = TelemetryWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
