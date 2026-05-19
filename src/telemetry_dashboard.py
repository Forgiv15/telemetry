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
CAN_COMPARE_TIMEOUT_S = 0.35


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
        self.can_compare_pending_can1: dict[tuple[int, int, tuple[int, ...]], deque[float]] = {}
        self.can_compare_pending_can2: dict[tuple[int, int, tuple[int, ...]], deque[float]] = {}
        self.can_compare_match_count = 0
        self.can_compare_miss_count = 0
        self.last_fps_reset = time.monotonic()
        self.fps_counter = 0
        self.can_bits_window_can1: deque[tuple[float, int]] = deque()
        self.can_bits_window_can2: deque[tuple[float, int]] = deque()
        self.can_utilization_can1 = 0.0
        self.can_utilization_can2 = 0.0

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
        self.raw_tab = QWidget()

        self.tabs.addTab(self.overview_tab, "Overview")
        self.tabs.addTab(self.pdu_tab, "PDU Control + Diagnostics")
        self.tabs.addTab(self.asm_tab, "ASM330 Debug")
        self.tabs.addTab(self.raw_tab, "Raw")

        self._build_overview_tab()
        self._build_pdu_tab()
        self._build_asm_tab()
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

    def _on_serial_line(self, line: str):
        self.rx_queue.put(line)

    def _send_serial_line(self, line: str):
        self.serial_worker.send_line(line)

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

    def _send_can_frame(self, can_id: int, data: list[int], bus: str = "1", extended: bool = False):
        payload = " ".join(f"{b & 0xFF:02X}" for b in data)
        frame_type = "E" if extended else "S"
        self._send_serial_line(f"TX,{bus},{frame_type},{can_id:X},{len(data)},{payload}")

    def _reset_can_compare(self):
        self.can_compare_pending_can1.clear()
        self.can_compare_pending_can2.clear()
        self.can_compare_match_count = 0
        self.can_compare_miss_count = 0

    def _prune_can_compare(self, now: float):
        for pending_map in (self.can_compare_pending_can1, self.can_compare_pending_can2):
            empty_keys = []
            for signature, timestamps in pending_map.items():
                while timestamps and (now - timestamps[0]) > CAN_COMPARE_TIMEOUT_S:
                    timestamps.popleft()
                    self.can_compare_miss_count += 1
                if not timestamps:
                    empty_keys.append(signature)
            for signature in empty_keys:
                pending_map.pop(signature, None)

    def _track_can_compare(self, bus_name: str, can_id: int, dlc: int, payload: list[int]):
        now = time.monotonic()
        self._prune_can_compare(now)

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

        own_map.setdefault(signature, deque()).append(now)

    def _estimate_can_frame_bits(self, extended: bool, dlc: int) -> int:
        payload_bits = max(0, min(8, dlc)) * 8
        base_bits = (67 if extended else 47) + payload_bits
        return int(base_bits * 1.2)

    def _prune_util_window(self, now: float):
        for window in (self.can_bits_window_can1, self.can_bits_window_can2):
            while window and (now - window[0][0]) > CAN_UTIL_WINDOW_S:
                window.popleft()

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

        if line.startswith("IMU,"):
            self._handle_imu_line(line)
            return

        if line.startswith("ASMDBG") or line.startswith("BOOT,ASM330") or line.startswith("ASM330,"):
            self._handle_asm_line(line)
            return

        if line.startswith("CMD,"):
            self.asm_log.appendPlainText(line)

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
        self._track_can_compare(bus_name, can_id, dlc, payload)

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

        self._prune_can_compare(now)
        pending_total = sum(len(items) for items in self.can_compare_pending_can1.values()) + sum(
            len(items) for items in self.can_compare_pending_can2.values()
        )

        self.overview_stats_label.setText(
            f"CAN1 RX: {self.rx_frames_can1}   CAN2 RX: {self.rx_frames_can2}   Total: {self.rx_frames_total}   Serial FPS: {fps}"
        )
        self.can1_util_bar.setValue(int(self.can_utilization_can1))
        self.can2_util_bar.setValue(int(self.can_utilization_can2))
        self.can1_util_label.setText(f"{self.can_utilization_can1:.1f}%")
        self.can2_util_label.setText(f"{self.can_utilization_can2:.1f}%")
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
