import serial
import threading
import queue
import tkinter as tk
from tkinter import ttk
from tkinter import simpledialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from serial.tools import list_ports
from collections import deque
import time

# ================= CONFIG =================

SERIAL_PORT = "COM8"
BAUD_RATE = 921600

EFUSE_COUNT = 8
HISTORY_SECONDS = 5
UPDATE_RATE_HZ = 20
HISTORY_LENGTH = HISTORY_SECONDS * UPDATE_RATE_HZ
WATERFALL_MAX_LINES = 500

CAN_ID_CONTROL = 0x400
CAN_ID_TELEM_SUMMARY = 0x410
CAN_ID_TELEM_EXTRA = 0x411

# Debug/verbose telemetry (only when debug flag is armed)
CAN_ID_EFUSE_BASE = 0x600
CAN_ID_EFUSE_LAST = CAN_ID_EFUSE_BASE + EFUSE_COUNT - 1
CAN_ID_ADC_BASE = 0x610
CAN_ID_ADC_LAST = CAN_ID_ADC_BASE + EFUSE_COUNT - 1
CAN_ID_TEMP_BASE = 0x618
CAN_ID_TEMP_LAST = CAN_ID_TEMP_BASE + EFUSE_COUNT - 1
CAN_ID_MCU = 0x620
CAN_ID_ERR_DETAIL_BASE = 0x630
CAN_ID_ERR_DETAIL_LAST = CAN_ID_ERR_DETAIL_BASE + 7
CAN_ID_I2C_SCAN_BASE = 0x640
CAN_ID_I2C_SCAN_LAST = CAN_ID_I2C_SCAN_BASE + 3
CAN_ID_PMBUS_DBG_META = 0x660
CAN_ID_PMBUS_DBG_DATA = 0x661
CAN_ID_PMBUS_DBG_EXT = 0x662
CAN_ID_RETRY_MODE_STAT = 0x690
CAN_ID_ADC_REF_STAT = 0x691  # Existing constant

CONTROL_MAGIC = 0xA5
CONTROL_OP_OUTPUT = 0x01
CONTROL_OP_RETRY_MODE = 0x02
CONTROL_OP_CLEAR_FAULTS = 0x03
CONTROL_OP_ADC_REF = 0x04
CONTROL_OP_NOP = 0x00
CONTROL_FLAG_DEBUG = 0x01

EFUSE_NAMES = [
    "Hybrid",
    "Vent1",
    "Vent2",
    "IGN",
    "Fuel",
    "WP1",
    "WP2",
    "12V",
]

OUTPUT_SWITCH_NAMES = EFUSE_NAMES + ["Other Fused"]

RETRY_MODE_NAMES = {
    1: "Fast SC Retry",
    2: "Race Mode",
    3: "Test Mode",
}

ADC_REFERENCE_NAMES = {
    0: "External 2.048V",
    1: "Internal 2.048V",
}

PMBUS_CMD_NAMES = {
    0x03: "CLEAR_FAULTS",
    0x79: "STATUS_WORD",
    0x8B: "READ_VOUT",
    0x89: "READ_IIN",
    0x97: "READ_PIN",
    0xA0: "ADC_COMPARE",
}

MCU_ERR_NAMES = {
    0: "OK",
    1: "NACK",
    2: "TIMEOUT",
    3: "PEC",
    4: "BUS",
    10: "CAN_TX",
    20: "ENABLE_FAIL",
    21: "SCAN_NO_ACK",
    30: "ADC_MISMATCH",
}

PMBUS_STATUS_NAMES = {
    0: "OK",
    1: "NACK",
    2: "TIMEOUT",
    3: "PEC",
    4: "BUS",
}

SERCOM_ERR_NAMES = {
    0: "NONE",
    1: "NAK",
    2: "BUS",
}

PMBUS_PEC_MODE_NAMES = {
    0: "REQ",
    1: "OFF",
    2: "AUTO",
}

PMBUS_TRACE_FLAG_NAMES = {
    0x01: "PEC",
    0x02: "FB_NOPEC",
}

PMBUS_OP_NAMES = {
    1: "SEND_BYTE",
    2: "WRITE_BYTE",
    3: "WRITE_WORD",
    4: "READ_BYTE",
    5: "READ_WORD",
}

PMBUS_FAULT_NAMES = [
    "OP_READ",
    "OP_WRITE",
    "ACK",
    "NACK",
    "TIMEOUT",
    "PEC_ERR",
    "BUS_ERR",
    "PARAM_ERR",
    "START_ERR",
]

TPS_STATUS_CML_BITS = [
    (7, "INV_CMD"),
    (6, "INV_DATA"),
    (5, "INV_PEC"),
    (4, "MEM_FLT"),
    (0, "OTHER"),
]
# ================= STATUS WORD BIT NAMES =================

TPS_STATUS_WORD_BITS = [
    "OUT_STATUS",       # 15
    "IOUT_STATUS",      # 14
    "INPUT_STATUS",     # 13
    "MFR_STATUS",       # 12
    "PGOODB",           # 11
    "RES10",            # 10
    "RES9",             # 9
    "UNKNOWN",          # 8
    "BUSY",             # 7
    "FET_OFF",          # 6
    "RES5",             # 5
    "RES4",             # 4
    "VIN_UV_FLT",       # 3
    "TEMP_FLT",         # 2
    "CML_ERR",          # 1
    "OTHER",            # 0
]

# ================= DATA STORAGE =================

data_queue = queue.Queue()

voltage = [0]*EFUSE_COUNT
current = [0]*EFUSE_COUNT
power   = [0]*EFUSE_COUNT
status_word = [0]*EFUSE_COUNT
adc_current = [0]*EFUSE_COUNT
adc_voltage = [0]*EFUSE_COUNT
adc_diff = [0]*EFUSE_COUNT
adc_flags = [0]*EFUSE_COUNT
cml_status = [0]*EFUSE_COUNT
temperature_c = [None]*EFUSE_COUNT

current_history = [deque(maxlen=HISTORY_LENGTH) for _ in range(EFUSE_COUNT)]

mcu_shunt = 0
flt_bits = 0
system_flags = 0
mcu_dbg_stage = 0
mcu_dbg_error = 0
mcu_dbg_fail_channel = 0xFF
mcu_dbg_fail_command = 0
active_retry_mode = 0
retry_mode_applied = 0
active_adc_reference = 0
adc_reference_applied = 0
sum_current_da = 0
vin_avg_mv = 0
temp_avg_c = 0.0
temp_peak_c = 0.0
err_flags = 0
debug_active = 0

active_error_details = {}
i2c_scan_blocks = {}
pmbus_debug_pending = {}
pmbus_debug_lines = deque(maxlen=400)

system_flag_names = [
    "ENABLE_FAIL",
    "READ_FAIL",
    "PEC_FAIL",
    "TIMEOUT",
    "NACK_OR_BUS",
    "ZERO_TELEM",
    "CAN_TX_FAIL",
    "SCAN_NO_ACK",
]

dashboard_flag_names = [
    "FLT",
    "FLTM",
    "SDC",
    "BSPD",
    "BOTS",
    "INERTIA",
]

selected_serial_port = SERIAL_PORT
active_serial_port = None
serial_connected = False
serial_error = "Not connected"
serial_instance = None
serial_state_lock = threading.Lock()
telemetry_lock = threading.Lock()


def list_available_ports():
    return [port.device for port in list_ports.comports()]


def set_target_port(port_name):
    global selected_serial_port
    with serial_state_lock:
        selected_serial_port = port_name


def disconnect_serial_port():
    global selected_serial_port, serial_instance
    with serial_state_lock:
        selected_serial_port = None
        if serial_instance is not None:
            try:
                serial_instance.close()
            except Exception:
                pass


def get_serial_state():
    with serial_state_lock:
        return serial_connected, active_serial_port, serial_error, selected_serial_port


def send_serial_line(line):
    with serial_state_lock:
        ser = serial_instance
        connected = serial_connected

    if (not connected) or (ser is None) or (not ser.is_open):
        return False

    try:
        ser.write((line + "\n").encode("ascii"))
        ser.flush()
        return True
    except Exception:
        return False


def send_can_standard_frame(can_id, data_bytes):
    payload = " ".join(f"{byte & 0xFF:02X}" for byte in data_bytes)
    return send_serial_line(f"TX,S,{can_id:03X},{len(data_bytes)},{payload}")


def send_control_frame(opcode, param0=0, param1=0, debug=False):
    flags = CONTROL_FLAG_DEBUG if debug else 0
    return send_can_standard_frame(CAN_ID_CONTROL, [opcode & 0xFF, param0 & 0xFF, param1 & 0xFF, flags, CONTROL_MAGIC])


def send_output_control(channel_index, enabled, debug=False):
    return send_control_frame(CONTROL_OP_OUTPUT, channel_index, 1 if enabled else 0, debug)


def send_retry_mode_control(mode, debug=False):
    return send_control_frame(CONTROL_OP_RETRY_MODE, mode & 0xFF, 0, debug)


def send_fault_clear_command(channel_index=0xFF, debug=False):
    return send_control_frame(CONTROL_OP_CLEAR_FAULTS, channel_index & 0xFF, 0, debug)


def send_adc_reference_control(reference, debug=False):
    return send_control_frame(CONTROL_OP_ADC_REF, reference & 0xFF, 0, debug)


def send_debug_pulse():
    return send_control_frame(CONTROL_OP_NOP, 0, 0, debug=True)


def parse_bridge_can_line(line):
    parts = line.split(",", 4)
    if len(parts) < 5:
        return None

    try:
        timestamp_ms = int(parts[0].strip())
    except ValueError:
        return None

    frame_type = parts[1].strip().upper()
    if frame_type not in ("S", "E"):
        return None

    try:
        can_id = int(parts[2].strip(), 16)
        dlc = int(parts[3].strip())
    except ValueError:
        return None

    if dlc < 0 or dlc > 8:
        return None

    payload_str = parts[4].strip()
    payload_tokens = payload_str.split() if payload_str else []
    data_bytes = []

    for token in payload_tokens[:dlc]:
        try:
            data_bytes.append(int(token, 16))
        except ValueError:
            return None

    return {
        "timestamp_ms": timestamp_ms,
        "frame_type": frame_type,
        "can_id": can_id,
        "dlc": dlc,
        "data": data_bytes,
    }


def decode_can_frame(frame):
    global mcu_shunt, flt_bits, system_flags
    global mcu_dbg_stage, mcu_dbg_error, mcu_dbg_fail_channel, mcu_dbg_fail_command
    global active_error_details, i2c_scan_blocks, pmbus_debug_pending, pmbus_debug_lines
    global adc_current, adc_voltage, adc_diff, adc_flags
    global cml_status, temperature_c, active_retry_mode, retry_mode_applied
    global active_adc_reference, adc_reference_applied
    global sum_current_da, vin_avg_mv, temp_avg_c, temp_peak_c, err_flags, debug_active

    can_id = frame["can_id"]
    data = frame["data"]

    with telemetry_lock:
        if can_id == CAN_ID_TELEM_SUMMARY:
            if len(data) < 8:
                return

            vin_avg_mv = data[0] | (data[1] << 8)
            sum_current_da = data[2] | (data[3] << 8)
            temp_avg_raw = data[4] | (data[5] << 8)
            if temp_avg_raw & 0x8000:
                temp_avg_raw -= 0x10000
            temp_avg_c = temp_avg_raw / 10.0
            err_flags = data[6]
            flt_bits = data[7]

        elif can_id == CAN_ID_TELEM_EXTRA:
            if len(data) < 4:
                return

            temp_peak_raw = data[0] | (data[1] << 8)
            if temp_peak_raw & 0x8000:
                temp_peak_raw -= 0x10000
            temp_peak_c = temp_peak_raw / 10.0
            system_flags = data[2]
            debug_active = data[3]

        elif CAN_ID_EFUSE_BASE <= can_id <= CAN_ID_EFUSE_LAST:
            if len(data) < 8:
                return

            idx = can_id - CAN_ID_EFUSE_BASE
            voltage[idx] = data[0] | (data[1] << 8)
            current[idx] = data[2] | (data[3] << 8)
            power[idx] = data[4] | (data[5] << 8)
            status_word[idx] = data[6] | (data[7] << 8)
            current_history[idx].append(current[idx])

        elif CAN_ID_ADC_BASE <= can_id <= CAN_ID_ADC_LAST:
            if len(data) < 7:
                return

            idx = can_id - CAN_ID_ADC_BASE
            adc_current[idx] = data[0] | (data[1] << 8)
            adc_voltage[idx] = data[2] | (data[3] << 8)
            adc_diff[idx] = data[4] | (data[5] << 8)
            adc_flags[idx] = data[6]
            cml_status[idx] = data[7] if len(data) >= 8 else 0

        elif CAN_ID_TEMP_BASE <= can_id <= CAN_ID_TEMP_LAST:
            if len(data) < 3:
                return

            idx = can_id - CAN_ID_TEMP_BASE
            temp_raw = data[0] | (data[1] << 8)
            if temp_raw & 0x8000:
                temp_raw -= 0x10000
            temperature_c[idx] = (temp_raw / 10.0) if data[2] != 0 else None

        elif can_id == CAN_ID_MCU:
            if len(data) < 4:
                return

            flt_bits = data[0]
            system_flags = data[1]
            mcu_shunt = data[2] | (data[3] << 8)

            if len(data) >= 8:
                mcu_dbg_stage = data[4]
                mcu_dbg_error = data[5]
                mcu_dbg_fail_channel = data[6]
                mcu_dbg_fail_command = data[7]

        elif can_id == CAN_ID_RETRY_MODE_STAT:
            if len(data) < 2:
                return

            active_retry_mode = data[0]
            retry_mode_applied = data[1]

        elif can_id == CAN_ID_ADC_REF_STAT:
            if len(data) < 2:
                return

            active_adc_reference = data[0]
            adc_reference_applied = data[1]

        elif CAN_ID_ERR_DETAIL_BASE <= can_id <= CAN_ID_ERR_DETAIL_LAST:
            if len(data) < 6:
                return

            slot = can_id - CAN_ID_ERR_DETAIL_BASE
            active = data[1] != 0

            if active:
                active_error_details[slot] = {
                    "err": data[2],
                    "channel": data[3],
                    "command": data[4],
                    "ttl": data[5],
                }
            else:
                if slot in active_error_details:
                    del active_error_details[slot]

        elif CAN_ID_I2C_SCAN_BASE <= can_id <= CAN_ID_I2C_SCAN_LAST:
            if len(data) < 6:
                return

            slot = can_id - CAN_ID_I2C_SCAN_BASE
            i2c_scan_blocks[slot] = {
                "base": data[1],
                "mask": data[2],
                "found": data[3],
                "start": data[4],
                "end": data[5],
            }

        elif can_id == CAN_ID_PMBUS_DBG_META:
            if len(data) < 7:
                return

            seq = data[0]
            entry = pmbus_debug_pending.get(seq, {})
            entry["meta"] = {
                "seq": seq,
                "op": data[1],
                "addr": data[2],
                "cmd": data[3],
                "status": data[4],
                "fault_flags": data[5] | (data[6] << 8),
                "sercom_error": data[7] if len(data) >= 8 else 0,
            }
            pmbus_debug_pending[seq] = entry

            if "data" in entry:
                append_pmbus_trace_line(seq)

        elif can_id == CAN_ID_PMBUS_DBG_DATA:
            if len(data) < 8:
                return

            seq = data[0]
            entry = pmbus_debug_pending.get(seq, {})
            entry["data"] = {
                "seq": seq,
                "tx0": data[1],
                "tx1": data[2],
                "tx2": data[3],
                "rx0": data[4],
                "rx1": data[5],
                "rx2": data[6],
                "tx_len": (data[7] >> 4) & 0x0F,
                "rx_len": data[7] & 0x0F,
            }
            pmbus_debug_pending[seq] = entry

            if "meta" in entry:
                append_pmbus_trace_line(seq)

        elif can_id == CAN_ID_PMBUS_DBG_EXT:
            if len(data) < 7:
                return

            seq = data[0]
            entry = pmbus_debug_pending.get(seq, {})
            entry["ext"] = {
                "seq": seq,
                "tx3": data[1],
                "pec_calc": data[2],
                "pec_rx": data[3],
                "trace_flags": data[4],
                "pec_mode": data[5],
                "sercom_error_last": data[6],
            }
            pmbus_debug_pending[seq] = entry

            if "meta" in entry and "data" in entry:
                append_pmbus_trace_line(seq)


def fault_flags_to_text(flags):
    labels = []
    for bit, name in enumerate(PMBUS_FAULT_NAMES):
        if (flags >> bit) & 1:
            labels.append(name)
    return "|".join(labels) if labels else "NONE"


def append_pmbus_trace_line(seq):
    global pmbus_debug_pending, pmbus_debug_lines

    entry = pmbus_debug_pending.get(seq)
    if not entry:
        return
    if "meta" not in entry or "data" not in entry:
        return

    meta = entry["meta"]
    dat = entry["data"]
    ext = entry.get("ext", {})

    op_name = PMBUS_OP_NAMES.get(meta["op"], f"OP{meta['op']}")
    st_name = PMBUS_STATUS_NAMES.get(meta["status"], f"S{meta['status']}")
    flags_text = fault_flags_to_text(meta["fault_flags"])
    serr_name = SERCOM_ERR_NAMES.get(meta.get("sercom_error", 0), f"E{meta.get('sercom_error', 0)}")
    trace_flag_bits = ext.get("trace_flags", 0)
    trace_labels = [name for mask, name in PMBUS_TRACE_FLAG_NAMES.items() if (trace_flag_bits & mask) != 0]
    trace_text = "|".join(trace_labels) if trace_labels else "-"
    pec_mode = PMBUS_PEC_MODE_NAMES.get(ext.get("pec_mode", 255), f"M{ext.get('pec_mode', 255)}")

    line = (
        f"SEQ:{meta['seq']:03d} {op_name} "
        f"A:0x{meta['addr']:02X} C:0x{meta['cmd']:02X} "
        f"ST:{st_name} "
        f"TX[{dat['tx_len']}]:{dat['tx0']:02X} {dat['tx1']:02X} {dat['tx2']:02X} {ext.get('tx3', 0):02X} "
        f"RX[{dat['rx_len']}]:{dat['rx0']:02X} {dat['rx1']:02X} {dat['rx2']:02X} "
        f"PEC:{ext.get('pec_calc', 0):02X}/{ext.get('pec_rx', 0):02X} "
        f"SE:{serr_name} PM:{pec_mode} TF:{trace_text} F:{flags_text}"
    )

    pmbus_debug_lines.append(line)
    del pmbus_debug_pending[seq]


def decode_legacy_csv(line):
    global mcu_shunt, flt_bits, system_flags
    global sum_current_da, vin_avg_mv, temp_avg_c, temp_peak_c, err_flags, debug_active

    parts = line.split(",")

    if parts[0] == "410" and len(parts) >= 7:
        with telemetry_lock:
            vin_avg_mv = int(parts[1])
            sum_current_da = int(parts[2])
            temp_avg_c = int(parts[3]) / 10.0
            err_flags = int(parts[5], 16)
            flt_bits = int(parts[6], 16)
        return True

    if parts[0] == "411" and len(parts) >= 5:
        with telemetry_lock:
            temp_peak_c = int(parts[1]) / 10.0
            system_flags = int(parts[2], 16)
            debug_active = int(parts[3], 16)
        return True

    if parts[0] == "510" and len(parts) >= 6:
        idx = int(parts[1])
        if 0 <= idx < EFUSE_COUNT:
            with telemetry_lock:
                voltage[idx] = int(parts[2])
                current[idx] = int(parts[3])
                power[idx] = int(parts[4])
                status_word[idx] = int(parts[5], 16)
                current_history[idx].append(current[idx])
        return True

    if parts[0] == "520" and len(parts) >= 4:
        with telemetry_lock:
            mcu_shunt = int(parts[1])
            flt_bits = int(parts[2], 16)
            system_flags = int(parts[3], 16)
        return True

    return False

# ================= SERIAL THREAD =================

def serial_thread():
    global mcu_shunt, flt_bits, system_flags
    global serial_connected, active_serial_port, serial_error, serial_instance

    ser = None

    while True:
        with serial_state_lock:
            target_port = selected_serial_port

        if not target_port:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None

            with serial_state_lock:
                serial_instance = None
                active_serial_port = None
                serial_connected = False
                serial_error = "Disconnected"

            time.sleep(0.2)
            continue

        if ser is None or not ser.is_open or ser.port != target_port:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None

            try:
                ser = serial.Serial(target_port, BAUD_RATE, timeout=1)
                with serial_state_lock:
                    serial_instance = ser
                    active_serial_port = target_port
                    serial_connected = True
                    serial_error = ""
            except Exception as exc:
                with serial_state_lock:
                    serial_instance = None
                    active_serial_port = None
                    serial_connected = False
                    serial_error = str(exc)
                time.sleep(1.0)
                continue

        try:
            line = ser.readline().decode(errors="replace").strip()
            if not line:
                continue

            data_queue.put(line)

            frame = parse_bridge_can_line(line)
            if frame is not None:
                decode_can_frame(frame)
            else:
                try:
                    decode_legacy_csv(line)
                except (ValueError, IndexError):
                    pass

        except Exception as exc:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None

            with serial_state_lock:
                serial_instance = None
                active_serial_port = None
                serial_connected = False
                serial_error = str(exc)

            time.sleep(0.3)

# ================= GUI =================

class PDUDashboard:

    def __init__(self, root):
        self.root = root
        root.title("Professional PDU Dashboard")

        self.page_index = 0
        self.serial_port_var = tk.StringVar(value=SERIAL_PORT)
        self.debug_enabled_var = tk.BooleanVar(value=False)
        self.last_debug_keepalive = 0.0

        self.build_menu()
        self.build_serial_panel()
        self.build_output_controls()

        self.build_top()
        self.build_dashboard_flags()
        self.build_plots()
        self.build_diagnostics_window()

        self.refresh_ports()

        self.update_gui()

    def build_menu(self):
        menu_bar = tk.Menu(self.root)
        serial_menu = tk.Menu(menu_bar, tearoff=0)
        self.ports_menu = tk.Menu(serial_menu, tearoff=0)

        serial_menu.add_cascade(label="Select COM Port", menu=self.ports_menu)
        serial_menu.add_command(label="Refresh Ports", command=self.refresh_ports)
        serial_menu.add_separator()
        serial_menu.add_command(label="Connect", command=self.connect_selected_port)
        serial_menu.add_command(label="Disconnect", command=self.disconnect_port)

        menu_bar.add_cascade(label="Serial", menu=serial_menu)
        self.root.config(menu=menu_bar)

    def build_serial_panel(self):
        serial_frame = tk.LabelFrame(self.root, text="Serial Connection")
        serial_frame.pack(fill="x", pady=5)

        ttk.Label(serial_frame, text="Port:").pack(side="left", padx=(8, 4))

        self.port_combo = ttk.Combobox(
            serial_frame,
            textvariable=self.serial_port_var,
            state="readonly",
            width=12,
        )
        self.port_combo.pack(side="left", padx=4, pady=4)

        ttk.Button(serial_frame, text="Refresh", command=self.refresh_ports).pack(
            side="left", padx=4
        )
        ttk.Button(serial_frame, text="Connect", command=self.connect_selected_port).pack(
            side="left", padx=4
        )
        ttk.Button(serial_frame, text="Disconnect", command=self.disconnect_port).pack(
            side="left", padx=4
        )
        ttk.Button(serial_frame, text="Diagnostics Window", command=self.show_diagnostics_window).pack(
            side="left", padx=4
        )

        self.serial_status_label = tk.Label(serial_frame, text="Status: Disconnected", fg="red")
        self.serial_status_label.pack(side="left", padx=10)

    def refresh_ports(self):
        ports = list_available_ports()

        self.port_combo["values"] = ports

        selected = self.serial_port_var.get()
        if ports:
            if selected not in ports:
                self.serial_port_var.set(ports[0])
        else:
            self.serial_port_var.set("")

        self.ports_menu.delete(0, "end")
        if ports:
            for port in ports:
                self.ports_menu.add_radiobutton(
                    label=port,
                    value=port,
                    variable=self.serial_port_var,
                    command=self.connect_selected_port,
                )
        else:
            self.ports_menu.add_command(label="No COM ports found")

    def connect_selected_port(self):
        port_name = self.serial_port_var.get().strip()
        if port_name:
            set_target_port(port_name)

    def disconnect_port(self):
        disconnect_serial_port()

    def build_dashboard_flags(self):
        flag_frame = tk.LabelFrame(self.root, text="System Flags")
        flag_frame.pack(fill="x", pady=5)

        self.flt_labels = []
        flt_frame = tk.Frame(flag_frame)
        flt_frame.pack(anchor="w", padx=4, pady=2)

        for i, name in enumerate(dashboard_flag_names):
            lbl = tk.Label(flt_frame, text=name, width=10, relief="groove")
            lbl.grid(row=0, column=i, padx=2)
            self.flt_labels.append(lbl)

        self.sys_labels = []
        sys_frame = tk.Frame(flag_frame)
        sys_frame.pack(anchor="w", padx=4, pady=2)

        for i in range(8):
            lbl = tk.Label(sys_frame, text=system_flag_names[i], width=11, relief="groove")
            lbl.grid(row=0, column=i, padx=2)
            self.sys_labels.append(lbl)

    def build_output_controls(self):
        control_frame = tk.LabelFrame(self.root, text="Output Control")
        control_frame.pack(fill="x", pady=5)

        self.output_switch_vars = []
        self.output_switch_buttons = []

        for index, name in enumerate(OUTPUT_SWITCH_NAMES):
            var = tk.IntVar(value=1)
            btn = tk.Checkbutton(
                control_frame,
                text=name,
                variable=var,
                indicatoron=False,
                width=14,
                command=lambda idx=index: self.on_output_switch_toggled(idx),
            )
            btn.grid(row=index // 5, column=index % 5, padx=3, pady=3, sticky="ew")
            self.output_switch_vars.append(var)
            self.output_switch_buttons.append(btn)

        self.retry_mode_var = tk.IntVar(value=2)
        retry_frame = tk.LabelFrame(control_frame, text="TPS25990 Retry Mode")
        retry_frame.grid(row=2, column=0, columnspan=5, sticky="w", padx=3, pady=4)

        for column, mode in enumerate((1, 2, 3)):
            tk.Radiobutton(
                retry_frame,
                text=RETRY_MODE_NAMES[mode],
                variable=self.retry_mode_var,
                value=mode,
                command=self.on_retry_mode_changed,
            ).grid(row=0, column=column, padx=4, pady=2, sticky="w")

        self.retry_mode_status_label = tk.Label(retry_frame, text="Active: unknown")
        self.retry_mode_status_label.grid(row=1, column=0, columnspan=3, sticky="w", padx=4)

        tk.Button(
            retry_frame,
            text="Clear Faults",
            command=self.on_clear_faults_clicked,
            width=14,
        ).grid(row=0, column=3, padx=6, pady=2, sticky="w")

        self.adc_reference_var = tk.IntVar(value=0)
        adc_ref_frame = tk.LabelFrame(control_frame, text="ADC Reference")
        adc_ref_frame.grid(row=3, column=0, columnspan=5, sticky="w", padx=3, pady=4)

        for column, reference in enumerate((0, 1)):
            tk.Radiobutton(
                adc_ref_frame,
                text=ADC_REFERENCE_NAMES[reference],
                variable=self.adc_reference_var,
                value=reference,
                command=self.on_adc_reference_changed,
            ).grid(row=0, column=column, padx=4, pady=2, sticky="w")

        self.adc_reference_status_label = tk.Label(adc_ref_frame, text="Active: unknown")
        self.adc_reference_status_label.grid(row=1, column=0, columnspan=2, sticky="w", padx=4)

        self.refresh_output_switch_colors()

    def refresh_output_switch_colors(self):
        for index, btn in enumerate(self.output_switch_buttons):
            active = self.output_switch_vars[index].get() != 0
            btn.config(bg="green" if active else "red", activebackground="green" if active else "red")

    def on_output_switch_toggled(self, index):
        enabled = self.output_switch_vars[index].get() != 0
        if not send_output_control(index, enabled):
            self.output_switch_vars[index].set(0 if enabled else 1)
        self.refresh_output_switch_colors()

    def on_retry_mode_changed(self):
        mode = self.retry_mode_var.get()
        if not send_retry_mode_control(mode):
            return

    def on_clear_faults_clicked(self):
        send_fault_clear_command(0xFF)

    def on_adc_reference_changed(self):
        reference = self.adc_reference_var.get()
        if not send_adc_reference_control(reference):
            return

    def on_debug_toggle(self):
        self.last_debug_keepalive = 0.0
        if self.debug_enabled_var.get():
            send_debug_pulse()
        else:
            send_control_frame(CONTROL_OP_NOP, 0, 0, debug=False)

    def build_diagnostics_window(self):
        self.diag_window = tk.Toplevel(self.root)
        self.diag_window.title("PDU Diagnostics")
        self.diag_window.geometry("1100x700")

        self.build_mcu(self.diag_window)
        self.build_serial_waterfall(self.diag_window)

        self.diag_window.protocol("WM_DELETE_WINDOW", self.diag_window.withdraw)

    def show_diagnostics_window(self):
        if self.diag_window.state() == "withdrawn":
            self.diag_window.deiconify()
        self.diag_window.lift()
        self.diag_window.focus_force()

    # ================= TOP EFUSE PANELS =================

    def build_top(self):
        self.efuse_frames = []

        top_frame = tk.Frame(self.root)
        top_frame.pack()

        for i in range(EFUSE_COUNT):
            frame = tk.LabelFrame(top_frame, text=f"{EFUSE_NAMES[i]}", padx=5, pady=5)
            frame.grid(row=i//4, column=i%4, padx=5, pady=5)

            v_label = tk.Label(frame, text="V: 0 mV")
            v_label.pack()

            i_label = tk.Label(frame, text="I: 0 mA")
            i_label.pack()

            p_label = tk.Label(frame, text="P: 0 (10mW)")
            p_label.pack()

            adc_label = tk.Label(frame, text="ADC I: 0 mA @ 0 mV")
            adc_label.pack()

            temp_label = tk.Label(frame, text="T: --.- C")
            temp_label.pack()

            compare_label = tk.Label(frame, text="ADC diff: n/a", width=22, relief="groove", bg="light gray")
            compare_label.pack(pady=(2, 2))

            cml_frame = tk.Frame(frame)
            cml_frame.pack()

            cml_labels = []
            for cml_index, (_, cml_name) in enumerate(TPS_STATUS_CML_BITS):
                lbl = tk.Label(cml_frame, text=cml_name, width=10, relief="groove")
                lbl.grid(row=0, column=cml_index, padx=1, pady=1)
                cml_labels.append(lbl)

            bit_frame = tk.Frame(frame)
            bit_frame.pack()

            bit_labels = []

            for b in range(16):
                lbl = tk.Label(bit_frame, text=TPS_STATUS_WORD_BITS[b],
                               width=12, relief="groove")
                lbl.grid(row=b//4, column=b%4)
                bit_labels.append(lbl)

            self.efuse_frames.append((v_label, i_label, p_label, adc_label, temp_label, compare_label, cml_labels, bit_labels))

    # ================= MCU PANEL =================

    def build_mcu(self, parent):
        mcu_frame = tk.LabelFrame(parent, text="MCU Telemetry / PMBus Diagnostics")
        mcu_frame.pack(fill="x", pady=5)

        self.shunt_label = tk.Label(mcu_frame, text="Shunt: 0 mA")
        self.shunt_label.pack()

        self.vin_label = tk.Label(mcu_frame, text="VIN avg: 0 mV")
        self.vin_label.pack()

        self.sum_current_label = tk.Label(mcu_frame, text="Sum I: 0.0 A")
        self.sum_current_label.pack()

        self.temp_avg_label = tk.Label(mcu_frame, text="Temp avg: 0.0 C  peak: 0.0 C")
        self.temp_avg_label.pack()

        self.err_flags_label = tk.Label(mcu_frame, text="Err flags: 0x00  Debug: 0")
        self.err_flags_label.pack()

        tk.Checkbutton(
            mcu_frame,
            text="Debug CAN (hold)",
            variable=self.debug_enabled_var,
            command=self.on_debug_toggle,
        ).pack(pady=2)

        self.mcu_debug_label = tk.Label(
            mcu_frame,
            text="Stage:0  Err:0  FailCh:-  FailCmd:0x00",
        )
        self.mcu_debug_label.pack()

        self.mcu_debug_detail_label = tk.Label(
            mcu_frame,
            text="Error: OK",
        )
        self.mcu_debug_detail_label.pack()

        error_list_frame = tk.LabelFrame(mcu_frame, text="Active Errors (1s TTL)")
        error_list_frame.pack(fill="x", padx=4, pady=4)
        self.error_listbox = tk.Listbox(error_list_frame, height=6)
        self.error_listbox.pack(fill="x", padx=4, pady=4)

        rename_btn = tk.Button(mcu_frame, text="Rename Flags",
                               command=self.rename_flags)
        rename_btn.pack()

    def rename_flags(self):
        for i in range(8):
            new_name = simpledialog.askstring("Rename",
                        f"New name for FLAG{i}")
            if new_name:
                system_flag_names[i] = new_name
                self.sys_labels[i].config(text=new_name)

    # ================= PLOT SECTION =================

    def build_plots(self):
        plot_frame = tk.LabelFrame(self.root, text="Current History")
        plot_frame.pack(fill="both", expand=True)

        control_frame = tk.Frame(plot_frame)
        control_frame.pack()

        tk.Button(control_frame, text="Prev",
                  command=self.prev_page).pack(side="left")

        tk.Button(control_frame, text="Next",
                  command=self.next_page).pack(side="left")

        self.figure, self.ax = plt.subplots(2, 1, figsize=(8,5))
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def build_serial_waterfall(self, parent):
        waterfall_frame = tk.LabelFrame(parent, text="CAN/Serial Waterfall")
        waterfall_frame.pack(fill="both", expand=True, padx=6, pady=6)

        self.waterfall_text = tk.Text(waterfall_frame, height=24, state="disabled")
        self.waterfall_text.pack(fill="both", expand=True)

    def append_waterfall_line(self, text_line):
        self.waterfall_text.configure(state="normal")
        self.waterfall_text.insert("end", text_line + "\n")

        line_count = int(self.waterfall_text.index("end-1c").split(".")[0])
        if line_count > WATERFALL_MAX_LINES:
            self.waterfall_text.delete("1.0", f"{line_count - WATERFALL_MAX_LINES + 1}.0")

        self.waterfall_text.see("end")
        self.waterfall_text.configure(state="disabled")

    def prev_page(self):
        self.page_index = (self.page_index - 1) % 4

    def next_page(self):
        self.page_index = (self.page_index + 1) % 4

    # ================= UPDATE LOOP =================

    def update_gui(self):

        # Serial status + waterfall
        connected, active_port, last_error, _ = get_serial_state()
        if connected and active_port:
            self.serial_status_label.config(text=f"Status: Connected ({active_port})", fg="green")
        else:
            if last_error and last_error != "Disconnected":
                self.serial_status_label.config(text=f"Status: Disconnected ({last_error})", fg="red")
            else:
                self.serial_status_label.config(text="Status: Disconnected", fg="red")

        # Keep debug TTL alive while switch is enabled
        if self.debug_enabled_var.get():
            now = time.monotonic()
            if now - self.last_debug_keepalive >= 0.5:
                if send_debug_pulse():
                    self.last_debug_keepalive = now
        else:
            self.last_debug_keepalive = time.monotonic()

        drained = 0
        while drained < 200:
            try:
                message = data_queue.get_nowait()
            except queue.Empty:
                break
            self.append_waterfall_line(message)
            drained += 1

        with telemetry_lock:
            local_voltage = list(voltage)
            local_current = list(current)
            local_power = list(power)
            local_status_word = list(status_word)
            local_adc_current = list(adc_current)
            local_adc_voltage = list(adc_voltage)
            local_adc_diff = list(adc_diff)
            local_adc_flags = list(adc_flags)
            local_cml_status = list(cml_status)
            local_temperature_c = list(temperature_c)
            local_histories = [list(history) for history in current_history]
            local_mcu_shunt = mcu_shunt
            local_flt_bits = flt_bits
            local_system_flags = system_flags
            local_mcu_dbg_stage = mcu_dbg_stage
            local_mcu_dbg_error = mcu_dbg_error
            local_mcu_dbg_fail_channel = mcu_dbg_fail_channel
            local_mcu_dbg_fail_command = mcu_dbg_fail_command
            local_active_retry_mode = active_retry_mode
            local_retry_mode_applied = retry_mode_applied
            local_active_adc_reference = active_adc_reference
            local_adc_reference_applied = adc_reference_applied
            local_active_error_details = dict(active_error_details)
            local_sum_current_da = sum_current_da
            local_vin_avg_mv = vin_avg_mv
            local_temp_avg_c = temp_avg_c
            local_temp_peak_c = temp_peak_c
            local_err_flags = err_flags
            local_debug_active = debug_active

        # Update efuse panels
        for i in range(EFUSE_COUNT):
            v_lbl, i_lbl, p_lbl, adc_lbl, temp_lbl, cmp_lbl, cml_lbls, bit_lbls = self.efuse_frames[i]

            v_lbl.config(text=f"V: {local_voltage[i]} mV")
            i_lbl.config(text=f"I: {local_current[i]} mA")
            p_lbl.config(text=f"P: {local_power[i]} (10mW)")
            adc_lbl.config(text=f"ADC I: {local_adc_current[i]} mA @ {local_adc_voltage[i]} mV")
            if local_temperature_c[i] is None:
                temp_lbl.config(text="T: --.- C")
            else:
                temp_lbl.config(text=f"T: {local_temperature_c[i]:.1f} C")

            if local_adc_flags[i] & 0x04:
                cmp_lbl.config(text=f"ADC diff: {local_adc_diff[i]} mA", bg="red")
            elif local_adc_flags[i] & 0x02:
                cmp_lbl.config(text=f"ADC diff: {local_adc_diff[i]} mA", bg="green")
            else:
                cmp_lbl.config(text="ADC diff: n/a", bg="light gray")

            for cml_index, (bit_pos, _) in enumerate(TPS_STATUS_CML_BITS):
                bit_active = ((local_cml_status[i] >> bit_pos) & 1) != 0
                cml_lbls[cml_index].config(bg="red" if bit_active else "green")

            for b in range(16):
                bit = (local_status_word[i] >> (15-b)) & 1
                bit_lbls[b].config(bg="red" if bit else "green")

        # MCU
        self.shunt_label.config(text=f"Shunt: {local_mcu_shunt} mA")
        self.vin_label.config(text=f"VIN avg: {local_vin_avg_mv} mV")
        self.sum_current_label.config(text=f"Sum I: {local_sum_current_da/10.0:.1f} A")
        self.temp_avg_label.config(text=f"Temp avg: {local_temp_avg_c:.1f} C  peak: {local_temp_peak_c:.1f} C")
        self.err_flags_label.config(text=f"Err flags: 0x{local_err_flags:02X}  Debug: {local_debug_active}")

        fail_ch_text = "-" if local_mcu_dbg_fail_channel == 0xFF else str(local_mcu_dbg_fail_channel)
        err_name = MCU_ERR_NAMES.get(local_mcu_dbg_error, f"UNK_{local_mcu_dbg_error}")
        cmd_name = PMBUS_CMD_NAMES.get(local_mcu_dbg_fail_command, f"CMD_0x{local_mcu_dbg_fail_command:02X}")

        self.mcu_debug_label.config(
            text=f"Stage:{local_mcu_dbg_stage}  Err:{local_mcu_dbg_error}  FailCh:{fail_ch_text}  FailCmd:0x{local_mcu_dbg_fail_command:02X}"
        )
        self.mcu_debug_detail_label.config(
            text=f"Error:{err_name}  Command:{cmd_name}"
        )

        self.error_listbox.delete(0, tk.END)
        if local_active_error_details:
            for slot in sorted(local_active_error_details.keys()):
                entry = local_active_error_details[slot]
                err_code = entry["err"]
                ch = entry["channel"]
                cmd = entry["command"]
                ttl = entry["ttl"]

                err_name = MCU_ERR_NAMES.get(err_code, f"UNK_{err_code}")
                cmd_name = PMBUS_CMD_NAMES.get(cmd, f"CMD_0x{cmd:02X}")
                ch_text = "-" if ch == 0xFF else str(ch)

                self.error_listbox.insert(
                    tk.END,
                    f"ERR:{err_code}({err_name}) CH:{ch_text} CMD:0x{cmd:02X}({cmd_name}) TTL:{ttl*100}ms",
                )
        else:
            self.error_listbox.insert(tk.END, "No active errors")

        for i in range(len(self.flt_labels)):
            active = (local_flt_bits >> i) & 1
            self.flt_labels[i].config(bg="red" if active else "green")

        for i in range(8):
            active = (local_system_flags >> i) & 1
            self.sys_labels[i].config(bg="red" if active else "green")

        self.refresh_output_switch_colors()

        active_retry_text = RETRY_MODE_NAMES.get(local_active_retry_mode, "Unknown")
        applied_text = "applied" if local_retry_mode_applied else "pending"
        self.retry_mode_status_label.config(text=f"Active: {active_retry_text} ({applied_text})")
        if local_active_retry_mode in RETRY_MODE_NAMES:
            self.retry_mode_var.set(local_active_retry_mode)

        active_adc_reference_text = ADC_REFERENCE_NAMES.get(local_active_adc_reference, "Unknown")
        adc_ref_applied_text = "applied" if local_adc_reference_applied else "pending"
        self.adc_reference_status_label.config(text=f"Active: {active_adc_reference_text} ({adc_ref_applied_text})")
        if local_active_adc_reference in ADC_REFERENCE_NAMES:
            self.adc_reference_var.set(local_active_adc_reference)

        # Plots
        self.ax[0].clear()
        self.ax[1].clear()

        base = self.page_index * 2
        idx1 = base
        idx2 = base + 1

        self.ax[0].plot(local_histories[idx1])
        self.ax[0].set_title(f"{EFUSE_NAMES[idx1]} Current (mA)")

        self.ax[1].plot(local_histories[idx2])
        self.ax[1].set_title(f"{EFUSE_NAMES[idx2]} Current (mA)")

        self.canvas.draw()

        self.root.after(int(1000/UPDATE_RATE_HZ), self.update_gui)

# ================= MAIN =================

threading.Thread(target=serial_thread, daemon=True).start()

root = tk.Tk()
app = PDUDashboard(root)
root.mainloop()
