"""Constants for the OBD-II BLE integration.

Supports both standard SAE OBD-II PIDs (mode 01) and manufacturer-specific
PIDs (mode 22) for vehicles like the XPENG G6, with CAN ECU routing.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Callable, Final, Optional

DOMAIN: Final = "ha_odb"

# ── BLE GATT UUIDs ────────────────────────────────────────────────────────
# ── Veepeak OBDCheck BLE / BLE+ (primary target) ────────────────────────
# Service: 0000fff0, Write char: 0000fff1, Notify char: 0000fff2
OBD_SERVICE_UUID: Final = "0000fff0-0000-1000-8000-00805f9b34fb"
OBD_CHAR_WRITE_UUID: Final = "0000fff1-0000-1000-8000-00805f9b34fb"
OBD_CHAR_NOTIFY_UUID: Final = "0000fff2-0000-1000-8000-00805f9b34fb"

# ── Vgate iCar Pro / generic ELM327 BLE (shared write+notify char) ──────
VGT_SERVICE_UUID: Final = "e7810a71-73ae-499d-8c15-faa9aef0c3f2"
VGT_CHAR_UUID: Final = "bef8d6c9-9c21-4c9e-b632-bd58c1009f9f"

# Discovery UUID list (tried in order for both service matching & GATT discovery)
DISCOVERY_SERVICE_UUIDS: Final = [
    OBD_SERVICE_UUID,    # Veepeak / FFF0 primary
    VGT_SERVICE_UUID,    # Vgate / iCar Pro
    "0000ffe0-0000-1000-8000-00805f9b34fb",  # HM-10 based
    "49535343-fe7d-4ae5-8fa9-9fafd205e455",  # TI CC254x based
]

# ── Timeouts & intervals (seconds) ────────────────────────────────────────
BLE_CONNECT_TIMEOUT: Final = 15.0
BLE_COMMAND_TIMEOUT: Final = 5.0
POLL_INTERVAL_DEFAULT: Final = 30
SCAN_TIMEOUT: Final = 60

# Config entry / options keys
CONF_POLL_INTERVAL: Final = "poll_interval"
CONF_VEHICLE_PROFILE: Final = "vehicle_profile"
INITIAL_SETUP_TIMEOUT: Final = 30
MAX_COMMAND_RETRIES: Final = 3

# ── ELM327 protocol ───────────────────────────────────────────────────────
ELM_CR: Final = b"\r"
ELM_PROMPT: Final = b">"


class OBDMode(StrEnum):
    """OBD-II diagnostic modes."""

    CURRENT_DATA = "01"
    FREEZE_FRAME = "02"
    DTCS = "03"
    CLEAR_DTCS = "04"
    OXYGEN_SENSOR = "05"
    VEHICLE_INFO = "09"
    MANUFACTURER = "22"  # Manufacturer-specific (used by XPENG G6 et al.)


class PIDPriority(StrEnum):
    HIGH = "high"
    LOW = "low"


# ── ELM327 initialization commands ───────────────────────────────────────┐
# Generic SAE-standard adapter init (no CAN header setup)
ELM_INIT_SAE: Final = [
    "AT Z",    # Reset
    "AT E0",   # Echo off
    "AT L0",   # Linefeed off
    "AT S0",   # Spaces off
    "AT H0",   # Headers off
    "AT AT0",  # Adaptive timing off
    "AT ST0",  # Short timeout
    "AT SP0",  # Auto protocol select
]

# XPENG G6 init — sets up CAN bus with BMS (704) as default header
# Commands from the CarSOC project (verified v4 profile):
#   AT H1       — Headers on
#   AT SP6      — Protocol 6 (CAN 11-bit 500kbps)
#   AT S0       — Spaces off
#   AT M0       — Memory off
#   AT AT1      — Adaptive timing on
#   AT FCSM1    — Flow control mode 1
#   AT SH 704   — Set header to BMS (0x704)
#   AT CRA 784  — Set CAN receive address to BMS response (0x784)
#   AT FCSH 704 — Set flow control header to BMS
ELM_INIT_G6: Final = [
    "AT Z",
    "AT E0",
    "AT L0",
    "AT S0",
    "AT H1",
    "AT SP6",
    "AT M0",
    "AT AT1",
    "AT FCSM1",
    "AT SH 704",
    "AT CRA 784",
    "AT FCSH 704",
]

# ECU-specific ELM327 header-switch sequences for G6
# BMS: set header 704, listen on 784
ELM_HEADER_BMS: Final = "AT SH 704; AT CRA 784; AT FCSH 704"
# VCU: set header 7E0, listen on 7E8
ELM_HEADER_VCU: Final = "AT SH 7E0; AT CRA 7E8; AT FCSH 7E0"

# ── PID probe groups (mode 01 SAE support discovery) ─────────────────────
PID_SUPPORT_PROBES: Final = ["00", "20", "40", "60", "80", "A0", "C0"]


# ══════════════════════════════════════════════════════════════════════════
# PID definitions
# ══════════════════════════════════════════════════════════════════════════

@dataclass
class OBDPid:
    """Definition of a single OBD-II PID."""

    pid: str                      # Hex PID e.g. "0C", "221109"
    name: str                     # Human-readable sensor name
    description: str
    mode: OBDMode = OBDMode.CURRENT_DATA
    data_bytes: int = 1           # Expected response data bytes
    formula: str = "A"            # Formula string (WiCAN or ABC notation)
    unit: Optional[str] = None
    device_class: Optional[str] = None
    state_class: str = "measurement"
    icon: str = "mdi:gauge"
    suggested_display_precision: int = 1
    entity_category: Optional[str] = None
    priority: PIDPriority = PIDPriority.HIGH
    # CAN bus routing (for mode 22 / manufacturer PIDs)
    can_header: Optional[str] = None  # e.g. "704" (BMS), "7E0" (VCU)
    can_listen: Optional[str] = None  # e.g. "784" (BMS response), "7E8" (VCU response)


# ── SAE standard PID formulas (ABC notation) ──────────────────────────────

_SAE_RPM = "((A*256+B)/4)"
_SAE_SPEED = "A"
_SAE_TEMP = "(A-40)"
_SAE_PERCENT = "(A*100/255)"
_SAE_PRESSURE = "A"
_SAE_FUEL_PRESS = "(A*3)"
_SAE_VOLTAGE = "((A*256+B)/1000)"
_SAE_TIMING = "((A-128)/2)"
_SAE_AIRFLOW = "(((A*256)+B)/100)"
_SAE_O2_V = "(A/200)"
_SAE_RUNTIME = "((A*256)+B)"
_SAE_DISTANCE = "((A*256)+B)"
_SAE_ODOMETER = "((A<<24)+(B<<16)+(C<<8)+D)/10"

# ── SAE standard PIDs (mode 01) ──────────────────────────────────────────

SAE_PIDS: dict[str, OBDPid] = {
    "04": OBDPid(
        pid="04", name="Calculated Engine Load",
        description="Calculated engine load value",
        formula=_SAE_PERCENT, unit="%", device_class="power_factor",
        icon="mdi:engine",
    ),
    "05": OBDPid(
        pid="05", name="Coolant Temperature",
        description="Engine coolant temperature",
        formula=_SAE_TEMP, unit="°C", device_class="temperature",
        icon="mdi:thermometer", suggested_display_precision=0,
    ),
    "0A": OBDPid(
        pid="0A", name="Fuel Pressure",
        description="Fuel rail pressure",
        formula=_SAE_FUEL_PRESS, unit="kPa", device_class="pressure",
        icon="mdi:fuel", suggested_display_precision=0,
    ),
    "0B": OBDPid(
        pid="0B", name="Intake Manifold Pressure",
        description="Intake manifold absolute pressure",
        formula=_SAE_PRESSURE, unit="kPa", device_class="pressure",
        icon="mdi:gauge",
    ),
    "0C": OBDPid(
        pid="0C", name="Engine RPM",
        description="Engine revolutions per minute",
        formula=_SAE_RPM, unit="rpm", icon="mdi:tachometer",
        suggested_display_precision=0,
    ),
    "0D": OBDPid(
        pid="0D", name="Vehicle Speed",
        description="Vehicle speed",
        formula=_SAE_SPEED, unit="km/h", device_class="speed",
        icon="mdi:speedometer", suggested_display_precision=0,
    ),
    "0E": OBDPid(
        pid="0E", name="Timing Advance",
        description="Ignition timing advance for cylinder #1",
        formula=_SAE_TIMING, unit="°", icon="mdi:engine",
    ),
    "0F": OBDPid(
        pid="0F", name="Intake Air Temperature",
        description="Intake air temperature",
        formula=_SAE_TEMP, unit="°C", device_class="temperature",
        icon="mdi:thermometer", suggested_display_precision=0,
    ),
    "10": OBDPid(
        pid="10", name="Mass Air Flow",
        description="Mass air flow rate",
        formula=_SAE_AIRFLOW, unit="g/s", icon="mdi:air-filter",
    ),
    "11": OBDPid(
        pid="11", name="Throttle Position",
        description="Absolute throttle position",
        formula=_SAE_PERCENT, unit="%", icon="mdi:gauge",
    ),
    "14": OBDPid(
        pid="14", name="O2 Sensor 1",
        description="Oxygen sensor 1 — voltage",
        formula=_SAE_O2_V, unit="V", device_class="voltage",
        icon="mdi:lambda",
    ),
    "15": OBDPid(
        pid="15", name="O2 Sensor 2",
        description="Oxygen sensor 2 — voltage",
        formula=_SAE_O2_V, unit="V", device_class="voltage",
        icon="mdi:lambda",
    ),
    "1F": OBDPid(
        pid="1F", name="Run Time Since Start",
        description="Run time since engine start",
        formula=_SAE_RUNTIME, unit="s", device_class="duration",
        icon="mdi:timer-outline", suggested_display_precision=0,
    ),
    "21": OBDPid(
        pid="21", name="Distance with MIL On",
        description="Distance traveled with malfunction indicator lamp on",
        formula=_SAE_DISTANCE, unit="km", device_class="distance",
        icon="mdi:car-wrench", suggested_display_precision=0,
    ),
    "2F": OBDPid(
        pid="2F", name="Fuel Level",
        description="Fuel tank level input",
        formula=_SAE_PERCENT, unit="%", icon="mdi:fuel",
    ),
    "33": OBDPid(
        pid="33", name="Barometric Pressure",
        description="Absolute barometric pressure",
        formula=_SAE_PRESSURE, unit="kPa", device_class="pressure",
        icon="mdi:weather-cloudy",
    ),
    "42": OBDPid(
        pid="42", name="Control Module Voltage",
        description="Control module voltage",
        formula=_SAE_VOLTAGE, unit="V", device_class="voltage",
        icon="mdi:car-battery",
    ),
    "45": OBDPid(
        pid="45", name="Relative Throttle",
        description="Relative throttle position",
        formula=_SAE_PERCENT, unit="%", icon="mdi:gauge",
    ),
    "46": OBDPid(
        pid="46", name="Ambient Air Temperature",
        description="Ambient air temperature",
        formula=_SAE_TEMP, unit="°C", device_class="temperature",
        icon="mdi:thermometer", suggested_display_precision=0,
    ),
    "4D": OBDPid(
        pid="4D", name="Run Time MIL On",
        description="Run time with malfunction indicator lamp on",
        formula=_SAE_RUNTIME, unit="min", device_class="duration",
        icon="mdi:timer-alert-outline", suggested_display_precision=0,
    ),
    "5C": OBDPid(
        pid="5C", name="Engine Oil Temperature",
        description="Engine oil temperature",
        formula=_SAE_TEMP, unit="°C", device_class="temperature",
        icon="mdi:oil-temperature", suggested_display_precision=0,
    ),
    "A6": OBDPid(
        pid="A6", name="Odometer",
        description="Odometer reading",
        formula=_SAE_ODOMETER, unit="km", device_class="distance",
        state_class="total_increasing", icon="mdi:counter",
        suggested_display_precision=0,
    ),
}

# ── XPENG G6 Manufacturer PIDs (mode 22, WiCAN-v4 corrected) ─────────────
# Source: CarSOC project / local_vehicle_profiles.dart
# BMS = CAN header 704 (listen 784), VCU = CAN header 7E0 (listen 7E8)

XPENG_G6_PIDS: dict[str, OBDPid] = {
    # ═══ BMS PIDs (704→784) ═══
    "221109": OBDPid(
        pid="221109", name="SOC", mode=OBDMode.MANUFACTURER,
        description="Battery State of Charge",
        formula="[B4:B5]/10", unit="%", device_class="battery",
        icon="mdi:battery", suggested_display_precision=0,
        can_header="704", can_listen="784",
    ),
    "22110A": OBDPid(
        pid="22110A", name="SOH", mode=OBDMode.MANUFACTURER,
        description="Battery State of Health",
        formula="[B4:B5]/10", unit="%", device_class="battery",
        icon="mdi:battery-heart", suggested_display_precision=0,
        priority=PIDPriority.LOW, can_header="704", can_listen="784",
    ),
    "221101": OBDPid(
        pid="221101", name="HV Voltage", mode=OBDMode.MANUFACTURER,
        description="HV Battery Voltage",
        formula="[B4:B5]/10", unit="V", device_class="voltage",
        icon="mdi:lightning-bolt", can_header="704", can_listen="784",
    ),
    "221103": OBDPid(
        pid="221103", name="HV Current", mode=OBDMode.MANUFACTURER,
        description="HV Battery Current (negative = charging)",
        formula="[B4:B5]/2-1600", unit="A", device_class="current",
        icon="mdi:current-dc", can_header="704", can_listen="784",
    ),
    "221105": OBDPid(
        pid="221105", name="Max Cell Voltage", mode=OBDMode.MANUFACTURER,
        description="Maximum cell voltage",
        formula="[B4:B5]/1000", unit="V", device_class="voltage",
        icon="mdi:battery-plus", priority=PIDPriority.LOW,
        can_header="704", can_listen="784",
    ),
    "221106": OBDPid(
        pid="221106", name="Min Cell Voltage", mode=OBDMode.MANUFACTURER,
        description="Minimum cell voltage",
        formula="[B4:B5]/1000", unit="V", device_class="voltage",
        icon="mdi:battery-minus", priority=PIDPriority.LOW,
        can_header="704", can_listen="784",
    ),
    "221107": OBDPid(
        pid="221107", name="Max Battery Temp", mode=OBDMode.MANUFACTURER,
        description="Maximum battery cell temperature",
        formula="B4-40", unit="°C", device_class="temperature",
        icon="mdi:thermometer-alert", suggested_display_precision=0,
        can_header="704", can_listen="784",
    ),
    "221108": OBDPid(
        pid="221108", name="Min Battery Temp", mode=OBDMode.MANUFACTURER,
        description="Minimum battery cell temperature",
        formula="B4-40", unit="°C", device_class="temperature",
        icon="mdi:thermometer", priority=PIDPriority.LOW,
        suggested_display_precision=0, can_header="704", can_listen="784",
    ),
    "221112": OBDPid(
        pid="221112", name="Battery Power", mode=OBDMode.MANUFACTURER,
        description="Battery net power (positive = discharge)",
        formula="[B4:B5]/4", unit="kW", device_class="power",
        icon="mdi:flash", can_header="704", can_listen="784",
    ),
    "221120": OBDPid(
        pid="221120", name="Cumulative Charge", mode=OBDMode.MANUFACTURER,
        description="Battery cumulative charging",
        formula="A<<24+B<<16+C<<8+D", unit="Ah",
        device_class="energy_storage", state_class="total_increasing",
        icon="mdi:battery-charging", priority=PIDPriority.LOW,
        can_header="704", can_listen="784",
    ),
    "221121": OBDPid(
        pid="221121", name="Cumulative Discharge", mode=OBDMode.MANUFACTURER,
        description="Battery cumulative discharging",
        formula="A<<24+B<<16+C<<8+D", unit="Ah",
        device_class="energy_storage", state_class="total_increasing",
        icon="mdi:battery-arrow-down", priority=PIDPriority.LOW,
        can_header="704", can_listen="784",
    ),
    "221118": OBDPid(
        pid="221118", name="CLTC Range", mode=OBDMode.MANUFACTURER,
        description="CLTC rated range",
        formula="[B4:B5]", unit="km", device_class="distance",
        icon="mdi:map-marker-distance", priority=PIDPriority.LOW,
        can_header="704", can_listen="784",
    ),
    "22112D": OBDPid(
        pid="22112D", name="Charge Status", mode=OBDMode.MANUFACTURER,
        description="BMS charge status (0=not charging, 2=charging)",
        formula="B4", icon="mdi:ev-station",
        can_header="704", can_listen="784",
    ),
    "221130": OBDPid(
        pid="221130", name="Charge Limit", mode=OBDMode.MANUFACTURER,
        description="Charge limit setting",
        formula="[B4:B5]-10", unit="%", icon="mdi:battery-lock",
        priority=PIDPriority.LOW, can_header="704", can_listen="784",
    ),
    # BMS-hosted vehicle telemetry (v4 WiCAN-corrected)
    "220101": OBDPid(
        pid="220101", name="Odometer", mode=OBDMode.MANUFACTURER,
        description="Odometer — BMS-hosted, 3-byte",
        formula="[B4:B6]", unit="km", device_class="distance",
        state_class="total_increasing", icon="mdi:counter",
        suggested_display_precision=0, priority=PIDPriority.LOW,
        can_header="704", can_listen="784",
    ),
    "220102": OBDPid(
        pid="220102", name="12V Battery", mode=OBDMode.MANUFACTURER,
        description="12V auxiliary battery voltage — BMS-hosted",
        formula="B4/10", unit="V", device_class="voltage",
        icon="mdi:car-battery", can_header="704", can_listen="784",
    ),

    # ═══ VCU PIDs (7E0→7E8) ═══
    "220104": OBDPid(
        pid="220104", name="Vehicle Speed", mode=OBDMode.MANUFACTURER,
        description="Vehicle speed",
        formula="[B4:B5]/100", unit="km/h", device_class="speed",
        icon="mdi:speedometer", suggested_display_precision=0,
        can_header="7E0", can_listen="7E8",
    ),
    "220313": OBDPid(
        pid="220313", name="Accelerator Pedal", mode=OBDMode.MANUFACTURER,
        description="Accelerator pedal position",
        formula="B4/2", unit="%", icon="mdi:gauge",
        priority=PIDPriority.LOW, can_header="7E0", can_listen="7E8",
    ),
    "220317": OBDPid(
        pid="220317", name="Front Motor RPM", mode=OBDMode.MANUFACTURER,
        description="Front motor RPM (G6: -16000 offset)",
        formula="[B4:B5]-16000", unit="rpm", icon="mdi:engine",
        priority=PIDPriority.LOW, can_header="7E0", can_listen="7E8",
    ),
    "220318": OBDPid(
        pid="220318", name="Rear Motor RPM", mode=OBDMode.MANUFACTURER,
        description="Rear motor RPM (G6: -16000 offset)",
        formula="[B4:B5]-16000", unit="rpm", icon="mdi:engine",
        priority=PIDPriority.LOW, can_header="7E0", can_listen="7E8",
    ),
    "220319": OBDPid(
        pid="220319", name="Front Motor Torque", mode=OBDMode.MANUFACTURER,
        description="Front motor torque request",
        formula="[B4:B5]/4-500", unit="Nm", icon="mdi:engine",
        priority=PIDPriority.LOW, can_header="7E0", can_listen="7E8",
    ),
    "22031A": OBDPid(
        pid="22031A", name="Rear Motor Torque", mode=OBDMode.MANUFACTURER,
        description="Rear motor torque request",
        formula="[B4:B5]/4-500", unit="Nm", icon="mdi:engine",
        priority=PIDPriority.LOW, can_header="7E0", can_listen="7E8",
    ),
    "22031D": OBDPid(
        pid="22031D", name="Charging HVIL", mode=OBDMode.MANUFACTURER,
        description="Charging HVIL status",
        formula="B4", icon="mdi:ev-station",
        can_header="7E0", can_listen="7E8",
    ),
    "22031E": OBDPid(
        pid="22031E", name="VCU SoC", mode=OBDMode.MANUFACTURER,
        description="VCU-side State of Charge",
        formula="[B4:B5]/10", unit="%", device_class="battery",
        icon="mdi:battery", priority=PIDPriority.LOW,
        can_header="7E0", can_listen="7E8",
    ),
    "22031F": OBDPid(
        pid="22031F", name="DC Charge Current", mode=OBDMode.MANUFACTURER,
        description="DC fast charge current",
        formula="[B4:B5]/10-1200", unit="A", device_class="current",
        icon="mdi:current-dc", can_header="7E0", can_listen="7E8",
    ),
    "220320": OBDPid(
        pid="220320", name="DC Charge Voltage", mode=OBDMode.MANUFACTURER,
        description="DC fast charge voltage",
        formula="[B4:B5]", unit="V", device_class="voltage",
        icon="mdi:lightning-bolt", can_header="7E0", can_listen="7E8",
    ),
    "220321": OBDPid(
        pid="220321", name="Brake Pressure", mode=OBDMode.MANUFACTURER,
        description="Brake main cylinder pressure — was misidentified as AC_CHG_A in v3",
        formula="[B4:B5]/5", unit="bar", device_class="pressure",
        icon="mdi:car-brake-alert", priority=PIDPriority.LOW,
        can_header="7E0", can_listen="7E8",
    ),
    "220322": OBDPid(
        pid="220322", name="Fast Charge Temp 1", mode=OBDMode.MANUFACTURER,
        description="Fast charging temperature 1 — was misidentified as AC_CHG_V in v3",
        formula="B4-40", unit="°C", device_class="temperature",
        icon="mdi:thermometer", priority=PIDPriority.LOW,
        can_header="7E0", can_listen="7E8",
    ),
    "220323": OBDPid(
        pid="220323", name="Fast Charge Temp 2", mode=OBDMode.MANUFACTURER,
        description="Fast charging temperature 2",
        formula="B4-40", unit="°C", device_class="temperature",
        icon="mdi:thermometer", priority=PIDPriority.LOW,
        can_header="7E0", can_listen="7E8",
    ),
    "220324": OBDPid(
        pid="220324", name="Slow Charge Temp 1", mode=OBDMode.MANUFACTURER,
        description="Slow charging temperature 1",
        formula="B4-40", unit="°C", device_class="temperature",
        icon="mdi:thermometer", priority=PIDPriority.LOW,
        can_header="7E0", can_listen="7E8",
    ),
    "220325": OBDPid(
        pid="220325", name="Slow Charge Temp 2", mode=OBDMode.MANUFACTURER,
        description="Slow charging temperature 2 — was misidentified as INV_T in v3",
        formula="B4-40", unit="°C", device_class="temperature",
        icon="mdi:thermometer", priority=PIDPriority.LOW,
        can_header="7E0", can_listen="7E8",
    ),
    "220326": OBDPid(
        pid="220326", name="Slow Charge Temp 3", mode=OBDMode.MANUFACTURER,
        description="Slow charging temperature 3",
        formula="B4-40", unit="°C", device_class="temperature",
        icon="mdi:thermometer", priority=PIDPriority.LOW,
        can_header="7E0", can_listen="7E8",
    ),
    "220327": OBDPid(
        pid="220327", name="Motor Temp", mode=OBDMode.MANUFACTURER,
        description="Traction motor coolant temperature",
        formula="B4/2-40", unit="°C", device_class="temperature",
        icon="mdi:engine-coolant", priority=PIDPriority.LOW,
        can_header="7E0", can_listen="7E8",
    ),
    "220328": OBDPid(
        pid="220328", name="Coolant Temp", mode=OBDMode.MANUFACTURER,
        description="Battery coolant temperature",
        formula="B4/2-40", unit="°C", device_class="temperature",
        icon="mdi:coolant-temperature", priority=PIDPriority.LOW,
        can_header="7E0", can_listen="7E8",
    ),
}


# ══════════════════════════════════════════════════════════════════════════
# Formula parser (WiCAN + ABC notation, adapted from CarSOC / obd_pid_config.dart)
# ══════════════════════════════════════════════════════════════════════════

def _tokenize_formula(expr: str) -> list[str]:
    """Split a formula string into tokens (numbers, operators, parentheses)."""
    tokens: list[str] = []
    buf = ""
    for ch in expr.replace(" ", ""):
        if ch in "+-*/()<>|&":
            if buf:
                tokens.append(buf)
                buf = ""
            tokens.append(ch)
        else:
            buf += ch
    if buf:
        tokens.append(buf)
    # Merge << and >>
    merged: list[str] = []
    i = 0
    while i < len(tokens):
        if i + 1 < len(tokens) and tokens[i] == "<" and tokens[i + 1] == "<":
            merged.append("<<")
            i += 2
        elif i + 1 < len(tokens) and tokens[i] == ">" and tokens[i + 1] == ">":
            merged.append(">>")
            i += 2
        else:
            merged.append(tokens[i])
            i += 1
    return merged


def _eval_simple(expr: str) -> float:
    """Evaluate a simple numeric expression with + - * / << >>."""
    tokens = _tokenize_formula(expr)
    # Recursive descent: << and >> first, then * and /, then + and -
    def _one_op(ops: list[str], fn):
        nonlocal tokens
        i = 0
        while i < len(tokens):
            if tokens[i] in ops:
                left = float(tokens[i - 1])
                right = float(tokens[i + 1])
                tokens[i - 1 : i + 2] = [str(fn(left, right, tokens[i]))]
                continue
            i += 1
        return tokens

    tokens = _one_op(["<<", ">>"], lambda a, b, op: float(int(a) << int(b)) if op == "<<" else float(int(a) >> int(b)))
    tokens = _one_op(["*", "/"], lambda a, b, op: a * b if op == "*" else (a / b if b != 0 else 0.0))
    tokens = _one_op(["+", "-"], lambda a, b, op: a + b if op == "+" else a - b)
    return float(tokens[0])


def parse_obd_response(response: str, formula: str) -> Optional[float]:
    """Parse an OBD-II response using WiCAN or ABC formula notation.

    WiCAN notation: B4 = byte 4, [B4:B5] = bytes 4-5 as 16-bit big-endian.
    ABC notation: A/B/C/D = sequential data bytes after header.

    Returns None on error/NO DATA/negative response.
    """
    # Clean the raw response
    clean = response.replace(" ", "").replace(">", "").replace("\r", "").upper().strip()

    # Error detection
    if any(err in clean for err in ("ERROR", "STOPPED", "SEARCHING", "UNABLE", "NO DATA", "?")):
        return None
    if len(clean) < 6:
        return None

    # Handle multi-frame ISO-TP (sequence prefixes like "0:", "1:", "2:")
    is_multi_frame = bool(re.search(r"[0-9A-F]:", clean))
    if is_multi_frame:
        clean = re.sub(r"[0-9A-F]:", "", clean)
        if len(clean) >= 3:
            clean = clean[3:]  # Skip 3-char length header

    # Extract all bytes
    all_bytes: list[int] = []
    if len(clean) % 2 != 0:
        clean = clean[:-1]
    for i in range(0, len(clean) - 1, 2):
        hb = clean[i : i + 2]
        if re.match(r"^[0-9A-F]{2}$", hb):
            all_bytes.append(int(hb, 16))

    if not all_bytes or len(all_bytes) < 2:
        return None

    # Strip 3-nibble CAN header (784, 7E8, etc.) for WiCAN byte indexing
    if len(all_bytes) >= 3:
        first_three = clean[:3]
        if re.match(r"^7[0-9A-F]{2}$", first_three):
            all_bytes = all_bytes[1:] if len(all_bytes) > 1 and all_bytes[0] == 0x07 else all_bytes[2:]
            # Actually the 3 nibbles = 1.5 bytes so strip first 2 chars and reparse
            clean = clean[3:]
            # Rebuild bytes
            all_bytes = []
            if len(clean) % 2 != 0:
                clean = clean[:-1]
            for i in range(0, len(clean) - 1, 2):
                hb = clean[i : i + 2]
                if re.match(r"^[0-9A-F]{2}$", hb):
                    all_bytes.append(int(hb, 16))

    if not all_bytes or len(all_bytes) < 2:
        return None

    # Negative response check: service byte 7F at position after length byte
    if len(all_bytes) >= 3 and all_bytes[1] == 0x7F:
        return None

    # ── Formula evaluation ──
    uses_wican = bool(re.search(r"\[?B\d", formula))
    uses_abc = bool(re.search(r"[ABCDE]", formula)) and not uses_wican

    expr = formula

    if uses_wican:
        # [B4:B5] → multi-byte big-endian value
        expr = re.sub(
            r"\[B(\d+):B(\d+)\]",
            lambda m: str(
                sum(
                    (all_bytes[i] << (8 * (int(m[2]) - i)))
                    if i < len(all_bytes)
                    else 0
                    for i in range(int(m[1]), int(m[2]) + 1)
                )
            ),
            expr,
        )
        # B4 → single byte value
        expr = re.sub(
            r"B(\d+)",
            lambda m: str(all_bytes[int(m[1])]) if int(m[1]) < len(all_bytes) else "0",
            expr,
        )
    elif uses_abc:
        # Find data start: skip ECU header, length byte, service byte 62, PID echo
        # Data bytes are after the service byte pattern
        data_start = 0
        for i, b in enumerate(all_bytes):
            if b == 0x62 and i > 0:  # Positive response service byte
                data_start = i + 1
                # Skip PID echo (2-3 more bytes for mode 22)
                if data_start + 2 < len(all_bytes):
                    data_start += 2 if len(all_bytes[data_start:]) > 3 else 3
                break
        data = all_bytes[data_start:] if data_start < len(all_bytes) else []
        abc_map = ["A", "B", "C", "D", "E"]
        for idx, label in enumerate(abc_map):
            expr = re.sub(
                rf"\b{label}\b",
                str(data[idx]) if idx < len(data) else "0",
                expr,
            )

    try:
        return _eval_simple(expr)
    except (ValueError, ZeroDivisionError):
        return None
