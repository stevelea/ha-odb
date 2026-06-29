"""ESPHome external component: OBD-II BLE client for XPENG G6 / SAE vehicles.

Connects to a Veepeak OBDCheck BLE (or compatible ELM327 BLE adapter) via
ESP32 NimBLE, polls OBD-II PIDs, and exposes them as ESPHome sensors.

REQUIRES Arduino framework (not ESP-IDF). Add this to your config:
    esp32:
      board: esp32dev
      framework:
        type: arduino

    esp32_ble_tracker:

Example usage:
    external_components:
      - source: github://stevelea/ha-odb@main
        components: [obd_ble]
        refresh: 0s

    obd_ble:
      mac_address: "8C:DE:52:DE:FA:CF"
      profile: xpeng_g6
      update_interval: 30s
"""

import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32", "esp32_ble_tracker"]
AUTO_LOAD = ["sensor"]

obd_ble_ns = cg.esphome_ns.namespace("obd_ble")
OBDComponent = obd_ble_ns.class_("OBDComponent", cg.PollingComponent)

CONF_MAC_ADDRESS = "mac_address"
CONF_PROFILE = "profile"
CONF_UPDATE_INTERVAL = "update_interval"


def _validate_mac(value):
    """Validate a BLE MAC address like 8C:DE:52:DE:FA:CF or 8c:de:52:de:fa:cf."""
    value = cv.string(value)
    if not re.match(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$", value):
        raise cv.Invalid(f"Invalid MAC address: {value}")
    return value


# PID definitions (name, unit, device_class, state_class, icon)
# These are passed via config; C++ handles PID hex + formula
G6_PIDS = [
    ("SOC", "%", "battery", "measurement", "mdi:battery"),
    ("SOH", "%", "battery", "measurement", "mdi:battery-heart"),
    ("HV Voltage", "V", "voltage", "measurement", "mdi:lightning-bolt"),
    ("HV Current", "A", "current", "measurement", "mdi:current-dc"),
    ("Max Cell Voltage", "V", "voltage", "measurement", "mdi:battery-plus"),
    ("Min Cell Voltage", "V", "voltage", "measurement", "mdi:battery-minus"),
    ("Max Battery Temp", "°C", "temperature", "measurement", "mdi:thermometer-alert"),
    ("Min Battery Temp", "°C", "temperature", "measurement", "mdi:thermometer"),
    ("CLTC Range", "km", "distance", "measurement", "mdi:map-marker-distance"),
    ("Cumulative Charge", "Ah", "energy", "total_increasing", "mdi:battery-charging"),
    ("Cumulative Dischg", "Ah", "energy", "total_increasing", "mdi:battery-arrow-down"),
    ("Charge Status", "", "", "", "mdi:ev-station"),
    ("Charge Limit", "%", "", "measurement", "mdi:battery-lock"),
    ("Odometer", "km", "distance", "total_increasing", "mdi:counter"),
    ("12V Battery", "V", "voltage", "measurement", "mdi:car-battery"),
    ("Vehicle Speed", "km/h", "speed", "measurement", "mdi:speedometer"),
    ("Accelerator Pedal", "%", "", "measurement", "mdi:gauge"),
    ("Front Motor RPM", "rpm", "", "measurement", "mdi:engine"),
    ("Rear Motor RPM", "rpm", "", "measurement", "mdi:engine"),
    ("Front Motor Torque", "Nm", "", "measurement", "mdi:engine"),
    ("Rear Motor Torque", "Nm", "", "measurement", "mdi:engine"),
    ("Charging HVIL", "", "", "", "mdi:ev-station"),
    ("VCU SoC", "%", "battery", "measurement", "mdi:battery"),
    ("DC Charge Current", "A", "current", "measurement", "mdi:current-dc"),
    ("DC Charge Voltage", "V", "voltage", "measurement", "mdi:lightning-bolt"),
    ("Brake Pressure", "bar", "pressure", "measurement", "mdi:car-brake-alert"),
    ("Fast Charge Temp 1", "°C", "temperature", "measurement", "mdi:thermometer"),
    ("Fast Charge Temp 2", "°C", "temperature", "measurement", "mdi:thermometer"),
    ("Slow Charge Temp 1", "°C", "temperature", "measurement", "mdi:thermometer"),
    ("Slow Charge Temp 2", "°C", "temperature", "measurement", "mdi:thermometer"),
    ("Slow Charge Temp 3", "°C", "temperature", "measurement", "mdi:thermometer"),
    ("Motor Temp", "°C", "temperature", "measurement", "mdi:engine-coolant"),
    ("Coolant Temp", "°C", "temperature", "measurement", "mdi:coolant-temperature"),
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OBDComponent),
        cv.Required(CONF_MAC_ADDRESS): _validate_mac,
        cv.Optional(CONF_PROFILE, default="xpeng_g6"): cv.one_of("sae", "xpeng_g6"),
        cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
    }
).extend(cv.polling_component_schema("30s"))


async def to_code(config):
    """Create the C++ component from config and generate sensors."""
    # Add NimBLE-Arduino library dependency
    cg.add_library("h2zero/NimBLE-Arduino", "2.2.3")
    cg.add_build_flag("-DCONFIG_NIMBLE_CPP_ENABLE_ADVERTISEMENT=0")
    cg.add_build_flag("-DCONFIG_NIMBLE_CPP_ENABLE_GAP=1")
    cg.add_build_flag("-DCONFIG_NIMBLE_CPP_ENABLE_GATT_CLIENT=1")

    # Create the main component
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Set MAC address (strip colons)
    mac_clean = config[CONF_MAC_ADDRESS].replace(":", "").replace("-", "").lower()
    cg.add(var.set_mac_address(cg.RawExpression(f'"{mac_clean}"')))
    cg.add(var.set_profile(config[CONF_PROFILE]))

    # Create sensors from PID definitions via raw codegen
    sensor_type = cg.esphome_ns.namespace("sensor").class_("Sensor")
    from esphome.core import ID

    if config[CONF_PROFILE] == "xpeng_g6":
        for i, (name, unit, dev_cls, state_cls, icon) in enumerate(G6_PIDS):
            sid = ID(f"obd_sensor_{i}", is_declaration=True, type=sensor_type)
            sens = cg.Pvariable(sid)
            cg.add(cg.App.register_component(sens))
            cg.add(sens.set_name(f"obd_{name}"))
            if unit:
                cg.add(sens.set_unit_of_measurement(unit))
            if dev_cls:
                cg.add(sens.set_device_class(dev_cls))
            if state_cls:
                cg.add(sens.set_state_class(state_cls))
            if icon:
                cg.add(sens.set_icon(icon))
            cg.add(var.add_sensor(sens))
