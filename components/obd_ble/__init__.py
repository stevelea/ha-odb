"""ESPHome external component: OBD-II BLE client for XPENG G6 / SAE vehicles.

Connects to a Veepeak OBDCheck BLE (or compatible ELM327 BLE adapter) via
ESP32 NimBLE, polls OBD-II PIDs, and exposes them as ESPHome sensors.

REQUIRES Arduino framework (not ESP-IDF). Add this to your config:
    esp32:
      board: esp32dev
      framework:
        type: arduino

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
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32", "esp32_ble_tracker"]
AUTO_LOAD = ["sensor"]

obd_ble_ns = cg.esphome_ns.namespace("obd_ble")
OBDComponent = obd_ble_ns.class_("OBDComponent", cg.PollingComponent)


def _validate_mac(value):
    """Validate a BLE MAC address like 8C:DE:52:DE:FA:CF or 8c:de:52:de:fa:cf."""
    value = cv.string(value)
    if not re.match(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$", value):
        raise cv.Invalid(f"Invalid MAC address: {value}")
    return value


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OBDComponent),
        cv.Required("mac_address"): _validate_mac,
        cv.Optional("profile", default="xpeng_g6"): cv.one_of("sae", "xpeng_g6"),
        cv.Optional("update_interval", default="30s"): cv.positive_time_period_milliseconds,
    }
).extend(cv.polling_component_schema("30s"))


async def to_code(config):
    """Create the C++ component from config."""
    # Add NimBLE-Arduino library dependency
    cg.add_library("h2zero/NimBLE-Arduino", "2.2.3")

    # NimBLE configuration — minimise RAM, only GATT client needed
    cg.add_build_flag("-DCONFIG_NIMBLE_CPP_ENABLE_ADVERTISEMENT=0")
    cg.add_build_flag("-DCONFIG_NIMBLE_CPP_ENABLE_GAP=1")
    cg.add_build_flag("-DCONFIG_NIMBLE_CPP_ENABLE_GATT_CLIENT=1")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Convert MAC to hex bytes string (without colons) for C++
    mac_clean = config["mac_address"].replace(":", "").replace("-", "").lower()
    cg.add(var.set_mac_address(cg.RawExpression(f'"{mac_clean}"')))
    cg.add(var.set_profile(config["profile"]))
