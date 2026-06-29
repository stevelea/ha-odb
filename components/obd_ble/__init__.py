"""ESPHome external component: OBD-II BLE client for XPENG G6 / SAE vehicles.

Connects to a Veepeak OBDCheck BLE (or compatible ELM327 BLE adapter) via
ESP32 NimBLE, polls OBD-II PIDs, and exposes them as ESPHome sensors.

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

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_MAC_ADDRESS,
    CONF_PROFILE,
    CONF_UPDATE_INTERVAL,
)

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["sensor", "text_sensor"]

obd_ble_ns = cg.esphome_ns.namespace("obd_ble")
OBDComponent = obd_ble_ns.class_("OBDComponent", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OBDComponent),
        cv.Required("mac_address"): cv.mac_address,
        cv.Optional(CONF_PROFILE, default="xpeng_g6"): cv.one_of("sae", "xpeng_g6"),
        cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
    }
).extend(cv.polling_component_schema("30s"))


async def to_code(config):
    """Create the C++ component from config."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_mac_address(config["mac_address"].as_hex()))
    cg.add(var.set_profile(config[CONF_PROFILE]))

    # Create sensors are done in C++ setup() based on profile PIDs
    # Sensor creation is handled via the component's internal sensor registry
