"""ESPHome external component: OBD-II BLE client

Uses ESPHome's built-in ble_client for BLE connection + GATT.
Our component ONLY handles the ELM327 protocol and sensor publishing.
No NimBLE-Arduino, no custom BLE scanning.

Example usage:
    esp32_ble_tracker:

    ble_client:
      - mac_address: "8C:DE:52:DE:FA:CF"
        id: veepeak

    obd_ble:
      ble_client_id: veepeak
      profile: xpeng_g6
      update_interval: 30s
"""

import re
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, ble_client
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32", "esp32_ble_tracker", "ble_client"]
AUTO_LOAD = ["sensor"]

obd_ble_ns = cg.esphome_ns.namespace("obd_ble")
OBDComponent = obd_ble_ns.class_("OBDComponent", cg.PollingComponent, cg.Component)

CONF_BLE_CLIENT_ID = "ble_client_id"
CONF_PROFILE = "profile"
CONF_UPDATE_INTERVAL = "update_interval"

G6_PIDS = [
    ("SOC","%","battery","measurement","mdi:battery"),
    ("SOH","%","battery","measurement","mdi:battery-heart"),
    ("HV Voltage","V","voltage","measurement","mdi:lightning-bolt"),
    ("HV Current","A","current","measurement","mdi:current-dc"),
    ("Max Cell Voltage","V","voltage","measurement","mdi:battery-plus"),
    ("Min Cell Voltage","V","voltage","measurement","mdi:battery-minus"),
    ("Max Battery Temp","°C","temperature","measurement","mdi:thermometer-alert"),
    ("Min Battery Temp","°C","temperature","measurement","mdi:thermometer"),
    ("CLTC Range","km","distance","measurement","mdi:map-marker-distance"),
    ("Cumulative Charge","Ah","energy","total_increasing","mdi:battery-charging"),
    ("Cumulative Dischg","Ah","energy","total_increasing","mdi:battery-arrow-down"),
    ("Charge Status","","","","mdi:ev-station"),
    ("Charge Limit","%","","measurement","mdi:battery-lock"),
    ("Odometer","km","distance","total_increasing","mdi:counter"),
    ("12V Battery","V","voltage","measurement","mdi:car-battery"),
    ("Vehicle Speed","km/h","speed","measurement","mdi:speedometer"),
    ("Accelerator Pedal","%","","measurement","mdi:gauge"),
    ("Front Motor RPM","rpm","","measurement","mdi:engine"),
    ("Rear Motor RPM","rpm","","measurement","mdi:engine"),
    ("Front Motor Torque","Nm","","measurement","mdi:engine"),
    ("Rear Motor Torque","Nm","","measurement","mdi:engine"),
    ("Charging HVIL","","","","mdi:ev-station"),
    ("VCU SoC","%","battery","measurement","mdi:battery"),
    ("DC Charge Current","A","current","measurement","mdi:current-dc"),
    ("DC Charge Voltage","V","voltage","measurement","mdi:lightning-bolt"),
    ("Brake Pressure","bar","pressure","measurement","mdi:car-brake-alert"),
    ("Fast Charge Temp 1","°C","temperature","measurement","mdi:thermometer"),
    ("Fast Charge Temp 2","°C","temperature","measurement","mdi:thermometer"),
    ("Slow Charge Temp 1","°C","temperature","measurement","mdi:thermometer"),
    ("Slow Charge Temp 2","°C","temperature","measurement","mdi:thermometer"),
    ("Slow Charge Temp 3","°C","temperature","measurement","mdi:thermometer"),
    ("Motor Temp","°C","temperature","measurement","mdi:engine-coolant"),
    ("Coolant Temp","°C","temperature","measurement","mdi:coolant-temperature"),
]

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OBDComponent),
    cv.Required(CONF_BLE_CLIENT_ID): cv.use_id(ble_client.BLEClient),
    cv.Optional(CONF_PROFILE, default="xpeng_g6"): cv.one_of("sae","xpeng_g6"),
    cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
}).extend(cv.polling_component_schema("30s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Get the ble_client reference
    parent = await cg.get_variable(config[CONF_BLE_CLIENT_ID])
    cg.add(var.set_ble_client(parent))
    cg.add(var.set_profile(config[CONF_PROFILE]))

    # Create sensors
    sensor_type = cg.esphome_ns.namespace("sensor").class_("Sensor")
    from esphome.core import ID

    if config[CONF_PROFILE] == "xpeng_g6":
        for i, (name, unit, dev_cls, state_cls, icon) in enumerate(G6_PIDS):
            sid = ID(f"obd_sensor_{i}", is_declaration=True, type=sensor_type)
            sens = cg.new_Pvariable(sid)
            cg.add(var.add_sensor(sens))
