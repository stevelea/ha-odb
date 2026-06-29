# ESPHome OBD-II BLE Bridge

ESPHome external component that connects to a **Veepeak OBDCheck BLE** (or compatible ELM327 BLE) adapter via ESP32 NimBLE, polls OBD-II PIDs, and exposes them as Home Assistant sensors — all through MQTT auto-discovery.

No Android app, no Python bridge, no HA Bluetooth integration needed. Just an ESP32 near your OBD port.

## Supported vehicles

| Profile | PIDs | Protocol |
|---------|------|----------|
| `xpeng_g6` | 33 sensors: SoC, SOH, HV battery, cell voltages, motor RPM/torque, charging, temps, odometer, 12V | Mode 22 (CAN 500kbps, BMS 704 + VCU 7E0) |
| `sae` | Coming soon | Mode 01 (auto protocol) |

## Requirements

- **ESP32** (ESP32, ESP32-C3, ESP32-S3)
- **Arduino framework** (not ESP-IDF) — NimBLE-Arduino requires it
- **Veepeak OBDCheck BLE** or **BLE+** (works with Vgate iCar Pro BLE and other ELM327 BLE adapters using FFF0 or E781 service UUID)
- **MQTT broker** accessible from the ESP32 (e.g. Home Assistant Mosquitto addon)

## Quick start

### 1. Create your ESPHome YAML

```yaml
esphome:
  name: obd-bridge

esp32:
  board: esp32dev
  framework:
    type: arduino  # ⬅ REQUIRED — NimBLE needs Arduino, not ESP-IDF

external_components:
  - source: github://stevelea/ha-odb@main
    components: [obd_ble]
    refresh: 0s

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

mqtt:
  broker: 192.168.1.88
  username: !secret mqtt_user
  password: !secret mqtt_password
  discovery: true
  discovery_prefix: homeassistant

logger:

ota:
  - platform: esphome
    password: !secret ota_password

obd_ble:
  mac_address: "8C:DE:52:DE:FA:CF"
  profile: xpeng_g6
  update_interval: 30s
```

See [`esp32-obd-bridge.yaml`](esp32-obd-bridge.yaml) for a complete example with uptime/WiFi diagnostic sensors.

### 2. Flash

```bash
esphome run obd-bridge.yaml
```

### 3. Done

Sensors appear automatically in Home Assistant under the **OBD Bridge** device. No manual entity creation needed.

## G6 sensors exposed

| Sensor | PID | Unit | Update |
|--------|-----|------|--------|
| SoC | 221109 | % | 30s |
| SOH | 22110A | % | 6min |
| HV Voltage | 221101 | V | 30s |
| HV Current | 221103 | A | 30s |
| Max Cell Voltage | 221105 | V | 6min |
| Min Cell Voltage | 221106 | V | 6min |
| Max Battery Temp | 221107 | °C | 30s |
| Min Battery Temp | 221108 | °C | 6min |
| CLTC Range | 221118 | km | 6min |
| Cumulative Charge | 221120 | Ah | 6min |
| Cumulative Discharge | 221121 | Ah | 6min |
| Charge Status | 22112D | — | 30s |
| Charge Limit | 221130 | % | 6min |
| Odometer | 220101 | km | 6min |
| 12V Battery | 220102 | V | 30s |
| Vehicle Speed | 220104 | km/h | 30s |
| Accelerator Pedal | 220313 | % | 6min |
| Front Motor RPM | 220317 | rpm | 6min |
| Rear Motor RPM | 220318 | rpm | 6min |
| Front Motor Torque | 220319 | Nm | 6min |
| Rear Motor Torque | 22031A | Nm | 6min |
| Charging HVIL | 22031D | — | 30s |
| VCU SoC | 22031E | % | 6min |
| DC Charge Current | 22031F | A | 30s |
| DC Charge Voltage | 220320 | V | 30s |
| Brake Pressure | 220321 | bar | 6min |
| Fast Charge Temp 1/2 | 220322-3 | °C | 6min |
| Slow Charge Temp 1/2/3 | 220324-6 | °C | 6min |
| Motor Temp | 220327 | °C | 6min |
| Coolant Temp | 220328 | °C | 6min |

High-priority sensors poll every 30s. Low-priority sensors poll every 12 cycles (~6 min) and cache values between polls.

## Adapter compatibility

| Adapter | Service UUID | Works? |
|---------|-------------|--------|
| **Veepeak OBDCheck BLE / BLE+** | `0000fff0` (FFF1 write, FFF2 notify) | ✅ Primary target |
| Vgate iCar Pro BLE | `e7810a71` (shared bef8d6c9) | ✅ Fallback auto-detected |
| Generic ELM327 BLE (HM-10 / CC254x) | `0000ffe0` / `49535343` | ⚠️ Untested, may work |

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `framework: arduino` not set | ESPHome 2026.x defaults to ESP-IDF. Add `framework: {type: arduino}` under `esp32:` |
| `NimBLEDevice.h` not found | Arduino framework + `h2zero/NimBLE-Arduino` library required. Reflash after adding `framework: arduino`. |
| Adapter not found during scan | Unplug/replug adapter to force fresh BLE advertisement. Car must be on or OBD port must have constant 12V. |
| Connection fails repeatedly | ESP32 needs to be within ~3m of the OBD port. Try different mounting position. |
| Wrong/suspicious values | The G6 profile uses v4 WiCAN-corrected PIDs. Firmware variants may need formula adjustments — open an issue. |
| All sensors show "unknown" | Check MQTT broker connectivity. Ensure `discovery: true` and `discovery_prefix: homeassistant` are set. |
