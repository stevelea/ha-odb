# ESPHome OBD-II BLE Bridge

ESPHome external component that connects to a **Veepeak OBDCheck BLE** (or compatible ELM327 BLE) adapter via ESP32 NimBLE, polls OBD-II PIDs, and exposes them as Home Assistant sensors — all through MQTT auto-discovery.

No Android app, no Python bridge, no HA Bluetooth integration needed. Just an ESP32 near your OBD port.

## Supported vehicles

| Profile | PIDs | Protocol |
|---------|------|----------|
| `xpeng_g6` | 33 sensors: SoC, SOH, HV battery, cell voltages, motor RPM/torque, charging, temps, odometer, 12V | Mode 22 (CAN 500kbps, BMS 704 + VCU 7E0) |
| `sae` | Coming soon (standard SAE mode 01 PIDs) | Mode 01 (auto protocol) |

## Quick start

### 1. Copy the files

Copy the `esphome-obd/components/` folder into your ESPHome config directory, or reference it from GitHub:

```yaml
external_components:
  - source: github://stevelea/ha-odb@main
    components: [obd_ble]
    refresh: 0s
```

### 2. Edit your ESPHome YAML

Use [`esp32-obd-bridge.yaml`](esp32-obd-bridge.yaml) as a template. Update:
- WiFi credentials
- MQTT broker IP (your Home Assistant)
- OBD adapter MAC address
- Vehicle profile

### 3. Flash

```bash
esphome run esp32-obd-bridge.yaml
```

### 4. Done

Sensors appear automatically in Home Assistant under the **OBD Bridge** device. No manual entity creation needed.

## Required hardware

- **ESP32** (any variant — ESP32, ESP32-C3, ESP32-S3)
- **Veepeak OBDCheck BLE** or **OBDCheck BLE+**
  - Also works with Vgate iCar Pro BLE and other ELM327 BLE adapters using the FFF0 or E781 service UUID

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

## Troubleshooting

**Adapter not found:** Make sure the adapter is plugged in and the car is on (or the OBD port has constant 12V). The adapter's power LED should be lit.

**Connection fails:** Try unplugging/replugging the adapter to force a fresh BLE advertisement. The ESP32 scans for 15 seconds on each connection attempt.

**Wrong values:** The G6 profile uses v4 WiCAN-corrected PIDs. If you're seeing suspicious values, the formula may need adjustment for your G6 firmware variant.

**BLE range:** The ESP32 needs to be within ~3-5 meters of the OBD port for reliable BLE. Mount it under the dash if possible.
