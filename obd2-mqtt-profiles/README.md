# obd2-mqtt G6 Profile

This profile adds XPENG G6 EV-specific OBD-II PIDs (mode 0x22, CAN 500kbps) to [adlerre/obd2-mqtt](https://github.com/adlerre/obd2-mqtt).

## Requirements

- ESP32 with [obd2-mqtt](https://github.com/adlerre/obd2-mqtt) firmware
- Veepeak OBDCheck BLE (or any ELM327 Bluetooth adapter)
- Bluetooth Classic (SPP) connection — NOT BLE

## How to use

1. Flash your ESP32 with `obd2-mqtt` firmware
2. Connect to the ESP32's Wi-Fi AP (`OBD2-MQTT-XXXX`)
3. Open http://192.168.4.1
4. Go to the **OBD** tab
5. Upload [`xpeng_g6_profile.json`](xpeng_g6_profile.json)
6. Reboot the ESP32

The G6 profile includes 15 BMS PIDs and 18 VCU PIDs for a total of 33 sensors. All data publishes to MQTT with Home Assistant auto-discovery.

## PID format notes

Mode 0x22 PIDs require the ELM327 to be configured for CAN 500kbps with proper ECU headers. The obd2-mqtt firmware sends an AT init string before polling. For the G6, add this to your firmware's OBD init:

```
AT H1; AT SP6; AT SH 704; AT CRA 784; AT FCSH 704
```

PID codes are in **decimal** (not hex). Mode 0x22 = decimal 34.

## Adaptation notes

The `scaleFactor` field uses ABC notation where:
- **A** = first response data byte after the OBD header
- **B** = second byte, etc.

For CAN responses with 3-nibble ECU headers (e.g., `784`), the data bytes start with an offset. You may need to adjust the `response` byte count and scaleFactor letters to match your adapter's response format.

If values are incorrect, check the raw OBD response in the ESP32 serial logs and adjust the byte offsets.
