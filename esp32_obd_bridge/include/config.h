// User configuration — edit these values before flashing
#pragma once

// ── WiFi ───────────────────────────────────────────────────────────────
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// ── MQTT (Home Assistant Mosquitto broker) ─────────────────────────────
#define MQTT_SERVER   "192.168.1.88"
#define MQTT_PORT     1883
#define MQTT_USER     ""          // Leave empty if no auth
#define MQTT_PASSWORD ""          // Leave empty if no auth
#define MQTT_CLIENT_ID "esp32_obd_g6"

// ── OBD-II BLE Adapter ─────────────────────────────────────────────────
#define OBD_BLE_ADDRESS "8c:de:52:de:fa:cf"   // Veepeak MAC (lowercase)

// ── Vehicle Profile ────────────────────────────────────────────────────
// "xpeng_g6" or "sae"
#define VEHICLE_PROFILE "xpeng_g6"

// ── Polling ────────────────────────────────────────────────────────────
#define POLL_INTERVAL_MS     30000   // High-priority PID poll interval
#define LOW_PRIORITY_MULT    12      // Low-priority PIDs every N cycles

// ── Home Assistant MQTT Discovery ──────────────────────────────────────
#define HA_DISCOVERY_PREFIX "homeassistant"
#define HA_DEVICE_NAME      "OBD-II Bridge"
#define HA_DEVICE_ID        "esp32_obd_g6"
#define HA_MODEL            "ESP32 OBD BLE Bridge"
#define HA_MANUFACTURER     "Veepeak/ESP32"
