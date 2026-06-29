// OBD-II PID definitions for XPENG G6 (v4 WiCAN-corrected)
// Ported from const.py — CarSOC project
#pragma once

#include <Arduino.h>
#include <vector>

enum class PIDPriority { HIGH, LOW };

struct OBDPid {
  const char* name;
  const char* pid;       // hex string e.g. "221109"
  const char* formula;   // WiCAN notation e.g. "[B4:B5]/10"
  const char* unit;
  const char* device_class;
  const char* icon;
  const char* state_class;
  PIDPriority priority;
  // CAN routing (G6 BMS=704, VCU=7E0)
  const char* can_header;  // "704" or "7E0" or nullptr
};

// ── XPENG G6 PIDs (BMS 704→784) ────────────────────────────────────────
static const OBDPid G6_BMS_PIDS[] = {
  {"SOC",               "221109", "[B4:B5]/10",           "%",  "battery",          "mdi:battery",                    "measurement",       PIDPriority::HIGH, "704"},
  {"SOH",               "22110A", "[B4:B5]/10",           "%",  "battery",          "mdi:battery-heart",              "measurement",       PIDPriority::LOW,  "704"},
  {"HV Voltage",        "221101", "[B4:B5]/10",           "V",  "voltage",          "mdi:lightning-bolt",             "measurement",       PIDPriority::HIGH, "704"},
  {"HV Current",        "221103", "[B4:B5]/2-1600",       "A",  "current",          "mdi:current-dc",                 "measurement",       PIDPriority::HIGH, "704"},
  {"Max Cell Voltage",  "221105", "[B4:B5]/1000",         "V",  "voltage",          "mdi:battery-plus",               "measurement",       PIDPriority::LOW,  "704"},
  {"Min Cell Voltage",  "221106", "[B4:B5]/1000",         "V",  "voltage",          "mdi:battery-minus",              "measurement",       PIDPriority::LOW,  "704"},
  {"Max Battery Temp",  "221107", "B4-40",                "°C", "temperature",      "mdi:thermometer-alert",          "measurement",       PIDPriority::HIGH, "704"},
  {"Min Battery Temp",  "221108", "B4-40",                "°C", "temperature",      "mdi:thermometer",                "measurement",       PIDPriority::LOW,  "704"},
  {"CLTC Range",        "221118", "[B4:B5]",              "km", "distance",         "mdi:map-marker-distance",        "measurement",       PIDPriority::LOW,  "704"},
  {"Cumulative Charge", "221120", "A<<24+B<<16+C<<8+D",   "Ah", "energy_storage",   "mdi:battery-charging",           "total_increasing",  PIDPriority::LOW,  "704"},
  {"Cumulative Dischg", "221121", "A<<24+B<<16+C<<8+D",   "Ah", "energy_storage",   "mdi:battery-arrow-down",         "total_increasing",  PIDPriority::LOW,  "704"},
  {"Charge Status",     "22112D", "B4",                   "",   "",                 "mdi:ev-station",                 "measurement",       PIDPriority::HIGH, "704"},
  {"Charge Limit",      "221130", "[B4:B5]-10",           "%",  "",                 "mdi:battery-lock",               "measurement",       PIDPriority::LOW,  "704"},
  {"Odometer",          "220101", "[B4:B6]",              "km", "distance",         "mdi:counter",                    "total_increasing",  PIDPriority::LOW,  "704"},
  {"12V Battery",       "220102", "B4/10",                "V",  "voltage",          "mdi:car-battery",                "measurement",       PIDPriority::HIGH, "704"},
};

// ── XPENG G6 PIDs (VCU 7E0→7E8) ───────────────────────────────────────
static const OBDPid G6_VCU_PIDS[] = {
  {"Vehicle Speed",     "220104", "[B4:B5]/100",          "km/h","speed",            "mdi:speedometer",                "measurement",       PIDPriority::HIGH, "7E0"},
  {"Accelerator Pedal", "220313", "B4/2",                 "%",   "",                 "mdi:gauge",                      "measurement",       PIDPriority::LOW,  "7E0"},
  {"Front Motor RPM",   "220317", "[B4:B5]-16000",        "rpm", "",                 "mdi:engine",                     "measurement",       PIDPriority::LOW,  "7E0"},
  {"Rear Motor RPM",    "220318", "[B4:B5]-16000",        "rpm", "",                 "mdi:engine",                     "measurement",       PIDPriority::LOW,  "7E0"},
  {"Front Motor Torque","220319", "[B4:B5]/4-500",        "Nm",  "",                 "mdi:engine",                     "measurement",       PIDPriority::LOW,  "7E0"},
  {"Rear Motor Torque", "22031A", "[B4:B5]/4-500",        "Nm",  "",                 "mdi:engine",                     "measurement",       PIDPriority::LOW,  "7E0"},
  {"Charging HVIL",     "22031D", "B4",                   "",    "",                 "mdi:ev-station",                 "measurement",       PIDPriority::HIGH, "7E0"},
  {"VCU SoC",           "22031E", "[B4:B5]/10",           "%",   "battery",          "mdi:battery",                    "measurement",       PIDPriority::LOW,  "7E0"},
  {"DC Charge Current", "22031F", "[B4:B5]/10-1200",      "A",   "current",          "mdi:current-dc",                 "measurement",       PIDPriority::HIGH, "7E0"},
  {"DC Charge Voltage", "220320", "[B4:B5]",              "V",   "voltage",          "mdi:lightning-bolt",             "measurement",       PIDPriority::HIGH, "7E0"},
  {"Brake Pressure",    "220321", "[B4:B5]/5",            "bar", "pressure",         "mdi:car-brake-alert",            "measurement",       PIDPriority::LOW,  "7E0"},
  {"Fast Charge Temp 1","220322", "B4-40",                "°C",  "temperature",      "mdi:thermometer",                "measurement",       PIDPriority::LOW,  "7E0"},
  {"Fast Charge Temp 2","220323", "B4-40",                "°C",  "temperature",      "mdi:thermometer",                "measurement",       PIDPriority::LOW,  "7E0"},
  {"Slow Charge Temp 1","220324", "B4-40",                "°C",  "temperature",      "mdi:thermometer",                "measurement",       PIDPriority::LOW,  "7E0"},
  {"Slow Charge Temp 2","220325", "B4-40",                "°C",  "temperature",      "mdi:thermometer",                "measurement",       PIDPriority::LOW,  "7E0"},
  {"Slow Charge Temp 3","220326", "B4-40",                "°C",  "temperature",      "mdi:thermometer",                "measurement",       PIDPriority::LOW,  "7E0"},
  {"Motor Temp",        "220327", "B4/2-40",              "°C",  "temperature",      "mdi:engine-coolant",             "measurement",       PIDPriority::LOW,  "7E0"},
  {"Coolant Temp",      "220328", "B4/2-40",              "°C",  "temperature",      "mdi:coolant-temperature",        "measurement",       PIDPriority::LOW,  "7E0"},
};

// ── Total counts ───────────────────────────────────────────────────────
#define G6_BMS_COUNT (sizeof(G6_BMS_PIDS) / sizeof(OBDPid))
#define G6_VCU_COUNT (sizeof(G6_VCU_PIDS) / sizeof(OBDPid))
