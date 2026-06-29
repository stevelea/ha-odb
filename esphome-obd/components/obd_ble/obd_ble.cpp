// ESPHome external component: OBD-II BLE client implementation
#include "obd_ble.h"
#include "esphome/core/log.h"
#include <NimBLEDevice.h>
#include <NimBLEClient.h>
#include <NimBLERemoteService.h>
#include <NimBLERemoteCharacteristic.h>
#include <cstdlib>
#include <cstring>
#include <algorithm>

namespace esphome {
namespace obd_ble {

static const char* TAG = "obd_ble";

// ── BLE GATT UUIDs ────────────────────────────────────────────────────
static const NimBLEUUID SVC_UUID_FFF0("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_WRITE_UUID("0000fff1-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_NOTIFY_UUID("0000fff2-0000-1000-8000-00805f9b34fb");
// Vgate fallback
static const NimBLEUUID SVC_UUID_E781("e7810a71-73ae-499d-8c15-faa9aef0c3f2");
static const NimBLEUUID CHAR_SHARED_UUID("bef8d6c9-9c21-4c9e-b632-bd58c1009f9f");

// ── ELM327 constants ──────────────────────────────────────────────────
static const char ELM_PROMPT = '>';
static const uint32_t CMD_TIMEOUT_MS = 5000;
static const uint32_t RECONNECT_DELAY_MS = 10000;
static const uint32_t MIN_RESPONSE_WAIT_MS = 200;

// ── G6 PID definitions (ported from const.py / CarSOC v4) ─────────────
static const OBDPidDef G6_BMS[] = {
  {"SOC",               "221109", "[B4:B5]/10",           "%",  "battery",          "measurement",      "mdi:battery",                    true,  "704"},
  {"SOH",               "22110A", "[B4:B5]/10",           "%",  "battery",          "measurement",      "mdi:battery-heart",              false, "704"},
  {"HV Voltage",        "221101", "[B4:B5]/10",           "V",  "voltage",          "measurement",      "mdi:lightning-bolt",             true,  "704"},
  {"HV Current",        "221103", "[B4:B5]/2-1600",       "A",  "current",          "measurement",      "mdi:current-dc",                 true,  "704"},
  {"Max Cell Voltage",  "221105", "[B4:B5]/1000",         "V",  "voltage",          "measurement",      "mdi:battery-plus",               false, "704"},
  {"Min Cell Voltage",  "221106", "[B4:B5]/1000",         "V",  "voltage",          "measurement",      "mdi:battery-minus",              false, "704"},
  {"Max Battery Temp",  "221107", "B4-40",                "°C", "temperature",      "measurement",      "mdi:thermometer-alert",          true,  "704"},
  {"Min Battery Temp",  "221108", "B4-40",                "°C", "temperature",      "measurement",      "mdi:thermometer",                false, "704"},
  {"CLTC Range",        "221118", "[B4:B5]",              "km", "distance",         "measurement",      "mdi:map-marker-distance",        false, "704"},
  {"Cumulative Charge", "221120", "A<<24+B<<16+C<<8+D",   "Ah", "energy",           "total_increasing", "mdi:battery-charging",           false, "704"},
  {"Cumulative Dischg", "221121", "A<<24+B<<16+C<<8+D",   "Ah", "energy",           "total_increasing", "mdi:battery-arrow-down",         false, "704"},
  {"Charge Status",     "22112D", "B4",                   "",   "",                 "",                 "mdi:ev-station",                 true,  "704"},
  {"Charge Limit",      "221130", "[B4:B5]-10",           "%",  "",                 "measurement",      "mdi:battery-lock",               false, "704"},
  {"Odometer",          "220101", "[B4:B6]",              "km", "distance",         "total_increasing", "mdi:counter",                    false, "704"},
  {"12V Battery",       "220102", "B4/10",                "V",  "voltage",          "measurement",      "mdi:car-battery",                true,  "704"},
};

static const OBDPidDef G6_VCU[] = {
  {"Vehicle Speed",     "220104", "[B4:B5]/100",          "km/h","speed",           "measurement",      "mdi:speedometer",                true,  "7E0"},
  {"Accelerator Pedal", "220313", "B4/2",                 "%",   "",                 "measurement",      "mdi:gauge",                      false, "7E0"},
  {"Front Motor RPM",   "220317", "[B4:B5]-16000",        "rpm", "",                 "measurement",      "mdi:engine",                     false, "7E0"},
  {"Rear Motor RPM",    "220318", "[B4:B5]-16000",        "rpm", "",                 "measurement",      "mdi:engine",                     false, "7E0"},
  {"Front Motor Torque","220319", "[B4:B5]/4-500",        "Nm",  "",                 "measurement",      "mdi:engine",                     false, "7E0"},
  {"Rear Motor Torque", "22031A", "[B4:B5]/4-500",        "Nm",  "",                 "measurement",      "mdi:engine",                     false, "7E0"},
  {"Charging HVIL",     "22031D", "B4",                   "",    "",                 "",                 "mdi:ev-station",                 true,  "7E0"},
  {"VCU SoC",           "22031E", "[B4:B5]/10",           "%",   "battery",          "measurement",      "mdi:battery",                    false, "7E0"},
  {"DC Charge Current", "22031F", "[B4:B5]/10-1200",      "A",   "current",          "measurement",      "mdi:current-dc",                 true,  "7E0"},
  {"DC Charge Voltage", "220320", "[B4:B5]",              "V",   "voltage",          "measurement",      "mdi:lightning-bolt",             true,  "7E0"},
  {"Brake Pressure",    "220321", "[B4:B5]/5",            "bar", "pressure",         "measurement",      "mdi:car-brake-alert",            false, "7E0"},
  {"Fast Charge Temp 1","220322", "B4-40",                "°C",  "temperature",      "measurement",      "mdi:thermometer",                false, "7E0"},
  {"Fast Charge Temp 2","220323", "B4-40",                "°C",  "temperature",      "measurement",      "mdi:thermometer",                false, "7E0"},
  {"Slow Charge Temp 1","220324", "B4-40",                "°C",  "temperature",      "measurement",      "mdi:thermometer",                false, "7E0"},
  {"Slow Charge Temp 2","220325", "B4-40",                "°C",  "temperature",      "measurement",      "mdi:thermometer",                false, "7E0"},
  {"Slow Charge Temp 3","220326", "B4-40",                "°C",  "temperature",      "measurement",      "mdi:thermometer",                false, "7E0"},
  {"Motor Temp",        "220327", "B4/2-40",              "°C",  "temperature",      "measurement",      "mdi:engine-coolant",             false, "7E0"},
  {"Coolant Temp",      "220328", "B4/2-40",              "°C",  "temperature",      "measurement",      "mdi:coolant-temperature",        false, "7E0"},
};

static const int G6_BMS_COUNT = sizeof(G6_BMS) / sizeof(OBDPidDef);
static const int G6_VCU_COUNT = sizeof(G6_VCU) / sizeof(OBDPidDef);

// ── ELM327 AT init commands (G6 profile) ──────────────────────────────
static const char* ELM_INIT_G6[] = {
  "AT Z", "AT E0", "AT L0", "AT S0", "AT H1",
  "AT SP6", "AT M0", "AT AT1", "AT FCSM1",
  "AT SH 704", "AT CRA 784", "AT FCSH 704",
};

// ── Helpers ───────────────────────────────────────────────────────────
static std::string bytes_to_hex(const uint8_t* data, size_t len) {
  std::string out;
  out.reserve(len * 2);
  char buf[3];
  for (size_t i = 0; i < len; i++) {
    snprintf(buf, sizeof(buf), "%02X", data[i]);
    out += buf;
  }
  return out;
}

static int hex_to_int(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

static std::vector<int> parse_hex_bytes(const std::string& hex) {
  std::vector<int> bytes;
  for (size_t i = 0; i + 1 < hex.length(); i += 2) {
    bytes.push_back((hex_to_int(hex[i]) << 4) | hex_to_int(hex[i + 1]));
  }
  return bytes;
}

// ── OBDComponent implementation ───────────────────────────────────────

void OBDComponent::setup() {
  ESP_LOGI(TAG, "Setting up OBD BLE component...");
  ESP_LOGI(TAG, "MAC: %s  Profile: %s", mac_address_.c_str(), profile_.c_str());

  // Build PID list from profile
  if (profile_ == "xpeng_g6") {
    for (int i = 0; i < G6_BMS_COUNT; i++) pids_.push_back(G6_BMS[i]);
    for (int i = 0; i < G6_VCU_COUNT; i++) pids_.push_back(G6_VCU[i]);
  } else {
    // SAE profile — placeholder, not implemented yet
    ESP_LOGW(TAG, "SAE profile not implemented yet");
  }

  ESP_LOGI(TAG, "Loaded %d PIDs", pids_.size());

  // Create sensors for each PID
  for (const auto& pid : pids_) {
    auto* sens = new sensor::Sensor();
    sens->set_name(std::string("obd_") + pid.name);
    if (strlen(pid.unit) > 0) sens->set_unit_of_measurement(pid.unit);
    if (strlen(pid.device_class) > 0) sens->set_device_class(pid.device_class);
    if (strlen(pid.state_class) > 0) {
      if (strcmp(pid.state_class, "total_increasing") == 0)
        sens->set_state_class(sensor::STATE_CLASS_TOTAL_INCREASING);
      else
        sens->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
    }
    App.register_sensor(sens);
    sensors_.push_back(sens);
    last_values_.push_back(NAN);
  }

  // Initialize NimBLE
  NimBLEDevice::init("esp32-obd");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  ESP_LOGI(TAG, "Setup complete. Will attempt BLE connection on first update.");
}

void OBDComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "OBD-II BLE Component:");
  ESP_LOGCONFIG(TAG, "  MAC Address: %s", mac_address_.c_str());
  ESP_LOGCONFIG(TAG, "  Profile: %s", profile_.c_str());
  ESP_LOGCONFIG(TAG, "  PIDs: %d", pids_.size());
}

void OBDComponent::update() {
  // Called on the polling interval. Start a new polling cycle.
  if (state_ == PollState::IDLE && client_ != nullptr && client_->isConnected()) {
    poll_cycle_++;
    current_pid_index_ = 0;
    bms_done_ = false;
    state_ = PollState::POLL_BMS;
    state_start_ms_ = millis();
    ESP_LOGD(TAG, "Starting poll cycle %d", poll_cycle_);
  }
}

void OBDComponent::loop() {
  uint32_t now = millis();

  switch (state_) {
    case PollState::IDLE:
      // First run — initiate connection
      if (client_ == nullptr) {
        state_ = PollState::CONNECTING;
        state_start_ms_ = now;
      }
      break;

    case PollState::CONNECTING:
      if (now - state_start_ms_ > 1000) {  // debounce
        if (connect_ble()) {
          state_ = PollState::DISCOVERING;
        } else {
          ESP_LOGW(TAG, "BLE connection failed, retrying in %dms", RECONNECT_DELAY_MS);
          state_start_ms_ = now;  // reset timer for retry
        }
      }
      break;

    case PollState::DISCOVERING:
      discover_services();
      if (char_write_ != nullptr && char_notify_ != nullptr) {
        ESP_LOGI(TAG, "GATT services discovered, starting ELM327 init");
        init_cmd_index_ = 0;
        state_ = PollState::INIT_ELM;
      }
      break;

    case PollState::INIT_ELM: {
      int count = profile_ == "xpeng_g6" ? 
        sizeof(ELM_INIT_G6) / sizeof(char*) : 0;
      if (init_cmd_index_ >= count) {
        ESP_LOGI(TAG, "ELM327 init complete");
        current_ecu_ = "bms";
        state_ = PollState::IDLE;
        break;
      }
      const char* cmd = ELM_INIT_G6[init_cmd_index_];
      send_at_command(cmd);
      current_command_ = cmd;
      state_ = PollState::INIT_WAIT;
      break;
    }

    case PollState::INIT_WAIT:
      // Wait for response (handled in on_notify)
      if (now - state_start_ms_ > CMD_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Init command '%s' timed out, continuing", current_command_.c_str());
        init_cmd_index_++;
        state_ = PollState::INIT_ELM;
      }
      break;

    case PollState::POLL_BMS:
    case PollState::POLL_VCU: {
      // Select current PID list
      bool is_bms = (state_ == PollState::POLL_BMS);
      size_t total = pids_.size();
      
      // Find the right PID
      while (current_pid_index_ < total) {
        const auto& pid = pids_[current_pid_index_];
        bool pid_is_bms = (pid.can_header && strcmp(pid.can_header, "704") == 0);
        bool pid_is_vcu = (pid.can_header && strcmp(pid.can_header, "7E0") == 0);
        
        if ((is_bms && pid_is_bms) || (!is_bms && pid_is_vcu)) {
          // Skip low-priority every LOW_PRIORITY_MULT cycles
          if (!pid.high_priority && (poll_cycle_ % 12 != 1)) {
            current_pid_index_++;
            continue;
          }
          break;
        }
        current_pid_index_++;
      }

      if (is_bms && current_pid_index_ >= total) {
        // BMS done, switch to VCU
        current_pid_index_ = 0;
        bms_done_ = true;
      }

      if (!is_bms && current_pid_index_ >= total) {
        // All done
        state_ = PollState::IDLE;
        break;
      }

      // Switch ECU if needed
      const char* target_ecu = is_bms ? "bms" : "vcu";
      if (current_ecu_ != target_ecu) {
        switch_ecu_header(target_ecu);
        current_ecu_ = target_ecu;
      }

      // Send query
      const auto& pid = pids_[current_pid_index_];
      std::string cmd = std::string("22 ") + pid.pid_hex;
      send_obd_query(cmd);
      current_command_ = pid.pid_hex;
      state_ = PollState::WAIT_RESPONSE;
      state_start_ms_ = now;
      break;
    }

    case PollState::WAIT_RESPONSE:
      // Handled in on_notify callback
      if (now - state_start_ms_ > CMD_TIMEOUT_MS) {
        ESP_LOGD(TAG, "PID %s timed out", current_command_.c_str());
        current_pid_index_++;
        state_ = (bms_done_ && current_pid_index_ >= pids_.size()) 
                  ? PollState::IDLE 
                  : (bms_done_ ? PollState::POLL_VCU : PollState::POLL_BMS);
      }
      break;
  }
}

// ── BLE connection ────────────────────────────────────────────────────

bool OBDComponent::connect_ble() {
  ESP_LOGI(TAG, "Scanning for OBD adapter %s...", mac_address_.c_str());

  NimBLEScan* scan = NimBLEDevice::getScan();
  NimBLEAdvertisedDevice* device = scan->getDeviceByAddress(mac_address_, 15000);
  
  if (device == nullptr) {
    ESP_LOGW(TAG, "Device %s not found in scan", mac_address_.c_str());
    return false;
  }

  ESP_LOGI(TAG, "Found %s, connecting...", device->getName().c_str());

  if (client_ != nullptr) {
    NimBLEDevice::deleteClient(client_);
    client_ = nullptr;
  }

  client_ = NimBLEDevice::createClient();
  client_->setConnectTimeout(15);

  if (!client_->connect(device)) {
    ESP_LOGW(TAG, "Failed to connect to %s", mac_address_.c_str());
    NimBLEDevice::deleteClient(client_);
    client_ = nullptr;
    return false;
  }

  ESP_LOGI(TAG, "Connected! MTU: %d", client_->getMTU());
  return true;
}

void OBDComponent::discover_services() {
  if (client_ == nullptr) return;

  // Try Veepeak FFF0 service first
  svc_ = client_->getService(SVC_UUID_FFF0);
  if (svc_ != nullptr) {
    char_write_ = svc_->getCharacteristic(CHAR_WRITE_UUID);
    char_notify_ = svc_->getCharacteristic(CHAR_NOTIFY_UUID);
    if (char_write_ && char_notify_) {
      ESP_LOGI(TAG, "Found Veepeak service (FFF0), FFF1 write, FFF2 notify");
    }
  }

  // Fallback: Vgate
  if (char_write_ == nullptr) {
    svc_ = client_->getService(SVC_UUID_E781);
    if (svc_ != nullptr) {
      char_write_ = svc_->getCharacteristic(CHAR_SHARED_UUID);
      char_notify_ = char_write_;
      if (char_write_) {
        ESP_LOGI(TAG, "Found Vgate service (E781), shared char");
      }
    }
  }

  // Subscribe to notifications
  if (char_notify_ != nullptr) {
    char_notify_->subscribe(true, notify_callback, this);
    ESP_LOGI(TAG, "Subscribed to notify characteristic");
  }
}

// ── ELM327 protocol ───────────────────────────────────────────────────

bool OBDComponent::send_at_command(const std::string& cmd) {
  if (char_write_ == nullptr) return false;
  std::string payload = cmd + "\r";
  ESP_LOGD(TAG, ">> %s", cmd.c_str());
  return char_write_->writeValue((const uint8_t*)payload.c_str(), payload.length(), false);
}

bool OBDComponent::send_obd_query(const std::string& pid_hex) {
  if (char_write_ == nullptr) return false;
  std::string payload = std::string("22 ") + pid_hex + "\r";
  ESP_LOGD(TAG, ">> 22 %s", pid_hex.c_str());
  return char_write_->writeValue((const uint8_t*)payload.c_str(), payload.length(), false);
}

void OBDComponent::switch_ecu_header(const std::string& ecu) {
  if (ecu == "bms") {
    send_at_command("AT SH 704");
    delay(50);
    send_at_command("AT CRA 784");
    delay(50);
    send_at_command("AT FCSH 704");
    delay(50);
    ESP_LOGD(TAG, "Switched to BMS (704)");
  } else if (ecu == "vcu") {
    send_at_command("AT SH 7E0");
    delay(50);
    send_at_command("AT CRA 7E8");
    delay(50);
    send_at_command("AT FCSH 7E0");
    delay(50);
    ESP_LOGD(TAG, "Switched to VCU (7E0)");
  }
}

// ── Notification callback ─────────────────────────────────────────────

void OBDComponent::notify_callback(NimBLERemoteCharacteristic* pChar,
                                   uint8_t* pData, size_t length,
                                   bool isNotify, void* arg) {
  auto* self = static_cast<OBDComponent*>(arg);
  self->on_notify(pData, length);
}

void OBDComponent::on_notify(uint8_t* data, size_t length) {
  // Convert to string
  for (size_t i = 0; i < length; i++) {
    if (data[i] == ELM_PROMPT) {
      // Prompt received — process buffer
      std::string response = rx_buffer_;
      rx_buffer_.clear();

      // Clean response
      std::string clean;
      for (char c : response) {
        if (c != ' ' && c != '\r' && c != '\n' && c != '>') {
          clean += c;
        }
      }

      if (clean.empty()) return;

      // Dispatch based on state
      if (state_ == PollState::INIT_WAIT) {
        ESP_LOGD(TAG, "Init response: %s", clean.c_str());
        init_cmd_index_++;
        state_ = PollState::INIT_ELM;

      } else if (state_ == PollState::WAIT_RESPONSE) {
        // Parse the PID response
        const auto& pid = pids_[current_pid_index_];
        float value = parse_response(clean, pid.formula);
        
        if (!std::isnan(value)) {
          publish_sensor(current_pid_index_, value);
          ESP_LOGD(TAG, "%s = %.2f %s", pid.name, value, pid.unit);
        } else {
          ESP_LOGD(TAG, "%s: no data", pid.name);
        }

        current_pid_index_++;
        state_ = (bms_done_ && current_pid_index_ >= pids_.size())
                  ? PollState::IDLE
                  : (bms_done_ ? PollState::POLL_VCU : PollState::POLL_BMS);
      }
    } else {
      rx_buffer_ += (char)data[i];
    }
  }
}

// ── Formula parser (WiCAN notation) ───────────────────────────────────

float OBDComponent::parse_response(const std::string& response, const std::string& formula) {
  // Error detection
  if (response.find("ERROR") != std::string::npos ||
      response.find("STOPPED") != std::string::npos ||
      response.find("SEARCHING") != std::string::npos ||
      response.find("NO DATA") != std::string::npos ||
      response.find("UNABLE") != std::string::npos) {
    return NAN;
  }

  std::string clean = response;

  // Strip 3-nibble CAN header (784, 7E8, etc.)
  if (clean.length() >= 3) {
    std::string first3 = clean.substr(0, 3);
    if ((first3[0] == '7' || first3[0] == '7') && 
        isxdigit(first3[1]) && isxdigit(first3[2])) {
      clean = clean.substr(3);
    }
  }

  if (clean.length() < 6) return NAN;

  auto bytes = parse_hex_bytes(clean);
  if (bytes.size() < 2) return NAN;

  // Negative response check
  if (bytes.size() >= 3 && bytes[1] == 0x7F) return NAN;

  // Simple expression evaluator
  std::string expr = formula;
  size_t pos;

  // [B4:B5] → multi-byte big-endian
  while ((pos = expr.find("[B")) != std::string::npos) {
    size_t colon = expr.find(':', pos);
    size_t close = expr.find(']', colon);
    if (colon == std::string::npos || close == std::string::npos) break;
    
    int start = atoi(expr.c_str() + pos + 2);
    int end = atoi(expr.c_str() + colon + 2);
    
    int64_t val = 0;
    for (int i = start; i <= end && i < (int)bytes.size(); i++) {
      val = (val << 8) | bytes[i];
    }
    
    std::string val_str = std::to_string(val);
    expr.replace(pos, close - pos + 1, val_str);
  }

  // B4 → single byte
  while ((pos = expr.find('B')) != std::string::npos) {
    int idx = 0;
    size_t i = pos + 1;
    while (i < expr.length() && isdigit(expr[i])) {
      idx = idx * 10 + (expr[i] - '0');
      i++;
    }
    int bval = (idx < (int)bytes.size()) ? bytes[idx] : 0;
    expr.replace(pos, i - pos, std::to_string(bval));
  }

  // A/B/C/D notation — find 62 service byte and extract data
  if (expr.find('A') != std::string::npos) {
    int data_start = 0;
    for (size_t i = 0; i + 1 < bytes.size(); i++) {
      if (bytes[i] == 0x62) {
        data_start = i + 1;
        if (data_start + 2 < (int)bytes.size()) data_start += 2;
        break;
      }
    }
    for (size_t i = 0; i < expr.length(); i++) {
      char ch = expr[i];
      if (ch >= 'A' && ch <= 'D') {
        int idx = ch - 'A';
        int bval = (data_start + idx < (int)bytes.size()) ? bytes[data_start + idx] : 0;
        expr.replace(i, 1, std::to_string(bval));
      }
    }
  }

  // Evaluate the expression (supports + - * / << >>)
  auto eval_simple = [](const std::string& s) -> float {
    char* end;
    float result = strtof(s.c_str(), &end);
    if (end == s.c_str()) return NAN;

    // Handle remaining operators manually
    std::string remaining(end);
    if (remaining.empty()) return result;

    // Simple left-to-right evaluation for + - * /
    size_t pp = 0;
    while (pp < remaining.length()) {
      char op = remaining[pp];
      pp++;
      float next = strtof(remaining.c_str() + pp, &end);
      if (end == remaining.c_str() + pp) break;
      pp = end - remaining.c_str();
      
      if (op == '+') result += next;
      else if (op == '-') result -= next;
      else if (op == '*') result *= next;
      else if (op == '/') result = (next != 0) ? result / next : 0;
    }
    return result;
  };

  return eval_simple(expr);
}

// ── Sensor publishing ─────────────────────────────────────────────────

void OBDComponent::publish_sensor(size_t idx, float value) {
  if (idx >= sensors_.size()) return;
  last_values_[idx] = value;
  sensors_[idx]->publish_state(value);
}

}  // namespace obd_ble
}  // namespace esphome
