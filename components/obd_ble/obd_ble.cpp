// ESPHome external component: OBD-II BLE client implementation
// ESPHome 2026.6 + NimBLE-Arduino 2.2.3 compatible
#include "obd_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
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
static const NimBLEUUID SVC_UUID_E781("e7810a71-73ae-499d-8c15-faa9aef0c3f2");
static const NimBLEUUID CHAR_SHARED_UUID("bef8d6c9-9c21-4c9e-b632-bd58c1009f9f");

// ── ELM327 constants ──────────────────────────────────────────────────
static const char ELM_PROMPT = '>';
static const uint32_t CMD_TIMEOUT_MS = 5000;
static const uint32_t RECONNECT_DELAY_MS = 10000;

// ── G6 PID definitions in C++ (formulas + hex + priority + ECU) ───────
static const OBDPidDef G6_BMS[] = {
  {"SOC",               "221109", "[B4:B5]/10",           true,  "704"},
  {"SOH",               "22110A", "[B4:B5]/10",           false, "704"},
  {"HV Voltage",        "221101", "[B4:B5]/10",           true,  "704"},
  {"HV Current",        "221103", "[B4:B5]/2-1600",       true,  "704"},
  {"Max Cell Voltage",  "221105", "[B4:B5]/1000",         false, "704"},
  {"Min Cell Voltage",  "221106", "[B4:B5]/1000",         false, "704"},
  {"Max Battery Temp",  "221107", "B4-40",                true,  "704"},
  {"Min Battery Temp",  "221108", "B4-40",                false, "704"},
  {"CLTC Range",        "221118", "[B4:B5]",              false, "704"},
  {"Cumulative Charge", "221120", "A<<24+B<<16+C<<8+D",   false, "704"},
  {"Cumulative Dischg", "221121", "A<<24+B<<16+C<<8+D",   false, "704"},
  {"Charge Status",     "22112D", "B4",                   true,  "704"},
  {"Charge Limit",      "221130", "[B4:B5]-10",           false, "704"},
  {"Odometer",          "220101", "[B4:B6]",              false, "704"},
  {"12V Battery",       "220102", "B4/10",                true,  "704"},
};

static const OBDPidDef G6_VCU[] = {
  {"Vehicle Speed",     "220104", "[B4:B5]/100",          true,  "7E0"},
  {"Accelerator Pedal", "220313", "B4/2",                 false, "7E0"},
  {"Front Motor RPM",   "220317", "[B4:B5]-16000",        false, "7E0"},
  {"Rear Motor RPM",    "220318", "[B4:B5]-16000",        false, "7E0"},
  {"Front Motor Torque","220319", "[B4:B5]/4-500",        false, "7E0"},
  {"Rear Motor Torque", "22031A", "[B4:B5]/4-500",        false, "7E0"},
  {"Charging HVIL",     "22031D", "B4",                   true,  "7E0"},
  {"VCU SoC",           "22031E", "[B4:B5]/10",           false, "7E0"},
  {"DC Charge Current", "22031F", "[B4:B5]/10-1200",      true,  "7E0"},
  {"DC Charge Voltage", "220320", "[B4:B5]",              true,  "7E0"},
  {"Brake Pressure",    "220321", "[B4:B5]/5",            false, "7E0"},
  {"Fast Charge Temp 1","220322", "B4-40",                false, "7E0"},
  {"Fast Charge Temp 2","220323", "B4-40",                false, "7E0"},
  {"Slow Charge Temp 1","220324", "B4-40",                false, "7E0"},
  {"Slow Charge Temp 2","220325", "B4-40",                false, "7E0"},
  {"Slow Charge Temp 3","220326", "B4-40",                false, "7E0"},
  {"Motor Temp",        "220327", "B4/2-40",              false, "7E0"},
  {"Coolant Temp",      "220328", "B4/2-40",              false, "7E0"},
};

static const int G6_BMS_COUNT = sizeof(G6_BMS) / sizeof(OBDPidDef);
static const int G6_VCU_COUNT = sizeof(G6_VCU) / sizeof(OBDPidDef);

// ── ELM327 AT init commands (G6) ──────────────────────────────────────
static const char* ELM_INIT_G6[] = {
  "AT Z", "AT E0", "AT L0", "AT S0", "AT H1",
  "AT SP6", "AT M0", "AT AT1", "AT FCSM1",
  "AT SH 704", "AT CRA 784", "AT FCSH 704",
};

// ── Helpers ───────────────────────────────────────────────────────────
static std::vector<int> hex_to_bytes(const std::string& hex) {
  std::vector<int> bytes;
  for (size_t i = 0; i + 1 < hex.length(); i += 2) {
    int h = hex[i];
    int l = hex[i + 1];
    int hi = (h >= '0' && h <= '9') ? h - '0' : (h >= 'A' && h <= 'F') ? h - 'A' + 10 : (h >= 'a' && h <= 'f') ? h - 'a' + 10 : 0;
    int lo = (l >= '0' && l <= '9') ? l - '0' : (l >= 'A' && l <= 'F') ? l - 'A' + 10 : (l >= 'a' && l <= 'f') ? l - 'a' + 10 : 0;
    bytes.push_back((hi << 4) | lo);
  }
  return bytes;
}

// ── OBDComponent ──────────────────────────────────────────────────────

void OBDComponent::setup() {
  ESP_LOGI(TAG, "OBD BLE component: MAC=%s profile=%s", mac_address_.c_str(), profile_.c_str());

  // Build PID list in C++
  if (profile_ == "xpeng_g6") {
    for (int i = 0; i < G6_BMS_COUNT; i++) pids_.push_back(G6_BMS[i]);
    for (int i = 0; i < G6_VCU_COUNT; i++) pids_.push_back(G6_VCU[i]);
  }

  ESP_LOGI(TAG, "Loaded %d PID definitions, %d sensors", pids_.size(), sensors_.size());
  ESP_LOGI(TAG, "Will attempt BLE connection on first update cycle.");
}

void OBDComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "OBD-II BLE Component:");
  ESP_LOGCONFIG(TAG, "  MAC: %s  Profile: %s", mac_address_.c_str(), profile_.c_str());
  ESP_LOGCONFIG(TAG, "  PIDs: %d  Sensors: %d", pids_.size(), sensors_.size());
}

void OBDComponent::update() {
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
      if (client_ == nullptr || !client_->isConnected()) {
        state_ = PollState::CONNECTING;
        state_start_ms_ = now;
      }
      break;

    case PollState::CONNECTING:
      if (now - state_start_ms_ > 5000) {
        if (connect_ble()) {
          state_ = PollState::DISCOVERING;
        } else {
          ESP_LOGW(TAG, "Connection failed, retry in %dms", RECONNECT_DELAY_MS);
          state_start_ms_ = now;
        }
      }
      break;

    case PollState::DISCOVERING:
      discover_services();
      if (char_write_ != nullptr && char_notify_ != nullptr) {
        ESP_LOGI(TAG, "GATT services ready, starting ELM327 init");
        init_cmd_index_ = 0;
        state_ = PollState::INIT_ELM;
      } else {
        ESP_LOGW(TAG, "Service discovery incomplete, retrying...");
        state_ = PollState::CONNECTING;
        state_start_ms_ = now;
      }
      break;

    case PollState::INIT_ELM: {
      int count = profile_ == "xpeng_g6" ? sizeof(ELM_INIT_G6)/sizeof(char*) : 0;
      if (init_cmd_index_ >= count) {
        ESP_LOGI(TAG, "ELM327 init complete, ready to poll");
        current_ecu_ = "bms";
        state_ = PollState::IDLE;
        break;
      }
      send_at_command(ELM_INIT_G6[init_cmd_index_]);
      current_command_ = ELM_INIT_G6[init_cmd_index_];
      state_ = PollState::INIT_WAIT;
      state_start_ms_ = now;
      break;
    }

    case PollState::INIT_WAIT:
      if (now - state_start_ms_ > CMD_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Init '%s' timed out, continuing", current_command_.c_str());
        init_cmd_index_++;
        state_ = PollState::INIT_ELM;
      }
      break;

    case PollState::POLL_BMS:
    case PollState::POLL_VCU: {
      bool is_bms = (state_ == PollState::POLL_BMS);
      size_t total = pids_.size();

      // Find the next PID matching the current ECU
      while (current_pid_index_ < total) {
        const auto& pid = pids_[current_pid_index_];
        bool pid_is_bms = pid.can_header && strcmp(pid.can_header, "704") == 0;
        bool pid_is_vcu = pid.can_header && strcmp(pid.can_header, "7E0") == 0;

        if ((is_bms && pid_is_bms) || (!is_bms && pid_is_vcu)) {
          if (!pid.high_priority && (poll_cycle_ % 12 != 1)) {
            current_pid_index_++;
            continue;
          }
          break;
        }
        current_pid_index_++;
      }

      if (is_bms && current_pid_index_ >= total) {
        // All BMS done, switch to VCU
        current_pid_index_ = 0;
        bms_done_ = true;
      }
      if (!is_bms && current_pid_index_ >= total) {
        state_ = PollState::IDLE;
        break;
      }

      const char* target_ecu = is_bms ? "bms" : "vcu";
      if (current_ecu_ != target_ecu) {
        switch_ecu_header(target_ecu);
        current_ecu_ = target_ecu;
      }

      const auto& pid = pids_[current_pid_index_];
      send_obd_query(std::string("22 ") + pid.pid_hex);
      current_command_ = pid.pid_hex;
      state_ = PollState::WAIT_RESPONSE;
      state_start_ms_ = now;
      break;
    }

    case PollState::WAIT_RESPONSE:
      if (now - state_start_ms_ > CMD_TIMEOUT_MS) {
        ESP_LOGD(TAG, "PID %s timeout", current_command_.c_str());
        current_pid_index_++;
        state_ = (bms_done_ && current_pid_index_ >= pids_.size()) 
                  ? PollState::IDLE 
                  : (bms_done_ ? PollState::POLL_VCU : PollState::POLL_BMS);
      }
      break;
  }
}

// ── BLE connection (NimBLE-Arduino 2.2.3 API) ─────────────────────────

bool OBDComponent::connect_ble() {
  ESP_LOGI(TAG, "Scanning for %s...", mac_address_.c_str());

  NimBLEScan* scan = NimBLEDevice::getScan();
  NimBLEScanResults results = scan->getResults(10000);  // 10s scan

  const NimBLEAdvertisedDevice* device = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    device = results.getDevice(i);
    if (device->getAddress().toString() == mac_address_) {
      break;
    }
    device = nullptr;
  }

  if (device == nullptr) {
    ESP_LOGW(TAG, "Device %s not found in scan results", mac_address_.c_str());
    return false;
  }

  ESP_LOGI(TAG, "Found %s (%s), connecting...", device->getName().c_str(), mac_address_.c_str());

  if (client_ != nullptr) {
    NimBLEDevice::deleteClient(client_);
  }

  client_ = NimBLEDevice::createClient();
  client_->setConnectTimeout(15);

  if (!client_->connect(*device)) {
    ESP_LOGW(TAG, "Failed to connect");
    NimBLEDevice::deleteClient(client_);
    client_ = nullptr;
    return false;
  }

  ESP_LOGI(TAG, "Connected! MTU=%d", client_->getMTU());
  return true;
}

void OBDComponent::discover_services() {
  if (client_ == nullptr) return;

  svc_ = client_->getService(SVC_UUID_FFF0);
  if (svc_ != nullptr) {
    char_write_ = svc_->getCharacteristic(CHAR_WRITE_UUID);
    char_notify_ = svc_->getCharacteristic(CHAR_NOTIFY_UUID);
    if (char_write_ && char_notify_)
      ESP_LOGI(TAG, "Found Veepeak service (FFF0)");
  }

  if (char_write_ == nullptr) {
    svc_ = client_->getService(SVC_UUID_E781);
    if (svc_ != nullptr) {
      char_write_ = svc_->getCharacteristic(CHAR_SHARED_UUID);
      char_notify_ = char_write_;
      if (char_write_)
        ESP_LOGI(TAG, "Found Vgate service (E781)");
    }
  }

  // NimBLE 2.2.3: subscribe uses std::function without void* context.
  // Use a capturing lambda and route through a static pointer.
  if (char_notify_ != nullptr) {
    static OBDComponent* notify_target = nullptr;
    notify_target = this;
    char_notify_->subscribe(true, [](NimBLERemoteCharacteristic* pChar,
                                     uint8_t* pData, size_t length,
                                     bool isNotify) {
      if (notify_target != nullptr)
        notify_target->on_notify(pData, length);
    });
    ESP_LOGI(TAG, "Subscribed to notifications");
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
  std::string payload = "22 " + pid_hex + "\r";
  ESP_LOGD(TAG, ">> 22 %s", pid_hex.c_str());
  return char_write_->writeValue((const uint8_t*)payload.c_str(), payload.length(), false);
}

void OBDComponent::switch_ecu_header(const std::string& ecu) {
  if (ecu == "bms") {
    send_at_command("AT SH 704"); delay(60);
    send_at_command("AT CRA 784"); delay(60);
    send_at_command("AT FCSH 704"); delay(60);
    ESP_LOGD(TAG, "Switched to BMS (704)");
  } else if (ecu == "vcu") {
    send_at_command("AT SH 7E0"); delay(60);
    send_at_command("AT CRA 7E8"); delay(60);
    send_at_command("AT FCSH 7E0"); delay(60);
    ESP_LOGD(TAG, "Switched to VCU (7E0)");
  }
}

// ── Notification handler ──────────────────────────────────────────────

void OBDComponent::on_notify(uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (data[i] == ELM_PROMPT) {
      // Prompt received — process accumulated buffer
      std::string clean;
      for (char c : rx_buffer_) {
        if (c != ' ' && c != '\r' && c != '\n' && c != '>') clean += c;
      }
      rx_buffer_.clear();

      if (clean.empty()) return;

      if (state_ == PollState::INIT_WAIT) {
        init_cmd_index_++;
        state_ = PollState::INIT_ELM;

      } else if (state_ == PollState::WAIT_RESPONSE && current_pid_index_ < pids_.size()) {
        const auto& pid = pids_[current_pid_index_];
        float value = parse_response(clean, pid.formula);

        if (!std::isnan(value) && current_pid_index_ < sensors_.size()) {
          publish_sensor(current_pid_index_, value);
          ESP_LOGD(TAG, "%s = %.2f", pid.name, value);
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

// ── Formula parser ────────────────────────────────────────────────────

float OBDComponent::parse_response(const std::string& response, const std::string& formula) {
  if (response.find("ERROR") != std::string::npos ||
      response.find("NO DATA") != std::string::npos ||
      response.find("SEARCHING") != std::string::npos ||
      response.find("UNABLE") != std::string::npos ||
      response.find("STOPPED") != std::string::npos) {
    return NAN;
  }

  std::string clean = response;
  // Strip 3-char CAN header (784, 7E8, etc.)
  if (clean.length() >= 3 && clean[0] == '7') {
    clean = clean.substr(3);
  }
  if (clean.length() < 6) return NAN;

  auto bytes = hex_to_bytes(clean);
  if (bytes.size() < 2) return NAN;
  if (bytes.size() >= 3 && bytes[1] == 0x7F) return NAN;

  std::string expr = formula;
  size_t pos;

  // [B4:B5] → big-endian multi-byte
  while ((pos = expr.find("[B")) != std::string::npos) {
    size_t colon = expr.find(':', pos);
    size_t close = expr.find(']', colon);
    if (colon == std::string::npos || close == std::string::npos) break;
    int start = atoi(expr.c_str() + pos + 2);
    int end = atoi(expr.c_str() + colon + 2);
    int64_t val = 0;
    for (int i = start; i <= end && i < (int)bytes.size(); i++)
      val = (val << 8) | bytes[i];
    expr.replace(pos, close - pos + 1, std::to_string(val));
  }

  // B4 → single byte
  while ((pos = expr.find('B')) != std::string::npos) {
    int idx = 0;
    size_t i = pos + 1;
    while (i < expr.length() && isdigit(expr[i]))
      idx = idx * 10 + (expr[i++] - '0');
    int bval = (idx < (int)bytes.size()) ? bytes[idx] : 0;
    expr.replace(pos, i - pos, std::to_string(bval));
  }

  // A/B/C/D → data bytes after 62 service byte
  if (expr.find('A') != std::string::npos) {
    int ds = 0;
    for (size_t i = 0; i + 1 < bytes.size(); i++) {
      if (bytes[i] == 0x62) { ds = i + 1; if (ds + 2 < (int)bytes.size()) ds += 2; break; }
    }
    for (size_t i = 0; i < expr.length(); i++) {
      if (expr[i] >= 'A' && expr[i] <= 'D') {
        int idx = expr[i] - 'A';
        int bval = (ds + idx < (int)bytes.size()) ? bytes[ds + idx] : 0;
        std::string val_str = std::to_string(bval);
        expr.replace(i, 1, val_str);
        i += val_str.length() - 1;
      }
    }
  }

  // Evaluate: handle << >> * / + - left-to-right
  auto tok = [](const std::string& s, size_t& p) -> float {
    while (p < s.length() && isspace(s[p])) p++;
    if (p >= s.length()) return 0;
    char* end;
    float v = strtof(s.c_str() + p, &end);
    p = end - s.c_str();
    return v;
  };

  size_t pp = 0;
  float result = tok(expr, pp);
  while (pp < expr.length()) {
    while (pp < expr.length() && isspace(expr[pp])) pp++;
    if (pp >= expr.length()) break;
    char op = expr[pp++];
    float rhs = tok(expr, pp);
    if (op == '+') result += rhs;
    else if (op == '-') result -= rhs;
    else if (op == '*') result *= rhs;
    else if (op == '/') result = (rhs != 0) ? result / rhs : 0;
    else if (op == '<' && pp < expr.length() && expr[pp] == '<') { pp++; result = (int)result << (int)rhs; }
    else if (op == '>' && pp < expr.length() && expr[pp] == '>') { pp++; result = (int)result >> (int)rhs; }
  }
  return result;
}

// ── Sensor publishing ─────────────────────────────────────────────────

void OBDComponent::publish_sensor(size_t idx, float value) {
  if (idx >= sensors_.size()) return;
  last_values_[idx] = value;
  sensors_[idx]->publish_state(value);
}

}  // namespace obd_ble
}  // namespace esphome
