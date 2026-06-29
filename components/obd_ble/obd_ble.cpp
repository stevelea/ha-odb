// ESPHome external component: OBD-II BLE client — ESP-IDF NimBLE implementation
#include "obd_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cstdlib>
#include <cstring>
#include <algorithm>

namespace esphome {
namespace obd_ble {

static const char* TAG = "obd_ble";

// ── Global instance for NimBLE callbacks ──────────────────────────────
OBDComponent* g_obd_component = nullptr;

// ── Veepeak OBDCheck BLE UUIDs ────────────────────────────────────────
static const char* SVC_UUID_STR   = "0000fff0-0000-1000-8000-00805f9b34fb";
static const char* WRITE_UUID_STR = "0000fff1-0000-1000-8000-00805f9b34fb";
static const char* NOTIFY_UUID_STR = "0000fff2-0000-1000-8000-00805f9b34fb";

static BLEUUID128 svc_uuid;
static BLEUUID128 write_uuid;
static BLEUUID128 notify_uuid;

// ── ELM327 constants ──────────────────────────────────────────────────
static const char ELM_PROMPT = '>';
static const uint32_t CMD_TIMEOUT_MS = 5000;
static const uint32_t RECONNECT_DELAY_MS = 10000;

// ── G6 PID definitions ────────────────────────────────────────────────
static const OBDPidDef G6_BMS[] = {
  {"SOC","221109","[B4:B5]/10",true,"704"},
  {"SOH","22110A","[B4:B5]/10",false,"704"},
  {"HV Voltage","221101","[B4:B5]/10",true,"704"},
  {"HV Current","221103","[B4:B5]/2-1600",true,"704"},
  {"Max Cell Voltage","221105","[B4:B5]/1000",false,"704"},
  {"Min Cell Voltage","221106","[B4:B5]/1000",false,"704"},
  {"Max Battery Temp","221107","B4-40",true,"704"},
  {"Min Battery Temp","221108","B4-40",false,"704"},
  {"CLTC Range","221118","[B4:B5]",false,"704"},
  {"Cumulative Charge","221120","A<<24+B<<16+C<<8+D",false,"704"},
  {"Cumulative Dischg","221121","A<<24+B<<16+C<<8+D",false,"704"},
  {"Charge Status","22112D","B4",true,"704"},
  {"Charge Limit","221130","[B4:B5]-10",false,"704"},
  {"Odometer","220101","[B4:B6]",false,"704"},
  {"12V Battery","220102","B4/10",true,"704"},
};
static const OBDPidDef G6_VCU[] = {
  {"Vehicle Speed","220104","[B4:B5]/100",true,"7E0"},
  {"Accelerator Pedal","220313","B4/2",false,"7E0"},
  {"Front Motor RPM","220317","[B4:B5]-16000",false,"7E0"},
  {"Rear Motor RPM","220318","[B4:B5]-16000",false,"7E0"},
  {"Front Motor Torque","220319","[B4:B5]/4-500",false,"7E0"},
  {"Rear Motor Torque","22031A","[B4:B5]/4-500",false,"7E0"},
  {"Charging HVIL","22031D","B4",true,"7E0"},
  {"VCU SoC","22031E","[B4:B5]/10",false,"7E0"},
  {"DC Charge Current","22031F","[B4:B5]/10-1200",true,"7E0"},
  {"DC Charge Voltage","220320","[B4:B5]",true,"7E0"},
  {"Brake Pressure","220321","[B4:B5]/5",false,"7E0"},
  {"Fast Charge Temp 1","220322","B4-40",false,"7E0"},
  {"Fast Charge Temp 2","220323","B4-40",false,"7E0"},
  {"Slow Charge Temp 1","220324","B4-40",false,"7E0"},
  {"Slow Charge Temp 2","220325","B4-40",false,"7E0"},
  {"Slow Charge Temp 3","220326","B4-40",false,"7E0"},
  {"Motor Temp","220327","B4/2-40",false,"7E0"},
  {"Coolant Temp","220328","B4/2-40",false,"7E0"},
};
static const int G6_BMS_COUNT = sizeof(G6_BMS)/sizeof(OBDPidDef);
static const int G6_VCU_COUNT = sizeof(G6_VCU)/sizeof(OBDPidDef);

static const char* ELM_INIT_G6[] = {
  "AT Z","AT E0","AT L0","AT S0","AT H1",
  "AT SP6","AT M0","AT AT1","AT FCSM1",
  "AT SH 704","AT CRA 784","AT FCSH 704",
};

// ── BLEUUID128 ────────────────────────────────────────────────────────
BLEUUID128 BLEUUID128::from_string(const char* s) {
  BLEUUID128 u{};
  const char* p = s;
  for (int i = 15; i >= 0 && *p; p++) {
    if (*p == '-') continue;
    int hi = (*p >= '0' && *p <= '9') ? *p - '0' : (*p >= 'a' && *p <= 'f') ? *p - 'a' + 10 : (*p >= 'A' && *p <= 'F') ? *p - 'A' + 10 : 0;
    p++;
    if (!*p) break;
    int lo = (*p >= '0' && *p <= '9') ? *p - '0' : (*p >= 'a' && *p <= 'f') ? *p - 'a' + 10 : (*p >= 'A' && *p <= 'F') ? *p - 'A' + 10 : 0;
    u.u[i--] = (hi << 4) | lo;
  }
  return u;
}
ble_uuid_any_t BLEUUID128::to_nimble() const {
  ble_uuid_any_t result;
  result.u.type = BLE_UUID_TYPE_128;
  memcpy(result.u128.value, u, 16);
  return result;
}

// ── Helpers ────────────────────────────────────────────────────────────
static std::vector<int> hex_to_bytes(const std::string& hex) {
  std::vector<int> bytes;
  for (size_t i = 0; i + 1 < hex.length(); i += 2) {
    int h = hex[i], l = hex[i+1];
    int hi = (h>='0'&&h<='9')?h-'0':(h>='A'&&h<='F')?h-'A'+10:(h>='a'&&h<='f')?h-'a'+10:0;
    int lo = (l>='0'&&l<='9')?l-'0':(l>='A'&&l<='F')?l-'A'+10:(l>='a'&&l<='f')?l-'a'+10:0;
    bytes.push_back((hi<<4)|lo);
  }
  return bytes;
}
static void mac_str_to_bytes(const std::string& s, uint8_t out[6]) {
  int j = 0;
  for (size_t i = 0; i < s.length() && j < 6; i += 2) {
    while (i < s.length() && (s[i] == ':' || s[i] == '-')) i++;
    if (i + 1 >= s.length()) break;
    int hi = (s[i]>='0'&&s[i]<='9')?s[i]-'0':(s[i]>='a'&&s[i]<='f')?s[i]-'a'+10:(s[i]>='A'&&s[i]<='F')?s[i]-'A'+10:0;
    int lo = (s[i+1]>='0'&&s[i+1]<='9')?s[i+1]-'0':(s[i+1]>='a'&&s[i+1]<='f')?s[i+1]-'a'+10:(s[i+1]>='A'&&s[i+1]<='F')?s[i+1]-'A'+10:0;
    out[j++] = (hi<<4)|lo;
  }
}

// ═══════════════════════════════════════════════════════════════════════
// OBDComponent implementation
// ═══════════════════════════════════════════════════════════════════════

void OBDComponent::setup() {
  g_obd_component = this;

  // Parse UUIDs
  svc_uuid = BLEUUID128::from_string(SVC_UUID_STR);
  write_uuid = BLEUUID128::from_string(WRITE_UUID_STR);
  notify_uuid = BLEUUID128::from_string(NOTIFY_UUID_STR);

  // Parse MAC
  mac_str_to_bytes(mac_address_, target_addr_);

  ESP_LOGI(TAG, "OBD: MAC=%s profile=%s", mac_address_.c_str(), profile_.c_str());

  if (profile_ == "xpeng_g6") {
    for (int i = 0; i < G6_BMS_COUNT; i++) pids_.push_back(G6_BMS[i]);
    for (int i = 0; i < G6_VCU_COUNT; i++) pids_.push_back(G6_VCU[i]);
  }
  ESP_LOGI(TAG, "PIDs=%d sensors=%d", pids_.size(), sensors_.size());

  // Register as BLE device listener with the tracker
  esp32_ble_tracker::global_esp32_ble_tracker->register_listener(this);
  ESP_LOGI(TAG, "Registered as BLE device listener — waiting for target...");
}

void OBDComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "OBD-II BLE:");
  ESP_LOGCONFIG(TAG, "  MAC: %s  Profile: %s", mac_address_.c_str(), profile_.c_str());
  ESP_LOGCONFIG(TAG, "  PIDs: %d  Sensors: %d", pids_.size(), sensors_.size());
}

// ── ESPBTDeviceListener: called by tracker when a device is seen ──────

bool OBDComponent::parse_device(const esp32_ble_tracker::ESPBTDevice& device) {
  if (connected_ || state_ != PollState::IDLE) return false;

  // Compare MAC
  auto addr = device.address_64();
  if (memcmp(addr, target_addr_, 6) == 0) {
    ESP_LOGI(TAG, "*** Target found by tracker! RSSI=%d, starting connection...", device.get_rssi());
    start_connect();
    return true;
  }
  return false;
}

// ── update / loop ─────────────────────────────────────────────────────

void OBDComponent::update() {
  if (state_ == PollState::IDLE && connected_ && notify_enabled_) {
    poll_cycle_++;
    current_pid_index_ = 0;
    bms_done_ = false;
    state_ = PollState::POLL_BMS;
    state_start_ms_ = millis();
  }
}

void OBDComponent::loop() {
  uint32_t now = millis();

  switch (state_) {
    case PollState::IDLE:
      break;  // Waiting for parse_device to trigger connection

    case PollState::WAIT_CONNECT:
      if (connected_) {
        ESP_LOGI(TAG, "Connected! Discovering services...");
        state_ = PollState::DISCOVER_SVC;
        discover_services();
      } else if (now - state_start_ms_ > 15000) {
        ESP_LOGW(TAG, "Connection timeout, retrying...");
        connected_ = false;
        state_ = PollState::IDLE;
        state_start_ms_ = now;
      }
      break;

    case PollState::DISCOVER_SVC:
      // Service discovery is async via callbacks
      if (svc_start_handle_ != 0 && now - state_start_ms_ > 5000) {
        ESP_LOGW(TAG, "Service discovery timeout");
        state_ = PollState::IDLE;
      }
      break;

    case PollState::DISCOVER_CHR:
      // Characteristic discovery is async via callbacks  
      if (chr_write_handle_ != 0 && chr_notify_handle_ != 0) {
        state_ = PollState::SUBSCRIBE;
        // Enable notifications
        uint8_t val[] = {0x01, 0x00};
        ble_gattc_write_flat(conn_handle_, cccd_handle_, val, 2, gatt_write_cb, this);
      } else if (now - state_start_ms_ > 10000) {
        ESP_LOGW(TAG, "Characteristic discovery timeout");
        state_ = PollState::IDLE;
      }
      break;

    case PollState::SUBSCRIBE:
      if (notify_enabled_) {
        ESP_LOGI(TAG, "Notifications enabled, starting ELM327 init");
        init_cmd_index_ = 0;
        state_ = PollState::INIT_ELM;
      } else if (now - state_start_ms_ > 5000) {
        ESP_LOGW(TAG, "Subscription timeout");
        state_ = PollState::IDLE;
      }
      break;

    case PollState::INIT_ELM: {
      int count = sizeof(ELM_INIT_G6)/sizeof(char*);
      if (init_cmd_index_ >= count) {
        ESP_LOGI(TAG, "ELM327 init done, ready to poll");
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
        init_cmd_index_++;
        state_ = PollState::INIT_ELM;
      }
      break;

    case PollState::POLL_BMS:
    case PollState::POLL_VCU: {
      bool is_bms = (state_ == PollState::POLL_BMS);
      size_t total = pids_.size();

      while (current_pid_index_ < total) {
        const auto& pid = pids_[current_pid_index_];
        bool pid_is_bms = pid.can_header && strcmp(pid.can_header, "704") == 0;
        bool pid_is_vcu = pid.can_header && strcmp(pid.can_header, "7E0") == 0;
        if ((is_bms && pid_is_bms) || (!is_bms && pid_is_vcu)) {
          if (!pid.high_priority && (poll_cycle_ % 12 != 1)) { current_pid_index_++; continue; }
          break;
        }
        current_pid_index_++;
      }

      if (is_bms && current_pid_index_ >= total) { current_pid_index_ = 0; bms_done_ = true; }
      if (!is_bms && current_pid_index_ >= total) { state_ = PollState::IDLE; break; }

      const char* target_ecu = is_bms ? "bms" : "vcu";
      if (current_ecu_ != target_ecu) { switch_ecu_header(target_ecu); current_ecu_ = target_ecu; }

      const auto& pid = pids_[current_pid_index_];
      send_obd_query(pid.pid_hex);
      current_command_ = pid.pid_hex;
      state_ = PollState::WAIT_RESPONSE;
      state_start_ms_ = now;
      break;
    }

    case PollState::WAIT_RESPONSE:
      if (now - state_start_ms_ > CMD_TIMEOUT_MS) {
        current_pid_index_++;
        state_ = (bms_done_ && current_pid_index_ >= pids_.size()) 
                  ? PollState::IDLE : (bms_done_ ? PollState::POLL_VCU : PollState::POLL_BMS);
      }
      break;
  }
}

// ── BLE Connection ────────────────────────────────────────────────────

void OBDComponent::start_connect() {
  if (connected_) return;

  // Stop the tracker's scan to free the radio for connection
  esp32_ble_tracker::global_esp32_ble_tracker->stop_scan();

  // Set up our own address
  ble_addr_t addr;
  addr.type = BLE_ADDR_RANDOM;
  memcpy(addr.val, target_addr_, 6);

  // Use a random address for ourselves (required by NimBLE when tracker is init'd)
  uint8_t own_addr[6] = {0xC0, 0xDE, 0x00, 0x00, 0x00, 0x00};
  ble_hs_id_set_rnd(own_addr);

  int rc = ble_gap_connect(BLE_OWN_ADDR_RANDOM, &addr, 15000, nullptr, gap_event_cb, this);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_gap_connect failed: %d", rc);
    // Restart tracker scan
    esp32_ble_tracker::global_esp32_ble_tracker->start_scan();
    return;
  }

  state_ = PollState::WAIT_CONNECT;
  state_start_ms_ = millis();
  ESP_LOGI(TAG, "Connection request sent, waiting...");
}

// ── GAP Event callback ────────────────────────────────────────────────

int OBDComponent::gap_event_cb(struct ble_gap_event* ev, void* arg) {
  auto* self = static_cast<OBDComponent*>(arg);
  self->handle_gap_event(ev);
  return 0;
}

void OBDComponent::handle_gap_event(struct ble_gap_event* ev) {
  switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (ev->connect.status == 0) {
        conn_handle_ = ev->connect.conn_handle;
        connected_ = true;
        ESP_LOGI(TAG, "GAP connected, handle=%d", conn_handle_);
      } else {
        ESP_LOGW(TAG, "Connection failed, status=%d", ev->connect.status);
        connected_ = false;
        esp32_ble_tracker::global_esp32_ble_tracker->start_scan();
      }
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      ESP_LOGW(TAG, "Disconnected, reason=%d", ev->disconnect.reason);
      connected_ = false;
      notify_enabled_ = false;
      conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
      state_ = PollState::IDLE;
      // Restart tracker scan for re-discovery
      esp32_ble_tracker::global_esp32_ble_tracker->start_scan();
      break;
    case BLE_GAP_EVENT_NOTIFY_RX: {
      // Received notification from the device
      auto* attr = &ev->notify_rx.attr_handle;
      // Only process notifications from our notify characteristic
      if (ev->notify_rx.attr_handle == chr_notify_handle_) {
        process_notify(ev->notify_rx.om->om_data, ev->notify_rx.om->om_len);
      }
      break;
    }
    case BLE_GAP_EVENT_MTU:
      ESP_LOGI(TAG, "MTU updated: %d", ev->mtu.value);
      break;
    default:
      break;
  }
}

// ── Service Discovery ──────────────────────────────────────────────────

void OBDComponent::discover_services() {
  ble_uuid_any_t u = svc_uuid.to_nimble();
  int rc = ble_gattc_disc_svc_by_uuid(conn_handle_, &u, gatt_disc_svc_cb, this);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_gattc_disc_svc_by_uuid failed: %d", rc);
  }
}

int OBDComponent::gatt_disc_svc_cb(uint16_t conn_handle,
                                    const struct ble_gatt_error* error,
                                    const struct ble_gatt_svc* svc,
                                    void* arg) {
  auto* self = static_cast<OBDComponent*>(arg);
  if (error->status != 0) {
    ESP_LOGW(TAG, "Service discovery error: %d", error->status);
    return 0;
  }
  if (svc != nullptr) {
    self->svc_start_handle_ = svc->start_handle;
    self->svc_end_handle_ = svc->end_handle;
    ESP_LOGI(TAG, "Found service: handles %d-%d", svc->start_handle, svc->end_handle);
    self->state_ = PollState::DISCOVER_CHR;
    self->state_start_ms_ = millis();
    self->discover_characteristics();
  }
  return 0;
}

// ── Characteristic Discovery ──────────────────────────────────────────

void OBDComponent::discover_characteristics() {
  ble_uuid_any_t w = write_uuid.to_nimble();
  int rc = ble_gattc_disc_chrs_by_uuid(conn_handle_, svc_start_handle_, svc_end_handle_, &w, gatt_disc_chr_cb, this);
  if (rc != 0) {
    ESP_LOGW(TAG, "Write char discovery failed: %d", rc);
  }
}

int OBDComponent::gatt_disc_chr_cb(uint16_t conn_handle,
                                    const struct ble_gatt_error* error,
                                    const struct ble_gatt_chr* chr,
                                    void* arg) {
  auto* self = static_cast<OBDComponent*>(arg);
  if (error->status != 0) {
    // Try notify characteristic
    ble_uuid_any_t n = notify_uuid.to_nimble();
    ble_gattc_disc_chrs_by_uuid(self->conn_handle_, self->svc_start_handle_, self->svc_end_handle_, &n, gatt_disc_chr_cb, self);
    return 0;
  }
  if (chr != nullptr) {
    // Check which characteristic this is
    ble_uuid_any_t u;
    ble_uuid_init_from_att_mbuf(&u, chr->uuid);

    if (self->chr_write_handle_ == 0) {
      self->chr_write_handle_ = chr->val_handle;
      ESP_LOGI(TAG, "Write char: handle=%d", chr->val_handle);
      // Now discover notify characteristic
      ble_uuid_any_t n = notify_uuid.to_nimble();
      ble_gattc_disc_chrs_by_uuid(self->conn_handle_, self->svc_start_handle_, self->svc_end_handle_, &n, gatt_disc_chr_cb, self);
    } else if (self->chr_notify_handle_ == 0) {
      self->chr_notify_handle_ = chr->val_handle;
      ESP_LOGI(TAG, "Notify char: handle=%d", chr->val_handle);
      // Find CCCD descriptor handle (next handle after the characteristic)
      self->cccd_handle_ = chr->val_handle + (chr->properties & BLE_GATT_CHR_PROP_NOTIFY ? 1 : 0);
      if (self->cccd_handle_ == 0) self->cccd_handle_ = chr->val_handle + 1;
      ESP_LOGI(TAG, "CCCD handle: %d", self->cccd_handle_);
    }
  }
  return 0;
}

// ── GATT Write callback ───────────────────────────────────────────────

int OBDComponent::gatt_write_cb(uint16_t conn_handle,
                                 const struct ble_gatt_error* error,
                                 struct ble_gatt_attr* attr,
                                 void* arg) {
  auto* self = static_cast<OBDComponent*>(arg);
  if (error->status == 0) {
    if (attr->handle == self->cccd_handle_) {
      self->notify_enabled_ = true;
      ESP_LOGI(TAG, "CCCD write OK — notifications enabled");
    }
  } else {
    ESP_LOGW(TAG, "GATT write error: %d", error->status);
  }
  return 0;
}

// ── ELM327 Commands ───────────────────────────────────────────────────

bool OBDComponent::send_at_command(const std::string& cmd) {
  if (conn_handle_ == BLE_HS_CONN_HANDLE_NONE || chr_write_handle_ == 0) return false;
  std::string payload = cmd + "\r";
  int rc = ble_gattc_write_no_rsp_flat(conn_handle_, chr_write_handle_, payload.data(), payload.size());
  ESP_LOGD(TAG, ">> %s (rc=%d)", cmd.c_str(), rc);
  return rc == 0;
}

bool OBDComponent::send_obd_query(const std::string& pid_hex) {
  if (conn_handle_ == BLE_HS_CONN_HANDLE_NONE || chr_write_handle_ == 0) return false;
  std::string payload = "22 " + pid_hex + "\r";
  int rc = ble_gattc_write_no_rsp_flat(conn_handle_, chr_write_handle_, payload.data(), payload.size());
  ESP_LOGD(TAG, ">> 22 %s (rc=%d)", pid_hex.c_str(), rc);
  return rc == 0;
}

void OBDComponent::switch_ecu_header(const std::string& ecu) {
  if (ecu == "bms") {
    send_at_command("AT SH 704"); delay(60);
    send_at_command("AT CRA 784"); delay(60);
    send_at_command("AT FCSH 704"); delay(60);
  } else if (ecu == "vcu") {
    send_at_command("AT SH 7E0"); delay(60);
    send_at_command("AT CRA 7E8"); delay(60);
    send_at_command("AT FCSH 7E0"); delay(60);
  }
}

// ── Notification processing ───────────────────────────────────────────

void OBDComponent::process_notify(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (data[i] == ELM_PROMPT) {
      std::string clean;
      for (char c : rx_buffer_) {
        if (c != ' ' && c != '\r' && c != '\n' && c != '\t' && c != '>') clean += c;
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
        }
        current_pid_index_++;
        state_ = (bms_done_ && current_pid_index_ >= pids_.size())
                   ? PollState::IDLE : (bms_done_ ? PollState::POLL_VCU : PollState::POLL_BMS);
      }
    } else {
      rx_buffer_ += (char)data[i];
    }
  }
}

// ── Formula parser (unchanged from previous) ──────────────────────────

float OBDComponent::parse_response(const std::string& response, const std::string& formula) {
  if (response.find("ERROR") != std::string::npos ||
      response.find("NO DATA") != std::string::npos ||
      response.find("SEARCHING") != std::string::npos ||
      response.find("UNABLE") != std::string::npos ||
      response.find("STOPPED") != std::string::npos) return NAN;

  std::string clean = response;
  if (clean.length() >= 3 && clean[0] == '7') clean = clean.substr(3);
  if (clean.length() < 6) return NAN;

  auto bytes = hex_to_bytes(clean);
  if (bytes.size() < 2) return NAN;
  if (bytes.size() >= 3 && bytes[1] == 0x7F) return NAN;

  std::string expr = formula;
  size_t pos;
  while ((pos = expr.find("[B")) != std::string::npos) {
    size_t colon = expr.find(':', pos), close = expr.find(']', colon);
    if (colon == std::string::npos || close == std::string::npos) break;
    int start = atoi(expr.c_str()+pos+2), end = atoi(expr.c_str()+colon+2);
    int64_t val = 0;
    for (int i = start; i <= end && i < (int)bytes.size(); i++) val = (val<<8)|bytes[i];
    expr.replace(pos, close-pos+1, std::to_string(val));
  }
  while ((pos = expr.find('B')) != std::string::npos) {
    int idx = 0; size_t i = pos+1;
    while (i < expr.length() && isdigit(expr[i])) idx = idx*10+(expr[i++]-'0');
    int bval = (idx < (int)bytes.size()) ? bytes[idx] : 0;
    expr.replace(pos, i-pos, std::to_string(bval));
  }
  if (expr.find('A') != std::string::npos) {
    int ds = 0;
    for (size_t i = 0; i+1 < bytes.size(); i++)
      if (bytes[i]==0x62){ds=i+1;if(ds+2<(int)bytes.size())ds+=2;break;}
    for (size_t i = 0; i < expr.length(); i++)
      if (expr[i]>='A'&&expr[i]<='D') {
        int bval = (ds+expr[i]-'A' < (int)bytes.size()) ? bytes[ds+expr[i]-'A'] : 0;
        auto s = std::to_string(bval); expr.replace(i,1,s); i+=s.length()-1;
      }
  }
  // Evaluate
  auto tok = [](const std::string& s, size_t& p)->float{
    while(p<s.length()&&isspace(s[p]))p++;
    char* e; float v=strtof(s.c_str()+p,&e); p=e-s.c_str(); return v;
  };
  size_t pp=0; float result=tok(expr,pp);
  while(pp<expr.length()){
    while(pp<expr.length()&&isspace(expr[pp]))pp++;
    if(pp>=expr.length())break;
    char op=expr[pp++]; float rhs=tok(expr,pp);
    if(op=='+')result+=rhs; else if(op=='-')result-=rhs; else if(op=='*')result*=rhs;
    else if(op=='/')result=(rhs!=0)?result/rhs:0;
    else if(op=='<'&&pp<expr.length()&&expr[pp]=='<'){pp++;result=(int)result<<(int)rhs;}
    else if(op=='>'&&pp<expr.length()&&expr[pp]=='>'){pp++;result=(int)result>>(int)rhs;}
  }
  return result;
}

void OBDComponent::publish_sensor(size_t idx, float value) {
  if (idx >= sensors_.size()) return;
  last_values_[idx] = value;
  sensors_[idx]->publish_state(value);
}

}  // namespace obd_ble
}  // namespace esphome
