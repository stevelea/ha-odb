// ESPHome external component: OBD-II BLE client — uses ESP-IDF NimBLE directly
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"

#include <host/ble_hs.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <services/gatt/ble_svc_gatt.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>

#include <functional>
#include <vector>
#include <string>

namespace esphome {
namespace obd_ble {

// ── PID definition structure ──────────────────────────────────────────
struct OBDPidDef {
  const char* name;
  const char* pid_hex;
  const char* formula;
  bool high_priority;
  const char* can_header;
};

// ── Polling state machine ─────────────────────────────────────────────
enum class PollState {
  IDLE,
  CONNECTING,
  WAIT_CONNECT,
  DISCOVER_SVC,
  DISCOVER_CHR,
  SUBSCRIBE,
  INIT_ELM,
  INIT_WAIT,
  POLL_BMS,
  POLL_VCU,
  WAIT_RESPONSE,
};

// ── NimBLE UUID helpers ───────────────────────────────────────────────
struct BLEUUID128 {
  uint8_t u[16];
  static BLEUUID128 from_string(const char* s);
  ble_uuid_any_t to_nimble() const;
};

// ── Main component class ──────────────────────────────────────────────
class OBDComponent : public PollingComponent,
                     public esp32_ble_tracker::ESPBTDeviceListener {
 public:
  OBDComponent() : PollingComponent(30000) {}

  void set_mac_address(const std::string& mac) { mac_address_ = mac; }
  void set_profile(const std::string& profile) { profile_ = profile; }
  void add_sensor(sensor::Sensor* sens) {
    sensors_.push_back(sens);
    last_values_.push_back(NAN);
  }

  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ESPBTDeviceListener — called by esp32_ble_tracker for every discovered device
  bool parse_device(const esp32_ble_tracker::ESPBTDevice& device) override;

 protected:
  void start_connect();
  bool try_connect();
  void handle_gap_event(struct ble_gap_event* ev);
  void discover_services();
  void discover_characteristics();
  bool send_at_command(const std::string& cmd);
  bool send_obd_query(const std::string& pid_hex);
  float parse_response(const std::string& response, const std::string& formula);
  void switch_ecu_header(const std::string& ecu);
  void publish_sensor(size_t idx, float value);
  void process_notify(const uint8_t* data, size_t length);

  // MAC address
  std::string mac_address_;
  std::string profile_;
  uint8_t target_addr_[6]{0};

  // Connection state (ESP-IDF NimBLE)
  uint16_t conn_handle_{BLE_HS_CONN_HANDLE_NONE};
  uint16_t svc_start_handle_{0};
  uint16_t svc_end_handle_{0};
  uint16_t chr_write_handle_{0};
  uint16_t chr_notify_handle_{0};
  uint16_t cccd_handle_{0};
  bool connected_{false};
  bool notify_enabled_{false};

  // Protocol state
  PollState state_{PollState::IDLE};
  uint32_t state_start_ms_{0};
  std::string rx_buffer_;
  std::string current_command_;
  size_t current_pid_index_{0};
  int init_cmd_index_{0};
  uint32_t poll_cycle_{0};

  // PID data
  std::vector<OBDPidDef> pids_;
  std::vector<sensor::Sensor*> sensors_;
  std::vector<float> last_values_;
  std::string current_ecu_;
  bool bms_done_{false};

  // Static callback trampoline (NimBLE uses C callbacks, need static fn)
  static int gap_event_cb(struct ble_gap_event* ev, void* arg);
  static int gatt_disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error* error,
                               const struct ble_gatt_svc* svc, void* arg);
  static int gatt_disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error* error,
                               const struct ble_gatt_chr* chr, void* arg);
  static int gatt_write_cb(uint16_t conn_handle, const struct ble_gatt_error* error,
                            struct ble_gatt_attr* attr, void* arg);
};

// ── Global pointer for NimBLE callbacks (single instance) ─────────────
extern OBDComponent* g_obd_component;

}  // namespace obd_ble
}  // namespace esphome
