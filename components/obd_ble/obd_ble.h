// ESPHome external component: OBD-II BLE client — C++ header
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include <NimBLEDevice.h>
#include <NimBLEClient.h>
#include <NimBLERemoteService.h>
#include <NimBLERemoteCharacteristic.h>
#include <NimBLEScan.h>
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
  const char* can_header;  // "704" (BMS), "7E0" (VCU), or nullptr
};

// ── Polling state machine ─────────────────────────────────────────────
enum class PollState {
  IDLE,
  SCANNING,
  CONNECTING,
  DISCOVERING,
  INIT_ELM,
  INIT_WAIT,
  POLL_BMS,
  POLL_VCU,
  WAIT_RESPONSE,
};

// ── Main component class ──────────────────────────────────────────────
class OBDComponent : public PollingComponent {
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

 protected:
  void start_scan();
  bool connect_ble();
  void discover_services();
  bool send_at_command(const std::string& cmd);
  bool send_obd_query(const std::string& pid_hex);
  float parse_response(const std::string& response, const std::string& formula);
  void switch_ecu_header(const std::string& ecu);
  void publish_sensor(size_t idx, float value);
  void on_notify(uint8_t* data, size_t length);

  // BLE objects
  std::string mac_address_;
  std::string profile_;
  NimBLEClient* client_{nullptr};
  NimBLERemoteService* svc_{nullptr};
  NimBLERemoteCharacteristic* char_write_{nullptr};
  NimBLERemoteCharacteristic* char_notify_{nullptr};

  // Protocol state
  PollState state_{PollState::IDLE};
  uint32_t state_start_ms_{0};
  std::string rx_buffer_;
  std::string current_command_;
  size_t current_pid_index_{0};
  int init_cmd_index_{0};
  uint32_t poll_cycle_{0};

  // PID data (sensors created in Python, formulas in C++)
  std::vector<OBDPidDef> pids_;
  std::vector<sensor::Sensor*> sensors_;
  std::vector<float> last_values_;
  std::string current_ecu_;
  bool bms_done_{false};
};

}  // namespace obd_ble
}  // namespace esphome
