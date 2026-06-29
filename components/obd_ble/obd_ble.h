// ESPHome external component: OBD-II BLE client
// Uses ESPHome's built-in ble_client for BLE — no NimBLE-Arduino needed
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/ble_client/ble_client.h"
#include <vector>
#include <string>

namespace esphome {
namespace obd_ble {

struct OBDPidDef {
  const char* name; const char* pid_hex; const char* formula;
  bool high_priority; const char* can_header;
};

enum class PollState {
  IDLE, INIT_ELM, INIT_WAIT, POLL_BMS, POLL_VCU, WAIT_RESPONSE,
};

class OBDComponent : public PollingComponent, public ble_client::BLEClientNode {
 public:
  OBDComponent() : PollingComponent(30000) {}

  void set_ble_client(ble_client::BLEClient* client) { ble_client_ = client; }
  void set_profile(const std::string& p) { profile_ = p; }
  void add_sensor(sensor::Sensor* s) { sensors_.push_back(s); last_values_.push_back(NAN); }

  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; }

  // BLEClientNode callbacks — called when ble_client connects / receives data
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param) override;
  void on_connect() override;
  void on_disconnect() override;

 protected:
  void start_init();
  bool send_at_command(const std::string& cmd);
  bool send_obd_query(const std::string& pid_hex);
  float parse_response(const std::string& r, const std::string& f);
  void switch_ecu_header(const std::string& ecu);
  void publish_sensor(size_t idx, float value);
  void process_notify(const uint8_t* data, size_t len);

  ble_client::BLEClient* ble_client_{nullptr};
  std::string profile_;
  int write_handle_{0};

  PollState state_{PollState::IDLE};
  uint32_t state_start_ms_{0};
  std::string rx_buffer_, current_command_, current_ecu_;
  size_t current_pid_index_{0};
  int init_cmd_index_{0};
  uint32_t poll_cycle_{0};
  bool bms_done_{false}, init_done_{false};

  std::vector<OBDPidDef> pids_;
  std::vector<sensor::Sensor*> sensors_;
  std::vector<float> last_values_;
};

}  // namespace obd_ble
}  // namespace esphome
