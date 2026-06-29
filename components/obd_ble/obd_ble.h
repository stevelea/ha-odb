#pragma once
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include <string>
#include <vector>
#include <cstdint>

namespace esphome {
namespace obd_ble {

struct OBDPidDef {
  const char* name;
  const char* pid_hex;
  const char* formula;
  bool high_priority;
  const char* can_header;  // "704"=BMS, "7E0"=VCU, nullptr=none
};

enum class PollState : uint8_t {
  IDLE = 0,
  INIT_ELM,
  INIT_WAIT,
  POLL_BMS,
  POLL_VCU,
  WAIT_RESPONSE,
};

class OBDComponent : public ble_client::BLEClientNode, public PollingComponent {
 public:
  OBDComponent() : PollingComponent(30000) {}  // default 30s
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;

  void set_sensors(const std::vector<sensor::Sensor*>& sensors) { sensors_ = sensors; }
  void set_mac_address(const std::string& mac) { mac_address_ = mac; }
  void set_profile(const std::string& profile) { profile_ = profile; }

  // BLEClientNode
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t* param) override;

 protected:
  void on_services_discovered();
  void on_write_complete();
  void process_response(const std::string& response);
  float parse_response(const std::string& response, const std::string& formula);
  void send_at_command(const std::string& cmd);
  void send_obd_query(const std::string& pid_hex);
  void switch_ecu_header(const std::string& ecu);
  void publish_sensor(size_t idx, float value);

  std::string mac_address_;
  std::string profile_;
  std::vector<OBDPidDef> pids_;
  std::vector<sensor::Sensor*> sensors_;
  std::vector<float> last_values_;

  PollState state_{PollState::IDLE};
  uint32_t state_start_ms_{0};
  uint32_t poll_cycle_{0};
  size_t current_pid_index_{0};
  bool bms_done_{false};
  bool init_done_{false};
  std::string current_ecu_;

  uint16_t svc_start_handle_{0};
  uint16_t svc_end_handle_{0};
  uint16_t chr_write_handle_{0};
  uint16_t chr_notify_handle_{0};
  uint16_t cccd_handle_{0};

  int init_cmd_index_{0};
  std::string current_command_;
  std::string rx_buffer_;
};

}  // namespace obd_ble
}  // namespace esphome
