#include "bus_t4.h"

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4";

void BusT4Device::send_cmd(T4Command cmd) {
  if (!parent_) {
    ESP_LOGW(TAG, "send_cmd: parent_ is NULL, cannot send command");
    return;
  }

  // DEP packet structure: [device] [command] [cmd_value] [offset]
  uint8_t message[4] = { OVIEW, RUN, cmd, 0x64 };
  T4Packet packet(target_address_, parent_->get_address(), DEP, message, sizeof(message));
  
  if (!write(&packet, pdMS_TO_TICKS(100))) {
    ESP_LOGW(TAG, "send_cmd: Failed to send command 0x%02X to queue", cmd);
  }
}

void BusT4Device::send_info_request(T4Target target, T4InfoCommand command) {
  if (!parent_) {
    ESP_LOGW(TAG, "send_info_request: parent_ is NULL");
    return;
  }

  // DMP GET packet structure: [target] [command] [request_type] [offset] [length]
  uint8_t message[5] = { target, command, REQ_GET, 0x00, 0x00 };
  T4Packet packet(target_address_, parent_->get_address(), DMP, message, sizeof(message));
  
  if (!write(&packet, pdMS_TO_TICKS(100))) {
    ESP_LOGW(TAG, "send_info_request: Failed to send info request to queue");
  }
}

void BusT4Device::send_config_set(T4InfoCommand param, uint8_t value) {
  if (!parent_) {
    ESP_LOGW(TAG, "send_config_set: parent_ is NULL");
    return;
  }

  // DMP SET packet structure: [target] [param] [request_type] [offset] [length] [value]
  uint8_t message[6] = { FOR_CU, param, REQ_SET, 0x00, 0x01, value };
  T4Packet packet(target_address_, parent_->get_address(), DMP, message, sizeof(message));
  
  if (!write(&packet, pdMS_TO_TICKS(100))) {
    ESP_LOGW(TAG, "send_config_set: Failed to send config set to queue");
  }
}

} // namespace esphome::bus_t4
