#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <functional>
#include <vector>
#include "esphome/components/uart/uart.h"
#include "t4_packet.h"

#ifdef USE_ESP_IDF
#include <driver/uart.h>
#include "esphome/components/uart/uart_component_esp_idf.h"
#endif

namespace esphome::bus_t4 {

// Forward declaration
class BusT4Device;

class BusT4Component final : public Component, public uart::UARTDevice {
 public:
  BusT4Component() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;

  // Read from queue - WITH SAFETY CHECKS
  bool read(T4Packet *packet, TickType_t xTicksToWait) {
    if (!packet) {
      return false;
    }
    if (rxQueue_ == nullptr) {
      return false;
    }
    return xQueueReceive(rxQueue_, packet, xTicksToWait);
  }

  // Write to queue - WITH SAFETY CHECKS
  bool write(T4Packet *packet, TickType_t xTicksToWait) {
    if (!packet) {
      return false;
    }
    if (txQueue_ == nullptr) {
      return false;
    }
    return xQueueSend(txQueue_, packet, xTicksToWait);
  }

  // Send raw bytes directly to UART (for debugging/testing)
  void write_raw(const uint8_t *data, size_t len);

  void set_address(const uint16_t address) {
    address_.address = static_cast<uint8_t>(address >> 8);
    address_.endpoint = static_cast<uint8_t>(address & 0xFF);
  }

  void set_startup_delay(uint32_t delay_ms) { startup_delay_ = delay_ms; }

  T4Source get_address() const { return address_; }

  // Register a device to receive packet callbacks
  void register_device(BusT4Device *device) { 
    if (device) {
      devices_.push_back(device);
    }
  }

 private:
  void rxTask();
  void txTask();
  static void rxTaskThunk(void *self) { static_cast<BusT4Component *>(self)->rxTask(); }
  static void txTaskThunk(void *self) { static_cast<BusT4Component *>(self)->txTask(); }

  // Send a proper BusT4 break signal (~1ms low pulse) before each packet.
  // Temporarily lowers baud rate to produce a longer break than a simple 0x00 at 19200.
  void send_break();

  T4Source address_;

  TaskHandle_t rxTask_ = nullptr;
  TaskHandle_t txTask_ = nullptr;

  QueueHandle_t rxQueue_ = nullptr;
  QueueHandle_t txQueue_ = nullptr;

  EventGroupHandle_t requestEvent_ = nullptr;
  
  std::vector<BusT4Device *> devices_;

  // Configurable startup delay: wait before UART TX/RX tasks begin processing.
  // Allows the bus to stabilize after power-on. Default 5 seconds.
  uint32_t startup_delay_{5000};

#ifdef USE_ESP_IDF
  // Cached UART port number for direct baud rate changes (lightweight register write).
  // Initialized to UART_NUM_MAX (invalid) — must be set from parent during setup().
  uart_port_t uart_num_ = UART_NUM_MAX;
#endif
};

enum { EB_REQUEST_FREE = 1, EB_REQUEST_PENDING = 2, EB_REQUEST_COMPLETE = 4 };

} // namespace esphome::bus_t4
