#include "bus_t4_component.h"
#include "bus_t4.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "t4_packet.h"

#ifdef USE_ESP_IDF
#include <esp_rom_sys.h>
#endif

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4";

// BusT4 break signal baud rate: sending 0x00 at this speed produces a ~1ms low pulse.
// Reference: pruwait/Nice_BusT4 uses 9200 for a 19200 baud bus.
static constexpr uint32_t T4_BAUD_BREAK = 9200;

void BusT4Component::setup() {
  // Create RX queue first with null check
  rxQueue_ = xQueueCreate(32, sizeof(T4Packet));
  if (rxQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create RX queue!");
    return;
  }
  ESP_LOGD(TAG, "RX queue created successfully");

  // Create TX queue
  txQueue_ = xQueueCreate(32, sizeof(T4Packet));
  if (txQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create TX queue!");
    vQueueDelete(rxQueue_);
    rxQueue_ = nullptr;
    return;
  }
  ESP_LOGD(TAG, "TX queue created successfully");

  // Create event group for request handling
  requestEvent_ = xEventGroupCreate();
  if (requestEvent_ != nullptr) {
    xEventGroupSetBits(requestEvent_, EB_REQUEST_FREE);
    ESP_LOGD(TAG, "Event group created successfully");
  } else {
    ESP_LOGW(TAG, "Failed to create event group");
  }

  // Create RX task with error checking
  BaseType_t rx_result = xTaskCreate(rxTaskThunk, "bus_t4_rx", 8192, this, 10, &rxTask_);
  if (rx_result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create RX task!");
    return;
  }
  ESP_LOGD(TAG, "RX task created");

  // Create TX task with error checking
  BaseType_t tx_result = xTaskCreate(txTaskThunk, "bus_t4_tx", 8192, this, 10, &txTask_);
  if (tx_result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create TX task!");
    return;
  }
  ESP_LOGD(TAG, "TX task created");

#ifdef USE_ESP_IDF
  // Cache the UART port number for fast baud rate changes during break signal.
  // uart_set_baudrate() is a lightweight hardware register write, much faster
  // than ESPHome's load_settings() which reinstalls the entire UART driver.
  // Note: On ESP-IDF, the UARTComponent is always IDFUARTComponent, so the
  // static_cast is safe under the USE_ESP_IDF guard.
  if (parent_) {
    auto *idf_uart = static_cast<uart::IDFUARTComponent *>(parent_);
    uart_num_ = static_cast<uart_port_t>(idf_uart->get_hw_serial_number());
    ESP_LOGD(TAG, "Cached UART port %d for break signal generation", uart_num_);
  }
#endif

  ESP_LOGI(TAG, "BusT4 component setup complete");
}

void BusT4Component::loop() {
  // Safety check: ensure queue exists
  if (rxQueue_ == nullptr) {
    return;
  }

  // Process received packets and dispatch to registered devices
  T4Packet packet;
  while (xQueueReceive(rxQueue_, &packet, 0)) {
    // Ignore packets FROM ourselves (TX echo on half-duplex bus)
    if (packet.header.from == address_) {
      ESP_LOGV(TAG, "Ignoring TX echo");
      continue;
    }

    // Dispatch to all registered devices
    for (auto *device : devices_) {
      device->on_packet(packet);
    }
  }
}

void BusT4Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BusT4:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", address_.address, address_.endpoint);
  ESP_LOGCONFIG(TAG, "  Startup delay: %dms", startup_delay_);
  ESP_LOGCONFIG(TAG, "  RX Queue: %s", rxQueue_ != nullptr ? "OK" : "FAILED");
  ESP_LOGCONFIG(TAG, "  TX Queue: %s", txQueue_ != nullptr ? "OK" : "FAILED");
  ESP_LOGCONFIG(TAG, "  RX Task: %s", rxTask_ != nullptr ? "RUNNING" : "STOPPED");
  ESP_LOGCONFIG(TAG, "  TX Task: %s", txTask_ != nullptr ? "RUNNING" : "STOPPED");
}

void BusT4Component::rxTask() {
  // Wait for UART bus to stabilize before receiving
  if (startup_delay_ > 0) {
    ESP_LOGI(TAG, "RX: waiting %dms for UART bus to stabilize...", startup_delay_);
    vTaskDelay(pdMS_TO_TICKS(startup_delay_));
    ESP_LOGI(TAG, "RX: startup delay complete, receiving enabled");
  }

  // Flush any parasitic bytes accumulated during startup (boot noise, VP230 echo, etc.)
#ifdef USE_ESP_IDF
  if (uart_num_ < UART_NUM_MAX) {
    uart_flush_input(uart_num_);
    ESP_LOGD(TAG, "RX: UART input buffer flushed");
  }
#else
  {
    uint8_t discard;
    uint32_t flushed = 0;
    while (parent_ && parent_->available()) {
      parent_->read_byte(&discard);
      flushed++;
    }
    if (flushed > 0) {
      ESP_LOGD(TAG, "RX: flushed %u parasitic byte(s) from startup", flushed);
    }
  }
#endif

  T4Packet packet;
  uint8_t expected_size = 0;
  // Protocol format: [BREAK] SYNC SIZE DATA[N] SIZE
  // The size byte is sent twice (start and end), no separate checksum byte.
  // Internal checksums are within the DATA portion.
  enum { WAIT_SYNC = 0, SIZE, DATA, TRAILING_SIZE } rx_state = WAIT_SYNC;

  ESP_LOGI(TAG, "RX task started");

  for (;;) {
    // Safety check: ensure parent and queue exist
    if (!parent_ || rxQueue_ == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    uint8_t byte;
    if (parent_->available() && parent_->read_byte(&byte) == true) {
      switch (rx_state) {
        case WAIT_SYNC:
          // Wait for SYNC byte (0x55), ignore break bytes (0x00) and others
          if (byte == T4_SYNC) {
            rx_state = SIZE;
          }
          break;

        case SIZE:
          if (byte > 0 && byte <= 60) {
            expected_size = byte;
            packet.size = 0;
            rx_state = DATA;
          } else {
            // Invalid size, go back to waiting for sync
            rx_state = WAIT_SYNC;
          }
          break;

        case DATA:
          packet.data[packet.size++] = byte;
          if (packet.size == expected_size)
            rx_state = TRAILING_SIZE;
          break;

        case TRAILING_SIZE:
          // Verify trailing size matches
          if (byte == expected_size) {
            // Verify header checksum (CRC1): XOR of bytes 0-5 should equal byte 6
            uint8_t header_check = packet.checksum(0, 6);
            if (header_check != packet.data[6]) {
              ESP_LOGW(TAG, "Header checksum mismatch: expected 0x%02X, got 0x%02X: %s",
                       header_check, packet.data[6],
                       format_hex_pretty(packet.data, packet.size).c_str());
              rx_state = WAIT_SYNC;
              break;
            }

            // Verify payload checksum (CRC2): XOR of bytes 7 to size-2 should equal byte size-1
            if (packet.size >= 9) {  // Minimum: 7 header + 1 payload + 1 CRC2
              uint8_t payload_check = packet.data[7];
              for (size_t i = 8; i < packet.size - 1; i++) {
                payload_check ^= packet.data[i];
              }
              if (payload_check != packet.data[packet.size - 1]) {
                ESP_LOGW(TAG, "Payload checksum mismatch: expected 0x%02X, got 0x%02X: %s",
                         payload_check, packet.data[packet.size - 1],
                         format_hex_pretty(packet.data, packet.size).c_str());
                rx_state = WAIT_SYNC;
                break;
              }
            }

            // Both checksums valid - accept packet
            ESP_LOGD(TAG, "Received packet: %s (%d bytes)",
                     format_hex_pretty(packet.data, packet.size).c_str(), packet.size);
            
            // Send to queue with timeout instead of portMAX_DELAY
            if (rxQueue_ != nullptr) {
              if (!xQueueSend(rxQueue_, &packet, pdMS_TO_TICKS(100))) {
                ESP_LOGW(TAG, "RX queue full, dropping packet");
              }
            }
          } else {
            ESP_LOGW(TAG, "Trailing size mismatch: expected 0x%02X, got 0x%02X", expected_size, byte);
          }
          rx_state = WAIT_SYNC;
          break;
      }
    }
    vTaskDelay(2);
  }

  rxTask_ = nullptr;
  vTaskDelete(nullptr);
}

void BusT4Component::txTask() {
  // Wait for UART bus to stabilize before transmitting
  if (startup_delay_ > 0) {
    ESP_LOGI(TAG, "TX: waiting %dms for UART bus to stabilize...", startup_delay_);
    vTaskDelay(pdMS_TO_TICKS(startup_delay_));
    ESP_LOGI(TAG, "TX: startup delay complete, transmission enabled");
  }

  TickType_t last_tx_time = 0;
  const TickType_t TX_MIN_INTERVAL = pdMS_TO_TICKS(100);  // Minimum 100ms between transmissions

  ESP_LOGI(TAG, "TX task started");

  for (;;) {
    // Safety check: ensure parent and queue exist
    if (!parent_ || txQueue_ == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    T4Packet packet;

    // Wait for packet with timeout (allows checking for queue items periodically)
    if (xQueueReceive(txQueue_, &packet, pdMS_TO_TICKS(10))) {
      // Ensure minimum interval between transmissions
      TickType_t now = xTaskGetTickCount();
      TickType_t elapsed = now - last_tx_time;
      if (elapsed < TX_MIN_INTERVAL) {
        vTaskDelay(TX_MIN_INTERVAL - elapsed);
      }

      ESP_LOGD(TAG, "Sending packet: %s", format_hex_pretty(packet.data, packet.size).c_str());
      send_break();
      parent_->write_byte(T4_SYNC);
      parent_->write_byte(packet.size);
      parent_->write_array(packet.data, packet.size);
      parent_->write_byte(packet.size);
      parent_->flush();

      last_tx_time = xTaskGetTickCount();
    }
  }

  txTask_ = nullptr;
  vTaskDelete(nullptr);
}

void BusT4Component::write_raw(const uint8_t *data, size_t len) {
  // Send raw bytes directly to UART with break prefix
  // Used for debugging/testing with user-provided hex commands
  if (!parent_) {
    ESP_LOGW(TAG, "Parent UART device not available");
    return;
  }

  send_break();
  parent_->write_array(data, len);
  parent_->flush();
}

void BusT4Component::send_break() {
  // BusT4 protocol requires a break signal (~1ms low pulse) before each packet.
  // This is achieved by sending 0x00 at a lower baud rate (9200 instead of 19200),
  // which produces a longer low-level signal on the bus.
  // Reference: pruwait/Nice_BusT4 send_array_cmd() implementation.
  if (!parent_) return;

#ifdef USE_ESP_IDF
  // Fast path: use ESP-IDF uart_set_baudrate() which is a lightweight hardware
  // register write. This does NOT reinstall the UART driver.
  // Note on RX safety: BusT4 is a half-duplex bus, so no device should be
  // transmitting while we are. The brief baud rate change (~1ms) won't affect
  // the RX task because the RX state machine already discards non-SYNC bytes.
  if (uart_num_ < UART_NUM_MAX) {
    uint32_t work_baud = parent_->get_baud_rate();
    uart_set_baudrate(uart_num_, T4_BAUD_BREAK);
    parent_->write_byte(T4_BREAK);
    parent_->flush();
    // Small delay to ensure the break byte is fully transmitted before restoring
    // the baud rate, matching the reference implementation's delayMicroseconds(90).
    esp_rom_delay_us(100);
    uart_set_baudrate(uart_num_, work_baud);
  } else {
    // UART port not cached (setup issue) — fall back to simple break
    ESP_LOGW(TAG, "UART port not cached, using simple break (may be too short)");
    parent_->write_byte(T4_BREAK);
    parent_->flush();
  }
#else
  // Fallback for non-IDF platforms: send break byte at normal speed.
  // This produces a shorter break (~520µs) which may not work with all controllers.
  parent_->write_byte(T4_BREAK);
  parent_->flush();
#endif
}

} // namespace esphome::bus_t4
