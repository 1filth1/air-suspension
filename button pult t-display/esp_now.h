#pragma once

// Compatibility adapter for libraries written for ESP32 Arduino Core 2.x.
// Core 3.x passes esp_now_recv_info_t instead of a separate sender MAC.
#include_next <esp_now.h>

#ifdef __cplusplus
using esp_now_legacy_recv_cb_t = void (*)(const uint8_t *mac_addr,
                                          const uint8_t *data,
                                          int data_len);

inline esp_now_legacy_recv_cb_t &schematikLegacyEspNowReceiveCallback() {
  static esp_now_legacy_recv_cb_t callback = nullptr;
  return callback;
}

inline void schematikLegacyEspNowReceiveAdapter(const esp_now_recv_info_t *info,
                                                const uint8_t *data,
                                                int data_len) {
  const esp_now_legacy_recv_cb_t callback = schematikLegacyEspNowReceiveCallback();
  if (callback != nullptr && info != nullptr) {
    callback(info->src_addr, data, data_len);
  }
}

inline esp_err_t esp_now_register_recv_cb(esp_now_legacy_recv_cb_t callback) {
  schematikLegacyEspNowReceiveCallback() = callback;
  return ::esp_now_register_recv_cb(schematikLegacyEspNowReceiveAdapter);
}
#endif
