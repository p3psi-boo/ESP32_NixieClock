#ifndef APP_UTILS_H
#define APP_UTILS_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "nvs.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_strlcpy(char *dst, const char *src, size_t dst_size);

// Convert ESP-IDF WiFi auth enum to stable display text.
const char *app_wifi_authmode_to_string(wifi_auth_mode_t authmode);

// Copy raw 32-byte SSID field to a guaranteed NUL-terminated string.
void app_wifi_format_ssid(const uint8_t ssid_raw[32], char out[33]);

esp_err_t app_nvs_get_str_or_default(nvs_handle_t handle,
                                     const char *key,
                                     char *out,
                                     size_t out_size,
                                     const char *default_value);

esp_err_t app_http_recv_body(httpd_req_t *req, char *buf, size_t buf_size);

// Send cJSON object as HTTP JSON response and handle allocation failures.
void app_http_send_json(httpd_req_t *req, cJSON *json);

#ifdef __cplusplus
}
#endif

#endif
