#include "app_utils.h"

#include <stdlib.h>
#include <string.h>

void app_strlcpy(char *dst, const char *src, size_t dst_size)
{
    if (dst == NULL || dst_size == 0) {
        return;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

const char *app_wifi_authmode_to_string(wifi_auth_mode_t authmode)
{
    switch (authmode) {
        case WIFI_AUTH_OPEN:
            return "Open";
        case WIFI_AUTH_WEP:
            return "WEP";
        case WIFI_AUTH_WPA_PSK:
            return "WPA";
        case WIFI_AUTH_WPA2_PSK:
            return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK:
            return "WPA/WPA2";
        case WIFI_AUTH_WPA3_PSK:
            return "WPA3";
        default:
            return "Other";
    }
}

void app_wifi_format_ssid(const uint8_t ssid_raw[32], char out[33])
{
    size_t len = 0;

    if (out == NULL) {
        return;
    }

    while (len < 32 && ssid_raw[len] != 0) {
        len++;
    }
    memcpy(out, ssid_raw, len);
    out[len] = '\0';
}

esp_err_t app_nvs_get_str_or_default(nvs_handle_t handle,
                                     const char *key,
                                     char *out,
                                     size_t out_size,
                                     const char *default_value)
{
    size_t required_size = out_size;
    esp_err_t ret;

    if (out == NULL || out_size == 0 || key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = nvs_get_str(handle, key, out, &required_size);
    if (ret != ESP_OK) {
        app_strlcpy(out, default_value, out_size);
    }

    return ret;
}

esp_err_t app_http_recv_body(httpd_req_t *req, char *buf, size_t buf_size)
{
    int ret;

    if (req == NULL || buf == NULL || buf_size < 2) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = httpd_req_recv(req, buf, buf_size - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        } else {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        }
        return ESP_FAIL;
    }

    buf[ret] = '\0';
    return ESP_OK;
}

void app_http_send_json(httpd_req_t *req, cJSON *json)
{
    char *response_string;

    if (req == NULL || json == NULL) {
        return;
    }

    response_string = cJSON_Print(json);
    if (response_string == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON encode failed");
        return;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, HTTPD_RESP_USE_STRLEN);
    free(response_string);
}
