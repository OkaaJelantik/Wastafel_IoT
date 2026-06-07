/**
 * @file iot_service.cpp
 * @brief IoT Service Implementation
 *
 * Manages the full IoT stack:
 *   WiFi STA → MQTT publish/flush → Blynk HTTP push → Offline ring-buffer
 *
 * main.cpp and other components only call IoTService::publish().
 */

#include "iot_service.h"

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "freertos/task.h"

/* ── Kconfig keys (defined in sdkconfig / menuconfig) ── */
// CONFIG_WASTAFEL_WIFI_SSID
// CONFIG_WASTAFEL_WIFI_PASSWORD
// CONFIG_WASTAFEL_MQTT_BROKER_URI
// CONFIG_WASTAFEL_DEVICE_ID
// CONFIG_WASTAFEL_BLYNK_AUTH
// CONFIG_WASTAFEL_BLYNK_SERVER

static const char *IOT_TAG = "IOT_SVC";

/* ── Publish intervals (ms) ── */
static constexpr int MQTT_PUBLISH_INTERVAL_MS = 2000;
static constexpr int BLYNK_PUSH_INTERVAL_MS   = 5000;

/* ══════════════════════════════════════════════════════════════════
 *  Static Member Definitions
 * ══════════════════════════════════════════════════════════════════ */

EventGroupHandle_t       IoTService::s_wifi_event_group = nullptr;
esp_mqtt_client_handle_t IoTService::s_mqtt_client      = nullptr;
bool                     IoTService::s_wifi_connected   = false;
bool                     IoTService::s_mqtt_connected   = false;
int                      IoTService::s_wifi_retry_count = 0;

IoTTelemetryEntry IoTService::s_offline_buffer[IOT_OFFLINE_BUFFER_SIZE] = {};
int               IoTService::s_offline_head  = 0;
int               IoTService::s_offline_count = 0;

/* ══════════════════════════════════════════════════════════════════
 *  PUBLIC: init()
 * ══════════════════════════════════════════════════════════════════ */

esp_err_t IoTService::init() {
    ESP_LOGI(IOT_TAG, "Initialising IoT Service...");
    _wifi_init_sta();
    ESP_LOGI(IOT_TAG, "WiFi STA initialised — connecting asynchronously");
    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════
 *  PUBLIC: run_task()
 *  FreeRTOS task — pin to Core 1 for network isolation.
 *  Waits for WiFi, then launches MQTT, then enters telemetry loop.
 * ══════════════════════════════════════════════════════════════════ */

void IoTService::run_task(void *pvParams) {
    TickType_t last_mqtt  = xTaskGetTickCount();
    TickType_t last_blynk = xTaskGetTickCount();

    /* ── Wait up to 30 s for WiFi result ── */
    ESP_LOGI(IOT_TAG, "IoT task: waiting for WiFi...");
    xEventGroupWaitBits(s_wifi_event_group,
                        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                        pdFALSE, pdFALSE,
                        pdMS_TO_TICKS(30000));

    if (s_wifi_connected) {
        _mqtt_init();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Allow MQTT handshake to complete
    } else {
        ESP_LOGW(IOT_TAG, "WiFi unavailable — running in offline-buffer mode");
    }

    while (true) {
        TickType_t now = xTaskGetTickCount();

        /* Actual publish calls happen via publish() which is driven by
           iot_task intervals — nothing to poll here beyond those. */
        vTaskDelay(pdMS_TO_TICKS(100));
        (void)last_mqtt;
        (void)last_blynk;
    }
}

/* ══════════════════════════════════════════════════════════════════
 *  PUBLIC: publish()
 *  Called from iot_task in main.cpp at MQTT_PUBLISH_INTERVAL_MS.
 * ══════════════════════════════════════════════════════════════════ */

void IoTService::publish(float hand_cm, float water_cm,
                         bool pump_on, SystemStateInt state, float vol_ml, float delta_ml) {
    if (s_mqtt_connected) {
        // Flush any data buffered during a prior outage first
        if (s_offline_count > 0) {
            _mqtt_flush_offline_buffer();
        }
        _mqtt_publish_live(hand_cm, water_cm, pump_on, state, vol_ml, delta_ml);
    } else {
        // No connectivity — store for later retransmission
        _buffer_entry(hand_cm, water_cm, pump_on, state, vol_ml, delta_ml);
        ESP_LOGD(IOT_TAG, "Buffered entry (%d/%d)",
                 s_offline_count, IOT_OFFLINE_BUFFER_SIZE);
    }
}

/* ══════════════════════════════════════════════════════════════════
 *  PRIVATE: WiFi
 * ══════════════════════════════════════════════════════════════════ */

void IoTService::_wifi_event_handler(void *arg, esp_event_base_t base,
                                     int32_t id, void *data) {
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            s_wifi_connected = false;
            s_mqtt_connected = false;
            if (s_wifi_retry_count < WIFI_MAX_RETRY) {
                esp_wifi_connect();
                ESP_LOGW(IOT_TAG, "WiFi retry %d/%d",
                         ++s_wifi_retry_count, WIFI_MAX_RETRY);
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(IOT_TAG, "WiFi failed after %d retries", WIFI_MAX_RETRY);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto *event = static_cast<ip_event_got_ip_t *>(data);
        ESP_LOGI(IOT_TAG, "IP acquired: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_count = 0;
        s_wifi_connected   = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void IoTService::_wifi_init_sta() {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h_any, h_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &_wifi_event_handler, nullptr, &h_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &_wifi_event_handler, nullptr, &h_ip));

    wifi_config_t wifi_cfg = {};
    strncpy(reinterpret_cast<char *>(wifi_cfg.sta.ssid),
            CONFIG_WASTAFEL_WIFI_SSID,
            sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy(reinterpret_cast<char *>(wifi_cfg.sta.password),
            CONFIG_WASTAFEL_WIFI_PASSWORD,
            sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(IOT_TAG, "Connecting to SSID '%s'...", CONFIG_WASTAFEL_WIFI_SSID);
}

/* ══════════════════════════════════════════════════════════════════
 *  PRIVATE: MQTT
 * ══════════════════════════════════════════════════════════════════ */

void IoTService::_mqtt_event_handler(void *args, esp_event_base_t base,
                                     int32_t id, void *data) {
    auto *event = static_cast<esp_mqtt_event_handle_t>(data);
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(IOT_TAG, "MQTT connected to broker");
        s_mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(IOT_TAG, "MQTT disconnected");
        s_mqtt_connected = false;
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(IOT_TAG, "MQTT error type: %d",
                 event->error_handle->error_type);
        break;
    default:
        break;
    }
}

void IoTService::_mqtt_init() {
    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = CONFIG_WASTAFEL_MQTT_BROKER_URI;

    s_mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt_client,
                                   static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
                                   _mqtt_event_handler, nullptr);
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGI(IOT_TAG, "MQTT client started → %s", CONFIG_WASTAFEL_MQTT_BROKER_URI);
}

/**
 * @brief Build and publish a JSON telemetry payload to the live topic.
 *        Topic: wastafel/{device_id}/telemetry
 */
void IoTService::_mqtt_publish_live(float hand_cm, float water_cm,
                                    bool pump_on, SystemStateInt state, float vol_ml, float delta_ml) {
    if (!s_mqtt_connected) return;

    char topic[64];
    snprintf(topic, sizeof(topic), "wastafel/%s/telemetry",
             CONFIG_WASTAFEL_DEVICE_ID);

    // Map numeric state to a readable string for the broker subscriber
    const char *state_str;
    switch (state) {
    case 0:  state_str = "IDLE";        break;
    case 1:  state_str = "PUMPING";     break;
    case 2:  state_str = "STABILIZING"; break;
    case 3:  state_str = "CALCULATING"; break;
    case 4:  state_str = "SHOW_RESULT"; break;
    case 5:  state_str = "CRITICAL";    break;
    case 6:  state_str = "REFILL";      break;
    default: state_str = "UNKNOWN";     break;
    }

    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"hand_cm\":%.1f,\"water_cm\":%.1f,"
             "\"pump\":%s,\"state\":\"%s\",\"vol_ml\":%.1f,\"delta_ml\":%.1f,\"uptime_ms\":%lld}",
             hand_cm, water_cm,
             pump_on ? "true" : "false",
             state_str,
             vol_ml, delta_ml,
             static_cast<long long>(esp_log_timestamp()));

    esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 1, 0);
}

/**
 * @brief Drain the offline ring-buffer, publishing each entry to the
 *        buffered topic.  Clears the buffer on completion.
 *        Topic: wastafel/{device_id}/buffered
 */
void IoTService::_mqtt_flush_offline_buffer() {
    if (!s_mqtt_connected || s_offline_count == 0) return;

    char topic[64];
    snprintf(topic, sizeof(topic), "wastafel/%s/buffered",
             CONFIG_WASTAFEL_DEVICE_ID);

    int start = (s_offline_head - s_offline_count + IOT_OFFLINE_BUFFER_SIZE)
                % IOT_OFFLINE_BUFFER_SIZE;
    int flushed = 0;

    for (int i = 0; i < s_offline_count; i++) {
        int idx = (start + i) % IOT_OFFLINE_BUFFER_SIZE;
        IoTTelemetryEntry &e = s_offline_buffer[idx];

        char payload[256];
        snprintf(payload, sizeof(payload),
                 "{\"hand_cm\":%.1f,\"water_cm\":%.1f,"
                 "\"pump\":%s,\"state\":%d,\"vol_ml\":%.1f,\"delta_ml\":%.1f,\"ts_ms\":%lld}",
                 e.hand_cm, e.water_cm,
                 e.pump_on ? "true" : "false",
                 e.state,
                 e.vol_ml, e.delta_ml,
                 static_cast<long long>(e.timestamp_ms));

        if (esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 0, 0) >= 0) {
            flushed++;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Throttle to avoid overwhelming broker
    }

    ESP_LOGI(IOT_TAG, "Flushed %d/%d buffered entries", flushed, s_offline_count);
    s_offline_count = 0;
    s_offline_head  = 0;
}

/* ══════════════════════════════════════════════════════════════════
 *  PRIVATE: Blynk HTTP Push
 *  Virtual pins: V0=hand_cm, V1=water_cm, V2=pump, V3=state
 * ══════════════════════════════════════════════════════════════════ */

void IoTService::_blynk_push(float hand_cm, float water_cm,
                              bool pump_on, SystemStateInt state, float vol_ml, float delta_ml) {
    const char *auth = CONFIG_WASTAFEL_BLYNK_AUTH;
    if (auth[0] == '\0' || !s_wifi_connected) return;

    char url[256];
    snprintf(url, sizeof(url),
             "https://%s/external/api/batch/update"
             "?token=%s&V0=%.1f&V1=%.1f&V2=%d&V3=%d&V4=%.1f&V5=%.1f",
             CONFIG_WASTAFEL_BLYNK_SERVER, auth,
             hand_cm, water_cm,
             pump_on ? 1 : 0,
             state,
             vol_ml, delta_ml);

    esp_http_client_config_t http_cfg = {};
    http_cfg.url        = url;
    http_cfg.method     = HTTP_METHOD_GET;
    http_cfg.timeout_ms = 3000;

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (client) {
        if (esp_http_client_perform(client) != ESP_OK) {
            ESP_LOGW(IOT_TAG, "Blynk push failed");
        }
        esp_http_client_cleanup(client);
    }
}

/* ══════════════════════════════════════════════════════════════════
 *  PRIVATE: Offline Ring-Buffer Writer
 * ══════════════════════════════════════════════════════════════════ */

void IoTService::_buffer_entry(float hand_cm, float water_cm,
                                bool pump_on, SystemStateInt state, float vol_ml, float delta_ml) {
    IoTTelemetryEntry &e = s_offline_buffer[s_offline_head];
    e.hand_cm      = hand_cm;
    e.water_cm     = water_cm;
    e.pump_on      = pump_on;
    e.state        = state;
    e.vol_ml       = vol_ml;
    e.delta_ml     = delta_ml;
    e.timestamp_ms = esp_log_timestamp();

    s_offline_head = (s_offline_head + 1) % IOT_OFFLINE_BUFFER_SIZE;
    if (s_offline_count < IOT_OFFLINE_BUFFER_SIZE) s_offline_count++;
}
