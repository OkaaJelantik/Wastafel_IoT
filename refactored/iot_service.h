/**
 * @file iot_service.h
 * @brief IoT Service — WiFi, MQTT, Blynk, and Offline Buffering
 *
 * Provides a single static façade so main.cpp and other components
 * only need one call to publish telemetry:
 *
 *   IoTService::publish(hand_cm, water_cm, pump_on, state);
 *
 * Internally the service:
 *   1. Buffers data when WiFi / MQTT is unavailable (ring buffer).
 *   2. Flushes the ring buffer when connectivity is restored.
 *   3. Mirrors data to Blynk via HTTP GET.
 *
 * Initialisation sequence (from app_main):
 *   IoTService::init();          // Start WiFi, register event handlers
 *   // — tasks are launched —
 *   // IoTService::run_task() is pinned to Core 1 by main.cpp
 */

#pragma once

#include "esp_err.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// ── Offline Ring-Buffer Size ──
static constexpr int IOT_OFFLINE_BUFFER_SIZE = 30;

// ── WiFi Event Bits ──
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1
#define WIFI_MAX_RETRY       10

// ── Forward declaration — SystemState is defined in main.cpp ──
// We use int here so iot_service has no dependency on main's enum.
// main.cpp casts SystemState to int before calling publish().
using SystemStateInt = int;

/* ─────────────────────────────────────────────────────────────────── */

/**
 * @brief POD entry stored in the offline ring-buffer.
 *        Mirrors TelemetryEntry from data_processing for IoT use.
 */
struct IoTTelemetryEntry {
    float   hand_cm;
    float   water_cm;
    bool    pump_on;
    int     state;
    int64_t timestamp_ms;
};

/* ════════════════════════════════════════════════════════════════════
 *  CLASS: IoTService (static singleton façade)
 * ════════════════════════════════════════════════════════════════════ */
class IoTService {
public:
    IoTService() = delete; // Pure static — no instances

    // ── Lifecycle ───────────────────────────────────────────────────

    /**
     * @brief Initialise NVS, netif, WiFi STA, and the offline buffer.
     *        Does NOT block; event group signals readiness asynchronously.
     */
    static esp_err_t init();

    /**
     * @brief FreeRTOS task body — runs telemetry loop on Core 1.
     *        Register with xTaskCreatePinnedToCore(IoTService::run_task, ...).
     */
    static void run_task(void *pvParams);

    // ── Publishing ───────────────────────────────────────────────────

    /**
     * @brief Publish one telemetry snapshot.
     *        - If MQTT is connected: publishes immediately (and flushes buffer).
     *        - Otherwise: stores in offline ring buffer for later.
     * @param state  Cast SystemState as int (avoid enum coupling)
     */
    static void publish(float hand_cm, float water_cm,
                        bool pump_on, SystemStateInt state);

    // ── Status Queries ──────────────────────────────────────────────
    static bool is_wifi_connected() { return s_wifi_connected; }
    static bool is_mqtt_connected() { return s_mqtt_connected; }

private:
    // WiFi
    static void   _wifi_event_handler(void *arg, esp_event_base_t base,
                                      int32_t id, void *data);
    static void   _wifi_init_sta();

    // MQTT
    static void   _mqtt_event_handler(void *args, esp_event_base_t base,
                                      int32_t id, void *data);
    static void   _mqtt_init();
    static void   _mqtt_publish_live(float hand_cm, float water_cm,
                                     bool pump_on, SystemStateInt state);
    static void   _mqtt_flush_offline_buffer();

    // Blynk
    static void   _blynk_push(float hand_cm, float water_cm,
                               bool pump_on, SystemStateInt state);

    // Offline ring buffer
    static void   _buffer_entry(float hand_cm, float water_cm,
                                 bool pump_on, SystemStateInt state);

    // ── Static State ─────────────────────────────────────────────────
    static EventGroupHandle_t        s_wifi_event_group;
    static esp_mqtt_client_handle_t  s_mqtt_client;
    static bool                      s_wifi_connected;
    static bool                      s_mqtt_connected;
    static int                       s_wifi_retry_count;

    static IoTTelemetryEntry s_offline_buffer[IOT_OFFLINE_BUFFER_SIZE];
    static int               s_offline_head;
    static int               s_offline_count;
};
