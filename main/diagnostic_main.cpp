
#include <stdio.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TRIG_PIN GPIO_NUM_32
#define ECHO_PIN GPIO_NUM_33

static const char *TAG = "RAW_DIAG";

extern "C" void app_main(void) {
    gpio_reset_pin(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ECHO_PIN);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);

    ESP_LOGI(TAG, "Starting raw diagnostic on GPIO%d (Trig) / GPIO%d (Echo)", TRIG_PIN, ECHO_PIN);

    while (1) {
        // Trigger
        gpio_set_level(TRIG_PIN, 0);
        esp_rom_delay_us(2);
        gpio_set_level(TRIG_PIN, 1);
        esp_rom_delay_us(10);
        gpio_set_level(TRIG_PIN, 0);

        // Wait for Echo HIGH
        int64_t start = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN) == 0) {
            if ((esp_timer_get_time() - start) > 50000) break; // Timeout 50ms
        }

        int64_t echo_start = esp_timer_get_time();
        
        // Wait for Echo LOW
        while (gpio_get_level(ECHO_PIN) == 1) {
             if ((esp_timer_get_time() - echo_start) > 50000) break;
        }
        int64_t echo_end = esp_timer_get_time();

        int64_t duration = echo_end - echo_start;
        
        if (duration < 0 || duration > 50000) {
            ESP_LOGW(TAG, "Invalid/Timeout: %lld us", duration);
        } else {
            ESP_LOGI(TAG, "Duration: %lld us (Approx %.2f cm)", duration, (float)duration / 58.0f);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
