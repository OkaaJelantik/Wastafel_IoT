idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "include"
    REQUIRES
        driver
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp-mqtt
        esp_http_client
        hcsr04
        i2c_lcd
)
