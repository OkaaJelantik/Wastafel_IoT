idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "include"
    REQUIRES
        driver
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        espressif__mqtt
        esp_http_client
        hcsr04
        i2c_lcd
    PRIV_REQUIRES
        esp_driver_touch_sens
)

# Disable Werror globally for this component to allow legacy drivers to build
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error")

