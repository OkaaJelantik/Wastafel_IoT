idf_component_register(
    SRCS "i2c_lcd.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2c
)
