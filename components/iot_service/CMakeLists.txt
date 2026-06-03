idf_component_register(SRCS "iot_service.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_wifi nvs_flash esp_event mqtt esp_http_client)
