idf_component_register(SRCS "display_service.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES common
                    PRIV_REQUIRES i2c_lcd driver data_processing)
