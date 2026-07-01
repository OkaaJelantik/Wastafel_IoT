idf_component_register(SRCS "hardware_manager.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES common
                    PRIV_REQUIRES hcsr04 driver)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error=cpp")
