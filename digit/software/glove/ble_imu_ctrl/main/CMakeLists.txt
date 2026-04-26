idf_component_register(
    SRCS "glove_frimware.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_driver_i2c esp_adc
)
