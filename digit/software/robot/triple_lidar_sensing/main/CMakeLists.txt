idf_component_register(
    SRCS "triple_lidar_sensing.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_i2c
)
