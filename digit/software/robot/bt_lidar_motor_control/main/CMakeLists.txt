idf_component_register(
    SRCS "bt_lidar_motor_control.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc bt nvs_flash
)
