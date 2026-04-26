idf_component_register(
    SRCS "robot_firmware.c" "camera_control.cpp" "app_httpd.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ledc
        bt
        nvs_flash
        esp_event
        esp_http_server
        esp_netif
        esp_psram
        esp_timer
        esp_wifi
)
