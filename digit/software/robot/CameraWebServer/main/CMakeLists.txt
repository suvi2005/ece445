idf_component_register(
    SRCS "main.cpp" "../app_httpd.cpp"
    INCLUDE_DIRS "." ".."
    REQUIRES
        esp_event
        esp_http_server
        esp_netif
        esp_psram
        esp_timer
        esp_wifi
        esp_driver_ledc
        nvs_flash
)
