#include <string.h>

#include "esp_camera.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

extern "C" void startCameraServer(void);
extern "C" void stopCameraServer(void);
extern "C" void setCameraFeedEnabled(bool enabled);
extern "C" bool isCameraFeedEnabled(void);
extern "C" int SCCB_Deinit(void);
extern "C" void setupLedFlash(int pin);

static const char *TAG = "robot_camera";

static const char *k_ap_ssid = "RobotCamera";
static const char *k_ap_password = "esp32camera";
static const int k_ap_channel = 1;
static const int k_max_connections = 4;

static bool s_wifi_started = false;
static bool s_server_started = false;
static bool s_camera_on = false;

static esp_err_t robot_camera_init_wifi_softap(void)
{
    if (s_wifi_started) {
        return ESP_OK;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");

    wifi_config_t wifi_config = {};
    strncpy(reinterpret_cast<char *>(wifi_config.ap.ssid),
            k_ap_ssid,
            sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(k_ap_ssid);
    strncpy(reinterpret_cast<char *>(wifi_config.ap.password),
            k_ap_password,
            sizeof(wifi_config.ap.password));
    wifi_config.ap.channel = k_ap_channel;
    wifi_config.ap.max_connection = k_max_connections;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(k_ap_password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG,
                        "esp_wifi_set_mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG,
                        "esp_wifi_set_config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");

    s_wifi_started = true;
    ESP_LOGI(TAG, "Camera SoftAP started: SSID=%s password=%s",
             k_ap_ssid,
             k_ap_password);
    return ESP_OK;
}

static bool robot_camera_is_valid_frame(const camera_fb_t *fb)
{
    if (fb == NULL || fb->buf == NULL || fb->len == 0) {
        return false;
    }

    if (fb->format == PIXFORMAT_JPEG) {
        return fb->len >= 2 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8;
    }

    return fb->width > 0 && fb->height > 0;
}

static esp_err_t robot_camera_verify_capture(void)
{
    for (int i = 0; i < 5; ++i) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb != NULL) {
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    for (int attempt = 1; attempt <= 15; ++attempt) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "Camera capture %d failed: null framebuffer", attempt);
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        const bool valid = robot_camera_is_valid_frame(fb);
        ESP_LOGI(TAG, "Camera capture %d: len=%u fmt=%d size=%ux%u valid=%d",
                 attempt,
                 static_cast<unsigned>(fb->len),
                 static_cast<int>(fb->format),
                 fb->width,
                 fb->height,
                 valid);
        esp_camera_fb_return(fb);

        if (valid) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }

    return ESP_FAIL;
}

static esp_err_t robot_camera_init_sensor(void)
{
    ESP_LOGI(TAG,
             "Free DMA RAM before camera init: %u bytes largest=%u",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));

    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_YUV422;
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    ESP_RETURN_ON_ERROR(esp_camera_init(&config), TAG,
                        "esp_camera_init failed");

    vTaskDelay(pdMS_TO_TICKS(1000));

    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL) {
        esp_camera_deinit();
        return ESP_FAIL;
    }

    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
    }

    s->set_vflip(s, 1);

#if defined(LED_GPIO_NUM)
    setupLedFlash(LED_GPIO_NUM);
#endif

    esp_err_t err = robot_camera_verify_capture();
    if (err != ESP_OK) {
        esp_camera_deinit();
        return err;
    }

    err = (esp_err_t)SCCB_Deinit();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SCCB release failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Released camera SCCB/I2C bus after sensor init");
    }

    s_camera_on = true;
    ESP_LOGI(TAG, "Camera sensor on");
    return ESP_OK;
}

extern "C" esp_err_t robot_camera_set_enabled(bool enabled)
{
    if (!enabled) {
        setCameraFeedEnabled(false);
        ESP_LOGI(TAG, "Camera feed off; server remains available");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(robot_camera_init_wifi_softap(), TAG,
                        "camera WiFi init failed");

    if (!s_camera_on) {
        esp_err_t err = robot_camera_init_sensor();
        if (err != ESP_OK) {
            return err;
        }
    }

    if (!s_server_started) {
        startCameraServer();
        s_server_started = true;
        ESP_LOGI(TAG, "Camera server ready at http://192.168.4.1");
    }

    setCameraFeedEnabled(true);
    return ESP_OK;
}

extern "C" esp_err_t robot_camera_start_on_boot(void)
{
    return robot_camera_set_enabled(true);
}

extern "C" esp_err_t robot_camera_toggle_recording(void)
{
    return robot_camera_set_enabled(!isCameraFeedEnabled());
}
