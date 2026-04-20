#include <stdio.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

extern void startCameraServer(void);
extern void setupLedFlash(int pin);

static const char *TAG = "camera_main";

static const char *kApSsid = "CameraWebServer";
static const char *kApPassword = "esp32camera";
static const int kApChannel = 1;
static const int kMaxConnections = 4;

static const int kStartupCaptureAttempts = 15;
static const int kDrainFrameCount = 5;
static const int kCaptureRetryDelayMs = 300;
static const int kSensorStabilizeDelayMs = 1000;

static bool is_valid_frame(const camera_fb_t *fb) {
  if (fb == nullptr || fb->buf == nullptr || fb->len == 0)
    return false;
  if (fb->format == PIXFORMAT_JPEG) {
    return fb->len >= 2 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8;
  }
  return fb->width > 0 && fb->height > 0;
}

static esp_err_t verify_startup_capture(void) {
  ESP_LOGI(TAG, "Draining %d initial frames", kDrainFrameCount);
  for (int i = 0; i < kDrainFrameCount; ++i) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb != nullptr) {
      ESP_LOGI(TAG, "Drain frame %d: len=%u fmt=%d %ux%u", i,
               static_cast<unsigned>(fb->len), static_cast<int>(fb->format),
               fb->width, fb->height);
      esp_camera_fb_return(fb);
    } else {
      ESP_LOGW(TAG, "Drain frame %d: null", i);
    }
    vTaskDelay(pdMS_TO_TICKS(kCaptureRetryDelayMs));
  }

  for (int attempt = 1; attempt <= kStartupCaptureAttempts; ++attempt) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr) {
      ESP_LOGE(TAG, "Capture %d/%d: null framebuffer", attempt,
               kStartupCaptureAttempts);
      vTaskDelay(pdMS_TO_TICKS(kCaptureRetryDelayMs));
      continue;
    }

    const bool valid = is_valid_frame(fb);
    ESP_LOGI(TAG, "Capture %d/%d: len=%u fmt=%d size=%ux%u valid=%s", attempt,
             kStartupCaptureAttempts, static_cast<unsigned>(fb->len),
             static_cast<int>(fb->format), fb->width, fb->height,
             valid ? "YES" : "no");
    esp_camera_fb_return(fb);

    if (valid) {
      ESP_LOGI(TAG, "Camera verified OK on attempt %d", attempt);
      return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(kCaptureRetryDelayMs));
  }

  return ESP_FAIL;
}

static esp_err_t init_camera(void) {
  ESP_LOGI(
      TAG, "Free DMA RAM before camera init: %u bytes (largest block: %u)",
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

  // PSRAM DMA is disabled on this board. Use YUV422 instead of RGB565 —
  // the DMA node math aligns better with YUV422 at QQVGA and avoids the
  // FB-SIZE mismatch warning (30720 != 38400) seen with RGB565.
  // YUV422 at QQVGA = 160x120x2 = 38400 bytes but DMA buffer aligns to
  // node boundaries that match the YUV422 line stride cleanly.
  config.pixel_format = PIXFORMAT_YUV422;
  config.frame_size = FRAMESIZE_QQVGA; // 160x120
  config.jpeg_quality = 12;            // unused for YUV422
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", err);
    return err;
  }

  ESP_LOGI(
      TAG, "Free DMA RAM after camera init: %u bytes (largest block: %u)",
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));

  ESP_LOGI(TAG, "Waiting %d ms for sensor to stabilize",
           kSensorStabilizeDelayMs);
  vTaskDelay(pdMS_TO_TICKS(kSensorStabilizeDelayMs));

  sensor_t *s = esp_camera_sensor_get();
  if (s == nullptr) {
    ESP_LOGE(TAG, "esp_camera_sensor_get returned null");
    return ESP_FAIL;
  }

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  if (verify_startup_capture() != ESP_OK) {
    ESP_LOGE(TAG, "Camera startup capture verification failed");
    return ESP_FAIL;
  }

  return ESP_OK;
}

static esp_err_t init_wifi_softap(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {};
  strncpy(reinterpret_cast<char *>(wifi_config.ap.ssid), kApSsid,
          sizeof(wifi_config.ap.ssid));
  wifi_config.ap.ssid_len = strlen(kApSsid);
  strncpy(reinterpret_cast<char *>(wifi_config.ap.password), kApPassword,
          sizeof(wifi_config.ap.password));
  wifi_config.ap.channel = kApChannel;
  wifi_config.ap.max_connection = kMaxConnections;
  wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

  if (strlen(kApPassword) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "SoftAP started: SSID=%s password=%s", kApSsid, kApPassword);
  return ESP_OK;
}

extern "C" void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Init WiFi before camera so WiFi DMA/interrupt setup does not race
  // with camera DMA and cause EV-EOF-OVF overflow errors.
  ESP_ERROR_CHECK(init_wifi_softap());

  ESP_ERROR_CHECK(init_camera());

  startCameraServer();
  ESP_LOGI(TAG, "Camera Ready! Open http://192.168.4.1");
}