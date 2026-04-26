#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

// -------------------- User Config --------------------

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
#define I2C_FREQ_HZ 100000

#define I2C_MUX_ADDR 0x70
#define DRV2605_ADDR 0x5A

// Two haptic drivers behind the mux.
#define HAPTIC_CHANNEL_A 0
#define HAPTIC_CHANNEL_B 1

// Two haptic enable pins.
#define HAPTIC_EN_GPIO_A GPIO_NUM_38
#define HAPTIC_EN_GPIO_B GPIO_NUM_37

// ERM motor, not LRA.
#define USE_LRA 0

// Strong simultaneous buzz settings.
#define STRONG_BUZZ_STRENGTH 255
#define STRONG_BUZZ_DURATION_MS 700
#define PAUSE_BETWEEN_BUZZES_MS 700

// -------------------- DRV2605L Registers --------------------

#define DRV2605_REG_STATUS 0x00
#define DRV2605_REG_MODE 0x01
#define DRV2605_REG_RTPIN 0x02
#define DRV2605_REG_LIBRARY 0x03
#define DRV2605_REG_WAVESEQ1 0x04
#define DRV2605_REG_WAVESEQ2 0x05
#define DRV2605_REG_WAVESEQ3 0x06
#define DRV2605_REG_WAVESEQ4 0x07
#define DRV2605_REG_WAVESEQ5 0x08
#define DRV2605_REG_WAVESEQ6 0x09
#define DRV2605_REG_WAVESEQ7 0x0A
#define DRV2605_REG_WAVESEQ8 0x0B
#define DRV2605_REG_GO 0x0C
#define DRV2605_REG_RATEDV 0x16
#define DRV2605_REG_CLAMPV 0x17
#define DRV2605_REG_FEEDBACK 0x1A
#define DRV2605_REG_CONTROL1 0x1B
#define DRV2605_REG_CONTROL2 0x1C
#define DRV2605_REG_CONTROL3 0x1D

// -------------------- DRV2605L Values --------------------

#define DRV2605_MODE_INTERNAL_TRIGGER 0x00
#define DRV2605_MODE_RTP 0x05

#define DRV2605_LIBRARY_ERM_A 1
#define DRV2605_LIBRARY_ERM_B 2
#define DRV2605_LIBRARY_ERM_C 3
#define DRV2605_LIBRARY_ERM_D 4
#define DRV2605_LIBRARY_ERM_E 5
#define DRV2605_LIBRARY_LRA 6

#define DRV2605_WAVEFORM_END 0x00

static const char *TAG = "DRV2605_SYNC_ERM";

// -------------------- I2C Helpers --------------------

static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_GPIO,
      .scl_io_num = I2C_SCL_GPIO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_FREQ_HZ,
      .clk_flags = 0,
  };

  esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}

static esp_err_t i2c_write_byte_to_addr(uint8_t addr, uint8_t value) {
  return i2c_master_write_to_device(I2C_PORT, addr, &value, 1,
                                    pdMS_TO_TICKS(100));
}

static bool i2c_probe_addr(uint8_t addr) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));

  i2c_cmd_link_delete(cmd);

  return ret == ESP_OK;
}

static void i2c_scan(const char *label) {
  ESP_LOGI(TAG, "Scanning %s", label);

  int devices_found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2c_probe_addr(addr)) {
      ESP_LOGI(TAG, "Found I2C address: 0x%02X", addr);
      devices_found++;
    }
  }

  ESP_LOGI(TAG, "Scan complete. Devices found: %d", devices_found);
}

// -------------------- GPIO / EN --------------------

static esp_err_t haptic_enable_pin(gpio_num_t pin) {
  gpio_config_t io_conf = {
      .pin_bit_mask = 1ULL << pin,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure haptic EN GPIO%d: %s", pin,
             esp_err_to_name(ret));
    return ret;
  }

  gpio_set_level(pin, 1);
  ESP_LOGI(TAG, "Haptic EN set HIGH on GPIO%d", pin);

  return ESP_OK;
}

static esp_err_t haptic_enable_init(void) {
  esp_err_t ret;

  ret = haptic_enable_pin(HAPTIC_EN_GPIO_A);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = haptic_enable_pin(HAPTIC_EN_GPIO_B);
  if (ret != ESP_OK) {
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(300));

  return ESP_OK;
}

// -------------------- Mux Helpers --------------------

static esp_err_t mux_select_channel(uint8_t channel) {
  if (channel > 7) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t value = 1 << channel;

  ESP_LOGI(TAG, "Selecting mux channel %u", channel);

  esp_err_t ret = i2c_write_byte_to_addr(I2C_MUX_ADDR, value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to select mux channel %u: %s", channel,
             esp_err_to_name(ret));
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(20));

  ESP_LOGI(TAG, "Selected mux channel %u", channel);
  return ESP_OK;
}

static esp_err_t mux_select_both_haptic_channels(void) {
  uint8_t value = (1 << HAPTIC_CHANNEL_A) | (1 << HAPTIC_CHANNEL_B);

  ESP_LOGI(TAG, "Selecting both haptic mux channels: 0x%02X", value);

  esp_err_t ret = i2c_write_byte_to_addr(I2C_MUX_ADDR, value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to select both mux channels: %s",
             esp_err_to_name(ret));
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(20));

  ESP_LOGI(TAG, "Both haptic mux channels selected");
  return ESP_OK;
}

static esp_err_t mux_disable_all_channels(void) {
  esp_err_t ret = i2c_write_byte_to_addr(I2C_MUX_ADDR, 0x00);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Mux disabled all channels");
  } else {
    ESP_LOGE(TAG, "Failed to disable mux channels: %s", esp_err_to_name(ret));
  }

  vTaskDelay(pdMS_TO_TICKS(20));

  return ret;
}

// -------------------- DRV2605L Helpers --------------------

static esp_err_t drv2605_write_reg(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};

  return i2c_master_write_to_device(I2C_PORT, DRV2605_ADDR, data, sizeof(data),
                                    pdMS_TO_TICKS(100));
}

static esp_err_t drv2605_read_reg(uint8_t reg, uint8_t *value) {
  return i2c_master_write_read_device(I2C_PORT, DRV2605_ADDR, &reg, 1, value, 1,
                                      pdMS_TO_TICKS(100));
}

static esp_err_t drv2605_stop(void) {
  esp_err_t ret;

  ret = drv2605_write_reg(DRV2605_REG_RTPIN, 0x00);
  if (ret != ESP_OK) {
    return ret;
  }

  return drv2605_write_reg(DRV2605_REG_GO, 0x00);
}

static esp_err_t drv2605_start(void) {
  return drv2605_write_reg(DRV2605_REG_GO, 0x01);
}

static esp_err_t drv2605_set_waveform(uint8_t slot, uint8_t waveform) {
  if (slot > 7) {
    return ESP_ERR_INVALID_ARG;
  }

  return drv2605_write_reg(DRV2605_REG_WAVESEQ1 + slot, waveform);
}

static esp_err_t drv2605_init_current_channel(uint8_t channel) {
  esp_err_t ret;

  ret = mux_select_channel(channel);
  if (ret != ESP_OK) {
    return ret;
  }

  if (!i2c_probe_addr(DRV2605_ADDR)) {
    ESP_LOGE(TAG, "DRV2605L not found at 0x%02X on mux channel %u",
             DRV2605_ADDR, channel);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "DRV2605L found at 0x%02X on mux channel %u", DRV2605_ADDR,
           channel);

  uint8_t status = 0;
  ret = drv2605_read_reg(DRV2605_REG_STATUS, &status);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "DRV2605L STATUS on channel %u = 0x%02X", channel, status);
  }

  ret = drv2605_write_reg(DRV2605_REG_MODE, DRV2605_MODE_INTERNAL_TRIGGER);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set internal trigger mode: %s",
             esp_err_to_name(ret));
    return ret;
  }

  // Increase drive limits.
  // If the motor gets hot or sounds bad, reduce these.
  ret = drv2605_write_reg(DRV2605_REG_RATEDV, 0xFF);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set RATEDV: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = drv2605_write_reg(DRV2605_REG_CLAMPV, 0xFF);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set CLAMPV: %s", esp_err_to_name(ret));
    return ret;
  }

  // Configure as ERM: FEEDBACK register bit 7 = 0.
  uint8_t feedback = 0;
  ret = drv2605_read_reg(DRV2605_REG_FEEDBACK, &feedback);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read FEEDBACK register: %s", esp_err_to_name(ret));
    return ret;
  }

  feedback &= ~0x80;

  ret = drv2605_write_reg(DRV2605_REG_FEEDBACK, feedback);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set ERM mode: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = drv2605_write_reg(DRV2605_REG_LIBRARY, DRV2605_LIBRARY_ERM_A);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to select ERM library: %s", esp_err_to_name(ret));
    return ret;
  }

  for (uint8_t i = 0; i < 8; i++) {
    ret = drv2605_set_waveform(i, DRV2605_WAVEFORM_END);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to clear waveform slot %u: %s", i,
               esp_err_to_name(ret));
      return ret;
    }
  }

  ESP_LOGI(TAG, "DRV2605L ERM init complete on mux channel %u", channel);

  return ESP_OK;
}

// This writes one command while both mux channels are enabled.
// Both DRV2605L chips receive the exact same I2C write.
static esp_err_t drv2605_sync_rtp_buzz(uint8_t strength, uint32_t duration_ms) {
  esp_err_t ret;

  ret = mux_select_both_haptic_channels();
  if (ret != ESP_OK) {
    return ret;
  }

  ESP_LOGI(TAG, "Starting simultaneous RTP buzz: strength=%u, duration=%lu ms",
           strength, duration_ms);

  ret = drv2605_stop();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to stop both drivers: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = drv2605_write_reg(DRV2605_REG_MODE, DRV2605_MODE_RTP);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set RTP mode on both drivers: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ret = drv2605_write_reg(DRV2605_REG_RTPIN, strength);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set RTP strength on both drivers: %s",
             esp_err_to_name(ret));
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(duration_ms));

  ret = drv2605_write_reg(DRV2605_REG_RTPIN, 0x00);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to stop RTP on both drivers: %s",
             esp_err_to_name(ret));
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(50));

  ret = drv2605_write_reg(DRV2605_REG_MODE, DRV2605_MODE_INTERNAL_TRIGGER);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to return both drivers to internal trigger mode: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "Simultaneous RTP buzz complete");

  return ESP_OK;
}

static esp_err_t drv2605_sync_play_effect(uint8_t effect) {
  esp_err_t ret;

  ret = mux_select_both_haptic_channels();
  if (ret != ESP_OK) {
    return ret;
  }

  ESP_LOGI(TAG, "Playing simultaneous built-in effect %u", effect);

  ret = drv2605_stop();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to stop both drivers: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = drv2605_write_reg(DRV2605_REG_MODE, DRV2605_MODE_INTERNAL_TRIGGER);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set internal trigger mode on both drivers: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ret = drv2605_set_waveform(0, effect);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set waveform 0 on both drivers: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ret = drv2605_set_waveform(1, DRV2605_WAVEFORM_END);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set waveform END on both drivers: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ret = drv2605_start();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start both drivers: %s", esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}

// -------------------- Main --------------------

void app_main(void) {
  ESP_LOGI(TAG, "Starting simultaneous dual DRV2605L ERM buzz test");
  ESP_LOGI(TAG, "Channel A = %u, Channel B = %u", HAPTIC_CHANNEL_A,
           HAPTIC_CHANNEL_B);

  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_ERROR_CHECK(haptic_enable_init());
  ESP_ERROR_CHECK(i2c_master_init());

  i2c_scan("root bus before mux select");

  if (!i2c_probe_addr(I2C_MUX_ADDR)) {
    ESP_LOGE(TAG, "Mux not found at 0x%02X", I2C_MUX_ADDR);
    return;
  }

  ESP_LOGI(TAG, "Mux found at 0x%02X", I2C_MUX_ADDR);

  ESP_ERROR_CHECK(mux_disable_all_channels());

  esp_err_t init_a = drv2605_init_current_channel(HAPTIC_CHANNEL_A);
  esp_err_t init_b = drv2605_init_current_channel(HAPTIC_CHANNEL_B);

  if (init_a != ESP_OK || init_b != ESP_OK) {
    ESP_LOGE(TAG, "Both haptic drivers must initialize for simultaneous mode.");
    ESP_LOGE(TAG, "init_a=%s, init_b=%s", esp_err_to_name(init_a),
             esp_err_to_name(init_b));
    mux_disable_all_channels();
    return;
  }

  ESP_LOGI(TAG, "Both haptic drivers initialized");
  ESP_LOGI(TAG, "Starting simultaneous buzz loop");

  while (1) {
    // Both motors receive the exact same RTP signal at the same time.
    drv2605_sync_rtp_buzz(STRONG_BUZZ_STRENGTH, STRONG_BUZZ_DURATION_MS);
    vTaskDelay(pdMS_TO_TICKS(PAUSE_BETWEEN_BUZZES_MS));

    // Both motors receive the same built-in waveform command at the same time.
    drv2605_sync_play_effect(1);
    vTaskDelay(pdMS_TO_TICKS(PAUSE_BETWEEN_BUZZES_MS));

    drv2605_sync_play_effect(47);
    vTaskDelay(pdMS_TO_TICKS(PAUSE_BETWEEN_BUZZES_MS));
  }
}