#include <stdbool.h>
#include <stdio.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FLEX_SENSOR_ADC_UNIT      ADC_UNIT_1
#define FLEX_SENSOR_ADC_ATTEN     ADC_ATTEN_DB_12
#define FLEX_SENSOR_ADC_BITWIDTH  ADC_BITWIDTH_12
#define FLEX_SENSOR_SAMPLE_MS     200
#define FLEX_SENSOR_AVG_SAMPLES   8

typedef struct {
    const char *name;
    int gpio_num;
    adc_channel_t channel;
    adc_cali_handle_t cali_handle;
    bool cali_enabled;
} flex_sensor_t;

static const char *TAG = "flex_sensor";

static flex_sensor_t flex_sensors[] = {
    {
        .name = "FLEX2_ADC",
        .gpio_num = 2,
        .channel = ADC_CHANNEL_1,
        .cali_handle = NULL,
        .cali_enabled = false,
    },
    {
        .name = "FLEX3_ADC",
        .gpio_num = 3,
        .channel = ADC_CHANNEL_2,
        .cali_handle = NULL,
        .cali_enabled = false,
    },
    {
        .name = "FLEX4_ADC",
        .gpio_num = 4,
        .channel = ADC_CHANNEL_3,
        .cali_handle = NULL,
        .cali_enabled = false,
    },
};

#define FLEX_SENSOR_COUNT (sizeof(flex_sensors) / sizeof(flex_sensors[0]))

static bool init_adc_calibration(adc_unit_t unit,
                                 adc_channel_t channel,
                                 adc_atten_t atten,
                                 adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t err = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = FLEX_SENSOR_ADC_BITWIDTH,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    calibrated = (err == ESP_OK);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = FLEX_SENSOR_ADC_BITWIDTH,
    };
    err = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    calibrated = (err == ESP_OK);
#else
    (void)unit;
    (void)channel;
    (void)atten;
#endif

    if (calibrated) {
        *out_handle = handle;
        ESP_LOGI(TAG, "ADC calibration enabled for channel %d", channel);
    } else {
        *out_handle = NULL;
        if (err == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG, "ADC calibration not supported for channel %d", channel);
        } else {
            ESP_LOGW(TAG, "ADC calibration unavailable for channel %d (%s)", channel, esp_err_to_name(err));
        }
    }

    return calibrated;
}

static esp_err_t read_sensor_raw_average(adc_oneshot_unit_handle_t adc_handle,
                                         adc_channel_t channel,
                                         int *raw_avg)
{
    int raw_sum = 0;

    for (int i = 0; i < FLEX_SENSOR_AVG_SAMPLES; ++i) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(adc_handle, channel, &raw);
        if (err != ESP_OK) {
            return err;
        }
        raw_sum += raw;
    }

    *raw_avg = raw_sum / FLEX_SENSOR_AVG_SAMPLES;
    return ESP_OK;
}

static esp_err_t read_and_log_sensor(adc_oneshot_unit_handle_t adc_handle, flex_sensor_t *sensor)
{
    int raw = 0;
    esp_err_t err = read_sensor_raw_average(adc_handle, sensor->channel, &raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s read failed: %s", sensor->name, esp_err_to_name(err));
        return err;
    }

    if (sensor->cali_enabled) {
        int voltage_mv = 0;
        err = adc_cali_raw_to_voltage(sensor->cali_handle, raw, &voltage_mv);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s calibration conversion failed: %s", sensor->name, esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "%s raw=%4d voltage=%4d mV", sensor->name, raw, voltage_mv);
    } else {
        ESP_LOGI(TAG, "%s raw=%4d", sensor->name, raw);
    }

    return ESP_OK;
}

void app_main(void)
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t adc_unit_config = {
        .unit_id = FLEX_SENSOR_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_unit_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = FLEX_SENSOR_ADC_ATTEN,
        .bitwidth = FLEX_SENSOR_ADC_BITWIDTH,
    };

    for (size_t i = 0; i < FLEX_SENSOR_COUNT; ++i) {
        flex_sensor_t *sensor = &flex_sensors[i];

        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, sensor->channel, &channel_config));
        sensor->cali_enabled = init_adc_calibration(
            FLEX_SENSOR_ADC_UNIT,
            sensor->channel,
            FLEX_SENSOR_ADC_ATTEN,
            &sensor->cali_handle
        );

        ESP_LOGI(
            TAG,
            "%s configured on GPIO%d (ADC1 channel %d)%s",
            sensor->name,
            sensor->gpio_num,
            sensor->channel,
            sensor->cali_enabled ? ", calibration enabled" : ", raw mode only"
        );
    }

    while (true) {
        for (size_t i = 0; i < FLEX_SENSOR_COUNT; ++i) {
            flex_sensor_t *sensor = &flex_sensors[i];
            (void)read_and_log_sensor(adc_handle, sensor);
        }

        vTaskDelay(pdMS_TO_TICKS(FLEX_SENSOR_SAMPLE_MS));
    }
}
