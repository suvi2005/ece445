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
#endif

    if (calibrated) {
        *out_handle = handle;
    } else {
        *out_handle = NULL;
    }

    return calibrated;
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

    for (size_t i = 0; i < sizeof(flex_sensors) / sizeof(flex_sensors[0]); ++i) {
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
        for (size_t i = 0; i < sizeof(flex_sensors) / sizeof(flex_sensors[0]); ++i) {
            flex_sensor_t *sensor = &flex_sensors[i];
            int raw = 0;

            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, sensor->channel, &raw));

            if (sensor->cali_enabled) {
                int voltage_mv = 0;
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(sensor->cali_handle, raw, &voltage_mv));
                ESP_LOGI(TAG, "%s raw=%4d voltage=%4d mV", sensor->name, raw, voltage_mv);
            } else {
                ESP_LOGI(TAG, "%s raw=%4d", sensor->name, raw);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(FLEX_SENSOR_SAMPLE_MS));
    }
}
