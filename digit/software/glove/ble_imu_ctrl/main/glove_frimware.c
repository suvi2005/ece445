#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#define I2C_PORT I2C_NUM_0
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQ_HZ 400000
#define I2C_TIMEOUT_MS 100

#define IMU_ADDR_LOW 0x6A
#define IMU_ADDR_HIGH 0x6B

#define BLE_DEVICE_NAME "ESP32S3_GLOVE"

#define COMMAND_UPDATE_MS 100
#define TILT_THRESHOLD_DEG 30.0f

#define REST_CALIBRATION_SAMPLES 30
#define REST_CALIBRATION_DELAY_MS 20

#define FLEX_SENSOR_ADC_UNIT      ADC_UNIT_1
#define FLEX_SENSOR_ADC_ATTEN     ADC_ATTEN_DB_12
#define FLEX_SENSOR_ADC_BITWIDTH  ADC_BITWIDTH_12

#define FLEX_SENSOR_COUNT 3
#define SPEED_MIN 1
#define SPEED_MAX 20

/*
 * Tune these for your actual sensors.
 * straight = raw ADC when finger is straight
 * bent     = raw ADC when finger is bent
 */
static const int FLEX_RAW_STRAIGHT[FLEX_SENSOR_COUNT] = {1800, 1400, 1400};
static const int FLEX_RAW_BENT[FLEX_SENSOR_COUNT]     = {2500, 2500, 2500};

/*
 * Movement threshold:
 * All 3 flex sensors must be above this threshold to allow movement.
 *
 * With 1400-2500 calibration:
 * 0.60 means raw needs to be about 2060+ to count as bent.
 */
#define FLEX_MOVE_BENT_THRESHOLD 0.60f

/*
 * Gesture thresholds:
 * Used only for LED / RECORD gesture patterns.
 */
#define FLEX_GESTURE_BENT_THRESHOLD     0.60f
#define FLEX_GESTURE_STRAIGHT_THRESHOLD 0.25f

/*
 * Real flick detection uses gyroscope angular velocity.
 *
 * Increase if normal hand movement triggers LED/RECORD.
 * Decrease if flicks are not detected.
 */
#define FLICK_GYRO_THRESHOLD_DPS 250.0f
#define FLICK_COOLDOWN_MS 400

static const char *TAG = "GLOVE_CTRL";

static const uint8_t REG_WHO_AM_I = 0x0F;
static const uint8_t REG_CTRL1_XL = 0x10;
static const uint8_t REG_CTRL2_G = 0x11;
static const uint8_t REG_CTRL3_C = 0x12;
static const uint8_t REG_OUT_TEMP_L = 0x20;
static const uint8_t REG_OUTX_L_G = 0x22;
static const uint8_t REG_OUTX_L_A = 0x28;

static const uint8_t WHO_AM_I_EXPECTED = 0x6C;

static const float ACCEL_MG_PER_LSB = 0.061f;
static const float GYRO_MDPS_PER_LSB = 8.75f;

static const int ACCEL_MAP[3] = {0, 1, 2};
static const int ACCEL_SIGN[3] = {-1, -1, -1};
static const int GYRO_MAP[3] = {0, 1, 2};
static const int GYRO_SIGN[3] = {-1, -1, -1};

typedef struct {
    const char *name;
    int gpio_num;
    adc_channel_t channel;
    int raw;
} flex_sensor_t;

typedef enum {
    FLEX_STATE_STRAIGHT = 0,
    FLEX_STATE_MID,
    FLEX_STATE_BENT,
} flex_state_t;

static flex_sensor_t flex_sensors[FLEX_SENSOR_COUNT] = {
    {
        .name = "FLEX2_ADC",
        .gpio_num = 2,
        .channel = ADC_CHANNEL_1,
        .raw = 0,
    },
    {
        .name = "FLEX3_ADC",
        .gpio_num = 3,
        .channel = ADC_CHANNEL_2,
        .raw = 0,
    },
    {
        .name = "FLEX4_ADC",
        .gpio_num = 4,
        .channel = ADC_CHANNEL_3,
        .raw = 0,
    },
};

static uint8_t s_imu_addr = 0;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_imu_dev = NULL;
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

static uint8_t s_own_addr_type;
static uint16_t s_command_val_handle;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_notify_enabled = false;
static char s_last_command[16] = "stop:0";

static float s_pitch_offset_deg = 0.0f;
static float s_roll_offset_deg = 0.0f;

void ble_store_config_init(void);

static const ble_uuid128_t s_service_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t s_char_uuid = BLE_UUID128_INIT(
    0xef, 0xcd, 0xab, 0xef, 0xcd, 0xab, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0xab, 0xef, 0xcd, 0xab);

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void ble_app_advertise(void);

static const struct ble_gatt_chr_def s_command_characteristics[] = {
    {
        .uuid = &s_char_uuid.u,
        .access_cb = gatt_access_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_command_val_handle,
    },
    {0},
};

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = s_command_characteristics,
    },
    {0},
};

static int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static float clamp_float(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static esp_err_t i2c_master_init(void) {
    const i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
        .flags.allow_pd = false,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg, &s_i2c_bus), TAG,
                        "i2c_new_master_bus failed");

    return ESP_OK;
}

static esp_err_t i2c_add_device(uint8_t addr, i2c_master_dev_handle_t *dev) {
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags.disable_ack_check = false,
    };

    return i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, dev);
}

static esp_err_t write_register(i2c_master_dev_handle_t dev,
                                uint8_t reg,
                                uint8_t value) {
    uint8_t payload[2] = {reg, value};
    return i2c_master_transmit(dev, payload, sizeof(payload), I2C_TIMEOUT_MS);
}

static esp_err_t read_registers(i2c_master_dev_handle_t dev,
                                uint8_t start_reg,
                                uint8_t *buffer,
                                size_t len) {
    return i2c_master_transmit_receive(dev,
                                       &start_reg,
                                       1,
                                       buffer,
                                       len,
                                       I2C_TIMEOUT_MS);
}

static int16_t combine_int16(uint8_t low_byte, uint8_t high_byte) {
    return (int16_t)((high_byte << 8) | low_byte);
}

static bool detect_imu(uint8_t addr) {
    i2c_master_dev_handle_t dev = NULL;
    uint8_t whoami = 0;

    if (i2c_add_device(addr, &dev) != ESP_OK) {
        return false;
    }

    if (read_registers(dev, REG_WHO_AM_I, &whoami, 1) != ESP_OK) {
        i2c_master_bus_rm_device(dev);
        return false;
    }

    ESP_LOGI(TAG, "Address 0x%02X WHO_AM_I = 0x%02X", addr, whoami);

    if (whoami != WHO_AM_I_EXPECTED) {
        i2c_master_bus_rm_device(dev);
        return false;
    }

    s_imu_addr = addr;
    s_imu_dev = dev;

    return true;
}

static esp_err_t init_imu(void) {
    ESP_RETURN_ON_ERROR(write_register(s_imu_dev, REG_CTRL3_C, 0x44), TAG,
                        "CTRL3_C init failed");

    ESP_RETURN_ON_ERROR(write_register(s_imu_dev, REG_CTRL1_XL, 0x40), TAG,
                        "CTRL1_XL init failed");

    ESP_RETURN_ON_ERROR(write_register(s_imu_dev, REG_CTRL2_G, 0x40), TAG,
                        "CTRL2_G init failed");

    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

static void remap_axes(float in_x,
                       float in_y,
                       float in_z,
                       const int map_idx[3],
                       const int sign_arr[3],
                       float *out_x,
                       float *out_y,
                       float *out_z) {
    const float v[3] = {in_x, in_y, in_z};

    *out_x = sign_arr[0] * v[map_idx[0]];
    *out_y = sign_arr[1] * v[map_idx[1]];
    *out_z = sign_arr[2] * v[map_idx[2]];
}

static esp_err_t read_imu(float *ax_g,
                          float *ay_g,
                          float *az_g,
                          float *gx_dps,
                          float *gy_dps,
                          float *gz_dps,
                          float *temp_c) {
    uint8_t gyro_buf[6] = {0};
    uint8_t accel_buf[6] = {0};
    uint8_t temp_buf[2] = {0};

    ESP_RETURN_ON_ERROR(read_registers(s_imu_dev,
                                       REG_OUTX_L_G,
                                       gyro_buf,
                                       sizeof(gyro_buf)),
                        TAG,
                        "gyro read failed");

    ESP_RETURN_ON_ERROR(read_registers(s_imu_dev,
                                       REG_OUTX_L_A,
                                       accel_buf,
                                       sizeof(accel_buf)),
                        TAG,
                        "accel read failed");

    ESP_RETURN_ON_ERROR(read_registers(s_imu_dev,
                                       REG_OUT_TEMP_L,
                                       temp_buf,
                                       sizeof(temp_buf)),
                        TAG,
                        "temp read failed");

    const int16_t gx_raw = combine_int16(gyro_buf[0], gyro_buf[1]);
    const int16_t gy_raw = combine_int16(gyro_buf[2], gyro_buf[3]);
    const int16_t gz_raw = combine_int16(gyro_buf[4], gyro_buf[5]);

    const int16_t ax_raw = combine_int16(accel_buf[0], accel_buf[1]);
    const int16_t ay_raw = combine_int16(accel_buf[2], accel_buf[3]);
    const int16_t az_raw = combine_int16(accel_buf[4], accel_buf[5]);

    const int16_t temp_raw = combine_int16(temp_buf[0], temp_buf[1]);

    const float ax0 = (ax_raw * ACCEL_MG_PER_LSB) / 1000.0f;
    const float ay0 = (ay_raw * ACCEL_MG_PER_LSB) / 1000.0f;
    const float az0 = (az_raw * ACCEL_MG_PER_LSB) / 1000.0f;

    const float gx0 = (gx_raw * GYRO_MDPS_PER_LSB) / 1000.0f;
    const float gy0 = (gy_raw * GYRO_MDPS_PER_LSB) / 1000.0f;
    const float gz0 = (gz_raw * GYRO_MDPS_PER_LSB) / 1000.0f;

    remap_axes(ax0, ay0, az0, ACCEL_MAP, ACCEL_SIGN, ax_g, ay_g, az_g);
    remap_axes(gx0, gy0, gz0, GYRO_MAP, GYRO_SIGN, gx_dps, gy_dps, gz_dps);

    *temp_c = 25.0f + ((float)temp_raw / 256.0f);

    return ESP_OK;
}

static float radians_to_degrees(float radians) {
    return radians * 180.0f / (float)M_PI;
}

static void compute_pitch_roll_deg(float ax,
                                   float ay,
                                   float az,
                                   float *pitch_deg,
                                   float *roll_deg) {
    *pitch_deg = radians_to_degrees(
        atan2f(ax, sqrtf((ay * ay) + (az * az))));

    *roll_deg = radians_to_degrees(
        atan2f(ay, sqrtf((ax * ax) + (az * az))));
}

static esp_err_t calibrate_rest_position(void) {
    float pitch_sum = 0.0f;
    float roll_sum = 0.0f;
    int valid_samples = 0;

    ESP_LOGI(TAG, "Hold glove in desired rest position for calibration");

    for (int i = 0; i < REST_CALIBRATION_SAMPLES; ++i) {
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;
        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;
        float temp_c = 0.0f;

        esp_err_t err = read_imu(&ax, &ay, &az, &gx, &gy, &gz, &temp_c);

        if (err == ESP_OK) {
            float pitch_deg = 0.0f;
            float roll_deg = 0.0f;

            compute_pitch_roll_deg(ax, ay, az, &pitch_deg, &roll_deg);

            pitch_sum += pitch_deg;
            roll_sum += roll_deg;
            valid_samples++;
        }

        vTaskDelay(pdMS_TO_TICKS(REST_CALIBRATION_DELAY_MS));
    }

    if (valid_samples == 0) {
        return ESP_FAIL;
    }

    s_pitch_offset_deg = pitch_sum / valid_samples;
    s_roll_offset_deg = roll_sum / valid_samples;

    ESP_LOGI(TAG,
             "Rest calibration done: pitch_offset=%.2f roll_offset=%.2f",
             s_pitch_offset_deg,
             s_roll_offset_deg);

    return ESP_OK;
}

static float axis_to_angle_deg(float tilt_axis, float neutral_axis) {
    return radians_to_degrees(atan2f(tilt_axis, fabsf(neutral_axis)));
}

static const char *get_tilt_command(float ax, float ay, float az) {
    const float neutral_axis = ax;
    const float fb_axis = -az;
    const float turn_axis = ay;

    const float fb_deg = axis_to_angle_deg(fb_axis, neutral_axis);
    const float turn_deg = axis_to_angle_deg(turn_axis, neutral_axis);

    if (fabsf(turn_deg) >= fabsf(fb_deg)) {
        if (turn_deg > TILT_THRESHOLD_DEG) {
            return "b";
        }

        if (turn_deg < -TILT_THRESHOLD_DEG) {
            return "f";
        }
    } else {
        if (fb_deg > TILT_THRESHOLD_DEG) {
            return "r";
        }

        if (fb_deg < -TILT_THRESHOLD_DEG) {
            return "l";
        }
    }

    return "stop";
}

static bool detect_flick(float gx, float gy, float gz) {
    const float gyro_magnitude = sqrtf((gx * gx) + (gy * gy) + (gz * gz));

    return gyro_magnitude >= FLICK_GYRO_THRESHOLD_DPS;
}

static esp_err_t flex_setup(void) {
    adc_oneshot_unit_init_cfg_t adc_unit_config = {
        .unit_id = FLEX_SENSOR_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_unit_config, &s_adc_handle),
                        TAG,
                        "adc_oneshot_new_unit failed");

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = FLEX_SENSOR_ADC_ATTEN,
        .bitwidth = FLEX_SENSOR_ADC_BITWIDTH,
    };

    for (size_t i = 0; i < FLEX_SENSOR_COUNT; ++i) {
        flex_sensor_t *sensor = &flex_sensors[i];

        ESP_RETURN_ON_ERROR(
            adc_oneshot_config_channel(s_adc_handle,
                                       sensor->channel,
                                       &channel_config),
            TAG,
            "adc_oneshot_config_channel failed");

        ESP_LOGI(TAG,
                 "%s configured on GPIO%d (ADC1 channel %d)",
                 sensor->name,
                 sensor->gpio_num,
                 sensor->channel);
    }

    return ESP_OK;
}

static esp_err_t read_flex_sensors(void) {
    for (size_t i = 0; i < FLEX_SENSOR_COUNT; ++i) {
        int raw = 0;

        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc_handle,
                                             flex_sensors[i].channel,
                                             &raw),
                            TAG,
                            "adc_oneshot_read failed");

        flex_sensors[i].raw = raw;
    }

    return ESP_OK;
}

static float flex_raw_to_percent(int raw, int straight_raw, int bent_raw) {
    if (straight_raw == bent_raw) {
        return 0.0f;
    }

    float percent = 0.0f;

    if (bent_raw > straight_raw) {
        percent = (float)(raw - straight_raw) /
                  (float)(bent_raw - straight_raw);
    } else {
        percent = (float)(straight_raw - raw) /
                  (float)(straight_raw - bent_raw);
    }

    return clamp_float(percent, 0.0f, 1.0f);
}

static flex_state_t classify_flex_state(float bend_percent) {
    if (bend_percent >= FLEX_MOVE_BENT_THRESHOLD) {
        return FLEX_STATE_BENT;
    }

    if (bend_percent <= FLEX_GESTURE_STRAIGHT_THRESHOLD) {
        return FLEX_STATE_STRAIGHT;
    }

    return FLEX_STATE_MID;
}

static int compute_speed_from_bend_percent(
    const float bend_percent[FLEX_SENSOR_COUNT]) {
    float sum_percent = 0.0f;

    for (size_t i = 0; i < FLEX_SENSOR_COUNT; ++i) {
        sum_percent += bend_percent[i];
    }

    const float avg_percent = sum_percent / (float)FLEX_SENSOR_COUNT;

    const int speed =
        SPEED_MIN +
        (int)lroundf(avg_percent * (float)(SPEED_MAX - SPEED_MIN));

    return clamp_int(speed, SPEED_MIN, SPEED_MAX);
}

static void build_payload(char *out,
                          size_t out_size,
                          const char *direction,
                          int speed,
                          const float bend_percent[FLEX_SENSOR_COUNT],
                          bool flick_event) {
    const bool flex2_move_bent =
        bend_percent[0] >= FLEX_MOVE_BENT_THRESHOLD;
    const bool flex3_move_bent =
        bend_percent[1] >= FLEX_MOVE_BENT_THRESHOLD;
    const bool flex4_move_bent =
        bend_percent[2] >= FLEX_MOVE_BENT_THRESHOLD;

    const bool all_move_bent =
        flex2_move_bent && flex3_move_bent && flex4_move_bent;

    const bool flex2_gesture_straight =
        bend_percent[0] <= FLEX_GESTURE_STRAIGHT_THRESHOLD;
    const bool flex3_gesture_straight =
        bend_percent[1] <= FLEX_GESTURE_STRAIGHT_THRESHOLD;

    const bool flex3_gesture_bent =
        bend_percent[1] >= FLEX_GESTURE_BENT_THRESHOLD;
    const bool flex4_gesture_bent =
        bend_percent[2] >= FLEX_GESTURE_BENT_THRESHOLD;

    /*
     * Special flick gestures.
     *
     * FLEX2 straight + FLEX3 bent + FLEX4 bent + flick => LED
     * FLEX2 straight + FLEX3 straight + FLEX4 bent + flick => RECORD
     */
    if (flick_event) {
        ESP_LOGI(TAG,
             "Flick flex check: F2=%.2f straight=%d | F3=%.2f straight=%d bent=%d | F4=%.2f bent=%d",
             bend_percent[0],
             flex2_gesture_straight,
             bend_percent[1],
             flex3_gesture_straight,
             flex3_gesture_bent,
             bend_percent[2],
             flex4_gesture_bent);
        if (flex2_gesture_straight &&
            flex3_gesture_bent &&
            flex4_gesture_bent) {
            snprintf(out, out_size, "LED");
            return;
        }

        if (flex2_gesture_straight &&
            flex3_gesture_straight &&
            flex4_gesture_bent) {
            snprintf(out, out_size, "RECORD");
            return;
        }
    }

    /*
     * Movement only happens if all 3 flex sensors are bent.
     */
    if (all_move_bent && strcmp(direction, "stop") != 0) {
        snprintf(out, out_size, "%s:%d", direction, speed);
        return;
    }

    snprintf(out, out_size, "stop:0");
}

static void update_command_value(const char *payload) {
    if (payload == NULL) {
        return;
    }

    strlcpy(s_last_command, payload, sizeof(s_last_command));

    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE &&
        s_notify_enabled &&
        s_command_val_handle != 0) {
        ble_gatts_chr_updated(s_command_val_handle);
        ESP_LOGI(TAG, "Sent command: %s", s_last_command);
    } else {
        ESP_LOGI(TAG, "Command: %s", s_last_command);
    }
}

static int gatt_access_cb(uint16_t conn_handle,
                          uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg) {
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, s_last_command, strlen(s_last_command));
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static void ble_app_advertise(void) {
    struct ble_hs_adv_fields fields = {0};
    struct ble_hs_adv_fields scan_rsp_fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    scan_rsp_fields.name = (uint8_t *)BLE_DEVICE_NAME;
    scan_rsp_fields.name_len = strlen(BLE_DEVICE_NAME);
    scan_rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&scan_rsp_fields);

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type,
                           NULL,
                           BLE_HS_FOREVER,
                           &adv_params,
                           gap_event_cb,
                           NULL);

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started as %s", BLE_DEVICE_NAME);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_notify_enabled = false;
            ESP_LOGI(TAG,
                     "BLE client connected, conn_handle=%u",
                     s_conn_handle);
        } else {
            ESP_LOGW(TAG,
                     "BLE connect failed, status=%d",
                     event->connect.status);
            ble_app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG,
                 "BLE client disconnected, reason=%d",
                 event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_command_val_handle) {
            s_notify_enabled = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG,
                     "Notify subscription changed: enabled=%d",
                     s_notify_enabled);

            if (s_notify_enabled) {
                update_command_value("stop:0");
            }
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete; restarting");
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG,
                 "MTU update: conn_handle=%u mtu=%u",
                 event->mtu.conn_handle,
                 event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void ble_on_reset(int reason) {
    ESP_LOGE(TAG, "NimBLE reset; reason=%d", reason);
}

static void ble_on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    ble_app_advertise();
}

static void ble_host_task(void *param) {
    (void)param;

    nimble_port_run();
    nimble_port_freertos_deinit();
}

static esp_err_t bluetooth_init(void) {
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(),
                            TAG,
                            "nvs erase failed");

        ESP_RETURN_ON_ERROR(nvs_flash_init(),
                            TAG,
                            "nvs init after erase failed");
    } else if (err != ESP_OK) {
        return err;
    }

    ESP_RETURN_ON_ERROR(nimble_port_init(),
                        TAG,
                        "nimble_port_init failed");

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

    if (rc != 0) {
        ESP_LOGW(TAG, "ble_svc_gap_device_name_set failed: %d", rc);
    }

    rc = ble_gatts_count_cfg(s_gatt_svcs);

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_gatt_svcs);

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return ESP_FAIL;
    }

    ble_store_config_init();
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE server initialized");

    return ESP_OK;
}

static esp_err_t imu_setup(void) {
    ESP_RETURN_ON_ERROR(i2c_master_init(),
                        TAG,
                        "i2c init failed");

    if (detect_imu(IMU_ADDR_LOW)) {
    } else if (detect_imu(IMU_ADDR_HIGH)) {
    } else {
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(init_imu(),
                        TAG,
                        "imu init failed");

    ESP_LOGI(TAG,
             "IMU init succeeded at address 0x%02X",
             s_imu_addr);

    ESP_RETURN_ON_ERROR(calibrate_rest_position(),
                        TAG,
                        "rest-position calibration failed");

    return ESP_OK;
}

static void glove_task(void *arg) {
    (void)arg;

    char previous_payload[16] = "";
    TickType_t last_flick_tick = 0;

    while (true) {
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;

        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;

        float temp_c = 0.0f;

        float bend_percent[FLEX_SENSOR_COUNT] = {0.0f};
        flex_state_t states[FLEX_SENSOR_COUNT] = {FLEX_STATE_MID};

        esp_err_t imu_err = read_imu(&ax, &ay, &az, &gx, &gy, &gz, &temp_c);

        if (imu_err != ESP_OK) {
            ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(imu_err));
            vTaskDelay(pdMS_TO_TICKS(COMMAND_UPDATE_MS));
            continue;
        }

        esp_err_t flex_err = read_flex_sensors();

        if (flex_err != ESP_OK) {
            ESP_LOGW(TAG, "Flex read failed: %s", esp_err_to_name(flex_err));
            vTaskDelay(pdMS_TO_TICKS(COMMAND_UPDATE_MS));
            continue;
        }

        for (size_t i = 0; i < FLEX_SENSOR_COUNT; ++i) {
            bend_percent[i] = flex_raw_to_percent(flex_sensors[i].raw,
                                                  FLEX_RAW_STRAIGHT[i],
                                                  FLEX_RAW_BENT[i]);

            states[i] = classify_flex_state(bend_percent[i]);

            // ESP_LOGI(TAG,
            //          "%s raw=%4d bend=%.2f state=%s",
            //          flex_sensors[i].name,
            //          flex_sensors[i].raw,
            //          bend_percent[i],
            //          (states[i] == FLEX_STATE_BENT) ? "BENT" :
            //          (states[i] == FLEX_STATE_STRAIGHT) ? "STRAIGHT" : "MID");
        }

        const char *direction = get_tilt_command(ax, ay, az);
        const int speed = compute_speed_from_bend_percent(bend_percent);

        const TickType_t now = xTaskGetTickCount();
        const TickType_t cooldown_ticks = pdMS_TO_TICKS(FLICK_COOLDOWN_MS);
        const bool cooldown_done = (now - last_flick_tick) > cooldown_ticks;

        bool flick_event = false;

        if (detect_flick(gx, gy, gz) && cooldown_done) {
            flick_event = true;
            last_flick_tick = now;
        }

        char payload[16] = "";

        build_payload(payload,
                      sizeof(payload),
                      direction,
                      speed,
                      bend_percent,
                      flick_event);

        if (strcmp(previous_payload, payload) != 0) {
            strlcpy(previous_payload, payload, sizeof(previous_payload));
            update_command_value(payload);
        } else {
            ESP_LOGI(TAG, "Command: %s", payload);
        }

        vTaskDelay(pdMS_TO_TICKS(COMMAND_UPDATE_MS));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting glove controller");

    if (imu_setup() != ESP_OK) {
        ESP_LOGE(TAG, "Could not find or initialize LSM6DSOX");
        return;
    }

    if (flex_setup() != ESP_OK) {
        ESP_LOGE(TAG, "Flex sensor init failed");
        return;
    }

    if (bluetooth_init() != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth init failed");
        return;
    }

    xTaskCreate(glove_task, "glove_task", 6144, NULL, 8, NULL);
}