#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/clk_tree_defs.h"

#define I2C_BUS_1_SDA 2
#define I2C_BUS_1_SCL 1

#define I2C_BUS_2_SDA 41
#define I2C_BUS_2_SCL 42

#define X_SHUT_TOF_2 45
#define X_SHUT_TOF_3 40

#define I2C_BUS_1 I2C_NUM_0
#define I2C_BUS_2 I2C_NUM_1
#define I2C_FREQ_HZ 400000
#define I2C_TIMEOUT_MS 100

#define VL53L0X_DEFAULT_ADDR 0x29
#define VL53L0X_SENSOR_2_ADDR 0x30
#define VL53L0X_SENSOR_3_ADDR 0x31

#define REG_SYSRANGE_START 0x00
#define REG_SYSTEM_INTERRUPT_CLEAR 0x0B
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define REG_RESULT_INTERRUPT_STATUS 0x13
#define REG_RESULT_RANGE_STATUS 0x14
#define REG_GPIO_HV_MUX_ACTIVE_HIGH 0x84
#define REG_I2C_SLAVE_DEVICE_ADDRESS 0x8A
#define REG_IDENTIFICATION_MODEL_ID 0xC0

#define RANGE_READ_TIMEOUT_MS 100

#define RETURN_ON_ERROR(expr, log_tag, message)                    \
    do {                                                           \
        esp_err_t __err_rc = (expr);                               \
        if (__err_rc != ESP_OK) {                                  \
            ESP_LOGE((log_tag), "%s: %s", (message),               \
                     esp_err_to_name(__err_rc));                   \
            return __err_rc;                                       \
        }                                                          \
    } while (0)

static const char *TAG = "triple_lidar";

typedef struct {
    const char *name;
    i2c_master_dev_handle_t handle;
    uint8_t address;
} tof_sensor_t;

static i2c_master_bus_handle_t s_bus_1 = NULL;
static i2c_master_bus_handle_t s_bus_2 = NULL;

static esp_err_t i2c_bus_init(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl, i2c_master_bus_handle_t *bus_handle)
{
    const i2c_master_bus_config_t config = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = 1,
        .flags.allow_pd = 0,
    };

    return i2c_new_master_bus(&config, bus_handle);
}

static esp_err_t i2c_add_device(i2c_master_bus_handle_t bus, uint8_t address, i2c_master_dev_handle_t *device_handle)
{
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags.disable_ack_check = 0,
    };

    return i2c_master_bus_add_device(bus, &config, device_handle);
}

static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t value)
{
    const uint8_t payload[] = { reg, value };
    return i2c_master_transmit(dev_handle, payload, sizeof(payload), I2C_TIMEOUT_MS);
}

static esp_err_t i2c_read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, value, 1, I2C_TIMEOUT_MS);
}

static esp_err_t i2c_read_regs(i2c_master_dev_handle_t dev_handle, uint8_t start_reg, uint8_t *buffer, size_t length)
{
    return i2c_master_transmit_receive(dev_handle, &start_reg, 1, buffer, length, I2C_TIMEOUT_MS);
}

static void scan_i2c_bus(i2c_master_bus_handle_t bus_handle, const char *label)
{
    ESP_LOGI(TAG, "Scanning %s...", label);
    bool found = false;

    for (uint8_t addr = 1; addr < 0x78; ++addr) {
        if (i2c_master_probe(bus_handle, addr, I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "%s found device at 0x%02X", label, addr);
            found = true;
        }
    }

    if (!found) {
        ESP_LOGW(TAG, "%s scan found no devices", label);
    }
}

static esp_err_t tof_set_shutdown_pin(gpio_num_t pin, uint32_t level)
{
    return gpio_set_level(pin, level);
}

static esp_err_t tof_shutdown_pins_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << X_SHUT_TOF_2) | (1ULL << X_SHUT_TOF_3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    RETURN_ON_ERROR(gpio_config(&cfg), TAG, "xshut pin init failed");
    RETURN_ON_ERROR(tof_set_shutdown_pin(X_SHUT_TOF_2, 0), TAG, "failed to hold TOF2 in reset");
    RETURN_ON_ERROR(tof_set_shutdown_pin(X_SHUT_TOF_3, 0), TAG, "failed to hold TOF3 in reset");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t vl53l0x_check_identity(i2c_master_dev_handle_t dev_handle, uint8_t address)
{
    uint8_t model_id = 0;
    RETURN_ON_ERROR(i2c_read_reg(dev_handle, REG_IDENTIFICATION_MODEL_ID, &model_id), TAG, "model ID read failed");

    if (model_id != 0xEE) {
        ESP_LOGE(TAG, "Unexpected model ID 0x%02X at 0x%02X", model_id, address);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t vl53l0x_basic_init(i2c_master_dev_handle_t dev_handle, uint8_t address)
{
    uint8_t value = 0;

    RETURN_ON_ERROR(vl53l0x_check_identity(dev_handle, address), TAG, "VL53L0X identity check failed");
    RETURN_ON_ERROR(i2c_read_reg(dev_handle, REG_GPIO_HV_MUX_ACTIVE_HIGH, &value), TAG, "GPIO mux read failed");
    RETURN_ON_ERROR(i2c_write_reg(dev_handle, REG_GPIO_HV_MUX_ACTIVE_HIGH, value & (uint8_t)~0x10), TAG, "GPIO mux write failed");
    RETURN_ON_ERROR(i2c_write_reg(dev_handle, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04), TAG, "interrupt config failed");
    RETURN_ON_ERROR(i2c_write_reg(dev_handle, REG_SYSTEM_INTERRUPT_CLEAR, 0x01), TAG, "interrupt clear failed");

    return ESP_OK;
}

static esp_err_t vl53l0x_set_address(i2c_master_dev_handle_t dev_handle, uint8_t old_address, uint8_t new_address)
{
    RETURN_ON_ERROR(
        i2c_write_reg(dev_handle, REG_I2C_SLAVE_DEVICE_ADDRESS, new_address & 0x7F),
        TAG,
        "set address failed");
    RETURN_ON_ERROR(
        i2c_master_device_change_address(dev_handle, new_address, I2C_TIMEOUT_MS),
        TAG,
        "device handle address update failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Device moved from 0x%02X to 0x%02X", old_address, new_address);
    return ESP_OK;
}

static esp_err_t vl53l0x_read_range_mm(i2c_master_dev_handle_t dev_handle, uint8_t address, uint16_t *range_mm)
{
    TickType_t start = xTaskGetTickCount();
    uint8_t status = 0;
    uint8_t raw_range[2] = {0};

    RETURN_ON_ERROR(i2c_write_reg(dev_handle, REG_SYSRANGE_START, 0x01), TAG, "failed to start range measurement");

    do {
        RETURN_ON_ERROR(i2c_read_reg(dev_handle, REG_RESULT_INTERRUPT_STATUS, &status), TAG, "failed to poll measurement status");
        if ((status & 0x07) != 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    } while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(RANGE_READ_TIMEOUT_MS));

    if ((status & 0x07) == 0) {
        ESP_LOGW(TAG, "Range read timed out at 0x%02X", address);
        return ESP_ERR_TIMEOUT;
    }

    RETURN_ON_ERROR(
        i2c_read_regs(dev_handle, REG_RESULT_RANGE_STATUS + 10, raw_range, sizeof(raw_range)),
        TAG,
        "failed to read range result");

    *range_mm = ((uint16_t)raw_range[0] << 8) | raw_range[1];

    return i2c_write_reg(dev_handle, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

static esp_err_t bring_up_tof_on_bus_2(
    gpio_num_t xshut_pin,
    uint8_t new_address,
    const char *label,
    i2c_master_dev_handle_t *dev_handle)
{
    ESP_LOGI(TAG, "Enabling %s on bus 2", label);
    RETURN_ON_ERROR(tof_set_shutdown_pin(xshut_pin, 1), TAG, "failed to release XSHUT");
    vTaskDelay(pdMS_TO_TICKS(50));

    scan_i2c_bus(s_bus_2, "I2C bus 2");
    RETURN_ON_ERROR(i2c_add_device(s_bus_2, VL53L0X_DEFAULT_ADDR, dev_handle), TAG, "add default-address device failed");
    RETURN_ON_ERROR(vl53l0x_basic_init(*dev_handle, VL53L0X_DEFAULT_ADDR), TAG, "basic init failed");
    RETURN_ON_ERROR(vl53l0x_set_address(*dev_handle, VL53L0X_DEFAULT_ADDR, new_address), TAG, "address assignment failed");
    RETURN_ON_ERROR(vl53l0x_basic_init(*dev_handle, new_address), TAG, "re-init after address change failed");

    ESP_LOGI(TAG, "%s now responding at 0x%02X", label, new_address);
    scan_i2c_bus(s_bus_2, "I2C bus 2");
    return ESP_OK;
}

static esp_err_t init_all_sensors(tof_sensor_t *sensors, size_t sensor_count)
{
    if (sensor_count < 3) {
        return ESP_ERR_INVALID_ARG;
    }

    RETURN_ON_ERROR(tof_shutdown_pins_init(), TAG, "shutdown pin setup failed");
    RETURN_ON_ERROR(i2c_bus_init(I2C_BUS_1, I2C_BUS_1_SDA, I2C_BUS_1_SCL, &s_bus_1), TAG, "I2C bus 1 init failed");
    RETURN_ON_ERROR(i2c_bus_init(I2C_BUS_2, I2C_BUS_2_SDA, I2C_BUS_2_SCL, &s_bus_2), TAG, "I2C bus 2 init failed");

    ESP_LOGI(TAG, "Stage 1: baseline scan with TOF2 and TOF3 held in reset");
    scan_i2c_bus(s_bus_1, "I2C bus 1");
    scan_i2c_bus(s_bus_2, "I2C bus 2");

    sensors[0].name = "TOF1";
    sensors[0].address = VL53L0X_DEFAULT_ADDR;
    RETURN_ON_ERROR(i2c_add_device(s_bus_1, sensors[0].address, &sensors[0].handle), TAG, "TOF1 device add failed");
    RETURN_ON_ERROR(vl53l0x_basic_init(sensors[0].handle, sensors[0].address), TAG, "TOF1 init failed");
    ESP_LOGI(TAG, "TOF1 on bus 1 responding at 0x%02X", sensors[0].address);

    ESP_LOGI(TAG, "Stage 2: release TOF2 only; expect device at default address 0x%02X", VL53L0X_DEFAULT_ADDR);
    sensors[1].name = "TOF2";
    sensors[1].address = VL53L0X_SENSOR_2_ADDR;
    RETURN_ON_ERROR(
        bring_up_tof_on_bus_2(X_SHUT_TOF_2, sensors[1].address, sensors[1].name, &sensors[1].handle),
        TAG,
        "TOF2 bring-up failed");

    ESP_LOGI(TAG, "Stage 3: release TOF3 only; expect 0x%02X and existing TOF2 at 0x%02X", VL53L0X_DEFAULT_ADDR, VL53L0X_SENSOR_2_ADDR);
    sensors[2].name = "TOF3";
    sensors[2].address = VL53L0X_SENSOR_3_ADDR;
    RETURN_ON_ERROR(
        bring_up_tof_on_bus_2(X_SHUT_TOF_3, sensors[2].address, sensors[2].name, &sensors[2].handle),
        TAG,
        "TOF3 bring-up failed");

    ESP_LOGI(TAG, "Stage 4: final scan; expect TOF2 at 0x%02X and TOF3 at 0x%02X", VL53L0X_SENSOR_2_ADDR, VL53L0X_SENSOR_3_ADDR);
    scan_i2c_bus(s_bus_1, "I2C bus 1");
    scan_i2c_bus(s_bus_2, "I2C bus 2");
    return ESP_OK;
}

void app_main(void)
{
    tof_sensor_t sensors[3] = {0};

    if (init_all_sensors(sensors, sizeof(sensors) / sizeof(sensors[0])) != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed. Check wiring, pullups, XSHUT pins, and power.");
        return;
    }

    ESP_LOGI(TAG, "Initialization complete. Starting continuous ranging.");
    while (true) {
        for (size_t i = 0; i < sizeof(sensors) / sizeof(sensors[0]); ++i) {
            uint16_t range_mm = 0;
            esp_err_t err = vl53l0x_read_range_mm(sensors[i].handle, sensors[i].address, &range_mm);

            if (err == ESP_OK) {
                ESP_LOGI(TAG, "%s addr=0x%02X range=%u mm", sensors[i].name, sensors[i].address, range_mm);
            } else {
                ESP_LOGW(TAG, "%s addr=0x%02X read failed: %s", sensors[i].name, sensors[i].address, esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
