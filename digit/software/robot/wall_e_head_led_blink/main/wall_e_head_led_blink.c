#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_UART_TX_GPIO GPIO_NUM_43
#define LED_UART_RX_GPIO GPIO_NUM_44
#define LED_GPIO_MASK ((1ULL << LED_UART_TX_GPIO) | (1ULL << LED_UART_RX_GPIO))
#define LED_ON_LEVEL 0
#define LED_OFF_LEVEL 1
#define BLINK_PERIOD_MS 150

static const char *TAG = "wall_e_head_led";

void app_main(void)
{
    gpio_config_t io_config = {
        .pin_bit_mask = LED_GPIO_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART pins as GPIO outputs: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Alternating active-low LEDs on GPIO %d and GPIO %d",
             LED_UART_TX_GPIO, LED_UART_RX_GPIO);

    while (true) {
        gpio_set_level(LED_UART_TX_GPIO, LED_ON_LEVEL);
        gpio_set_level(LED_UART_RX_GPIO, LED_OFF_LEVEL);
        ESP_LOGI(TAG, "GPIO %d LED on, GPIO %d LED off", LED_UART_TX_GPIO, LED_UART_RX_GPIO);
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_MS));

        gpio_set_level(LED_UART_TX_GPIO, LED_OFF_LEVEL);
        gpio_set_level(LED_UART_RX_GPIO, LED_ON_LEVEL);
        ESP_LOGI(TAG, "GPIO %d LED off, GPIO %d LED on", LED_UART_TX_GPIO, LED_UART_RX_GPIO);
        vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_MS));
    }
}
