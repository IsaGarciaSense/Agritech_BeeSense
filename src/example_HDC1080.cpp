#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "HDC1080.hpp"
#include "smart_sensor_sense.hpp"

#define TAG "HDC1080_MAIN"
constexpr gpio_num_t SDA_PIN = GPIO_NUM_5;
constexpr gpio_num_t SCL_PIN = GPIO_NUM_4;

I2C i2c(I2C_NUM_0, SDA_PIN, SCL_PIN, true);

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting HDC1080 Test...");
    HDC1080 sensor(i2c);

    esp_err_t err = i2c.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C: %s", esp_err_to_name(err));
        return;
    }

    err = sensor.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed: %s", esp_err_to_name(err));
        return;
    }

    char report[100];
    while (1) {
        err = sensor.measure();
        if (err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (sensor.getReport(report) == ESP_OK) {
                ESP_LOGI(TAG, "%s", report);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
