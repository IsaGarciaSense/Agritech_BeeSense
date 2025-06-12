#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "HDC1080.h"

// HDC1080: GPIO5 (SDA), GPIO4 (SCL) 

#define I2C_MASTER_SCL_IO          4
#define I2C_MASTER_SDA_IO          5
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0

#define TAG "MAIN"

HDC1080 hdc1080(I2C_MASTER_NUM, 0x40);

static esp_err_t i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;  // Recommended for ESP32-S3

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

extern "C" void app_main() {
    ESP_ERROR_CHECK(i2c_master_init());
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(hdc1080.init());
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Manufacturer ID: 0x%04X", hdc1080.readManufacturerId());
    ESP_LOGI(TAG, "Device ID: 0x%04X", hdc1080.readDeviceId());

    HDC1080_SerialNumber sn = hdc1080.readSerialNumber();
    ESP_LOGI(TAG, "Serial Number: %04X-%04X-%04X", sn.serialFirst, sn.serialMid, sn.serialLast);

    while (true) {
        double temp = hdc1080.readTemperature();
        double hum = hdc1080.readHumidity();
        ESP_LOGI(TAG, "Temperature: %.2f C", temp);
        ESP_LOGI(TAG, "Humidity: %.2f %%", hum);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
