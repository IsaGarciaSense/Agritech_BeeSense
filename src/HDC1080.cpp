#include "HDC1080.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "HDC1080"

HDC1080::HDC1080(I2C* i2c, uint8_t address)
    : i2c_(i2c), address_(address) {}

HDC1080::~HDC1080() {}

esp_err_t HDC1080::init() {
    // Configurar registro 0x02: modo secuencial T+H
    uint16_t config = 0x1000; // default config
    return writeRegister(REG_CONFIG, config);
}

esp_err_t HDC1080::readTemperature(float* temperatureC) {
    uint16_t raw = 0;
    esp_err_t err = readRegister(REG_TEMP, &raw);
    if (err != ESP_OK) return err;

    *temperatureC = ((float)raw / 65536.0f) * 165.0f - 40.0f;
    return ESP_OK;
}

esp_err_t HDC1080::readHumidity(float* humidityPercent) {
    uint16_t raw = 0;
    esp_err_t err = readRegister(REG_HUM, &raw);
    if (err != ESP_OK) return err;

    *humidityPercent = ((float)raw / 65536.0f) * 100.0f;
    return ESP_OK;
}

esp_err_t HDC1080::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t data[2] = {
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    return i2c_->write(address_, reg, data, sizeof(data));
}

esp_err_t HDC1080::readRegister(uint8_t reg, uint16_t* value) {
    // Escribir registro y luego leer 2 bytes
    uint8_t buffer[2];
    esp_err_t err = i2c_->read(address_, reg, buffer, 2);
    if (err != ESP_OK) return err;

    *value = (buffer[0] << 8) | buffer[1];
    return ESP_OK;
}
