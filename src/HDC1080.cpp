/*******************************************************************************
 * @file HDC1080.cpp
 * @brief Implementation of HDC1080 Temperature and Humidity Sensor Library
 *
 * @version v1.0.0
 * @date 2025-06-13
 * @author Sense-AI
 *******************************************************************************/

#include "HDC1080.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "HDC1080"

HDC1080::HDC1080(I2C &i2cInstance, uint8_t slaveAddress, uint32_t clkSpeed)
    : i2c_(i2cInstance), clkSpeed_(clkSpeed), slaveAddress_(slaveAddress), 
      temperature_(0.0f), humidity_(0.0f) {}

HDC1080::~HDC1080() {}

esp_err_t HDC1080::init(void) 
{
    esp_err_t err = i2c_.addDevice(&device_handle_, slaveAddress_, clkSpeed_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        return err;
    }

    readingDelay = (clkSpeed_ == 100000) ? 1000 : 500;

    err = readDeviceID();
    if (err != ESP_OK || deviceId_ != kDeviceId) {
        ESP_LOGE(TAG, "Invalid Device ID: 0x%04X", deviceId_);
        return ESP_ERR_NOT_FOUND;
    }

    hdc1080_config_t default_config = {
        .heater_enable = false,
        .resolution_temp = 0x00,
        .resolution_hum = 0x00
    };
    return configureSensor(default_config);
}

esp_err_t HDC1080::measure(void) {
    uint16_t config = 0x10;
    esp_err_t err = writeRegister(kConfigReg, config);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

esp_err_t HDC1080::getReport(char* _buff) const {
    if (_buff == nullptr) return ESP_ERR_INVALID_ARG;

    esp_err_t err = readTemperatureAndHumidity();
    if (err != ESP_OK) return err;

    snprintf(_buff, 100, "Temperature: %.2f C, Humidity: %.2f %%", temperature_, humidity_);
    return ESP_OK;
}

uint8_t HDC1080::getID(void) const {
    return kSensorId_;
}

esp_err_t HDC1080::readDeviceID(void) {
    return readRegister(kDevIdReg, &deviceId_);
}

uint16_t HDC1080::getDeviceID(void) const {
    return deviceId_;
}

esp_err_t HDC1080::setHeater(bool enable) {
    uint16_t config = 0;
    esp_err_t err = readRegister(kConfigReg, &config);
    if (err != ESP_OK) return err;

    if (enable) config |= 0x20;
    else config &= ~0x20;

    return writeRegister(kConfigReg, config);
}

esp_err_t HDC1080::readTemperatureAndHumidity(void) const {
    uint8_t data[4];
    esp_err_t err = i2c_.read(device_handle_, RegTemp, data, sizeof(data), readingDelay);
    if (err != ESP_OK) return err;

    uint16_t rawTemp = (data[0] << 8) | data[1];
    uint16_t rawHum  = (data[2] << 8) | data[3];

    temperature_ = ((float)rawTemp / 65536.0f) * 165.0f - 40.0f;
    humidity_ = ((float)rawHum / 65536.0f) * 100.0f;

    return ESP_OK;
}

esp_err_t HDC1080::configureSensor(const hdc1080_config_t& config) {
    uint16_t configReg = 0;
    if (config.heater_enable) configReg |= 0x20;
    configReg |= (config.resolution_temp & 0x03) << 10;
    configReg |= (config.resolution_hum & 0x03) << 8;
    return writeRegister(kConfigReg, configReg);
}

esp_err_t HDC1080::readRegister(uint8_t regAddr, uint16_t* value) const {
    if (value == nullptr) return ESP_ERR_INVALID_ARG;
    uint8_t data[2];
    esp_err_t err = i2c_.read(device_handle_, regAddr, data, 2, readingDelay);
    if (err != ESP_OK) return err;
    *value = (data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t HDC1080::writeRegister(uint8_t regAddr, uint16_t value) {
    uint8_t data[2] = { static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF) };
    return i2c_.write(device_handle_, regAddr, data, 2);
}
