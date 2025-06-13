#pragma once

#include "smart_sensor_sense.hpp"
#include "esp_err.h"

class HDC1080 {
public:
    explicit HDC1080(I2C* i2c, uint8_t address = 0x40);
    ~HDC1080();

    esp_err_t init();  // Configura el sensor para lectura de T y H
    esp_err_t readTemperature(float* temperatureC);
    esp_err_t readHumidity(float* humidityPercent);

private:
    I2C* i2c_;
    uint8_t address_;

    static constexpr uint8_t REG_CONFIG = 0x02;
    static constexpr uint8_t REG_TEMP   = 0x00;
    static constexpr uint8_t REG_HUM    = 0x01;

    esp_err_t writeRegister(uint8_t reg, uint16_t value);
    esp_err_t readRegister(uint8_t reg, uint16_t* value);
};
