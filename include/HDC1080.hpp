/*******************************************************************************
 * @file HDC1080.hpp
 * @brief Library for HDC1080 Temperature and Humidity Sensor
 *
 * @version v1.0.0
 * @date 2025-06-13
 * @author Isabelle Garcia
 *******************************************************************************/

// HDC1080.hpp
#pragma once

#include "smart_sensor_sense.hpp"
#include "esp_err.h"

typedef struct {
    bool heater_enable;
    uint8_t resolution_temp;
    uint8_t resolution_hum;
} hdc1080_config_t;

class HDC1080 : public Sensor {
public:
    static constexpr uint8_t kAddressAltLow  = 0x40;
    static constexpr uint16_t kDeviceId      = 0x1050;

    HDC1080(I2C& i2cInstance, uint8_t address = kAddressAltLow, uint32_t clkSpeed = 100000);
    ~HDC1080();

    esp_err_t init(void) override;
    esp_err_t measure(void) override;
    esp_err_t getReport(char* _buff) const override;
    uint8_t getID(void) const override;
    esp_err_t readDeviceID(void);
    uint16_t getDeviceID(void) const;
    esp_err_t setHeater(bool enable);

private:
    I2C& i2c_;
    i2c_master_dev_handle_t device_handle_;
    uint32_t clkSpeed_ = 0;
    uint8_t slaveAddress_ = 0;
    uint32_t readingDelay;

    uint16_t deviceId_ = 0;
    uint8_t kSensorId_ = 0x02;
    mutable float temperature_;
    mutable float humidity_;

    static constexpr uint8_t kDevIdReg = 0xFF;
    static constexpr uint8_t kConfigReg = 0x02;
    static constexpr uint8_t RegTemp   = 0x00;

    esp_err_t readRegister(uint8_t regAddr, uint16_t* value) const;
    esp_err_t writeRegister(uint8_t regAddr, uint16_t value);
    esp_err_t configureSensor(const hdc1080_config_t& config);
    esp_err_t readTemperatureAndHumidity(void) const;
};