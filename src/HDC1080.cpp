#include "HDC1080.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <math.h>

#define TAG "HDC1080"

HDC1080::HDC1080(i2c_port_t port, uint8_t address)
    : i2c_port(port), _address(address) {
}

esp_err_t HDC1080::init() {
    ESP_LOGI(TAG, "Initializing HDC1080 at address 0x%02X", _address);
    return setResolution(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);
}

esp_err_t HDC1080::setResolution(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature) {
    HDC1080_Registers reg = {};
    if (temperature == HDC1080_RESOLUTION_11BIT)
        reg.TemperatureMeasurementResolution = 0x01;

    switch (humidity) {
        case HDC1080_RESOLUTION_8BIT:
            reg.HumidityMeasurementResolution = 0x02;
            break;
        case HDC1080_RESOLUTION_11BIT:
            reg.HumidityMeasurementResolution = 0x01;
            break;
        default:
            break;
    }

    return writeRegister(reg);
}

HDC1080_SerialNumber HDC1080::readSerialNumber() {
    HDC1080_SerialNumber sernum = {};
    sernum.serialFirst = readData(HDC1080_SERIAL_ID_FIRST);
    sernum.serialMid = readData(HDC1080_SERIAL_ID_MID);
    sernum.serialLast = readData(HDC1080_SERIAL_ID_LAST);
    return sernum;
}

HDC1080_Registers HDC1080::readRegister() {
    HDC1080_Registers reg = {};
    reg.rawData = (readData(HDC1080_CONFIGURATION) >> 8);
    return reg;
}

esp_err_t HDC1080::writeRegister(HDC1080_Registers reg) {
    uint8_t data[3] = {
        HDC1080_CONFIGURATION,
        reg.rawData,
        0x00
    };

    return i2c_master_write_to_device(i2c_port, _address, data, sizeof(data), pdMS_TO_TICKS(100));
}

esp_err_t HDC1080::heatUp(uint8_t seconds) {
    HDC1080_Registers reg = readRegister();
    reg.Heater = 1;
    reg.ModeOfAcquisition = 1;
    ESP_ERROR_CHECK(writeRegister(reg));

    uint8_t cmd = HDC1080_TEMPERATURE;
    uint8_t buf[4];

    for (int i = 0; i < seconds * 66; i++) {
        i2c_master_write_to_device(i2c_port, _address, &cmd, 1, pdMS_TO_TICKS(50));
        vTaskDelay(pdMS_TO_TICKS(20));
        i2c_master_read_from_device(i2c_port, _address, buf, 4, pdMS_TO_TICKS(50));
    }

    reg.Heater = 0;
    reg.ModeOfAcquisition = 0;
    return writeRegister(reg);
}

double HDC1080::readTemperature() {
    uint16_t raw = readData(HDC1080_TEMPERATURE);
    return ((double)raw / 65536.0) * 165.0 - 40.0;
}

double HDC1080::readHumidity() {
    uint16_t raw = readData(HDC1080_HUMIDITY);
    return ((double)raw / 65536.0) * 100.0;
}

uint16_t HDC1080::readManufacturerId() {
    return readData(HDC1080_MANUFACTURER_ID);
}

uint16_t HDC1080::readDeviceId() {
    return readData(HDC1080_DEVICE_ID);
}

/*uint16_t HDC1080::readData(uint8_t pointer) {
    uint8_t data[2] = {0};
    i2c_master_write_to_device(i2c_port, _address, &pointer, 1, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(9));  // sensor requires delay
    i2c_master_read_from_device(i2c_port, _address, data, 2, pdMS_TO_TICKS(100));
    return (data[0] << 8) | data[1];
}*/



uint16_t HDC1080::readData(uint8_t pointer) {
    esp_err_t err;
    uint8_t data[2] = {0};

    // Send pointer register
    err = i2c_master_write_to_device(i2c_port, _address, &pointer, 1, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write pointer 0x%02X", pointer);
        return 0;
    }

    // REQUIRED delay after starting a measurement
    vTaskDelay(pdMS_TO_TICKS(15)); // Safer than minimum spec

    // Read 2 bytes back
    err = i2c_master_read_from_device(i2c_port, _address, data, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data for pointer 0x%02X", pointer);
        return 0;
    }

    uint16_t result = (data[0] << 8) | data[1];
    ESP_LOGD(TAG, "Read 0x%04X from pointer 0x%02X", result, pointer);
    return result;
}




double HDC1080::readT() {
    return readTemperature();
}

double HDC1080::readH() {
    return readHumidity();
}
