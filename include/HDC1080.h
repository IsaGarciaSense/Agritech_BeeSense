#ifndef _HDC1080_H_
#define _HDC1080_H_

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

typedef enum {
    HDC1080_RESOLUTION_8BIT,
    HDC1080_RESOLUTION_11BIT,
    HDC1080_RESOLUTION_14BIT,
} HDC1080_MeasurementResolution;

typedef enum {
    HDC1080_TEMPERATURE         = 0x00,
    HDC1080_HUMIDITY            = 0x01,
    HDC1080_CONFIGURATION       = 0x02,
    HDC1080_MANUFACTURER_ID     = 0xFE,
    HDC1080_DEVICE_ID           = 0xFF,
    HDC1080_SERIAL_ID_FIRST     = 0xFB,
    HDC1080_SERIAL_ID_MID       = 0xFC,
    HDC1080_SERIAL_ID_LAST      = 0xFD,
} HDC1080_Pointers;

typedef union {
    struct {
        uint16_t serialFirst;
        uint16_t serialMid;
        uint16_t serialLast;
    };
    uint8_t rawData[6];
} HDC1080_SerialNumber;

typedef union {
    struct {
        uint8_t HumidityMeasurementResolution : 2;
        uint8_t TemperatureMeasurementResolution : 1;
        uint8_t BatteryStatus : 1;
        uint8_t ModeOfAcquisition : 1;
        uint8_t Heater : 1;
        uint8_t ReservedAgain : 1;
        uint8_t SoftwareReset : 1;
    };
    uint8_t rawData;
} HDC1080_Registers;

class HDC1080 {
public:
    HDC1080(i2c_port_t port, uint8_t address);

    esp_err_t init();
    esp_err_t setResolution(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature);

    double readTemperature();
    double readHumidity();
    double readT(); // shortcut
    double readH(); // shortcut

    HDC1080_SerialNumber readSerialNumber();
    uint16_t readManufacturerId();
    uint16_t readDeviceId();

    HDC1080_Registers readRegister();
    esp_err_t writeRegister(HDC1080_Registers reg);

    esp_err_t heatUp(uint8_t seconds);

private:
    i2c_port_t i2c_port;
    uint8_t _address;
    uint16_t readData(uint8_t pointer);
};

#endif // _HDC1080_H_
