/*******************************************************************************
 * @file HDC1080.hpp
 * @brief Library for HDC1080 Temperature and Humidity Sensor
 *
 * @version v1.0.0
 * @date 2025-06-13
 * @author Isabelle Garcia
 *******************************************************************************/

#pragma once

#include "smart_sensor_sense.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Registros del HDC1080
#define HDC1080_TEMP_REG        0x00
#define HDC1080_HUMIDITY_REG    0x01
#define HDC1080_CONFIG_REG      0x02
#define HDC1080_SERIAL_ID_1     0xFB
#define HDC1080_SERIAL_ID_2     0xFC
#define HDC1080_SERIAL_ID_3     0xFD
#define HDC1080_MANUFACTURER_ID 0xFE
#define HDC1080_DEVICE_ID       0xFF

// Valores esperados de identificación
#define HDC1080_EXPECTED_MANUFACTURER_ID 0x5449  // "TI"
#define HDC1080_EXPECTED_DEVICE_ID       0x1050

// Configuración por defecto
#define HDC1080_DEFAULT_CONFIG   0x1000  // Modo secuencial, resolución 14-bit

// Rangos de operación
#define HDC1080_TEMP_MIN        -40.0f
#define HDC1080_TEMP_MAX        125.0f
#define HDC1080_HUMIDITY_MIN    0.0f
#define HDC1080_HUMIDITY_MAX    100.0f

/**
 * @brief Estructura para almacenar datos del sensor
 */
typedef struct {
    float temperature;  // Temperatura en °C
    float humidity;     // Humedad relativa en %
    bool valid;         // Indica si los datos son válidos
} hdc1080_data_t;

/**
 * @brief Configuración del HDC1080
 */
typedef struct {
    bool sequential_mode;       // true = modo secuencial, false = individual
    bool temp_resolution_14bit; // true = 14-bit, false = 11-bit
    bool hum_resolution_14bit;  // true = 14-bit, false = 11-bit (también 8-bit disponible)
    bool heater_enable;         // true = calentador habilitado
} hdc1080_config_t;

/**
 * @class HDC1080
 * @brief Clase para controlar el sensor HDC1080
 */
class HDC1080 {
public:
    /**
     * @brief Constructor
     * @param i2c_bus Puntero al bus I2C
     * @param device_address Dirección I2C del dispositivo (por defecto 0x40)
     * @param clock_speed Velocidad del reloj I2C en Hz
     */
    HDC1080(I2C* i2c_bus, uint8_t device_address = 0x40, uint32_t clock_speed = 100000);

    /**
     * @brief Destructor
     */
    ~HDC1080();

    /**
     * @brief Inicializa el sensor
     * @param config Configuración del sensor (usar nullptr para configuración por defecto)
     * @return esp_err_t ESP_OK si es exitoso
     */
    esp_err_t init(hdc1080_config_t* config = nullptr);

    /**
     * @brief Verifica si el sensor está conectado y funcionando
     * @return true si el sensor responde correctamente
     */
    bool isConnected();

    /**
     * @brief Lee temperatura y humedad en una sola operación (modo secuencial)
     * @return hdc1080_data_t Estructura con temperatura, humedad y estado de validez
     */
    hdc1080_data_t readBoth();

    /**
     * @brief Lee solo la temperatura
     * @return float Temperatura en °C (-999.0 si hay error)
     */
    float readTemperature();

    /**
     * @brief Lee solo la humedad
     * @return float Humedad en % (-999.0 si hay error)
     */
    float readHumidity();

    /**
     * @brief Obtiene el ID del fabricante
     * @return uint16_t ID del fabricante (0x5449 para TI)
     */
    uint16_t getManufacturerID();

    /**
     * @brief Obtiene el ID del dispositivo
     * @return uint16_t ID del dispositivo (0x1050 para HDC1080)
     */
    uint16_t getDeviceID();

    /**
     * @brief Obtiene los números de serie del dispositivo
     * @param serial_1 Puntero para almacenar el primer número de serie
     * @param serial_2 Puntero para almacenar el segundo número de serie
     * @param serial_3 Puntero para almacenar el tercer número de serie
     * @return esp_err_t ESP_OK si es exitoso
     */
    esp_err_t getSerialNumbers(uint16_t* serial_1, uint16_t* serial_2, uint16_t* serial_3);

    /**
     * @brief Habilita o deshabilita el calentador interno
     * @param enable true para habilitar, false para deshabilitar
     * @return esp_err_t ESP_OK si es exitoso
     */
    esp_err_t setHeater(bool enable);

    /**
     * @brief Realiza un reset por software del sensor
     * @return esp_err_t ESP_OK si es exitoso
     */
    esp_err_t softReset();

    /**
     * @brief Configura la resolución del sensor
     * @param temp_14bit true para 14-bit, false para 11-bit (temperatura)
     * @param hum_14bit true para 14-bit, false para 11-bit (humedad)
     * @return esp_err_t ESP_OK si es exitoso
     */
    esp_err_t setResolution(bool temp_14bit, bool hum_14bit);

    /**
     * @brief Verifica si los valores están en rango válido
     * @param temperature Temperatura a verificar
     * @param humidity Humedad a verificar
     * @return true si ambos valores están en rango
     */
    static bool isDataValid(float temperature, float humidity);

private:
    I2C* i2c_bus_;                              /**< Puntero al bus I2C */
    i2c_master_dev_handle_t device_handle_;     /**< Handle del dispositivo */
    uint8_t device_address_;                    /**< Dirección I2C del dispositivo */
    uint32_t clock_speed_;                      /**< Velocidad del reloj I2C */
    bool initialized_;                          /**< Estado de inicialización */
    hdc1080_config_t current_config_;          /**< Configuración actual */

    static const char* TAG;                     /**< Tag para logging */

    /**
     * @brief Lee un registro de 16 bits
     * @param reg_address Dirección del registro
     * @param timeout_ms Timeout en milisegundos
     * @return uint16_t Valor leído (0xFFFF si hay error)
     */
    uint16_t readRegister16(uint8_t reg_address, int timeout_ms = 15);

    /**
     * @brief Escribe un registro de 16 bits
     * @param reg_address Dirección del registro
     * @param value Valor a escribir
     * @return esp_err_t ESP_OK si es exitoso
     */
    esp_err_t writeRegister16(uint8_t reg_address, uint16_t value);

    /**
     * @brief Convierte valor raw de temperatura a °C
     * @param raw_value Valor raw del ADC
     * @return float Temperatura en °C
     */
    static float convertRawTemperature(uint16_t raw_value);

    /**
     * @brief Convierte valor raw de humedad a %
     * @param raw_value Valor raw del ADC
     * @return float Humedad en %
     */
    static float convertRawHumidity(uint16_t raw_value);

    /**
     * @brief Construye el valor de configuración basado en la estructura
     * @param config Configuración deseada
     * @return uint16_t Valor de configuración para el registro
     */
    static uint16_t buildConfigValue(const hdc1080_config_t& config);
};