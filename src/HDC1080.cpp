/*******************************************************************************
 * @file HDC1080.cpp
 * @brief Implementation of HDC1080 Temperature and Humidity Sensor Library
 *
 * @version v1.0.0
 * @date 2025-06-13
 * @author Sense-AI
 *******************************************************************************/

#include "HDC1080.hpp"

const char* HDC1080::TAG = "HDC1080";

HDC1080::HDC1080(I2C* i2c_bus, uint8_t device_address, uint32_t clock_speed)
    : i2c_bus_(i2c_bus), device_address_(device_address), clock_speed_(clock_speed), 
      initialized_(false)
{
    // Configuración por defecto
    current_config_ = {
        .sequential_mode = true,
        .temp_resolution_14bit = true,
        .hum_resolution_14bit = true,
        .heater_enable = false
    };
}

HDC1080::~HDC1080()
{
    // El destructor del bus I2C se encarga de la limpieza
}

esp_err_t HDC1080::init(hdc1080_config_t* config)
{
    if (!i2c_bus_) {
        ESP_LOGE(TAG, "Bus I2C no válido");
        return ESP_ERR_INVALID_ARG;
    }

    // Agregar dispositivo al bus I2C
    esp_err_t ret = i2c_bus_->addDevice(&device_handle_, device_address_, clock_speed_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error agregando dispositivo al bus I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verificar comunicación
    if (!isConnected()) {
        ESP_LOGE(TAG, "No se puede comunicar con el HDC1080");
        return ESP_ERR_NOT_FOUND;
    }

    // Aplicar configuración
    if (config) {
        current_config_ = *config;
    }

    uint16_t config_value = buildConfigValue(current_config_);
    ret = writeRegister16(HDC1080_CONFIG_REG, config_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando HDC1080: %s", esp_err_to_name(ret));
        return ret;
    }

    // Esperar a que se aplique la configuración
    vTaskDelay(pdMS_TO_TICKS(15));

    initialized_ = true;
    ESP_LOGI(TAG, "HDC1080 inicializado correctamente en dirección 0x%02X", device_address_);
    
    return ESP_OK;
}

bool HDC1080::isConnected()
{
    uint16_t manufacturer_id = getManufacturerID();
    uint16_t device_id = getDeviceID();
    
    bool connected = (manufacturer_id == HDC1080_EXPECTED_MANUFACTURER_ID) && 
                    (device_id == HDC1080_EXPECTED_DEVICE_ID);
    
    if (connected) {
        ESP_LOGI(TAG, "Manufacturer ID: 0x%04X, Device ID: 0x%04X", manufacturer_id, device_id);
    } else {
        ESP_LOGE(TAG, "IDs incorrectos - Manufacturer: 0x%04X (esperado: 0x%04X), Device: 0x%04X (esperado: 0x%04X)", 
                 manufacturer_id, HDC1080_EXPECTED_MANUFACTURER_ID, 
                 device_id, HDC1080_EXPECTED_DEVICE_ID);
    }
    
    return connected;
}

hdc1080_data_t HDC1080::readBoth()
{
    hdc1080_data_t result = {0.0f, 0.0f, false};
    
    if (!initialized_) {
        ESP_LOGE(TAG, "HDC1080 no inicializado");
        return result;
    }

    if (!current_config_.sequential_mode) {
        ESP_LOGW(TAG, "Modo secuencial no habilitado, usando lecturas individuales");
        result.temperature = readTemperature();
        vTaskDelay(pdMS_TO_TICKS(20));
        result.humidity = readHumidity();
        result.valid = (result.temperature != -999.0f) && (result.humidity != -999.0f);
        return result;
    }

    // En modo secuencial del HDC1080, necesitamos:
    // 1. Escribir el registro de temperatura para iniciar la conversión
    // 2. Esperar el tiempo de conversión
    // 3. Leer los 4 bytes (2 de temp + 2 de humedad)
    
    // Paso 1: Iniciar conversión escribiendo la dirección del registro de temperatura
    uint8_t temp_reg = HDC1080_TEMP_REG;
    esp_err_t ret = i2c_bus_->write(device_handle_, temp_reg, nullptr, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando conversión: %s", esp_err_to_name(ret));
        return result;
    }
    
    // Paso 2: Esperar tiempo de conversión (14ms para ambas mediciones)
    vTaskDelay(pdMS_TO_TICKS(15));
    
    // Paso 3: Leer los 4 bytes usando readStream (sin especificar registro)
    uint8_t data[4];
    ret = i2c_bus_->readStream(device_handle_, data, 4);
    
    if (ret == ESP_OK) {
        // Primeros 2 bytes: temperatura
        uint16_t raw_temp = (data[0] << 8) | data[1];
        result.temperature = convertRawTemperature(raw_temp);
        
        // Siguientes 2 bytes: humedad
        uint16_t raw_humidity = (data[2] << 8) | data[3];
        result.humidity = convertRawHumidity(raw_humidity);
        
        result.valid = isDataValid(result.temperature, result.humidity);
        
        ESP_LOGI(TAG, "Raw temp: 0x%04X (%d) | Raw humidity: 0x%04X (%d)", 
                 raw_temp, raw_temp, raw_humidity, raw_humidity);
        ESP_LOGI(TAG, "Bytes: [0x%02X, 0x%02X, 0x%02X, 0x%02X]", 
                 data[0], data[1], data[2], data[3]);
    } else {
        ESP_LOGE(TAG, "Error leyendo datos: %s", esp_err_to_name(ret));
    }
    
    return result;
}

float HDC1080::readTemperature()
{
    if (!initialized_) {
        ESP_LOGE(TAG, "HDC1080 no inicializado");
        return -999.0f;
    }

    // Iniciar conversión de temperatura
    uint8_t temp_reg = HDC1080_TEMP_REG;
    esp_err_t ret = i2c_bus_->write(device_handle_, temp_reg, nullptr, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando conversión de temperatura: %s", esp_err_to_name(ret));
        return -999.0f;
    }
    
    // Esperar tiempo de conversión (6.5ms para temperatura)
    vTaskDelay(pdMS_TO_TICKS(7));
    
    // Leer resultado
    uint8_t data[2];
    ret = i2c_bus_->readStream(device_handle_, data, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo temperatura: %s", esp_err_to_name(ret));
        return -999.0f;
    }
    
    uint16_t raw_temp = (data[0] << 8) | data[1];
    return convertRawTemperature(raw_temp);
}

float HDC1080::readHumidity()
{
    if (!initialized_) {
        ESP_LOGE(TAG, "HDC1080 no inicializado");
        return -999.0f;
    }

    // Iniciar conversión de humedad
    uint8_t hum_reg = HDC1080_HUMIDITY_REG;
    esp_err_t ret = i2c_bus_->write(device_handle_, hum_reg, nullptr, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando conversión de humedad: %s", esp_err_to_name(ret));
        return -999.0f;
    }
    
    // Esperar tiempo de conversión (6.35ms para humedad)
    vTaskDelay(pdMS_TO_TICKS(7));
    
    // Leer resultado
    uint8_t data[2];
    ret = i2c_bus_->readStream(device_handle_, data, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo humedad: %s", esp_err_to_name(ret));
        return -999.0f;
    }
    
    uint16_t raw_humidity = (data[0] << 8) | data[1];
    return convertRawHumidity(raw_humidity);
}

uint16_t HDC1080::getManufacturerID()
{
    return readRegister16(HDC1080_MANUFACTURER_ID, 10);
}

uint16_t HDC1080::getDeviceID()
{
    return readRegister16(HDC1080_DEVICE_ID, 10);
}

esp_err_t HDC1080::getSerialNumbers(uint16_t* serial_1, uint16_t* serial_2, uint16_t* serial_3)
{
    if (!serial_1 || !serial_2 || !serial_3) {
        return ESP_ERR_INVALID_ARG;
    }

    *serial_1 = readRegister16(HDC1080_SERIAL_ID_1, 10);
    *serial_2 = readRegister16(HDC1080_SERIAL_ID_2, 10);
    *serial_3 = readRegister16(HDC1080_SERIAL_ID_3, 10);
    
    if (*serial_1 == 0xFFFF || *serial_2 == 0xFFFF || *serial_3 == 0xFFFF) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t HDC1080::setHeater(bool enable)
{
    current_config_.heater_enable = enable;
    uint16_t config_value = buildConfigValue(current_config_);
    esp_err_t ret = writeRegister16(HDC1080_CONFIG_REG, config_value);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calentador %s", enable ? "habilitado" : "deshabilitado");
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    
    return ret;
}

esp_err_t HDC1080::softReset()
{
    uint16_t reset_config = 0x8000;  // Bit 15 = 1 para reset
    esp_err_t ret = writeRegister16(HDC1080_CONFIG_REG, reset_config);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Reset por software ejecutado");
        vTaskDelay(pdMS_TO_TICKS(50));  // Esperar tiempo de reset
        initialized_ = false;
    }
    
    return ret;
}

esp_err_t HDC1080::setResolution(bool temp_14bit, bool hum_14bit)
{
    current_config_.temp_resolution_14bit = temp_14bit;
    current_config_.hum_resolution_14bit = hum_14bit;
    
    uint16_t config_value = buildConfigValue(current_config_);
    esp_err_t ret = writeRegister16(HDC1080_CONFIG_REG, config_value);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Resolución configurada - Temp: %d-bit, Hum: %d-bit", 
                 temp_14bit ? 14 : 11, hum_14bit ? 14 : 11);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    
    return ret;
}

bool HDC1080::isDataValid(float temperature, float humidity)
{
    return (temperature >= HDC1080_TEMP_MIN && temperature <= HDC1080_TEMP_MAX) &&
           (humidity >= HDC1080_HUMIDITY_MIN && humidity <= HDC1080_HUMIDITY_MAX);
}

uint16_t HDC1080::readRegister16(uint8_t reg_address, int timeout_ms)
{
    // Para registros de solo lectura como IDs, usar el método directo
    uint8_t data[2];
    esp_err_t ret = i2c_bus_->read(device_handle_, reg_address, data, 2, timeout_ms * 1000);
    
    if (ret == ESP_OK) {
        return (data[0] << 8) | data[1];
    } else {
        ESP_LOGE(TAG, "Error leyendo registro 0x%02X: %s", reg_address, esp_err_to_name(ret));
        return 0xFFFF;
    }
}

esp_err_t HDC1080::writeRegister16(uint8_t reg_address, uint16_t value)
{
    uint8_t data[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return i2c_bus_->write(device_handle_, reg_address, data, 2);
}

float HDC1080::convertRawTemperature(uint16_t raw_value)
{
    return ((float)raw_value / 65536.0f) * 165.0f - 40.0f;
}

float HDC1080::convertRawHumidity(uint16_t raw_value)
{
    return ((float)raw_value / 65536.0f) * 100.0f;
}

uint16_t HDC1080::buildConfigValue(const hdc1080_config_t& config)
{
    uint16_t config_value = 0;
    
    if (config.sequential_mode) config_value |= (1 << 12);
    if (!config.temp_resolution_14bit) config_value |= (1 << 10);  // 0=14bit, 1=11bit
    if (!config.hum_resolution_14bit) config_value |= (1 << 8);    // 00=14bit, 01=11bit
    if (config.heater_enable) config_value |= (1 << 13);
    
    return config_value;
}