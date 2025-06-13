#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "smart_sensor_sense.hpp"

static const char *TAG = "HDC1080";

// Configuración I2C
#define I2C_SDA_PIN     GPIO_NUM_5
#define I2C_SCL_PIN     GPIO_NUM_4
#define I2C_PORT        I2C_NUM_0
#define HDC1080_ADDRESS 0x40
#define I2C_FREQ_HZ     100000

// Registros del HDC1080
#define HDC1080_TEMP_REG        0x00
#define HDC1080_HUMIDITY_REG    0x01
#define HDC1080_CONFIG_REG      0x02
#define HDC1080_SERIAL_ID_1     0xFB
#define HDC1080_SERIAL_ID_2     0xFC
#define HDC1080_SERIAL_ID_3     0xFD
#define HDC1080_MANUFACTURER_ID 0xFE
#define HDC1080_DEVICE_ID       0xFF

// Variables globales
I2C* i2c_bus = nullptr;
i2c_master_dev_handle_t hdc1080_handle;

void hdc1080_init(void)
{
    // Configurar el sensor HDC1080
    // Bit 12: Modo de adquisición (0 = temperatura o humedad, 1 = ambas secuencialmente)
    // Bit 11: Resolución temperatura (0 = 14bit, 1 = 11bit)  
    // Bit 10-8: Resolución humedad (00 = 14bit, 01 = 11bit, 10 = 8bit)
    // Bit 7: Voltaje de la batería (solo lectura)
    // Bit 6: Modo de calentamiento (1 = habilitado)
    // Bit 15: Reset de software (1 = reset)
    
    uint8_t config_data[2] = {0x10, 0x00}; // Modo secuencial activado, resolución máxima
    esp_err_t ret = i2c_bus->write(hdc1080_handle, HDC1080_CONFIG_REG, config_data, 2);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "HDC1080 configurado correctamente");
    } else {
        ESP_LOGE(TAG, "Error configurando HDC1080: %s", esp_err_to_name(ret));
    }
    
    vTaskDelay(pdMS_TO_TICKS(15)); // Esperar 15ms para que se aplique la configuración
}

void hdc1080_read_id(void)
{
    uint8_t manufacturer_id[2];
    uint8_t device_id[2];
    
    esp_err_t ret = i2c_bus->read(hdc1080_handle, HDC1080_MANUFACTURER_ID, manufacturer_id, 2, 1000);
    if (ret == ESP_OK) {
        uint16_t manuf_id = (manufacturer_id[0] << 8) | manufacturer_id[1];
        ESP_LOGI(TAG, "Manufacturer ID: 0x%04X (esperado: 0x5449)", manuf_id);
    }
    
    ret = i2c_bus->read(hdc1080_handle, HDC1080_DEVICE_ID, device_id, 2, 1000);
    if (ret == ESP_OK) {
        uint16_t dev_id = (device_id[0] << 8) | device_id[1];
        ESP_LOGI(TAG, "Device ID: 0x%04X (esperado: 0x1050)", dev_id);
    }
}

float hdc1080_read_temperature(void)
{
    uint8_t temp_data[2];
    
    // Leer temperatura
    esp_err_t ret = i2c_bus->read(hdc1080_handle, HDC1080_TEMP_REG, temp_data, 2, 15000); // 15ms timeout
    
    if (ret == ESP_OK) {
        uint16_t raw_temp = (temp_data[0] << 8) | temp_data[1];
        float temperature = ((float)raw_temp / 65536.0) * 165.0 - 40.0;
        return temperature;
    } else {
        ESP_LOGE(TAG, "Error leyendo temperatura: %s", esp_err_to_name(ret));
        return -999.0; // Valor de error
    }
}

float hdc1080_read_humidity(void)
{
    uint8_t humidity_data[2];
    
    // Leer humedad
    esp_err_t ret = i2c_bus->read(hdc1080_handle, HDC1080_HUMIDITY_REG, humidity_data, 2, 15000); // 15ms timeout
    
    if (ret == ESP_OK) {
        uint16_t raw_humidity = (humidity_data[0] << 8) | humidity_data[1];
        float humidity = ((float)raw_humidity / 65536.0) * 100.0;
        return humidity;
    } else {
        ESP_LOGE(TAG, "Error leyendo humedad: %s", esp_err_to_name(ret));
        return -999.0; // Valor de error
    }
}

void hdc1080_read_both(float* temperature, float* humidity)
{
    uint8_t data[4];
    
    // En modo secuencial, al leer temperatura se inician ambas mediciones
    esp_err_t ret = i2c_bus->read(hdc1080_handle, HDC1080_TEMP_REG, data, 4, 20000); // 20ms timeout
    
    if (ret == ESP_OK) {
        // Primeros 2 bytes: temperatura
        uint16_t raw_temp = (data[0] << 8) | data[1];
        *temperature = ((float)raw_temp / 65536.0) * 165.0 - 40.0;
        
        // Siguientes 2 bytes: humedad
        uint16_t raw_humidity = (data[2] << 8) | data[3];
        *humidity = ((float)raw_humidity / 65536.0) * 100.0;
    } else {
        ESP_LOGE(TAG, "Error leyendo temperatura y humedad: %s", esp_err_to_name(ret));
        *temperature = -999.0;
        *humidity = -999.0;
    }
}

void hdc1080_task(void *arg)
{
    // Crear instancia del bus I2C
    i2c_bus = new I2C(I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, true);
    
    // Inicializar el bus I2C
    esp_err_t ret = i2c_bus->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando I2C: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C inicializado correctamente");
    
    // Agregar dispositivo HDC1080 al bus
    ret = i2c_bus->addDevice(&hdc1080_handle, HDC1080_ADDRESS, I2C_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error agregando HDC1080: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "HDC1080 agregado al bus en dirección 0x%02X", HDC1080_ADDRESS);
    
    // Leer IDs del dispositivo para verificar comunicación
    hdc1080_read_id();
    
    // Configurar el sensor
    hdc1080_init();
    
    while (1) {
        float temperature, humidity;
        
        // Método 1: Leer ambos valores en una sola operación (recomendado)
        hdc1080_read_both(&temperature, &humidity);
        
        if (temperature != -999.0 && humidity != -999.0) {
            ESP_LOGI(TAG, "Temperatura: %.2f°C | Humedad: %.2f%%", temperature, humidity);
            
            // Verificar rangos válidos
            if (temperature < -40.0 || temperature > 125.0) {
                ESP_LOGW(TAG, "Temperatura fuera de rango!");
            }
            if (humidity < 0.0 || humidity > 100.0) {
                ESP_LOGW(TAG, "Humedad fuera de rango!");
            }
        }
        
        /* Método 2: Leer por separado (alternativo)
        float temp = hdc1080_read_temperature();
        vTaskDelay(pdMS_TO_TICKS(20)); // Esperar entre lecturas
        float hum = hdc1080_read_humidity();
        
        if (temp != -999.0 && hum != -999.0) {
            ESP_LOGI(TAG, "Temp: %.2f°C | Hum: %.2f%% (separado)", temp, hum);
        }
        */
        
        vTaskDelay(pdMS_TO_TICKS(2000)); // Leer cada 2 segundos
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Iniciando lectura del sensor HDC1080");
    ESP_LOGI(TAG, "Sensor de temperatura y humedad");
    
    // Crear tarea principal
    xTaskCreate(hdc1080_task, "hdc1080_task", 4096, NULL, 5, NULL);
}