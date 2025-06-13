#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "smart_sensor_sense.hpp"
#include "HDC1080.hpp"

static const char *TAG = "HDC1080_EXAMPLE";

// Configuración I2C
#define I2C_SDA_PIN     GPIO_NUM_5
#define I2C_SCL_PIN     GPIO_NUM_4
#define I2C_PORT        I2C_NUM_0

// Variables globales
I2C* i2c_bus = nullptr;
HDC1080* hdc_sensor = nullptr;

void sensor_task(void *arg)
{
    // Crear instancia del bus I2C
    i2c_bus = new I2C(I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, true);
    
    // Inicializar el bus I2C
    esp_err_t ret = i2c_bus->init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando I2C: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Bus I2C inicializado correctamente");
    
    // Crear instancia del sensor HDC1080
    hdc_sensor = new HDC1080(i2c_bus);
    
    // Configuración personalizada (opcional)
    hdc1080_config_t config = {
        .sequential_mode = true,        // Leer temp y humedad juntos
        .temp_resolution_14bit = true,  // Máxima resolución temperatura
        .hum_resolution_14bit = true,   // Máxima resolución humedad
        .heater_enable = false          // Calentador deshabilitado
    };
    
    // Inicializar el sensor
    ret = hdc_sensor->init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando HDC1080: %s", esp_err_to_name(ret));
        return;
    }
    
    // Obtener información del dispositivo
    uint16_t serial_1, serial_2, serial_3;
    if (hdc_sensor->getSerialNumbers(&serial_1, &serial_2, &serial_3) == ESP_OK) {
        ESP_LOGI(TAG, "Números de serie: 0x%04X, 0x%04X, 0x%04X", serial_1, serial_2, serial_3);
    }
    
    ESP_LOGI(TAG, "Iniciando lecturas del sensor...");
    
    while (1) {
        // Método 1: Leer ambos valores (recomendado para modo secuencial)
        hdc1080_data_t data = hdc_sensor->readBoth();
        
        if (data.valid) {
            ESP_LOGI(TAG, "Temperatura: %.2f°C | Humedad: %.2f%%", 
                     data.temperature, data.humidity);
            
        } else {
            ESP_LOGE(TAG, "Datos inválidos del sensor");
        }
        
        /* Método 2: Leer valores individuales
        float temperature = hdc_sensor->readTemperature();
        vTaskDelay(pdMS_TO_TICKS(50));
        float humidity = hdc_sensor->readHumidity();
        
        if (temperature != -999.0f && humidity != -999.0f) {
            ESP_LOGI(TAG, "Individual - Temp: %.2f°C, Hum: %.2f%%", temperature, humidity);
        }
        */
        
        vTaskDelay(pdMS_TO_TICKS(3000)); // Leer cada 3 segundos
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "=== HDC1080 Library Example ===");
    ESP_LOGI(TAG, "Sensor de temperatura y humedad de alta precisión");
    
    // Crear tarea del sensor
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}