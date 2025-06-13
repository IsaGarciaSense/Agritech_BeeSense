#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

// Configuración I2C
#define I2C_MASTER_SCL_IO           4    // Pin SCL (cambiar según tu board)
#define I2C_MASTER_SDA_IO           5    // Pin SDA (cambiar según tu board)
#define I2C_MASTER_NUM              I2C_NUM_0     // Puerto I2C número 0
#define I2C_MASTER_FREQ_HZ          100000 // Frecuencia 100kHz
#define I2C_MASTER_TX_BUF_DISABLE   0     // Buffer TX deshabilitado
#define I2C_MASTER_RX_BUF_DISABLE   0     // Buffer RX deshabilitado
#define I2C_MASTER_TIMEOUT_MS       1000  // Timeout en ms

static const char *TAG = "I2C_SCANNER";

/**
 * @brief Inicializar el maestro I2C
 */
static esp_err_t i2c_master_init(void)
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(i2c_master_port, conf.mode, 
                             I2C_MASTER_RX_BUF_DISABLE, 
                             I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Escanear dispositivos I2C
 */
static void i2c_scanner(void)
{
    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            
            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }
}

/**
 * @brief Tarea principal del scanner I2C
 */
void i2c_scanner_task(void *arg)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C inicializado exitosamente");
    
    ESP_LOGI(TAG, "Configuración I2C:");
    ESP_LOGI(TAG, "- SDA: GPIO%d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "- SCL: GPIO%d", I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "- Frecuencia: %d Hz", I2C_MASTER_FREQ_HZ);
    ESP_LOGI(TAG, "- Puerto: %d", I2C_MASTER_NUM);
    
    while (1) {
        ESP_LOGI(TAG, "Escaneando dispositivos I2C...");
        i2c_scanner();
        ESP_LOGI(TAG, "Escaneo completado");
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar 5 segundos
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Iniciando I2C Scanner");
    
    // Crear tarea del scanner
    xTaskCreate(i2c_scanner_task, "i2c_scanner_task", 4096, NULL, 10, NULL);
}