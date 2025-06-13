// /* Pin out del mini brain: 
// REED: GPIO17
// LED: GPIO38
// HDC1080: GPIO5 (SDA), GPIO4 (SCL) 
// */
#include "HDC1080.hpp"
#include "smart_sensor_sense.hpp"

extern "C" void app_main() {
    I2C i2c(I2C_NUM_0, GPIO_NUM_5, GPIO_NUM_4, 100000, true);
    i2c.init();

    HDC1080 hdc(&i2c);
    hdc.init();

    while (true) {
        float temp = 0, hum = 0;
        if (hdc.readTemperature(&temp) == ESP_OK &&
            hdc.readHumidity(&hum) == ESP_OK) {
            printf("T: %.2f °C, H: %.2f %%\n", temp, hum);
        } else {
            printf("Error leyendo HDC1080\n");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
