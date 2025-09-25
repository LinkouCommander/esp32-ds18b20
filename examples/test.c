#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "ds18b20.h"

int pin = 4; // GPIO pin where the DS18B20 is connected

void TestTask(void *pvParameters) {
    ds18b20_dev_t *dev;
    ds18b20_init(dev, (gpio_num_t)pin);

    while(1) {
        ds18b20_search_reset(dev);
        ds18b20_requestTemperatures(dev);
        
        DeviceAddress newaddr;
        while((ds18b20_search(dev, newaddr)) > 0) {
            if (ds18b20_validFamily(newaddr)) {
                float tempC = ds18b20_getTempC(dev, newaddr);
                ESP_LOGI("DS18B20", "Temperature: %.2f C", tempC);
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}

void app_main(void) {
    xTaskCreatePinnedToCore(TestTask, "TestTask", 3072, NULL, 5, NULL, 1);
}