/* Dual use Weather and Splot Phase Energy Meter

Intended to be used with CircuitSetup.us Split Phase Energy Meter and some extra tacked on sensors.

The MIT License (MIT)
Copyright (c) 2019 npotts
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.  
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_task_wdt.h"


#include "wifi-cfg.h"
#include "wxsensors.h"
#include "mqtt.client.h"


static const char *TAG = "main";

void heartbeat(void *parameter) {
  ESP_LOGI(TAG, "Heartbeat every %d", CONFIG_MQTT_TOPIC_HEARTBEAT_PERIOD);
  char *buf = malloc(16);
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(CONFIG_MQTT_TOPIC_HEARTBEAT_PERIOD));
    int i = snprintf(buf, 16, "%d", esp_log_timestamp());
    if (mqtt_publish_msg(CONFIG_MQTT_TOPIC_HEARTBEAT, buf, i) == -1) {
      ESP_LOGE(TAG, "Unable to publish heartbeat");
    }
  }
}

void setup() {
  //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "portTICK_PERIOD_MS = %d", portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wxstation_init();
    wifi_init_sta();
    mqtt_client_init();
    xTaskCreate(heartbeat,"heartbeat",2048, 0 ,1, 0);
}

void app_main(void) {
    setup();


    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    for(;;) {
        vTaskDelay(10);
    }
}


