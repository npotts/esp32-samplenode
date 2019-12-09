/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
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


