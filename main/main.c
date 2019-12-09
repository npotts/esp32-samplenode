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

void tick(void *parameter) {
  uint32_t *ticks = (uint32_t *)(parameter);
  ESP_LOGI(TAG, "Ticking every %d", *ticks); 
  while(1){
    vTaskDelay(pdMS_TO_TICKS(*ticks));
    ESP_LOGI(TAG, "tick");
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

    // TaskHandle_t tsk;
    // uint32_t ticker = 2000;
    // xTaskCreate(tick,"io_task",2048, &ticker ,1,&tsk);
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


