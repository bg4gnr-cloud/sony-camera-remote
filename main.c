#include <stdbool.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "mosaico_display.h"
#include "shutter_button.h"
#include "sony_liveview.h"

static const char *TAG = "sonycam_wifi";
static TaskHandle_t s_wifi_reconnect_task;
static EventGroupHandle_t s_wifi_events;
#define WIFI_HAS_IP_BIT BIT0

static void wifi_reconnect_task(void *arg)
{
    (void)arg;
    TickType_t retry_delay = pdMS_TO_TICKS(CONFIG_SONYCAM_WIFI_RECONNECT_INITIAL_MS);
    TickType_t maximum_delay =
        pdMS_TO_TICKS(CONFIG_SONYCAM_WIFI_RECONNECT_MAX_MS);
    if (maximum_delay < retry_delay) {
        maximum_delay = retry_delay;
    }

    while (true) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while ((xEventGroupGetBits(s_wifi_events) & WIFI_HAS_IP_BIT) == 0) {
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
                ESP_LOGE(TAG, "Could not start Wi-Fi connection: %s", esp_err_to_name(err));
            }

            const TickType_t attempt_start = xTaskGetTickCount();
            while ((xEventGroupGetBits(s_wifi_events) & WIFI_HAS_IP_BIT) == 0) {
                const TickType_t elapsed = xTaskGetTickCount() - attempt_start;
                if (elapsed >= retry_delay) {
                    break;
                }
                (void)ulTaskNotifyTake(pdTRUE, retry_delay - elapsed);
            }
            if ((xEventGroupGetBits(s_wifi_events) & WIFI_HAS_IP_BIT) == 0 &&
                retry_delay < maximum_delay) {
                retry_delay = retry_delay > maximum_delay / 2
                                  ? maximum_delay
                                  : retry_delay * 2;
            }
        }
        retry_delay = pdMS_TO_TICKS(CONFIG_SONYCAM_WIFI_RECONNECT_INITIAL_MS);
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        mosaico_display_set_wifi_state(MOSAICO_WIFI_CONNECTING);
        xTaskNotifyGive(s_wifi_reconnect_task);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event =
            (const wifi_event_sta_disconnected_t *)event_data;
        xEventGroupClearBits(s_wifi_events, WIFI_HAS_IP_BIT);
        mosaico_display_set_wifi_state(MOSAICO_WIFI_RECONNECTING);
        ESP_LOGW(TAG, "Disconnected from camera hotspot (reason=%d); reconnecting",
                 event->reason);
        xTaskNotifyGive(s_wifi_reconnect_task);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        xEventGroupClearBits(s_wifi_events, WIFI_HAS_IP_BIT);
        mosaico_display_set_wifi_state(MOSAICO_WIFI_RECONNECTING);
        xTaskNotifyGive(s_wifi_reconnect_task);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        xEventGroupSetBits(s_wifi_events, WIFI_HAS_IP_BIT);
        mosaico_display_set_wifi_state(MOSAICO_WIFI_CONNECTED);
        mosaico_display_set_liveview_state(MOSAICO_LIVEVIEW_STARTING);
        ESP_LOGI(TAG, "Connected to camera hotspot; IP=" IPSTR, IP2STR(&event->ip_info.ip));
        xTaskNotifyGive(s_wifi_reconnect_task);
        esp_err_t err = sony_liveview_start();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Could not start liveview task: %s", esp_err_to_name(err));
        }
        err = shutter_button_init();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Could not initialize shutter button: %s", esp_err_to_name(err));
        }
    }
}

static void connect_to_camera_hotspot(void)
{
    const size_t ssid_len = strlen(CONFIG_SONYCAM_WIFI_SSID);
    const size_t password_len = strlen(CONFIG_SONYCAM_WIFI_PASSWORD);
    wifi_config_t wifi_config = {0};

    if (ssid_len == 0 || ssid_len > sizeof(wifi_config.sta.ssid)) {
        ESP_LOGE(TAG, "Configure a non-empty camera SSID of at most 32 bytes in menuconfig");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_SIZE);
    }
    if (password_len > sizeof(wifi_config.sta.password)) {
        ESP_LOGE(TAG, "Camera hotspot password must be at most 64 bytes");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_SIZE);
    }

    memcpy(wifi_config.sta.ssid, CONFIG_SONYCAM_WIFI_SSID, ssid_len);
    memcpy(wifi_config.sta.password, CONFIG_SONYCAM_WIFI_PASSWORD, password_len);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        ESP_LOGE(TAG, "Could not create Wi-Fi event group");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    BaseType_t task_result = xTaskCreate(wifi_reconnect_task, "wifi_reconnect",
                                         3072, NULL, 5, &s_wifi_reconnect_task);
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Could not create Wi-Fi reconnect task");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(mosaico_display_init());
    connect_to_camera_hotspot();
}
