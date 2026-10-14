#include <stdint.h>

#include "button_gpio.h"
#include "iot_button.h"
#include "esp_check.h"
#include "esp_log.h"

#include "shutter_button.h"
#include "sony_liveview.h"

#define SHUTTER_BUTTON_GPIO 7
#define SHUTTER_BUTTON_ACTIVE_LEVEL 0
#define SHUTTER_FULL_PRESS_TIME_MS 600

static const char *TAG = "shutter_button";
static button_handle_t s_button;

static void shutter_button_event(void *button_handle, void *user_data)
{
    (void)user_data;
    const button_event_t event = iot_button_get_event(button_handle);
    esp_err_t err;

    switch (event) {
    case BUTTON_PRESS_DOWN:
        ESP_LOGI(TAG, "Half-press: starting autofocus");
        err = sony_liveview_focus_start();
        break;
    case BUTTON_LONG_PRESS_START:
        ESP_LOGI(TAG, "Full-press: taking picture");
        err = sony_liveview_take_picture();
        break;
    case BUTTON_PRESS_UP:
        ESP_LOGI(TAG, "Button released: canceling half-press");
        err = sony_liveview_focus_cancel();
        break;
    default:
        return;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not queue shutter action: %s", esp_err_to_name(err));
    }
}

esp_err_t shutter_button_init(void)
{
    if (s_button != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const button_config_t button_config = {
        .long_press_time = SHUTTER_FULL_PRESS_TIME_MS,
    };
    const button_gpio_config_t gpio_config = {
        .gpio_num = SHUTTER_BUTTON_GPIO,
        .active_level = SHUTTER_BUTTON_ACTIVE_LEVEL,
        .enable_power_save = true,
        .disable_pull = false,
    };
    ESP_RETURN_ON_ERROR(iot_button_new_gpio_device(&button_config, &gpio_config, &s_button),
                        TAG, "create ESP-Mosaico AI shutter button failed");
    ESP_RETURN_ON_ERROR(iot_button_register_cb(s_button, BUTTON_PRESS_DOWN, NULL,
                                               shutter_button_event, NULL),
                        TAG, "register half-press event failed");
    ESP_RETURN_ON_ERROR(iot_button_register_cb(s_button, BUTTON_LONG_PRESS_START, NULL,
                                               shutter_button_event, NULL),
                        TAG, "register full-press event failed");
    ESP_RETURN_ON_ERROR(iot_button_register_cb(s_button, BUTTON_PRESS_UP, NULL,
                                               shutter_button_event, NULL),
                        TAG, "register shutter release event failed");

    ESP_LOGI(TAG, "AI button ready: press=focus, hold %u ms=take photo",
             SHUTTER_FULL_PRESS_TIME_MS);
    return ESP_OK;
}
