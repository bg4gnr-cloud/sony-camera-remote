#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    MOSAICO_WIFI_CONNECTING,
    MOSAICO_WIFI_CONNECTED,
    MOSAICO_WIFI_RECONNECTING,
} mosaico_wifi_state_t;

typedef enum {
    MOSAICO_LIVEVIEW_STARTING,
    MOSAICO_LIVEVIEW_ACTIVE,
    MOSAICO_LIVEVIEW_RETRYING,
} mosaico_liveview_state_t;

esp_err_t mosaico_display_init(void);
esp_err_t mosaico_display_render_jpeg(const uint8_t *jpeg, size_t jpeg_size);
void mosaico_display_set_wifi_state(mosaico_wifi_state_t state);
void mosaico_display_set_liveview_state(mosaico_liveview_state_t state);
void mosaico_display_notify_new_frame(void);
