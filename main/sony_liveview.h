#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t sony_liveview_start(void);
esp_err_t sony_liveview_focus_start(void);
esp_err_t sony_liveview_focus_cancel(void);
esp_err_t sony_liveview_take_picture(void);

/*
 * Copies the most recent JPEG into destination.
 * On ESP_ERR_INVALID_SIZE, frame_size contains the required buffer size.
 */
esp_err_t sony_liveview_copy_latest_frame(uint8_t *destination, size_t capacity,
                                          size_t *frame_size);
