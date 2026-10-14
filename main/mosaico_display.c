#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/jpeg_decode.h"
#include "driver/spi_master.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_lcd_co5300.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "mosaico_display.h"
#include "sony_liveview.h"

#define LCD_WIDTH 480
#define LCD_HEIGHT 480
#define LCD_BITS_PER_PIXEL 16
#define LCD_DMA_LINES 8
#define LCD_MAX_TRANSFER_SIZE (LCD_WIDTH * LCD_DMA_LINES * LCD_BITS_PER_PIXEL / 8)
#define JPEG_MAX_SIZE (512U * 1024U)
#define JPEG_MAX_DIMENSION 2048U
#define DISPLAY_TASK_STACK_SIZE 8192U
#define DISPLAY_NOTIFY_FRAME (1U << 0)
#define LVGL_BUFFER_LINES 40
#define LVGL_STATUS_UPDATE_MS 500

#define MOSAICO_LCD_HOST SPI2_HOST
#define MOSAICO_LCD_CS GPIO_NUM_50
#define MOSAICO_LCD_DATA0 GPIO_NUM_36
#define MOSAICO_LCD_DATA1 GPIO_NUM_51
#define MOSAICO_LCD_DATA2 GPIO_NUM_35
#define MOSAICO_LCD_DATA3 GPIO_NUM_9
#define MOSAICO_LCD_POWER GPIO_NUM_60
#define MOSAICO_LCD_POWER_ON_LEVEL 0
#define MOSAICO_LCD_RESET_V1_0 GPIO_NUM_42
#define MOSAICO_LCD_RESET_V1_2 GPIO_NUM_44
#define MOSAICO_LCD_CLOCK_V1_0 GPIO_NUM_44
#define MOSAICO_LCD_CLOCK_V1_2 GPIO_NUM_42

static const char *TAG = "mosaico_display";

static const co5300_lcd_init_cmd_t s_init_commands[] = {
    {0x11, NULL, 0, 600},
    {0xFE, (uint8_t[]){0x20}, 1, 0},
    {0x19, (uint8_t[]){0x10}, 1, 0},
    {0x1C, (uint8_t[]){0xA0}, 1, 0},
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
    {0x63, (uint8_t[]){0xFF}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0xDF}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xDF}, 4, 0},
    {0x29, NULL, 0, 600},
};

static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_panel_io;
static jpeg_decoder_handle_t s_jpeg_decoder;
static uint8_t *s_jpeg_input;
static uint8_t *s_jpeg_output;
static size_t s_jpeg_output_capacity;
static uint16_t *s_scaled_frame;
static TaskHandle_t s_display_task;
static lv_display_t *s_lvgl_display;
static lv_obj_t *s_preview_image;
static lv_obj_t *s_wifi_label;
static lv_obj_t *s_liveview_label;
static lv_obj_t *s_fps_label;
static lv_image_dsc_t s_preview_image_descriptor;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static mosaico_wifi_state_t s_wifi_state = MOSAICO_WIFI_CONNECTING;
static mosaico_liveview_state_t s_liveview_state = MOSAICO_LIVEVIEW_STARTING;
static uint16_t s_preview_fps_tenths;
static uint32_t s_fps_sample_frames;
static int64_t s_last_fps_sample_us;
static int64_t s_last_rendered_frame_us;

static void refresh_status_ui(lv_timer_t *timer);

static void refresh_status_ui(lv_timer_t *timer)
{
    (void)timer;
    const int64_t now_us = esp_timer_get_time();
    mosaico_wifi_state_t wifi_state;
    mosaico_liveview_state_t liveview_state;
    uint16_t fps_tenths;

    portENTER_CRITICAL(&s_status_lock);
    if (s_last_rendered_frame_us == 0 ||
        now_us - s_last_rendered_frame_us >= 1000000) {
        s_preview_fps_tenths = 0;
        s_fps_sample_frames = 0;
        s_last_fps_sample_us = now_us;
    } else if (s_last_fps_sample_us != 0 &&
               now_us - s_last_fps_sample_us >= 1000000) {
        const uint64_t elapsed_us = (uint64_t)(now_us - s_last_fps_sample_us);
        const uint64_t fps =
            ((uint64_t)s_fps_sample_frames * 10000000ULL) / elapsed_us;
        s_preview_fps_tenths = fps > UINT16_MAX ? UINT16_MAX : (uint16_t)fps;
        s_fps_sample_frames = 0;
        s_last_fps_sample_us = now_us;
    }
    wifi_state = s_wifi_state;
    liveview_state = s_liveview_state;
    fps_tenths = s_preview_fps_tenths;
    portEXIT_CRITICAL(&s_status_lock);

    const char *wifi_text = "WIFI: CONNECTING";
    if (wifi_state == MOSAICO_WIFI_CONNECTED) {
        wifi_text = "WIFI: CONNECTED";
    } else if (wifi_state == MOSAICO_WIFI_RECONNECTING) {
        wifi_text = "WIFI: RECONNECTING";
    }

    const char *liveview_text = "LIVEVIEW: STARTING";
    if (liveview_state == MOSAICO_LIVEVIEW_ACTIVE) {
        liveview_text = "LIVEVIEW: ACTIVE";
    } else if (liveview_state == MOSAICO_LIVEVIEW_RETRYING) {
        liveview_text = "LIVEVIEW: RETRYING";
    }

    lv_label_set_text(s_wifi_label, wifi_text);
    lv_label_set_text(s_liveview_label, liveview_text);
    lv_label_set_text_fmt(s_fps_label, "PREVIEW: %u.%u FPS",
                          (unsigned)(fps_tenths / 10),
                          (unsigned)(fps_tenths % 10));
}

static esp_err_t configure_power_and_board_pins(gpio_num_t *lcd_clock, gpio_num_t *lcd_reset)
{
    uint16_t hardware_version = 0;
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_USER_DATA, &hardware_version,
                                                  sizeof(hardware_version) * 8U),
                        TAG, "read board hardware revision failed");

    if (hardware_version == 0x0100) {
        *lcd_clock = MOSAICO_LCD_CLOCK_V1_0;
        *lcd_reset = MOSAICO_LCD_RESET_V1_0;
    } else if (hardware_version == 0x0101 || hardware_version == 0x0102) {
        *lcd_clock = MOSAICO_LCD_CLOCK_V1_2;
        *lcd_reset = MOSAICO_LCD_RESET_V1_2;
    } else {
        ESP_LOGE(TAG, "Unsupported ESP-Mosaico board version 0x%04X", hardware_version);
        return ESP_ERR_NOT_SUPPORTED;
    }

    const gpio_config_t power_config = {
        .pin_bit_mask = 1ULL << MOSAICO_LCD_POWER,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&power_config), TAG, "configure display power failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(MOSAICO_LCD_POWER, MOSAICO_LCD_POWER_ON_LEVEL),
                        TAG, "enable display power failed");
    esp_rom_delay_us(100000);

    ESP_LOGI(TAG, "Board revision 0x%04X, LCD SCLK=%d RESET=%d",
             hardware_version, (int)*lcd_clock, (int)*lcd_reset);
    return ESP_OK;
}

static esp_err_t initialize_panel(void)
{
    gpio_num_t lcd_clock;
    gpio_num_t lcd_reset;
    ESP_RETURN_ON_ERROR(configure_power_and_board_pins(&lcd_clock, &lcd_reset),
                        TAG, "prepare ESP-Mosaico display");

    const spi_bus_config_t bus_config = CO5300_PANEL_BUS_QSPI_CONFIG(
        lcd_clock, MOSAICO_LCD_DATA0, MOSAICO_LCD_DATA1, MOSAICO_LCD_DATA2,
        MOSAICO_LCD_DATA3, LCD_MAX_TRANSFER_SIZE);
    ESP_RETURN_ON_ERROR(spi_bus_initialize(MOSAICO_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO),
                        TAG, "initialize CO5300 QSPI bus failed");

    esp_lcd_panel_io_spi_config_t io_config =
        CO5300_PANEL_IO_QSPI_CONFIG(MOSAICO_LCD_CS, NULL, NULL);
    io_config.flags.psram_dma_direct = false;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(
                            (esp_lcd_spi_bus_handle_t)MOSAICO_LCD_HOST, &io_config,
                            &s_panel_io),
                        TAG, "create CO5300 panel IO failed");

    const co5300_vendor_config_t vendor_config = {
        .init_cmds = s_init_commands,
        .init_cmds_size = sizeof(s_init_commands) / sizeof(s_init_commands[0]),
        .flags = {.use_qspi_interface = 1},
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = lcd_reset,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BITS_PER_PIXEL,
        .vendor_config = (void *)&vendor_config,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_co5300(s_panel_io, &panel_config, &s_panel),
                        TAG, "create CO5300 panel failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset CO5300 failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "initialize CO5300 failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true),
                        TAG, "turn on CO5300 failed");
    return ESP_OK;
}

static esp_err_t initialize_lvgl(void)
{
    const lvgl_port_cfg_t lvgl_config = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_config), TAG, "initialize LVGL port failed");

    lvgl_port_display_cfg_t display_config = {0};
    display_config.io_handle = s_panel_io;
    display_config.panel_handle = s_panel;
    display_config.buffer_size = LCD_WIDTH * LVGL_BUFFER_LINES;
    display_config.trans_size = LCD_WIDTH * LCD_DMA_LINES;
    display_config.double_buffer = true;
    display_config.hres = LCD_WIDTH;
    display_config.vres = LCD_HEIGHT;
    display_config.color_format = LV_COLOR_FORMAT_RGB565;
    display_config.flags.buff_spiram = true;
    s_lvgl_display = lvgl_port_add_disp(&display_config);
    if (s_lvgl_display == NULL) {
        ESP_LOGE(TAG, "Could not register CO5300 display with LVGL");
        return ESP_ERR_NO_MEM;
    }

    s_preview_image_descriptor = (lv_image_dsc_t) {
        .header = {
            .magic = LV_IMAGE_HEADER_MAGIC,
            .cf = LV_COLOR_FORMAT_RGB565,
            .w = LCD_WIDTH,
            .h = LCD_HEIGHT,
            .stride = LCD_WIDTH * sizeof(uint16_t),
        },
        .data_size = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t),
        .data = (const uint8_t *)s_scaled_frame,
    };

    if (!lvgl_port_lock(0)) {
        ESP_LOGE(TAG, "Could not lock LVGL during UI initialization");
        return ESP_FAIL;
    }
    lv_obj_t *screen = lv_display_get_screen_active(s_lvgl_display);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    s_preview_image = lv_image_create(screen);
    lv_image_set_src(s_preview_image, &s_preview_image_descriptor);
    lv_obj_set_pos(s_preview_image, 0, 0);

    lv_obj_t *status_bar = lv_obj_create(screen);
    lv_obj_set_size(status_bar, LCD_WIDTH, 52);
    lv_obj_align(status_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(status_bar, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(status_bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(status_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(status_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(status_bar, 6, LV_PART_MAIN);
    lv_obj_clear_flag(status_bar, LV_OBJ_FLAG_SCROLLABLE);

    s_wifi_label = lv_label_create(status_bar);
    lv_obj_set_pos(s_wifi_label, 0, 0);
    lv_obj_set_style_text_color(s_wifi_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    s_liveview_label = lv_label_create(status_bar);
    lv_obj_set_pos(s_liveview_label, 0, 22);
    lv_obj_set_style_text_color(s_liveview_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    s_fps_label = lv_label_create(status_bar);
    lv_obj_align(s_fps_label, LV_ALIGN_TOP_RIGHT, 0, 22);
    lv_obj_set_style_text_color(s_fps_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);

    (void)lv_timer_create(refresh_status_ui, LVGL_STATUS_UPDATE_MS, NULL);
    refresh_status_ui(NULL);
    lvgl_port_unlock();
    return ESP_OK;
}

static esp_err_t ensure_jpeg_output_buffer(size_t required_size)
{
    if (required_size <= s_jpeg_output_capacity) {
        return ESP_OK;
    }

    jpeg_decode_memory_alloc_cfg_t memory_config = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    size_t allocated_size = 0;
    uint8_t *new_buffer = jpeg_alloc_decoder_mem(required_size, &memory_config, &allocated_size);
    if (new_buffer == NULL) {
        ESP_LOGE(TAG, "Could not allocate %u bytes for JPEG decoder output",
                 (unsigned)required_size);
        return ESP_ERR_NO_MEM;
    }

    free(s_jpeg_output);
    s_jpeg_output = new_buffer;
    s_jpeg_output_capacity = allocated_size;
    return ESP_OK;
}

static uint32_t align_dimension(uint32_t value, uint32_t alignment)
{
    return (value + alignment - 1) / alignment * alignment;
}

static void scale_to_fit_letterbox(const uint16_t *source, uint32_t source_width,
                                   uint32_t source_height, uint32_t source_stride)
{
    memset(s_scaled_frame, 0, LCD_WIDTH * LCD_HEIGHT * sizeof(*s_scaled_frame));

    uint32_t destination_width;
    uint32_t destination_height;
    if (source_width >= source_height) {
        destination_width = LCD_WIDTH;
        destination_height = (uint32_t)(((uint64_t)source_height * LCD_WIDTH) / source_width);
    } else {
        destination_height = LCD_HEIGHT;
        destination_width = (uint32_t)(((uint64_t)source_width * LCD_HEIGHT) / source_height);
    }
    if (destination_width == 0) {
        destination_width = 1;
    }
    if (destination_height == 0) {
        destination_height = 1;
    }

    const uint32_t offset_x = (LCD_WIDTH - destination_width) / 2;
    const uint32_t offset_y = (LCD_HEIGHT - destination_height) / 2;
    for (uint32_t y = 0; y < destination_height; y++) {
        uint32_t source_y = (uint32_t)(((uint64_t)y * source_height) / destination_height);
        const uint16_t *source_row = source + source_y * source_stride;
        uint16_t *destination_row = s_scaled_frame + (offset_y + y) * LCD_WIDTH + offset_x;
        for (uint32_t x = 0; x < destination_width; x++) {
            uint32_t source_x = (uint32_t)(((uint64_t)x * source_width) / destination_width);
            destination_row[x] = source_row[source_x];
        }
    }
}

static void display_task(void *arg)
{
    (void)arg;
    while (true) {
        uint32_t notifications = 0;
        BaseType_t notified = xTaskNotifyWait(0, UINT32_MAX, &notifications, portMAX_DELAY);
        if (notified == pdTRUE && (notifications & DISPLAY_NOTIFY_FRAME) != 0) {
            size_t jpeg_size = 0;
            esp_err_t err = sony_liveview_copy_latest_frame(s_jpeg_input,
                                                            JPEG_MAX_SIZE,
                                                            &jpeg_size);
            if (err == ESP_OK) {
                err = mosaico_display_render_jpeg(s_jpeg_input, jpeg_size);
            }
            if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "Could not update AMOLED preview: %s", esp_err_to_name(err));
            }
        }
    }
}

esp_err_t mosaico_display_init(void)
{
    if (s_panel != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(initialize_panel(), TAG, "initialize ESP-Mosaico AMOLED");

    jpeg_decode_engine_cfg_t decoder_config = {
        .intr_priority = 0,
        .timeout_ms = 1000,
    };
    ESP_RETURN_ON_ERROR(jpeg_new_decoder_engine(&decoder_config, &s_jpeg_decoder),
                        TAG, "initialize hardware JPEG decoder failed");

    jpeg_decode_memory_alloc_cfg_t input_memory_config = {
        .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER,
    };
    size_t allocated_input_size = 0;
    s_jpeg_input = jpeg_alloc_decoder_mem(JPEG_MAX_SIZE, &input_memory_config,
                                          &allocated_input_size);
    if (s_jpeg_input == NULL || allocated_input_size < JPEG_MAX_SIZE) {
        ESP_LOGE(TAG, "Could not allocate JPEG input buffer in PSRAM");
        return ESP_ERR_NO_MEM;
    }

    s_scaled_frame = heap_caps_malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(*s_scaled_frame),
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_scaled_frame == NULL) {
        ESP_LOGE(TAG, "Could not allocate 480x480 display frame in PSRAM");
        return ESP_ERR_NO_MEM;
    }
    memset(s_scaled_frame, 0, LCD_WIDTH * LCD_HEIGHT * sizeof(*s_scaled_frame));
    ESP_RETURN_ON_ERROR(initialize_lvgl(), TAG, "initialize LVGL AMOLED UI");

    BaseType_t task_result = xTaskCreate(display_task, "mosaico_display",
                                         DISPLAY_TASK_STACK_SIZE, NULL, 4,
                                         &s_display_task);
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Could not create AMOLED rendering task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "CO5300 AMOLED ready: %dx%d; LVGL UI and hardware JPEG decoder enabled",
             LCD_WIDTH, LCD_HEIGHT);
    return ESP_OK;
}

esp_err_t mosaico_display_render_jpeg(const uint8_t *jpeg, size_t jpeg_size)
{
    if (s_panel == NULL || jpeg == NULL || jpeg_size == 0 || jpeg_size > JPEG_MAX_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (jpeg_size > UINT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (jpeg != s_jpeg_input) {
        memcpy(s_jpeg_input, jpeg, jpeg_size);
    }

    jpeg_decode_picture_info_t picture_info = {0};
    ESP_RETURN_ON_ERROR(jpeg_decoder_get_info(s_jpeg_input, jpeg_size, &picture_info),
                        TAG, "read JPEG image dimensions failed");
    if (picture_info.width == 0 || picture_info.height == 0 ||
        picture_info.width > JPEG_MAX_DIMENSION || picture_info.height > JPEG_MAX_DIMENSION) {
        ESP_LOGW(TAG, "Rejecting unsupported JPEG dimensions %ux%u",
                 (unsigned)picture_info.width, (unsigned)picture_info.height);
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t mcu_width = 8;
    uint32_t mcu_height = 8;
    if (picture_info.sample_method == JPEG_DOWN_SAMPLING_YUV420) {
        mcu_width = 16;
        mcu_height = 16;
    } else if (picture_info.sample_method == JPEG_DOWN_SAMPLING_YUV422) {
        mcu_width = 16;
    } else if (picture_info.sample_method != JPEG_DOWN_SAMPLING_YUV444 &&
               picture_info.sample_method != JPEG_DOWN_SAMPLING_GRAY) {
        ESP_LOGW(TAG, "Unsupported JPEG sampling method %d",
                 (int)picture_info.sample_method);
        return ESP_ERR_NOT_SUPPORTED;
    }
    uint32_t decoded_width = align_dimension(picture_info.width, mcu_width);
    uint32_t decoded_height = align_dimension(picture_info.height, mcu_height);
    size_t decoded_buffer_size = (size_t)decoded_width * decoded_height * sizeof(uint16_t);
    ESP_RETURN_ON_ERROR(ensure_jpeg_output_buffer(decoded_buffer_size),
                        TAG, "allocate hardware JPEG decoder output failed");

    const jpeg_decode_cfg_t decode_config = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_RGB,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    uint32_t decoded_size = 0;
    ESP_RETURN_ON_ERROR(jpeg_decoder_process(s_jpeg_decoder, &decode_config,
                                              s_jpeg_input, (uint32_t)jpeg_size,
                                              s_jpeg_output, (uint32_t)s_jpeg_output_capacity,
                                              &decoded_size),
                        TAG, "hardware JPEG decode failed");
    if (decoded_size < decoded_buffer_size) {
        ESP_LOGE(TAG, "JPEG decoder returned %u bytes, expected at least %u",
                 (unsigned)decoded_size, (unsigned)decoded_buffer_size);
        return ESP_ERR_INVALID_SIZE;
    }

    if (!lvgl_port_lock(0)) {
        ESP_LOGE(TAG, "Could not lock LVGL while updating the preview image");
        return ESP_FAIL;
    }
    scale_to_fit_letterbox((const uint16_t *)s_jpeg_output,
                           picture_info.width, picture_info.height, decoded_width);
    lv_obj_invalidate(s_preview_image);

    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_status_lock);
    if (s_last_fps_sample_us == 0) {
        s_last_fps_sample_us = now_us;
    }
    s_fps_sample_frames++;
    s_last_rendered_frame_us = now_us;
    portEXIT_CRITICAL(&s_status_lock);
    lvgl_port_unlock();
    return ESP_OK;
}

void mosaico_display_set_wifi_state(mosaico_wifi_state_t state)
{
    portENTER_CRITICAL(&s_status_lock);
    s_wifi_state = state;
    portEXIT_CRITICAL(&s_status_lock);
}

void mosaico_display_set_liveview_state(mosaico_liveview_state_t state)
{
    portENTER_CRITICAL(&s_status_lock);
    s_liveview_state = state;
    portEXIT_CRITICAL(&s_status_lock);
}

void mosaico_display_notify_new_frame(void)
{
    if (s_display_task != NULL) {
        xTaskNotify(s_display_task, DISPLAY_NOTIFY_FRAME, eSetBits);
    }
}
