idf_component_register(
    SRCS "main.c" "sony_liveview.c" "mosaico_display.c" "shutter_button.c"
    INCLUDE_DIRS "."
    REQUIRES button efuse esp_driver_gpio esp_driver_jpeg esp_driver_spi esp_event
             esp_http_client esp_netif esp_timer esp_wifi esp_lcd esp_lcd_co5300
             esp_hw_support esp_lvgl_port lvgl nvs_flash
)
