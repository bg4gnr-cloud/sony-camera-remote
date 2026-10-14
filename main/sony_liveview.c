#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "mosaico_display.h"
#include "sony_liveview.h"

#define MAX_JPEG_SIZE (512 * 1024)
#define LIVEVIEW_RX_SIZE (MAX_JPEG_SIZE + 1024)
#define HTTP_READ_SIZE 2048
#define API_RESPONSE_SIZE 1024
#define LIVEVIEW_TASK_STACK_SIZE 8192
#define CAMERA_COMMAND_TASK_STACK_SIZE 4096
#define CAMERA_COMMAND_QUEUE_LENGTH 8

static const char *TAG = "sony_liveview";
static const char *API_URL = CONFIG_SONYCAM_CAMERA_API_URL;

typedef enum {
    CAMERA_COMMAND_FOCUS_START,
    CAMERA_COMMAND_FOCUS_CANCEL,
    CAMERA_COMMAND_TAKE_PICTURE,
} camera_command_t;

typedef struct {
    char body[API_RESPONSE_SIZE];
    size_t length;
    bool overflow;
} api_response_t;

typedef struct {
    const char *current;
    const char *end;
} json_cursor_t;

static SemaphoreHandle_t s_frame_mutex;
static uint8_t *s_latest_frame;
static size_t s_latest_frame_size;
static TaskHandle_t s_liveview_task;
static TaskHandle_t s_camera_command_task;
static QueueHandle_t s_camera_command_queue;
static SemaphoreHandle_t s_api_mutex;

static void skip_json_whitespace(json_cursor_t *cursor)
{
    while (cursor->current < cursor->end &&
           (*cursor->current == ' ' || *cursor->current == '\t' ||
            *cursor->current == '\r' || *cursor->current == '\n')) {
        cursor->current++;
    }
}

static bool parse_json_string(json_cursor_t *cursor, char *output, size_t output_size)
{
    if (cursor->current >= cursor->end || *cursor->current++ != '"') {
        return false;
    }

    size_t written = 0;
    bool overflow = false;
    while (cursor->current < cursor->end) {
        unsigned char character = (unsigned char)*cursor->current++;
        if (character == '"') {
            if (output != NULL) {
                if (output_size == 0 || overflow) {
                    return false;
                }
                output[written] = '\0';
            }
            return true;
        }
        if (character < 0x20) {
            return false;
        }

        if (character == '\\') {
            if (cursor->current >= cursor->end) {
                return false;
            }
            character = (unsigned char)*cursor->current++;
            switch (character) {
            case '"':
            case '\\':
            case '/':
                break;
            case 'b':
                character = '\b';
                break;
            case 'f':
                character = '\f';
                break;
            case 'n':
                character = '\n';
                break;
            case 'r':
                character = '\r';
                break;
            case 't':
                character = '\t';
                break;
            case 'u': {
                if (cursor->end - cursor->current < 4) {
                    return false;
                }
                unsigned int codepoint = 0;
                for (int i = 0; i < 4; i++) {
                    unsigned char digit = (unsigned char)*cursor->current++;
                    codepoint <<= 4;
                    if (digit >= '0' && digit <= '9') {
                        codepoint |= digit - '0';
                    } else if (digit >= 'a' && digit <= 'f') {
                        codepoint |= digit - 'a' + 10;
                    } else if (digit >= 'A' && digit <= 'F') {
                        codepoint |= digit - 'A' + 10;
                    } else {
                        return false;
                    }
                }
                if (codepoint > 0x7F) {
                    return false;
                }
                character = (unsigned char)codepoint;
                break;
            }
            default:
                return false;
            }
        }

        if (output != NULL) {
            if (written + 1 >= output_size) {
                overflow = true;
            } else {
                output[written++] = (char)character;
            }
        }
    }
    return false;
}

static bool skip_json_value(json_cursor_t *cursor, unsigned int depth)
{
    if (depth > 16) {
        return false;
    }
    skip_json_whitespace(cursor);
    if (cursor->current >= cursor->end) {
        return false;
    }

    if (*cursor->current == '"') {
        return parse_json_string(cursor, NULL, 0);
    }
    if (*cursor->current == '{') {
        cursor->current++;
        skip_json_whitespace(cursor);
        if (cursor->current < cursor->end && *cursor->current == '}') {
            cursor->current++;
            return true;
        }
        while (true) {
            skip_json_whitespace(cursor);
            if (!parse_json_string(cursor, NULL, 0)) {
                return false;
            }
            skip_json_whitespace(cursor);
            if (cursor->current >= cursor->end || *cursor->current++ != ':') {
                return false;
            }
            if (!skip_json_value(cursor, depth + 1)) {
                return false;
            }
            skip_json_whitespace(cursor);
            if (cursor->current >= cursor->end) {
                return false;
            }
            char delimiter = *cursor->current++;
            if (delimiter == '}') {
                return true;
            }
            if (delimiter != ',') {
                return false;
            }
        }
    }
    if (*cursor->current == '[') {
        cursor->current++;
        skip_json_whitespace(cursor);
        if (cursor->current < cursor->end && *cursor->current == ']') {
            cursor->current++;
            return true;
        }
        while (true) {
            if (!skip_json_value(cursor, depth + 1)) {
                return false;
            }
            skip_json_whitespace(cursor);
            if (cursor->current >= cursor->end) {
                return false;
            }
            char delimiter = *cursor->current++;
            if (delimiter == ']') {
                return true;
            }
            if (delimiter != ',') {
                return false;
            }
        }
    }

    const char *value_start = cursor->current;
    while (cursor->current < cursor->end && *cursor->current != ',' &&
           *cursor->current != ']' && *cursor->current != '}' &&
           *cursor->current != ' ' && *cursor->current != '\t' &&
           *cursor->current != '\r' && *cursor->current != '\n') {
        cursor->current++;
    }
    return cursor->current > value_start;
}

static bool parse_result_array(json_cursor_t *cursor, char *url, size_t url_size)
{
    skip_json_whitespace(cursor);
    if (cursor->current >= cursor->end || *cursor->current++ != '[') {
        return false;
    }
    skip_json_whitespace(cursor);
    if (cursor->current < cursor->end && *cursor->current == ']') {
        cursor->current++;
        return false;
    }

    bool has_url = parse_json_string(cursor, url, url_size);
    if (!has_url && !skip_json_value(cursor, 1)) {
        return false;
    }
    skip_json_whitespace(cursor);
    while (cursor->current < cursor->end && *cursor->current == ',') {
        cursor->current++;
        if (!skip_json_value(cursor, 1)) {
            return false;
        }
        skip_json_whitespace(cursor);
    }
    if (cursor->current >= cursor->end || *cursor->current++ != ']') {
        return false;
    }
    return has_url;
}

static esp_err_t parse_api_response(const char *body, size_t body_length,
                                    char **liveview_url)
{
    json_cursor_t cursor = {.current = body, .end = body + body_length};
    char url[512] = {0};
    bool found_result = false;
    bool api_error = false;

    skip_json_whitespace(&cursor);
    if (cursor.current >= cursor.end || *cursor.current++ != '{') {
        return ESP_ERR_INVALID_RESPONSE;
    }
    skip_json_whitespace(&cursor);
    while (cursor.current < cursor.end && *cursor.current != '}') {
        char key[16];
        if (!parse_json_string(&cursor, key, sizeof(key))) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        skip_json_whitespace(&cursor);
        if (cursor.current >= cursor.end || *cursor.current++ != ':') {
            return ESP_ERR_INVALID_RESPONSE;
        }
        skip_json_whitespace(&cursor);

        if (strcmp(key, "error") == 0) {
            api_error = cursor.end - cursor.current < 4 ||
                        strncmp(cursor.current, "null", 4) != 0;
            if (!skip_json_value(&cursor, 1)) {
                return ESP_ERR_INVALID_RESPONSE;
            }
        } else if (strcmp(key, "result") == 0 && liveview_url != NULL) {
            found_result = parse_result_array(&cursor, url, sizeof(url));
            if (!found_result && cursor.current < cursor.end &&
                *(cursor.current - 1) != ']') {
                return ESP_ERR_INVALID_RESPONSE;
            }
        } else if (!skip_json_value(&cursor, 1)) {
            return ESP_ERR_INVALID_RESPONSE;
        }

        skip_json_whitespace(&cursor);
        if (cursor.current >= cursor.end) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        char delimiter = *cursor.current++;
        if (delimiter == '}') {
            break;
        }
        if (delimiter != ',') {
            return ESP_ERR_INVALID_RESPONSE;
        }
        skip_json_whitespace(&cursor);
    }

    skip_json_whitespace(&cursor);
    if (cursor.current != cursor.end || api_error) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (liveview_url != NULL) {
        if (!found_result || url[0] == '\0') {
            return ESP_ERR_INVALID_RESPONSE;
        }
        *liveview_url = strdup(url);
        if (*liveview_url == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

static esp_err_t api_event_handler(esp_http_client_event_t *event)
{
    api_response_t *response = event->user_data;

    if (event->event_id != HTTP_EVENT_ON_DATA || response == NULL) {
        return ESP_OK;
    }

    if ((size_t)event->data_len > sizeof(response->body) - response->length - 1) {
        response->overflow = true;
        return ESP_FAIL;
    }

    memcpy(response->body + response->length, event->data, event->data_len);
    response->length += event->data_len;
    response->body[response->length] = '\0';
    return ESP_OK;
}

static esp_err_t call_camera_api_unlocked(const char *method, char **liveview_url)
{
    char request[160];
    api_response_t response = {0};
    int request_length = snprintf(request, sizeof(request),
                                  "{\"method\":\"%s\",\"params\":[],\"id\":1,"
                                  "\"version\":\"1.0\"}",
                                  method);
    if (request_length < 0 || (size_t)request_length >= sizeof(request)) {
        return ESP_ERR_INVALID_SIZE;
    }

    const esp_http_client_config_t config = {
        .url = API_URL,
        .event_handler = api_event_handler,
        .user_data = &response,
        .timeout_ms = 10000,
        .buffer_size = 1024,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_set_method(client, HTTP_METHOD_POST);
    if (err == ESP_OK) {
        err = esp_http_client_set_header(client, "Content-Type", "application/json");
    }
    if (err == ESP_OK) {
        err = esp_http_client_set_post_field(client, request, request_length);
    }
    if (err == ESP_OK) {
        err = esp_http_client_perform(client);
    }
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status < 200 || status >= 300) {
            ESP_LOGE(TAG, "%s returned HTTP %d", method, status);
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        if (response.overflow) {
            ESP_LOGE(TAG, "%s response exceeded %d bytes", method, API_RESPONSE_SIZE - 1);
        } else if (response.length > 0) {
            ESP_LOGE(TAG, "%s failed: %s (response: %s)", method,
                     esp_err_to_name(err), response.body);
        } else {
            ESP_LOGE(TAG, "%s failed: %s", method, esp_err_to_name(err));
        }
        return err;
    }

    err = parse_api_response(response.body, response.length, liveview_url);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s returned invalid or unsuccessful JSON-RPC response: %s",
                 method, response.body);
    }
    return err;
}

static esp_err_t call_camera_api(const char *method, char **liveview_url)
{
    if (s_api_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_api_mutex, portMAX_DELAY);
    esp_err_t err = call_camera_api_unlocked(method, liveview_url);
    xSemaphoreGive(s_api_mutex);
    return err;
}

static void camera_command_task(void *arg)
{
    (void)arg;

    while (true) {
        camera_command_t command;
        if (xQueueReceive(s_camera_command_queue, &command, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        const char *method = NULL;
        switch (command) {
        case CAMERA_COMMAND_FOCUS_START:
            method = "actHalfPressShutter";
            break;
        case CAMERA_COMMAND_FOCUS_CANCEL:
            method = "cancelHalfPressShutter";
            break;
        case CAMERA_COMMAND_TAKE_PICTURE:
            method = "actTakePicture";
            break;
        default:
            ESP_LOGE(TAG, "Unknown camera command %d", (int)command);
            continue;
        }

        esp_err_t err = call_camera_api(method, NULL);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Camera command %s failed: %s", method, esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Camera command %s succeeded", method);
        }
    }
}

static esp_err_t enqueue_camera_command(camera_command_t command)
{
    if (s_camera_command_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(s_camera_command_queue, &command, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Camera command queue is full; dropping command %d", (int)command);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t sony_liveview_focus_start(void)
{
    return enqueue_camera_command(CAMERA_COMMAND_FOCUS_START);
}

esp_err_t sony_liveview_focus_cancel(void)
{
    return enqueue_camera_command(CAMERA_COMMAND_FOCUS_CANCEL);
}

esp_err_t sony_liveview_take_picture(void)
{
    return enqueue_camera_command(CAMERA_COMMAND_TAKE_PICTURE);
}

static bool save_latest_frame(const uint8_t *jpeg, size_t jpeg_size)
{
    if (jpeg_size < 4 || jpeg[0] != 0xFF || jpeg[1] != 0xD8 ||
        jpeg[jpeg_size - 2] != 0xFF || jpeg[jpeg_size - 1] != 0xD9) {
        ESP_LOGW(TAG, "Ignoring liveview payload without JPEG SOI/EOI markers");
        return false;
    }

    uint8_t *frame = malloc(jpeg_size);
    if (frame == NULL) {
        ESP_LOGE(TAG, "Could not allocate %u bytes for the latest JPEG",
                 (unsigned)jpeg_size);
        return false;
    }
    memcpy(frame, jpeg, jpeg_size);

    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    uint8_t *previous_frame = s_latest_frame;
    s_latest_frame = frame;
    s_latest_frame_size = jpeg_size;
    xSemaphoreGive(s_frame_mutex);
    free(previous_frame);

    mosaico_display_notify_new_frame();
    return true;
}

static size_t consume_stream_data(uint8_t *buffer, size_t *buffer_size)
{
    size_t frames_received = 0;

    while (*buffer_size >= 8) {
        size_t header_offset = 0;
        while (header_offset + 1 < *buffer_size &&
               !(buffer[header_offset] == 0xFF && buffer[header_offset + 1] == 0x01)) {
            header_offset++;
        }

        if (header_offset + 1 >= *buffer_size) {
            bool keep_last_byte = buffer[*buffer_size - 1] == 0xFF;
            if (keep_last_byte) {
                buffer[0] = 0xFF;
            }
            *buffer_size = keep_last_byte ? 1 : 0;
            return frames_received;
        }

        if (header_offset > 0) {
            memmove(buffer, buffer + header_offset, *buffer_size - header_offset);
            *buffer_size -= header_offset;
        }

        size_t jpeg_size = ((size_t)buffer[4] << 16) |
                           ((size_t)buffer[5] << 8) |
                           (size_t)buffer[6];
        size_t padding_size = buffer[7];
        if (jpeg_size == 0 || jpeg_size > MAX_JPEG_SIZE) {
            ESP_LOGW(TAG, "Discarding invalid liveview packet size %u",
                     (unsigned)jpeg_size);
            memmove(buffer, buffer + 1, --(*buffer_size));
            continue;
        }

        size_t packet_size = 8 + jpeg_size + padding_size;
        if (*buffer_size < packet_size) {
            return frames_received;
        }

        if (save_latest_frame(buffer + 8, jpeg_size)) {
            frames_received++;
        }
        memmove(buffer, buffer + packet_size, *buffer_size - packet_size);
        *buffer_size -= packet_size;
    }
    return frames_received;
}

static esp_err_t read_liveview_stream(const char *url)
{
    uint8_t *buffer = malloc(LIVEVIEW_RX_SIZE);
    uint8_t *read_buffer = malloc(HTTP_READ_SIZE);
    if (buffer == NULL || read_buffer == NULL) {
        free(buffer);
        free(read_buffer);
        return ESP_ERR_NO_MEM;
    }

    const esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 10000,
        .buffer_size = HTTP_READ_SIZE,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        free(buffer);
        free(read_buffer);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        (void)esp_http_client_fetch_headers(client);
        int status = esp_http_client_get_status_code(client);
        if (status < 200 || status >= 300) {
            ESP_LOGE(TAG, "Liveview stream returned HTTP %d", status);
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }

    if (err == ESP_OK) {
        mosaico_display_set_liveview_state(MOSAICO_LIVEVIEW_ACTIVE);
        ESP_LOGI(TAG, "Receiving liveview JPEG frames");
        size_t buffered = 0;
        uint32_t frame_count = 0;
        TickType_t last_report = xTaskGetTickCount();
        while (true) {
            int received = esp_http_client_read(client, (char *)read_buffer, HTTP_READ_SIZE);
            if (received <= 0) {
                err = received == 0 ? ESP_ERR_INVALID_STATE : ESP_FAIL;
                break;
            }

            if (buffered + (size_t)received > LIVEVIEW_RX_SIZE) {
                frame_count += consume_stream_data(buffer, &buffered);
                if (buffered + (size_t)received > LIVEVIEW_RX_SIZE) {
                    ESP_LOGE(TAG, "Liveview packet exceeds the receive buffer limit");
                    err = ESP_ERR_INVALID_SIZE;
                    break;
                }
            }
            memcpy(buffer + buffered, read_buffer, (size_t)received);
            buffered += (size_t)received;

            frame_count += consume_stream_data(buffer, &buffered);
            xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
            size_t latest_frame_size = s_latest_frame_size;
            xSemaphoreGive(s_frame_mutex);

            TickType_t now = xTaskGetTickCount();
            if (now - last_report >= pdMS_TO_TICKS(2000)) {
                ESP_LOGI(TAG, "Liveview active: %u JPEG frames received; latest is %u bytes",
                         (unsigned)frame_count, (unsigned)latest_frame_size);
                frame_count = 0;
                last_report = now;
            }
        }
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(buffer);
    free(read_buffer);
    return err;
}

static void liveview_task(void *arg)
{
    (void)arg;
    bool attempted_liveview = false;

    while (true) {
        char *stream_url = NULL;
        mosaico_display_set_liveview_state(attempted_liveview
                                               ? MOSAICO_LIVEVIEW_RETRYING
                                               : MOSAICO_LIVEVIEW_STARTING);
        esp_err_t err = call_camera_api("startLiveview", &stream_url);
        attempted_liveview = true;
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "startLiveview succeeded: %s", stream_url);
            err = read_liveview_stream(stream_url);
            free(stream_url);
            ESP_LOGW(TAG, "Liveview stream ended: %s", esp_err_to_name(err));
            esp_err_t stop_err = call_camera_api("stopLiveview", NULL);
            if (stop_err != ESP_OK) {
                ESP_LOGW(TAG, "stopLiveview failed: %s", esp_err_to_name(stop_err));
            }
        }

        mosaico_display_set_liveview_state(MOSAICO_LIVEVIEW_RETRYING);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

esp_err_t sony_liveview_start(void)
{
    if (s_liveview_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_frame_mutex = xSemaphoreCreateMutex();
    if (s_frame_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_api_mutex = xSemaphoreCreateMutex();
    if (s_api_mutex == NULL) {
        vSemaphoreDelete(s_frame_mutex);
        s_frame_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_camera_command_queue = xQueueCreate(CAMERA_COMMAND_QUEUE_LENGTH,
                                          sizeof(camera_command_t));
    if (s_camera_command_queue == NULL) {
        vSemaphoreDelete(s_api_mutex);
        s_api_mutex = NULL;
        vSemaphoreDelete(s_frame_mutex);
        s_frame_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    BaseType_t command_result = xTaskCreate(camera_command_task, "sony_camera_cmd",
                                            CAMERA_COMMAND_TASK_STACK_SIZE, NULL, 5,
                                            &s_camera_command_task);
    if (command_result != pdPASS) {
        vQueueDelete(s_camera_command_queue);
        s_camera_command_queue = NULL;
        vSemaphoreDelete(s_api_mutex);
        s_api_mutex = NULL;
        vSemaphoreDelete(s_frame_mutex);
        s_frame_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    BaseType_t result = xTaskCreate(liveview_task, "sony_liveview",
                                    LIVEVIEW_TASK_STACK_SIZE, NULL, 5,
                                    &s_liveview_task);
    if (result != pdPASS) {
        vTaskDelete(s_camera_command_task);
        s_camera_command_task = NULL;
        vQueueDelete(s_camera_command_queue);
        s_camera_command_queue = NULL;
        vSemaphoreDelete(s_api_mutex);
        s_api_mutex = NULL;
        vSemaphoreDelete(s_frame_mutex);
        s_frame_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t sony_liveview_copy_latest_frame(uint8_t *destination, size_t capacity,
                                          size_t *frame_size)
{
    if (destination == NULL || frame_size == NULL || s_frame_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    if (s_latest_frame == NULL) {
        xSemaphoreGive(s_frame_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    *frame_size = s_latest_frame_size;
    if (capacity < s_latest_frame_size) {
        xSemaphoreGive(s_frame_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(destination, s_latest_frame, s_latest_frame_size);
    xSemaphoreGive(s_frame_mutex);
    return ESP_OK;
}
