#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "usb/hid.h"
#include "usb/hid_host.h"
#include "usb/usb_host.h"

#include "HDMI.h"

#define HDMI_BRIDGE_VENDOR_ID_L             0x00
#define HDMI_BRIDGE_VENDOR_ID_H             0x01
#define HDMI_BRIDGE_DEVICE_ID_L             0x02
#define HDMI_BRIDGE_DEVICE_ID_H             0x03
#define HDMI_BRIDGE_REV_ID                  0x04
#define HDMI_BRIDGE_CTL_1_MODE              0x08
#define HDMI_BRIDGE_CTL_2_MODE              0x09
#define HDMI_BRIDGE_CTL_3_MODE              0x0A

#define HDMI_USB_TOUCH_QUEUE_LENGTH         10U
#define HDMI_USB_TOUCH_INIT_TIMEOUT_MS      3000U

typedef struct {
    hid_host_device_handle_t hid_device_handle;
    hid_host_driver_event_t event;
} HDMI_HIDHostEvent_t;

struct HDMI_Context {
    HDMI_Config_t config;
    esp_lcd_panel_handle_t panel_handle;
    lv_disp_draw_buf_t disp_buf;
    lv_disp_drv_t disp_drv;
    lv_disp_t *disp;
    esp_timer_handle_t lvgl_tick_timer;
    void *buf1;
    void *buf2;
    bool owns_draw_buffers;
    bool i2c_installed;
    volatile bool lvgl_flush_pending;

    lv_indev_drv_t indev_drv;
    lv_indev_t *indev;
    SemaphoreHandle_t touch_mutex;
    QueueHandle_t hid_host_event_queue;
    TaskHandle_t usb_lib_task_handle;
    TaskHandle_t hid_event_task_handle;
    TaskHandle_t usb_init_wait_task_handle;
    esp_err_t usb_host_init_err;
    bool usb_host_installed;
    bool hid_host_installed;
    bool usb_touch_shutdown;
    int32_t touch_x;
    int32_t touch_y;
    bool touch_pressed;
};

static const char *TAG = "HDMI";

#if !LV_USE_USER_DATA
static HDMI_Handle_t *s_active_touch_handle = NULL;
#endif

static bool HDMI_IsUSBConsoleEnabled(void)
{
#if CONFIG_IDF_TARGET_ESP32S3
#if defined(CONFIG_ESP_CONSOLE_USB_CDC) || defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG_ENABLED) || defined(CONFIG_USJ_ENABLE_USB_SERIAL_JTAG)
    return true;
#else
    return false;
#endif
#else
    return false;
#endif
}

static size_t HDMI_SelectDivisibleLineCount(uint32_t v_res, size_t preferred_lines)
{
    static const size_t candidate_lines[] = {40U, 32U, 24U, 20U, 16U, 12U, 10U, 8U, 6U, 5U, 4U, 3U, 2U, 1U};

    if (v_res == 0) {
        return preferred_lines == 0 ? 1U : preferred_lines;
    }

    if (preferred_lines != 0 && preferred_lines <= v_res && (v_res % preferred_lines) == 0U) {
        return preferred_lines;
    }

    for (size_t i = 0; i < sizeof(candidate_lines) / sizeof(candidate_lines[0]); i++) {
        size_t lines = candidate_lines[i];

        if (lines <= v_res && (v_res % lines) == 0U) {
            return lines;
        }
    }

    return 1U;
}

static TickType_t HDMI_GetI2CTimeoutTicks(const HDMI_I2CConfig_t *i2c_config)
{
    uint32_t timeout_ms = i2c_config->timeout_ms;

    if (timeout_ms == 0) {
        timeout_ms = HDMI_DEFAULT_I2C_TIMEOUT_MS;
    }

    {
        TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
        return ticks == 0 ? 1 : ticks;
    }
}

static const char *HDMI_HIDProtoName(uint8_t proto)
{
    switch (proto) {
    case HID_PROTOCOL_KEYBOARD:
        return "KEYBOARD";
    case HID_PROTOCOL_MOUSE:
        return "MOUSE";
    default:
        return "NONE";
    }
}

static void HDMI_LogHeapState(const char *stage)
{
    ESP_LOGI(TAG,
             "Heap %s: internal=%uB (largest=%uB), psram=%uB (largest=%uB)",
             stage,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}

static int32_t HDMI_MapTouchCoord(int32_t raw_coord, uint16_t raw_max, uint32_t screen_size)
{
    if (raw_max == 0 || screen_size == 0) {
        return 0;
    }

    if (raw_coord < 0) {
        raw_coord = 0;
    } else if (raw_coord > raw_max) {
        raw_coord = raw_max;
    }

    if (screen_size == 1) {
        return 0;
    }

    return (int32_t)(((int64_t)raw_coord * (int64_t)(screen_size - 1U)) / raw_max);
}

static void HDMI_SetTouchState(HDMI_Handle_t *handle, int32_t x, int32_t y, bool pressed)
{
    if (handle == NULL) {
        return;
    }

    if (handle->touch_mutex != NULL) {
        if (xSemaphoreTake(handle->touch_mutex, portMAX_DELAY) == pdTRUE) {
            handle->touch_x = x;
            handle->touch_y = y;
            handle->touch_pressed = pressed;
            xSemaphoreGive(handle->touch_mutex);
        }
    } else {
        handle->touch_x = x;
        handle->touch_y = y;
        handle->touch_pressed = pressed;
    }
}

static bool IRAM_ATTR HDMI_InvokeUserPanelCallback(esp_lcd_rgb_panel_general_cb_t callback,
                                                   esp_lcd_panel_handle_t panel,
                                                   const esp_lcd_rgb_panel_event_data_t *event_data,
                                                   void *user_ctx)
{
    if (callback == NULL) {
        return false;
    }

    return callback(panel, event_data, user_ctx);
}

static bool IRAM_ATTR HDMI_InternalPanelColorTransDoneCallback(esp_lcd_panel_handle_t panel,
                                                               const esp_lcd_rgb_panel_event_data_t *event_data,
                                                               void *user_ctx)
{
    HDMI_Handle_t *handle = (HDMI_Handle_t *)user_ctx;
    bool need_yield = false;

    if (handle != NULL) {
        if (handle->lvgl_flush_pending && !handle->config.use_panel_frame_buffers) {
            handle->lvgl_flush_pending = false;
            lv_disp_flush_ready(&handle->disp_drv);
        }

        if (handle->config.register_panel_callbacks) {
            need_yield |= HDMI_InvokeUserPanelCallback(
                handle->config.panel_callbacks.on_color_trans_done,
                panel,
                event_data,
                handle->config.panel_user_ctx);
        }
    }

    return need_yield;
}

static bool IRAM_ATTR HDMI_InternalPanelVSyncCallback(esp_lcd_panel_handle_t panel,
                                                      const esp_lcd_rgb_panel_event_data_t *event_data,
                                                      void *user_ctx)
{
    HDMI_Handle_t *handle = (HDMI_Handle_t *)user_ctx;
    bool need_yield = false;

    if (handle != NULL) {
        if (handle->lvgl_flush_pending && handle->config.use_panel_frame_buffers) {
            handle->lvgl_flush_pending = false;
            lv_disp_flush_ready(&handle->disp_drv);
        }

        if (handle->config.register_panel_callbacks) {
            need_yield |= HDMI_InvokeUserPanelCallback(
                handle->config.panel_callbacks.on_vsync,
                panel,
                event_data,
                handle->config.panel_user_ctx);
        }
    }

    return need_yield;
}

static bool IRAM_ATTR HDMI_InternalPanelFrameDoneCallback(esp_lcd_panel_handle_t panel,
                                                          const esp_lcd_rgb_panel_event_data_t *event_data,
                                                          void *user_ctx)
{
    HDMI_Handle_t *handle = (HDMI_Handle_t *)user_ctx;

    if (handle == NULL || !handle->config.register_panel_callbacks) {
        return false;
    }

    return HDMI_InvokeUserPanelCallback(
        handle->config.panel_callbacks.on_frame_buf_complete,
        panel,
        event_data,
        handle->config.panel_user_ctx);
}

static HDMI_Handle_t *HDMI_GetTouchHandleFromDrv(lv_indev_drv_t *indev_drv)
{
#if LV_USE_USER_DATA
    return (HDMI_Handle_t *)indev_drv->user_data;
#else
    (void)indev_drv;
    return s_active_touch_handle;
#endif
}

static void HDMI_USBTouchReadCallback(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    HDMI_Handle_t *handle = HDMI_GetTouchHandleFromDrv(indev_drv);
    int32_t x = 0;
    int32_t y = 0;
    bool pressed = false;

    if (handle == NULL) {
        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = 0;
        data->point.y = 0;
        return;
    }

    if (handle->touch_mutex != NULL) {
        if (xSemaphoreTake(handle->touch_mutex, portMAX_DELAY) == pdTRUE) {
            x = handle->touch_x;
            y = handle->touch_y;
            pressed = handle->touch_pressed;
            xSemaphoreGive(handle->touch_mutex);
        }
    } else {
        x = handle->touch_x;
        y = handle->touch_y;
        pressed = handle->touch_pressed;
    }

    data->point.x = (lv_coord_t)x;
    data->point.y = (lv_coord_t)y;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

static bool HDMI_DefaultUSBTouchParser(const HDMI_Handle_t *handle,
                                       const uint8_t *report,
                                       size_t report_len,
                                       int32_t *raw_x,
                                       int32_t *raw_y,
                                       bool *pressed)
{
    const HDMI_USBTouchConfig_t *touch = &handle->config.usb_touch;

    if (report == NULL || raw_x == NULL || raw_y == NULL || pressed == NULL) {
        return false;
    }

    if (report_len < touch->min_report_len) {
        return false;
    }

    *raw_x = ((int32_t)report[touch->x_msb_byte_index] << 8) | report[touch->x_lsb_byte_index];
    *raw_y = ((int32_t)report[touch->y_msb_byte_index] << 8) | report[touch->y_lsb_byte_index];
    *pressed = (report[touch->pressed_byte_index] & touch->pressed_bit_mask) != 0;

    return true;
}

static bool HDMI_ParseUSBTouchReport(const HDMI_Handle_t *handle,
                                     const uint8_t *report,
                                     size_t report_len,
                                     int32_t *raw_x,
                                     int32_t *raw_y,
                                     bool *pressed)
{
    if (handle->config.usb_touch.parser_cb != NULL) {
        return handle->config.usb_touch.parser_cb(
            report,
            report_len,
            handle->config.usb_touch.parser_user_ctx,
            raw_x,
            raw_y,
            pressed);
    }

    return HDMI_DefaultUSBTouchParser(handle, report, report_len, raw_x, raw_y, pressed);
}

static bool HDMI_HandleUSBTouchReport(HDMI_Handle_t *handle, const uint8_t *report, size_t report_len)
{
    const HDMI_USBTouchConfig_t *touch = &handle->config.usb_touch;
    uint32_t h_res = handle->config.panel_config.timings.h_res;
    uint32_t v_res = handle->config.panel_config.timings.v_res;
    int32_t raw_x = 0;
    int32_t raw_y = 0;
    int32_t touch_x = 0;
    int32_t touch_y = 0;
    int32_t mapped_x = 0;
    int32_t mapped_y = 0;
    uint16_t max_x = touch->raw_max_x;
    uint16_t max_y = touch->raw_max_y;
    bool pressed = false;

    if (!HDMI_ParseUSBTouchReport(handle, report, report_len, &raw_x, &raw_y, &pressed)) {
        return false;
    }

    touch_x = raw_x;
    touch_y = raw_y;
    if (touch->swap_xy) {
        int32_t temp_coord = touch_x;
        uint16_t temp_max = max_x;

        touch_x = touch_y;
        touch_y = temp_coord;
        max_x = max_y;
        max_y = temp_max;
    }

    mapped_x = HDMI_MapTouchCoord(touch_x, max_x, h_res);
    mapped_y = HDMI_MapTouchCoord(touch_y, max_y, v_res);

    if (touch->invert_x && h_res > 0) {
        mapped_x = (int32_t)(h_res - 1U) - mapped_x;
    }
    if (touch->invert_y && v_res > 0) {
        mapped_y = (int32_t)(v_res - 1U) - mapped_y;
    }

    HDMI_SetTouchState(handle, mapped_x, mapped_y, pressed);
    return true;
}

static void HDMI_HIDInterfaceCallback(hid_host_device_handle_t hid_device_handle,
                                      const hid_host_interface_event_t event,
                                      void *arg)
{
    HDMI_Handle_t *handle = (HDMI_Handle_t *)arg;
    uint8_t report_data[64] = {0};
    size_t report_len = 0;
    hid_host_dev_params_t dev_params = {0};

    if (handle == NULL) {
        return;
    }

    if (hid_host_device_get_params(hid_device_handle, &dev_params) != ESP_OK) {
        return;
    }

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        if (hid_host_device_get_raw_input_report_data(
                hid_device_handle,
                report_data,
                sizeof(report_data),
                &report_len) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read HID input report");
            return;
        }

        if (dev_params.sub_class == HID_SUBCLASS_BOOT_INTERFACE &&
            (dev_params.proto == HID_PROTOCOL_KEYBOARD || dev_params.proto == HID_PROTOCOL_MOUSE)) {
            return;
        }

        (void)HDMI_HandleUSBTouchReport(handle, report_data, report_len);
        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HID device disconnected, protocol '%s'", HDMI_HIDProtoName(dev_params.proto));
        HDMI_SetTouchState(handle, handle->touch_x, handle->touch_y, false);
        hid_host_device_close(hid_device_handle);
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "HID transfer error, protocol '%s'", HDMI_HIDProtoName(dev_params.proto));
        break;

    default:
        ESP_LOGW(TAG, "Unhandled HID interface event %d", event);
        break;
    }
}

static void HDMI_HandleHIDDeviceEvent(HDMI_Handle_t *handle, const HDMI_HIDHostEvent_t *hid_event)
{
    hid_host_dev_params_t dev_params = {0};

    if (hid_host_device_get_params(hid_event->hid_device_handle, &dev_params) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to query HID device parameters");
        return;
    }

    switch (hid_event->event) {
    case HID_HOST_DRIVER_EVENT_CONNECTED: {
        const hid_host_device_config_t dev_config = {
            .callback = HDMI_HIDInterfaceCallback,
            .callback_arg = handle,
        };
        esp_err_t err = hid_host_device_open(hid_event->hid_device_handle, &dev_config);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to open HID device: %s", esp_err_to_name(err));
            return;
        }

        ESP_LOGI(TAG, "HID device connected, protocol '%s'", HDMI_HIDProtoName(dev_params.proto));

        if (dev_params.sub_class == HID_SUBCLASS_BOOT_INTERFACE) {
            err = hid_class_request_set_protocol(hid_event->hid_device_handle, HID_REPORT_PROTOCOL_BOOT);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set HID boot protocol: %s", esp_err_to_name(err));
            } else if (dev_params.proto == HID_PROTOCOL_KEYBOARD) {
                err = hid_class_request_set_idle(hid_event->hid_device_handle, 0, 0);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to set HID idle: %s", esp_err_to_name(err));
                }
            }
        }

        err = hid_host_device_start(hid_event->hid_device_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start HID device: %s", esp_err_to_name(err));
        }
        break;
    }

    default:
        break;
    }
}

static void HDMI_HIDHostDeviceCallback(hid_host_device_handle_t hid_device_handle,
                                       const hid_host_driver_event_t event,
                                       void *arg)
{
    HDMI_Handle_t *handle = (HDMI_Handle_t *)arg;
    HDMI_HIDHostEvent_t hid_event = {
        .hid_device_handle = hid_device_handle,
        .event = event,
    };

    if (handle == NULL || handle->hid_host_event_queue == NULL) {
        return;
    }

    xQueueSend(handle->hid_host_event_queue, &hid_event, 0);
}

static void HDMI_HIDEventTask(void *pvParameters)
{
    HDMI_Handle_t *handle = (HDMI_Handle_t *)pvParameters;
    HDMI_HIDHostEvent_t hid_event;

    while (!handle->usb_touch_shutdown) {
        if (xQueueReceive(handle->hid_host_event_queue, &hid_event, pdMS_TO_TICKS(50)) == pdTRUE) {
            HDMI_HandleHIDDeviceEvent(handle, &hid_event);
        }
    }

    handle->hid_event_task_handle = NULL;
    vTaskDelete(NULL);
}

static void HDMI_USBLibTask(void *arg)
{
    HDMI_Handle_t *handle = (HDMI_Handle_t *)arg;
    usb_host_config_t host_config = {
        .skip_phy_setup = handle->config.usb_touch.skip_phy_setup,
        .intr_flags = handle->config.usb_touch.intr_flags,
    };

    handle->usb_host_init_err = usb_host_install(&host_config);
    if (handle->usb_host_init_err == ESP_OK) {
        handle->usb_host_installed = true;
    }

    if (handle->usb_init_wait_task_handle != NULL) {
        xTaskNotifyGive(handle->usb_init_wait_task_handle);
    }

    if (handle->usb_host_init_err != ESP_OK) {
        handle->usb_lib_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (!handle->usb_touch_shutdown) {
        uint32_t event_flags = 0;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (handle->usb_touch_shutdown) {
            break;
        }

        if (err != ESP_OK) {
            if (err != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "USB host event loop error: %s", esp_err_to_name(err));
            }
            continue;
        }

        if ((event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) != 0U) {
            usb_host_device_free_all();
            ESP_LOGI(TAG, "USB host event: NO_CLIENTS");
        }

        if ((event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) != 0U) {
            ESP_LOGI(TAG, "USB host event: ALL_FREE");
        }
    }

    handle->usb_lib_task_handle = NULL;
    vTaskDelete(NULL);
}

static void HDMI_NormalizeConfig(HDMI_Handle_t *handle)
{
    size_t required_report_len = HDMI_USB_TOUCH_DEFAULT_MIN_REPORT_LEN;

    if (handle->config.i2c.clk_speed_hz == 0) {
        handle->config.i2c.clk_speed_hz = HDMI_DEFAULT_I2C_CLK_SPEED_HZ;
    }

    if (handle->config.i2c.timeout_ms == 0) {
        handle->config.i2c.timeout_ms = HDMI_DEFAULT_I2C_TIMEOUT_MS;
    }

    if (handle->config.i2c.device_address == 0) {
        handle->config.i2c.device_address = HDMI_TFP410_I2C_ADDRESS;
    }

    if (handle->config.panel_config.data_width == 0) {
        handle->config.panel_config.data_width = HDMI_RGB_DATA_WIDTH;
    }

    if (handle->config.panel_config.dma_burst_size == 0) {
        handle->config.panel_config.dma_burst_size = 64;
    }

    if (handle->config.panel_config.num_fbs == 0) {
        handle->config.panel_config.num_fbs = handle->config.use_panel_frame_buffers ? 2 : 1;
    }

    if (handle->config.lvgl_draw_buffer_height == 0) {
        handle->config.lvgl_draw_buffer_height = HDMI_DEFAULT_DRAW_BUFFER_LINES;
    }

    if (handle->config.panel_config.timings.v_res != 0 &&
        handle->config.lvgl_draw_buffer_height > handle->config.panel_config.timings.v_res) {
        handle->config.lvgl_draw_buffer_height = handle->config.panel_config.timings.v_res;
    }

    if (handle->config.lvgl_tick_period_ms == 0) {
        handle->config.lvgl_tick_period_ms = HDMI_DEFAULT_LVGL_TICK_PERIOD_MS;
    }

    if (handle->config.usb_touch.raw_max_x == 0) {
        handle->config.usb_touch.raw_max_x = HDMI_USB_TOUCH_DEFAULT_RAW_MAX_X;
    }

    if (handle->config.usb_touch.raw_max_y == 0) {
        handle->config.usb_touch.raw_max_y = HDMI_USB_TOUCH_DEFAULT_RAW_MAX_Y;
    }

    if (handle->config.usb_touch.pressed_bit_mask == 0) {
        handle->config.usb_touch.pressed_bit_mask = HDMI_USB_TOUCH_DEFAULT_PRESSED_BIT_MASK;
    }

    if (handle->config.usb_touch.usb_lib_task_stack_size == 0) {
        handle->config.usb_touch.usb_lib_task_stack_size = HDMI_USB_TOUCH_DEFAULT_USB_LIB_STACK_SIZE;
    }

    if (handle->config.usb_touch.usb_lib_task_priority <= 0) {
        handle->config.usb_touch.usb_lib_task_priority = 2;
    }

    if (handle->config.usb_touch.hid_background_task_stack_size == 0) {
        handle->config.usb_touch.hid_background_task_stack_size = HDMI_USB_TOUCH_DEFAULT_HID_BG_STACK_SIZE;
    }

    if (handle->config.usb_touch.hid_background_task_priority <= 0) {
        handle->config.usb_touch.hid_background_task_priority = 5;
    }

    if (handle->config.usb_touch.hid_event_task_stack_size == 0) {
        handle->config.usb_touch.hid_event_task_stack_size = HDMI_USB_TOUCH_DEFAULT_HID_EVT_STACK_SIZE;
    }

    if (handle->config.usb_touch.hid_event_task_priority <= 0) {
        handle->config.usb_touch.hid_event_task_priority = 2;
    }

    if (handle->config.usb_touch.min_report_len == 0) {
        uint8_t max_index = handle->config.usb_touch.y_msb_byte_index;

        if (handle->config.usb_touch.pressed_byte_index > max_index) {
            max_index = handle->config.usb_touch.pressed_byte_index;
        }
        if (handle->config.usb_touch.x_lsb_byte_index > max_index) {
            max_index = handle->config.usb_touch.x_lsb_byte_index;
        }
        if (handle->config.usb_touch.x_msb_byte_index > max_index) {
            max_index = handle->config.usb_touch.x_msb_byte_index;
        }
        if (handle->config.usb_touch.y_lsb_byte_index > max_index) {
            max_index = handle->config.usb_touch.y_lsb_byte_index;
        }

        required_report_len = (size_t)max_index + 1U;
        if (required_report_len < HDMI_USB_TOUCH_DEFAULT_MIN_REPORT_LEN) {
            required_report_len = HDMI_USB_TOUCH_DEFAULT_MIN_REPORT_LEN;
        }
        handle->config.usb_touch.min_report_len = required_report_len;
    }
}

static esp_err_t HDMI_ValidateConfig(const HDMI_Handle_t *handle)
{
    if (handle->config.panel_config.timings.h_res == 0 ||
        handle->config.panel_config.timings.v_res == 0 ||
        handle->config.panel_config.timings.pclk_hz == 0) {
        ESP_LOGE(TAG, "Panel timing is incomplete");
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->config.panel_config.data_width != HDMI_RGB_DATA_WIDTH) {
        ESP_LOGE(TAG, "Only 16-bit RGB bus is supported by this component");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (handle->config.use_panel_frame_buffers && handle->config.panel_config.num_fbs > 2) {
        ESP_LOGE(TAG, "LVGL integration supports up to 2 panel frame buffers");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (handle->config.usb_touch.enabled &&
        (handle->config.usb_touch.raw_max_x == 0 || handle->config.usb_touch.raw_max_y == 0)) {
        ESP_LOGE(TAG, "USB touch raw range must be configured");
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->config.usb_touch.enabled && HDMI_IsUSBConsoleEnabled()) {
        ESP_LOGE(TAG, "USB touch host conflicts with USB CDC/USB Serial JTAG on ESP32-S3. Use UART console instead");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static esp_err_t HDMI_I2CRead(const HDMI_Handle_t *handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        handle->config.i2c.port,
        handle->config.i2c.device_address,
        &reg_addr,
        1,
        data,
        len,
        HDMI_GetI2CTimeoutTicks(&handle->config.i2c));
}

static esp_err_t HDMI_I2CWriteByte(const HDMI_Handle_t *handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(
        handle->config.i2c.port,
        handle->config.i2c.device_address,
        write_buf,
        sizeof(write_buf),
        HDMI_GetI2CTimeoutTicks(&handle->config.i2c));
}

static esp_err_t HDMI_ConfigI2C(HDMI_Handle_t *handle)
{
    if (!handle->config.i2c.install_driver) {
        return ESP_OK;
    }

    {
        i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = handle->config.i2c.sda_gpio_num,
            .scl_io_num = handle->config.i2c.scl_gpio_num,
            .sda_pullup_en = handle->config.i2c.sda_pullup_en,
            .scl_pullup_en = handle->config.i2c.scl_pullup_en,
            .master.clk_speed = handle->config.i2c.clk_speed_hz,
        };
        esp_err_t err = i2c_param_config(handle->config.i2c.port, &i2c_config);
        if (err != ESP_OK) {
            return err;
        }

        err = i2c_driver_install(handle->config.i2c.port, i2c_config.mode, 0, 0, 0);
        if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "I2C port %d already has a driver installed, reusing it", handle->config.i2c.port);
            return ESP_OK;
        }

        if (err == ESP_OK) {
            handle->i2c_installed = true;
        }

        return err;
    }
}

static esp_err_t HDMI_BridgeInit(HDMI_Handle_t *handle)
{
    uint8_t temp = 0;
    esp_err_t err = HDMI_I2CWriteByte(handle, HDMI_BRIDGE_CTL_1_MODE, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    err = HDMI_I2CWriteByte(handle, HDMI_BRIDGE_CTL_1_MODE, 0x0F);
    if (err != ESP_OK) {
        return err;
    }

    err = HDMI_I2CRead(handle, HDMI_BRIDGE_CTL_2_MODE, &temp, sizeof(temp));
    if (err != ESP_OK) {
        return err;
    }

    temp |= 0x39;
    err = HDMI_I2CWriteByte(handle, HDMI_BRIDGE_CTL_2_MODE, temp);
    if (err != ESP_OK) {
        return err;
    }

    err = HDMI_I2CWriteByte(handle, HDMI_BRIDGE_CTL_3_MODE, 0xF0);
    if (err != ESP_OK) {
        return err;
    }

    {
        uint8_t vendor_l = 0;
        uint8_t vendor_h = 0;
        uint8_t device_l = 0;
        uint8_t device_h = 0;
        uint8_t revision = 0;

        if (HDMI_I2CRead(handle, HDMI_BRIDGE_VENDOR_ID_L, &vendor_l, sizeof(vendor_l)) == ESP_OK &&
            HDMI_I2CRead(handle, HDMI_BRIDGE_VENDOR_ID_H, &vendor_h, sizeof(vendor_h)) == ESP_OK &&
            HDMI_I2CRead(handle, HDMI_BRIDGE_DEVICE_ID_L, &device_l, sizeof(device_l)) == ESP_OK &&
            HDMI_I2CRead(handle, HDMI_BRIDGE_DEVICE_ID_H, &device_h, sizeof(device_h)) == ESP_OK &&
            HDMI_I2CRead(handle, HDMI_BRIDGE_REV_ID, &revision, sizeof(revision)) == ESP_OK) {
            uint16_t vendor_id = ((uint16_t)vendor_h << 8) | vendor_l;
            uint16_t device_id = ((uint16_t)device_h << 8) | device_l;
            ESP_LOGI(TAG, "TFP410 detected: vendor=0x%04X device=0x%04X rev=0x%02X", vendor_id, device_id, revision);
        }
    }

    return ESP_OK;
}

static void HDMI_LVGLFlushCallback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    HDMI_Handle_t *handle = (HDMI_Handle_t *)drv->user_data;
    esp_err_t err;

    if (handle == NULL || handle->panel_handle == NULL) {
        lv_disp_flush_ready(drv);
        return;
    }

    handle->lvgl_flush_pending = true;
    err = esp_lcd_panel_draw_bitmap(
        handle->panel_handle,
        area->x1,
        area->y1,
        area->x2 + 1,
        area->y2 + 1,
        color_map);
    if (err != ESP_OK) {
        handle->lvgl_flush_pending = false;
        ESP_LOGW(TAG, "RGB draw_bitmap failed: %s", esp_err_to_name(err));
        lv_disp_flush_ready(drv);
    }
}

static void HDMI_LVGLIncTick(void *arg)
{
    const HDMI_Handle_t *handle = (const HDMI_Handle_t *)arg;
    lv_tick_inc(handle->config.lvgl_tick_period_ms);
}

static void *HDMI_AllocDrawBuffer(const HDMI_Handle_t *handle, size_t buffer_size, bool second_buffer)
{
    static const uint32_t prefer_internal_caps[] = {
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        MALLOC_CAP_8BIT,
    };
    static const uint32_t prefer_spiram_caps[] = {
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
        MALLOC_CAP_8BIT,
    };
    const uint32_t *caps_order = prefer_internal_caps;

    if (handle->config.usb_touch.enabled &&
        !handle->config.use_panel_frame_buffers &&
        second_buffer) {
        caps_order = prefer_spiram_caps;
    }

    for (size_t i = 0; i < sizeof(prefer_internal_caps) / sizeof(prefer_internal_caps[0]); i++) {
        void *buffer = heap_caps_malloc(buffer_size, caps_order[i]);
        if (buffer != NULL) {
            return buffer;
        }
    }

    return NULL;
}

static esp_err_t HDMI_InitPanel(HDMI_Handle_t *handle)
{
    esp_err_t err;
    esp_lcd_rgb_panel_event_callbacks_t panel_callbacks = {
        .on_color_trans_done = HDMI_InternalPanelColorTransDoneCallback,
        .on_vsync = HDMI_InternalPanelVSyncCallback,
        .on_frame_buf_complete = HDMI_InternalPanelFrameDoneCallback,
    };

    ESP_LOGI(TAG, "Installing RGB LCD panel driver");
    HDMI_LogHeapState("before panel");

    err = esp_lcd_new_rgb_panel(&handle->config.panel_config, &handle->panel_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_lcd_rgb_panel_register_event_callbacks(
        handle->panel_handle,
        &panel_callbacks,
        handle);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_lcd_panel_reset(handle->panel_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_lcd_panel_init(handle->panel_handle);
    if (err == ESP_OK) {
        HDMI_LogHeapState("after panel");
    }

    return err;
}

static esp_err_t HDMI_InitLVGL(HDMI_Handle_t *handle)
{
    uint32_t h_res = handle->config.panel_config.timings.h_res;
    uint32_t v_res = handle->config.panel_config.timings.v_res;
    uint32_t buffer_pixels = 0;

    if (!lv_is_initialized()) {
        lv_init();
    }

    if (handle->config.use_panel_frame_buffers) {
        esp_err_t err;

        if (handle->config.panel_config.num_fbs == 1) {
            err = esp_lcd_rgb_panel_get_frame_buffer(handle->panel_handle, 1, &handle->buf1);
            if (err != ESP_OK) {
                return err;
            }
            handle->buf2 = NULL;
        } else {
            err = esp_lcd_rgb_panel_get_frame_buffer(handle->panel_handle, 2, &handle->buf1, &handle->buf2);
            if (err != ESP_OK) {
                return err;
            }
        }

        buffer_pixels = h_res * v_res;
    } else {
        size_t buffer_pixels_per_buf = h_res * handle->config.lvgl_draw_buffer_height;
        size_t buffer_size = buffer_pixels_per_buf * sizeof(lv_color_t);

        HDMI_LogHeapState("before lvgl");

        handle->buf1 = HDMI_AllocDrawBuffer(handle, buffer_size, false);
        if (handle->buf1 == NULL) {
            return ESP_ERR_NO_MEM;
        }

        handle->owns_draw_buffers = true;

        handle->buf2 = HDMI_AllocDrawBuffer(handle, buffer_size, true);
        if (handle->buf2 == NULL) {
            return ESP_ERR_NO_MEM;
        }

        buffer_pixels = buffer_pixels_per_buf;
    }

    lv_disp_draw_buf_init(&handle->disp_buf, handle->buf1, handle->buf2, buffer_pixels);

    lv_disp_drv_init(&handle->disp_drv);
    handle->disp_drv.hor_res = h_res;
    handle->disp_drv.ver_res = v_res;
    handle->disp_drv.flush_cb = HDMI_LVGLFlushCallback;
    handle->disp_drv.draw_buf = &handle->disp_buf;
    handle->disp_drv.user_data = handle;
    handle->disp_drv.full_refresh = handle->config.lvgl_full_refresh;
    handle->disp = lv_disp_drv_register(&handle->disp_drv);

    if (handle->config.start_lvgl_tick_timer) {
        const esp_timer_create_args_t timer_args = {
            .callback = HDMI_LVGLIncTick,
            .arg = handle,
            .name = "hdmi_lvgl_tick",
        };
        esp_err_t err = esp_timer_create(&timer_args, &handle->lvgl_tick_timer);
        if (err != ESP_OK) {
            return err;
        }

        err = esp_timer_start_periodic(handle->lvgl_tick_timer, handle->config.lvgl_tick_period_ms * 1000);
        if (err != ESP_OK) {
            return err;
        }
    }

    HDMI_LogHeapState("after lvgl");

    return ESP_OK;
}

static esp_err_t HDMI_InitUSBTouchHost(HDMI_Handle_t *handle)
{
    if (!handle->config.usb_touch.enabled) {
        return ESP_OK;
    }

    HDMI_LogHeapState("before usb touch");

    handle->touch_mutex = xSemaphoreCreateMutex();
    if (handle->touch_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (!handle->config.usb_touch.manage_usb_host) {
        ESP_LOGI(TAG, "USB touch input enabled with external host management");
        return ESP_OK;
    }

    handle->hid_host_event_queue = xQueueCreate(
        HDMI_USB_TOUCH_QUEUE_LENGTH,
        sizeof(HDMI_HIDHostEvent_t));
    if (handle->hid_host_event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    handle->usb_init_wait_task_handle = xTaskGetCurrentTaskHandle();

    if (xTaskCreatePinnedToCore(
            HDMI_USBLibTask,
            "hdmi_usb_lib",
            handle->config.usb_touch.usb_lib_task_stack_size,
            handle,
            handle->config.usb_touch.usb_lib_task_priority,
            &handle->usb_lib_task_handle,
            handle->config.usb_touch.usb_lib_task_core_id) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(HDMI_USB_TOUCH_INIT_TIMEOUT_MS)) == 0) {
        ESP_LOGE(TAG, "USB host install timed out");
        return ESP_ERR_TIMEOUT;
    }

    handle->usb_init_wait_task_handle = NULL;

    if (handle->usb_host_init_err != ESP_OK) {
        return handle->usb_host_init_err;
    }

    {
        const hid_host_driver_config_t hid_host_driver_config = {
            .create_background_task = true,
            .task_priority = handle->config.usb_touch.hid_background_task_priority,
            .stack_size = handle->config.usb_touch.hid_background_task_stack_size,
            .core_id = handle->config.usb_touch.hid_background_task_core_id,
            .callback = HDMI_HIDHostDeviceCallback,
            .callback_arg = handle,
        };
        esp_err_t err = hid_host_install(&hid_host_driver_config);
        if (err != ESP_OK) {
            return err;
        }
        handle->hid_host_installed = true;
    }

    if (xTaskCreatePinnedToCore(
            HDMI_HIDEventTask,
            "hdmi_hid_evt",
            handle->config.usb_touch.hid_event_task_stack_size,
            handle,
            handle->config.usb_touch.hid_event_task_priority,
            &handle->hid_event_task_handle,
            handle->config.usb_touch.hid_event_task_core_id) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "USB touch support enabled");
    HDMI_LogHeapState("after usb touch host");
    return ESP_OK;
}

static esp_err_t HDMI_RegisterUSBTouchInput(HDMI_Handle_t *handle)
{
    if (!handle->config.usb_touch.enabled) {
        return ESP_OK;
    }

    lv_indev_drv_init(&handle->indev_drv);
    handle->indev_drv.type = LV_INDEV_TYPE_POINTER;
    handle->indev_drv.read_cb = HDMI_USBTouchReadCallback;
#if LV_USE_USER_DATA
    handle->indev_drv.user_data = handle;
#else
    s_active_touch_handle = handle;
#endif
    handle->indev = lv_indev_drv_register(&handle->indev_drv);
    if (handle->indev == NULL) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void HDMI_DeleteUSBTouch(HDMI_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    handle->usb_touch_shutdown = true;
    HDMI_SetTouchState(handle, handle->touch_x, handle->touch_y, false);

    if (handle->usb_host_installed) {
        usb_host_lib_unblock();
    }

    if (handle->hid_event_task_handle != NULL) {
        vTaskDelete(handle->hid_event_task_handle);
        handle->hid_event_task_handle = NULL;
    }

    if (handle->hid_host_installed) {
        esp_err_t err = hid_host_uninstall();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to uninstall HID host: %s", esp_err_to_name(err));
        }
        handle->hid_host_installed = false;
    }

    if (handle->usb_lib_task_handle != NULL) {
        vTaskDelete(handle->usb_lib_task_handle);
        handle->usb_lib_task_handle = NULL;
    }

    if (handle->usb_host_installed) {
        usb_host_device_free_all();
        {
            esp_err_t err = usb_host_uninstall();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to uninstall USB host: %s", esp_err_to_name(err));
            }
        }
        handle->usb_host_installed = false;
    }

    if (handle->indev != NULL) {
        lv_indev_delete(handle->indev);
        handle->indev = NULL;
    }

    if (handle->hid_host_event_queue != NULL) {
        vQueueDelete(handle->hid_host_event_queue);
        handle->hid_host_event_queue = NULL;
    }

    if (handle->touch_mutex != NULL) {
        vSemaphoreDelete(handle->touch_mutex);
        handle->touch_mutex = NULL;
    }

#if !LV_USE_USER_DATA
    if (s_active_touch_handle == handle) {
        s_active_touch_handle = NULL;
    }
#endif
}

void HDMI_GetDefaultConfig(HDMI_Config_t *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));

    config->i2c.port = I2C_NUM_0;
    config->i2c.sda_gpio_num = GPIO_NUM_5;
    config->i2c.scl_gpio_num = GPIO_NUM_4;
    config->i2c.clk_speed_hz = HDMI_DEFAULT_I2C_CLK_SPEED_HZ;
    config->i2c.timeout_ms = HDMI_DEFAULT_I2C_TIMEOUT_MS;
    config->i2c.device_address = HDMI_TFP410_I2C_ADDRESS;
    config->i2c.sda_pullup_en = false;
    config->i2c.scl_pullup_en = false;
    config->i2c.install_driver = true;

    config->usb_touch.enabled = false;
    config->usb_touch.manage_usb_host = true;
    config->usb_touch.skip_phy_setup = false;
    config->usb_touch.intr_flags = ESP_INTR_FLAG_LEVEL1;
    config->usb_touch.usb_lib_task_stack_size = HDMI_USB_TOUCH_DEFAULT_USB_LIB_STACK_SIZE;
    config->usb_touch.usb_lib_task_priority = 2;
    config->usb_touch.usb_lib_task_core_id = 0;
    config->usb_touch.hid_background_task_stack_size = HDMI_USB_TOUCH_DEFAULT_HID_BG_STACK_SIZE;
    config->usb_touch.hid_background_task_priority = 5;
    config->usb_touch.hid_background_task_core_id = 0;
    config->usb_touch.hid_event_task_stack_size = HDMI_USB_TOUCH_DEFAULT_HID_EVT_STACK_SIZE;
    config->usb_touch.hid_event_task_priority = 2;
    config->usb_touch.hid_event_task_core_id = -1;
    config->usb_touch.raw_max_x = HDMI_USB_TOUCH_DEFAULT_RAW_MAX_X;
    config->usb_touch.raw_max_y = HDMI_USB_TOUCH_DEFAULT_RAW_MAX_Y;
    config->usb_touch.swap_xy = false;
    config->usb_touch.invert_x = false;
    config->usb_touch.invert_y = false;
    config->usb_touch.pressed_byte_index = 1;
    config->usb_touch.pressed_bit_mask = HDMI_USB_TOUCH_DEFAULT_PRESSED_BIT_MASK;
    config->usb_touch.x_lsb_byte_index = 2;
    config->usb_touch.x_msb_byte_index = 3;
    config->usb_touch.y_lsb_byte_index = 4;
    config->usb_touch.y_msb_byte_index = 5;
    config->usb_touch.min_report_len = HDMI_USB_TOUCH_DEFAULT_MIN_REPORT_LEN;
    config->usb_touch.parser_cb = NULL;
    config->usb_touch.parser_user_ctx = NULL;

    config->panel_config.data_width = HDMI_RGB_DATA_WIDTH;
    config->panel_config.dma_burst_size = 64;
    config->panel_config.num_fbs = 2;
    config->panel_config.bounce_buffer_size_px = 6 * 1024;
    config->panel_config.clk_src = LCD_CLK_SRC_PLL240M;
    config->panel_config.disp_gpio_num = -1;
    config->panel_config.pclk_gpio_num = GPIO_NUM_8;
    config->panel_config.vsync_gpio_num = GPIO_NUM_7;
    config->panel_config.hsync_gpio_num = GPIO_NUM_6;
    config->panel_config.de_gpio_num = GPIO_NUM_15;
    config->panel_config.data_gpio_nums[0] = GPIO_NUM_16;
    config->panel_config.data_gpio_nums[1] = GPIO_NUM_17;
    config->panel_config.data_gpio_nums[2] = GPIO_NUM_18;
    config->panel_config.data_gpio_nums[3] = GPIO_NUM_3;
    config->panel_config.data_gpio_nums[4] = GPIO_NUM_46;
    config->panel_config.data_gpio_nums[5] = GPIO_NUM_0;
    config->panel_config.data_gpio_nums[6] = GPIO_NUM_9;
    config->panel_config.data_gpio_nums[7] = GPIO_NUM_10;
    config->panel_config.data_gpio_nums[8] = GPIO_NUM_11;
    config->panel_config.data_gpio_nums[9] = GPIO_NUM_12;
    config->panel_config.data_gpio_nums[10] = GPIO_NUM_13;
    config->panel_config.data_gpio_nums[11] = GPIO_NUM_14;
    config->panel_config.data_gpio_nums[12] = GPIO_NUM_21;
    config->panel_config.data_gpio_nums[13] = GPIO_NUM_47;
    config->panel_config.data_gpio_nums[14] = GPIO_NUM_48;
    config->panel_config.data_gpio_nums[15] = GPIO_NUM_45;
    config->panel_config.timings.pclk_hz = 27500000;
    config->panel_config.timings.h_res = 1024;
    config->panel_config.timings.v_res = 768;
    config->panel_config.timings.hsync_back_porch = 80;
    config->panel_config.timings.hsync_front_porch = 48;
    config->panel_config.timings.hsync_pulse_width = 32;
    config->panel_config.timings.vsync_back_porch = 6;
    config->panel_config.timings.vsync_front_porch = 3;
    config->panel_config.timings.vsync_pulse_width = 4;
    config->panel_config.timings.flags.pclk_active_neg = false;
    config->panel_config.timings.flags.vsync_idle_low = true;
    config->panel_config.timings.flags.hsync_idle_low = true;
    config->panel_config.timings.flags.de_idle_high = false;
    config->panel_config.timings.flags.pclk_idle_high = false;
    config->panel_config.flags.refresh_on_demand = false;
    config->panel_config.flags.fb_in_psram = true;

    config->panel_user_ctx = NULL;
    config->register_panel_callbacks = false;
    config->lvgl_draw_buffer_height = HDMI_DEFAULT_DRAW_BUFFER_LINES;
    config->lvgl_tick_period_ms = HDMI_DEFAULT_LVGL_TICK_PERIOD_MS;
    config->use_panel_frame_buffers = true;
    config->lvgl_full_refresh = true;
    config->start_lvgl_tick_timer = true;
}

void HDMI_ApplyRenderProfile(HDMI_Config_t *config, HDMI_RenderProfile_t profile)
{
    uint32_t h_res;
    uint32_t v_res;
    size_t lines;

    if (config == NULL) {
        return;
    }

    h_res = config->panel_config.timings.h_res == 0 ? 640U : config->panel_config.timings.h_res;
    v_res = config->panel_config.timings.v_res == 0 ? 480U : config->panel_config.timings.v_res;

    switch (profile) {
    case HDMI_RENDER_PROFILE_LOW_LATENCY:
        lines = HDMI_SelectDivisibleLineCount(v_res, config->usb_touch.enabled ? 20U : 40U);
        config->panel_config.num_fbs = 1;
        config->panel_config.bounce_buffer_size_px = h_res * lines;
        config->use_panel_frame_buffers = false;
        config->lvgl_full_refresh = false;
        config->lvgl_draw_buffer_height = lines;
        break;

    case HDMI_RENDER_PROFILE_HEAVY_ANIMATION:
        lines = HDMI_SelectDivisibleLineCount(v_res, 10U);
        config->panel_config.num_fbs = 2;
        config->panel_config.bounce_buffer_size_px = h_res * lines;
        config->use_panel_frame_buffers = true;
        config->lvgl_full_refresh = true;
        config->lvgl_draw_buffer_height = v_res;
        break;

    case HDMI_RENDER_PROFILE_DEFAULT:
    default:
        break;
    }
}

esp_err_t HDMI_Initialize(const HDMI_Config_t *config, HDMI_Handle_t **ret_handle)
{
    HDMI_Handle_t *handle;
    esp_err_t err;

    if (config == NULL || ret_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *ret_handle = NULL;

    handle = calloc(1, sizeof(*handle));
    if (handle == NULL) {
        return ESP_ERR_NO_MEM;
    }

    handle->config = *config;
    HDMI_NormalizeConfig(handle);

    err = HDMI_ValidateConfig(handle);
    if (err != ESP_OK) {
        HDMI_Delete(handle);
        return err;
    }

    err = HDMI_InitUSBTouchHost(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB touch host init failed: %s", esp_err_to_name(err));
        HDMI_Delete(handle);
        return err;
    }

    err = HDMI_ConfigI2C(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        HDMI_Delete(handle);
        return err;
    }

    err = HDMI_BridgeInit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TFP410 init failed: %s", esp_err_to_name(err));
        HDMI_Delete(handle);
        return err;
    }

    err = HDMI_InitPanel(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RGB panel init failed: %s", esp_err_to_name(err));
        HDMI_Delete(handle);
        return err;
    }

    err = HDMI_InitLVGL(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LVGL init failed: %s", esp_err_to_name(err));
        HDMI_Delete(handle);
        return err;
    }

    err = HDMI_RegisterUSBTouchInput(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LVGL touch input registration failed: %s", esp_err_to_name(err));
        HDMI_Delete(handle);
        return err;
    }

    *ret_handle = handle;
    return ESP_OK;
}

void HDMI_Delete(HDMI_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    HDMI_DeleteUSBTouch(handle);

    if (handle->lvgl_tick_timer != NULL) {
        esp_timer_stop(handle->lvgl_tick_timer);
        esp_timer_delete(handle->lvgl_tick_timer);
    }

    if (handle->disp != NULL) {
        lv_disp_remove(handle->disp);
    }

    if (handle->panel_handle != NULL) {
        esp_lcd_panel_del(handle->panel_handle);
    }

    if (handle->owns_draw_buffers) {
        if (handle->buf1 != NULL) {
            heap_caps_free(handle->buf1);
        }
        if (handle->buf2 != NULL) {
            heap_caps_free(handle->buf2);
        }
    }

    if (handle->i2c_installed) {
        i2c_driver_delete(handle->config.i2c.port);
    }

    free(handle);
}

lv_disp_t *HDMI_GetDisplay(HDMI_Handle_t *handle)
{
    return handle == NULL ? NULL : handle->disp;
}

esp_lcd_panel_handle_t HDMI_GetPanelHandle(HDMI_Handle_t *handle)
{
    return handle == NULL ? NULL : handle->panel_handle;
}

lv_indev_t *HDMI_GetTouchIndev(HDMI_Handle_t *handle)
{
    return handle == NULL ? NULL : handle->indev;
}

void HDMI_USBTouchSetState(HDMI_Handle_t *handle, int32_t x, int32_t y, bool pressed)
{
    HDMI_SetTouchState(handle, x, y, pressed);
}

bool HDMI_USBTouchHandleInputReport(HDMI_Handle_t *handle, const uint8_t *report, size_t report_len)
{
    if (handle == NULL || report == NULL) {
        return false;
    }

    return HDMI_HandleUSBTouchReport(handle, report, report_len);
}

uint32_t HDMI_LVGLTaskHandler(void)
{
    if (!lv_is_initialized()) {
        return 1;
    }

    {
        uint32_t delay_ms = lv_timer_handler();
        return delay_ms == 0 ? 1 : delay_ms;
    }
}

void HDMI_LVGLTask(void *pvParameters)
{
    (void)pvParameters;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(HDMI_LVGLTaskHandler()));
    }
}
