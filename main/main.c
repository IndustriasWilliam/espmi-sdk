#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"

#include "usb/hid.h"
#include "usb/hid_host.h"
#include "usb/usb_host.h"

#include "HDMI.h"
#include "lvgl.h"
#include "lvgl__lvgl/demos/lv_demos.h"

static lv_disp_t *disp = NULL;
static HDMI_Handle_t *hdmi = NULL;
static QueueHandle_t hid_host_event_queue = NULL;
static HDMI_USBTouchConfig_t usb_touch_config = {0};

typedef struct {
    hid_host_device_handle_t hid_device_handle;
    hid_host_driver_event_t event;
    void *arg;
} app_hid_host_event_t;

static BaseType_t app_create_task(TaskFunction_t task_func,
                                  const char *name,
                                  uint32_t stack_size,
                                  void *arg,
                                  UBaseType_t priority,
                                  int core_id,
                                  TaskHandle_t *task_handle)
{
    if (core_id >= 0) {
        return xTaskCreatePinnedToCore(task_func, name, stack_size, arg, priority, task_handle, core_id);
    }

    return xTaskCreate(task_func, name, stack_size, arg, priority, task_handle);
}

static void hid_host_generic_report_callback(const uint8_t *data, int length)
{
    if (hdmi == NULL) {
        return;
    }

    (void)HDMI_USBTouchHandleInputReport(hdmi, data, (size_t)length);
}

static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                        const hid_host_interface_event_t event,
                                        void *arg)
{
    uint8_t data[64] = {0};
    size_t data_length = 0;
    hid_host_dev_params_t dev_params = {0};

    (void)arg;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(
            hid_device_handle,
            data,
            sizeof(data),
            &data_length));

        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
            if (HID_PROTOCOL_KEYBOARD == dev_params.proto || HID_PROTOCOL_MOUSE == dev_params.proto) {
                break;
            }
        }

        hid_host_generic_report_callback(data, (int)data_length);
        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        if (hdmi != NULL) {
            HDMI_USBTouchSetState(hdmi, 0, 0, false);
        }
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    default:
        break;
    }
}

static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  hid_host_driver_event_t event,
                                  void *arg)
{
    hid_host_dev_params_t dev_params = {0};

    (void)arg;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event) {
    case HID_HOST_DRIVER_EVENT_CONNECTED: {
        const hid_host_device_config_t dev_config = {
            .callback = hid_host_interface_callback,
            .callback_arg = NULL,
        };

        ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
            ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT));
            if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
                ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle, 0, 0));
            }
        }
        ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
        break;
    }

    default:
        break;
    }
}

static void usb_lib_task(void *arg)
{
    TaskHandle_t notify_task = (TaskHandle_t)arg;
    const usb_host_config_t host_config = {
        .skip_phy_setup = usb_touch_config.skip_phy_setup,
        .intr_flags = usb_touch_config.intr_flags == 0 ? ESP_INTR_FLAG_LEVEL1 : usb_touch_config.intr_flags,
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskNotifyGive(notify_task);

    while (true) {
        uint32_t event_flags = 0;

        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
    }
}

static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                     hid_host_driver_event_t event,
                                     void *arg)
{
    const app_hid_host_event_t evt_queue = {
        .hid_device_handle = hid_device_handle,
        .event = event,
        .arg = arg,
    };

    if (hid_host_event_queue != NULL) {
        xQueueSend(hid_host_event_queue, &evt_queue, 0);
    }
}

static void hid_host_task(void *pvParameters)
{
    app_hid_host_event_t evt_queue;

    (void)pvParameters;

    while (true) {
        if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(50))) {
            hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event, evt_queue.arg);
        }
    }
}

static void start_usb_touch_host(const HDMI_USBTouchConfig_t *touch_config)
{
    BaseType_t task_created;
    usb_touch_config = *touch_config;

    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = touch_config->hid_background_task_priority,
        .stack_size = touch_config->hid_background_task_stack_size,
        .core_id = touch_config->hid_background_task_core_id,
        .callback = hid_host_device_callback,
        .callback_arg = NULL,
    };

    hid_host_event_queue = xQueueCreate(10, sizeof(app_hid_host_event_t));
    configASSERT(hid_host_event_queue != NULL);

    task_created = app_create_task(
        usb_lib_task,
        "usb_events",
        touch_config->usb_lib_task_stack_size,
        xTaskGetCurrentTaskHandle(),
        (UBaseType_t)touch_config->usb_lib_task_priority,
        touch_config->usb_lib_task_core_id,
        NULL);
    configASSERT(task_created == pdTRUE);

    configASSERT(ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000)) != 0);

    ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

    task_created = app_create_task(
        hid_host_task,
        "hid_task",
        touch_config->hid_event_task_stack_size,
        NULL,
        (UBaseType_t)touch_config->hid_event_task_priority,
        touch_config->hid_event_task_core_id,
        NULL);
    configASSERT(task_created == pdTRUE);
}

void app_main(void)
{
    static const bool enable_usb_touch = true;
    static const bool run_benchmark_demo = false;
    static const int hdmi_data_pins[HDMI_RGB_DATA_PINS] = {
        GPIO_NUM_16,
        GPIO_NUM_17,
        GPIO_NUM_18,
        GPIO_NUM_3,
        GPIO_NUM_46,
        GPIO_NUM_0,
        GPIO_NUM_9,
        GPIO_NUM_10,
        GPIO_NUM_11,
        GPIO_NUM_12,
        GPIO_NUM_13,
        GPIO_NUM_14,
        GPIO_NUM_21,
        GPIO_NUM_47,
        GPIO_NUM_48,
        GPIO_NUM_45
    };
    HDMI_Config_t hdmi_config;

    HDMI_GetDefaultConfig(&hdmi_config);

    hdmi_config.i2c.port = I2C_NUM_0;
    hdmi_config.i2c.sda_gpio_num = GPIO_NUM_5;
    hdmi_config.i2c.scl_gpio_num = GPIO_NUM_4;
    hdmi_config.i2c.clk_speed_hz = 100000;
    hdmi_config.i2c.install_driver = true;

    hdmi_config.panel_config.pclk_gpio_num = GPIO_NUM_8;
    hdmi_config.panel_config.vsync_gpio_num = GPIO_NUM_7;
    hdmi_config.panel_config.hsync_gpio_num = GPIO_NUM_6;
    hdmi_config.panel_config.de_gpio_num = GPIO_NUM_15;
    hdmi_config.panel_config.disp_gpio_num = -1;
    memcpy(hdmi_config.panel_config.data_gpio_nums, hdmi_data_pins, sizeof(hdmi_data_pins));

    hdmi_config.panel_config.timings.pclk_hz = 24000000;
    hdmi_config.panel_config.timings.h_res = 640;
    hdmi_config.panel_config.timings.v_res = 480;
    hdmi_config.panel_config.timings.hsync_back_porch = 80;
    hdmi_config.panel_config.timings.hsync_front_porch = 16;
    hdmi_config.panel_config.timings.hsync_pulse_width = 64;
    hdmi_config.panel_config.timings.vsync_back_porch = 13;
    hdmi_config.panel_config.timings.vsync_front_porch = 3;
    hdmi_config.panel_config.timings.vsync_pulse_width = 4;

    hdmi_config.usb_touch.enabled = enable_usb_touch;
    hdmi_config.usb_touch.manage_usb_host = false;
    hdmi_config.usb_touch.raw_max_x = 16383;
    hdmi_config.usb_touch.raw_max_y = 16282;
    hdmi_config.usb_touch.swap_xy = false;
    hdmi_config.usb_touch.invert_x = false;
    hdmi_config.usb_touch.invert_y = false;

    HDMI_ApplyRenderProfile(
        &hdmi_config,
        run_benchmark_demo ? HDMI_RENDER_PROFILE_HEAVY_ANIMATION : HDMI_RENDER_PROFILE_LOW_LATENCY);

    if (enable_usb_touch) {
        start_usb_touch_host(&hdmi_config.usb_touch);
    }

    ESP_ERROR_CHECK(HDMI_Initialize(&hdmi_config, &hdmi));
    disp = HDMI_GetDisplay(hdmi);
    (void)disp;

    if (run_benchmark_demo) {
        lv_demo_benchmark();
    } else {
        lv_demo_widgets();
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HDMI_LVGLTaskHandler()));
    }
}
