#ifndef HDMI_H_
#define HDMI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HDMI_RGB_DATA_WIDTH                         16U
#define HDMI_RGB_DATA_PINS                          16U
#define HDMI_DEFAULT_I2C_CLK_SPEED_HZ               100000UL
#define HDMI_DEFAULT_I2C_TIMEOUT_MS                 1000UL
#define HDMI_DEFAULT_LVGL_TICK_PERIOD_MS            2UL
#define HDMI_DEFAULT_DRAW_BUFFER_LINES              100U
#define HDMI_TFP410_I2C_ADDRESS                     0x38U
#define HDMI_USB_TOUCH_DEFAULT_RAW_MAX_X            16383U
#define HDMI_USB_TOUCH_DEFAULT_RAW_MAX_Y            16282U
#define HDMI_USB_TOUCH_DEFAULT_MIN_REPORT_LEN       6U
#define HDMI_USB_TOUCH_DEFAULT_PRESSED_BIT_MASK     0x40U
#define HDMI_USB_TOUCH_DEFAULT_USB_LIB_STACK_SIZE   4096U
#define HDMI_USB_TOUCH_DEFAULT_HID_BG_STACK_SIZE    4096U
#define HDMI_USB_TOUCH_DEFAULT_HID_EVT_STACK_SIZE   4096U

typedef struct {
    i2c_port_t port;
    gpio_num_t sda_gpio_num;
    gpio_num_t scl_gpio_num;
    uint32_t clk_speed_hz;
    uint32_t timeout_ms;
    uint8_t device_address;
    bool sda_pullup_en;
    bool scl_pullup_en;
    bool install_driver;
} HDMI_I2CConfig_t;

typedef bool (*HDMI_USBTouchReportParser_t)(const uint8_t *report,
                                            size_t report_len,
                                            void *user_ctx,
                                            int32_t *raw_x,
                                            int32_t *raw_y,
                                            bool *pressed);

typedef struct {
    bool enabled;
    bool manage_usb_host;
    bool skip_phy_setup;
    int intr_flags;
    size_t usb_lib_task_stack_size;
    int usb_lib_task_priority;
    int usb_lib_task_core_id;
    size_t hid_background_task_stack_size;
    int hid_background_task_priority;
    int hid_background_task_core_id;
    size_t hid_event_task_stack_size;
    int hid_event_task_priority;
    int hid_event_task_core_id;
    uint16_t raw_max_x;
    uint16_t raw_max_y;
    bool swap_xy;
    bool invert_x;
    bool invert_y;
    uint8_t pressed_byte_index;
    uint8_t pressed_bit_mask;
    uint8_t x_lsb_byte_index;
    uint8_t x_msb_byte_index;
    uint8_t y_lsb_byte_index;
    uint8_t y_msb_byte_index;
    size_t min_report_len;
    HDMI_USBTouchReportParser_t parser_cb;
    void *parser_user_ctx;
} HDMI_USBTouchConfig_t;

typedef enum {
    HDMI_RENDER_PROFILE_DEFAULT = 0,
    HDMI_RENDER_PROFILE_LOW_LATENCY,
    HDMI_RENDER_PROFILE_HEAVY_ANIMATION,
} HDMI_RenderProfile_t;

typedef struct {
    HDMI_I2CConfig_t i2c;
    HDMI_USBTouchConfig_t usb_touch;
    esp_lcd_rgb_panel_config_t panel_config;
    esp_lcd_rgb_panel_event_callbacks_t panel_callbacks;
    void *panel_user_ctx;
    bool register_panel_callbacks;
    size_t lvgl_draw_buffer_height;
    uint32_t lvgl_tick_period_ms;
    bool use_panel_frame_buffers;
    bool lvgl_full_refresh;
    bool start_lvgl_tick_timer;
} HDMI_Config_t;

typedef struct HDMI_Context HDMI_Handle_t;

void HDMI_GetDefaultConfig(HDMI_Config_t *config);
void HDMI_ApplyRenderProfile(HDMI_Config_t *config, HDMI_RenderProfile_t profile);
esp_err_t HDMI_Initialize(const HDMI_Config_t *config, HDMI_Handle_t **ret_handle);
void HDMI_Delete(HDMI_Handle_t *handle);
lv_disp_t *HDMI_GetDisplay(HDMI_Handle_t *handle);
esp_lcd_panel_handle_t HDMI_GetPanelHandle(HDMI_Handle_t *handle);
lv_indev_t *HDMI_GetTouchIndev(HDMI_Handle_t *handle);
void HDMI_USBTouchSetState(HDMI_Handle_t *handle, int32_t x, int32_t y, bool pressed);
bool HDMI_USBTouchHandleInputReport(HDMI_Handle_t *handle, const uint8_t *report, size_t report_len);
uint32_t HDMI_LVGLTaskHandler(void);
void HDMI_LVGLTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
