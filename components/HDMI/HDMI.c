#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "HDMI.h"
#include "../ESPMI/include/ESPMI.h"
#include "esp_attr.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_err.h"
#include "usb/usb_host.h"
#include "usb/hid_host.h"



#define HDMI_BRIDGE_ADDRESS0		0x70

/* vendor id low and high register*/
#define HDMI_BRIDGE_VENDOR_ID_L		0x00
#define HDMI_BRIDGE_VENDOR_ID_H		0x01

/*device id registers*/
#define HDMI_BRIDGE_DEVICE_ID_L		0x02
#define HDMI_BRIDGE_DEVICE_ID_H		0x03

/* revision registers*/
#define HDMI_BRIDGE_REV_ID			0x04

/* control mode register 1*/
#define HDMI_BRIDGE_CTL_1_MODE		0x08

/* control mode register 2*/
#define HDMI_BRIDGE_CTL_2_MODE		0x09

/* control mode register 3*/
#define HDMI_BRIDGE_CTL_3_MODE		0x0A

/* cfg register*/
#define HDMI_BRIDGE_CFG				0x0B

#define HDMI_BRIDGE_DE_DLY			0x32
#define HDMI_BRIDGE_CTL				0x33


#if CONFIG_DOUBLE_FB
#define LCD_NUM_FB             2
#else
#define LCD_NUM_FB             1
#endif
#define LVGL_TICK_PERIOD_MS    2


#define HDMI_BRIDGE_PCLK     (24 * 1000 * 1000)

#ifndef ESPMI_MINI
#define RGB_PIN_HSYNC          	6
#define RGB_PIN_VSYNC          	7
#define RGB_PIN_DE             	15
#define RGB_PIN_CLK          	 8
#define RGB_PIN_B0          	16 // B3
#define RGB_PIN_B1          	17 // B4
#define RGB_PIN_B2          	18 // B5
#define RGB_PIN_B3          	3 // B6
#define RGB_PIN_B4          	46 // B7
#define RGB_PIN_G0          	0 // G2
#define RGB_PIN_G1          	9 // G3
#define RGB_PIN_G2          	10 // G4
#define RGB_PIN_G3          	11 // G5
#define RGB_PIN_G4          	12 // G6
#define RGB_PIN_G5         		13 // G7
#define RGB_PIN_R0         		14  // R3
#define RGB_PIN_R1         		21  // R4
#define RGB_PIN_R2         		47 // R5
#define RGB_PIN_R3         		48 // R6
#define RGB_PIN_R4         		45 // R7
#define RGB_DISP_EN        		-1
#else
#define RGB_PIN_HSYNC          	10
#define RGB_PIN_VSYNC          	9
#define RGB_PIN_DE             	11
#define RGB_PIN_CLK          	 48
#define RGB_PIN_B0          	12 // B3
#define RGB_PIN_B1          	13 // B4
#define RGB_PIN_B2          	14 // B5
#define RGB_PIN_B3          	21 // B6
#define RGB_PIN_B4          	47 // B7
#define RGB_PIN_G0          	45 // G3
#define RGB_PIN_G1          	4 // G4
#define RGB_PIN_G2          	5 // G5
#define RGB_PIN_G3          	6 // G6
#define RGB_PIN_G4          	38 // G7
//#define RGB_PIN_G5         		13 // G7
#define RGB_PIN_R0         		39  // R2
#define RGB_PIN_R1         		40  // R3
#define RGB_PIN_R2         		41 // R4
#define RGB_PIN_R3         		42 // R5
#define RGB_PIN_R4         		44 // R6
#define RGB_PIN_R5         		43 // R7
#define RGB_DISP_EN        		-1
#endif

#define LCD_H_RES              640
#define	LCD_V_RES              480


static const char* TAG = "HDMI";
static esp_lcd_panel_handle_t panelHandle = NULL;
static lv_disp_draw_buf_t disp_buf;
static lv_disp_drv_t disp_drv;
static void *buf1 = NULL;
static void *buf2 = NULL;
static lv_disp_t *disp = NULL;
static lv_indev_drv_t indev_drv;

int x_coord = 0;
int y_coord = 0;
int touchscreen_interrupt = 0;
int isPressed = 0;

void touchscreen_read(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {
    if (touchscreen_interrupt) {

        // Preencher os dados da estrutura lv_indev_data_t
        data->point.x = x_coord;
        data->point.y = y_coord;

        // Definir o estado do toque (LV_INDEV_STATE_PR ou LV_INDEV_STATE_REL)
        data->state = isPressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;

        touchscreen_interrupt = false;  // Resetar a flag de interrupção
    }
}


void TouchScreenUSB_Initialize(void){

}

IRAM_ATTR static bool OnVSYNCCallBack(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
	TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup;
    vTaskNotifyGiveFromISR(task_to_notify, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}


static void LVGL_FlushCallback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}


static void LVGL_IncTick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static esp_err_t HDMI_BRIDGE_Init(void){

	esp_err_t err = ESP_OK;
#ifndef ESPMI_MINI
	uint8_t temp = 0;
	err |= ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, HDMI_BRIDGE_CTL_1_MODE, 0x00);
	vTaskDelay(100);
	err |= ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, HDMI_BRIDGE_CTL_1_MODE, 0x0F);

	err |= ESPMI_I2CRead(HDMI_ADDRESS >> 1, HDMI_BRIDGE_CTL_2_MODE, &temp, 1);

	temp |= 0x39;
	err |= ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, HDMI_BRIDGE_CTL_2_MODE, temp);
#else
	uint8_t buffer[2];
	uint16_t deviceId;

	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x98, 0x03);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x01, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x02, 0x18);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x03, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x14, 0x70);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x15, 0x20);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x16, 0x30);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x18, 0x46);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x40, 0x80);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x41, 0x10);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x49, 0xA8);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x55, 0x10);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x56, 0x98);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x96, 0xF6);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x73, 0x07);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x76, 0x1F);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x98, 0x03);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x99, 0x02);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x9A, 0xE0);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x9C, 0x30);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x9D, 0x61);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xA2, 0xA4);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xA3, 0xA4);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xA5, 0x04);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xAB, 0x40);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xAF, 0x16);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xBA, 0x60);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xD1, 0xFF);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xDE, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xE4, 0x60);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0xFA, 0x7D);

	//ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x17, 0x60);



	//Full rbg out

	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x18, 0xA8);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x19, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x1A, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x1B, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x1C, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x1D, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x1E, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x1F, 0x00);

	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x20, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x21, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x22, 0x08);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x23, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x24, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x25, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x26, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x27, 0x00);

	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x28, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x29, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x2A, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x2B, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x2C, 0x08);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x2D, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x2E, 0x00);
	ESPMI_I2CWriteByte(HDMI_ADDRESS >> 1, 0x2F, 0x00);

	ESPMI_I2CRead(HDMI_ADDRESS >> 1, 0xf5, buffer, 2);
	deviceId = (uint16_t)buffer[0] << 8;
	deviceId |= buffer[1];
	ESP_LOGI(TAG, "Device ADV7513 Chip ID = %i\r\n", deviceId);


#endif

	return err;
}


static esp_lcd_panel_handle_t HDMI_InitRGBBus(lv_disp_drv_t* disp_drive, _Bool refresh_on_demand, esp_lcd_rgb_panel_vsync_cb_t vsync_callback, const void* user_data){
    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .psram_trans_align = 64,
        .num_fbs = LCD_NUM_FB,
//#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 20	 * LCD_H_RES,
//#endif
        .clk_src = LCD_CLK_SRC_PLL240M,
        .disp_gpio_num = RGB_DISP_EN,
        .pclk_gpio_num = RGB_PIN_CLK,
        .vsync_gpio_num = RGB_PIN_VSYNC,
        .hsync_gpio_num = RGB_PIN_HSYNC,
        .de_gpio_num = RGB_PIN_DE,
        .data_gpio_nums = {
            RGB_PIN_B0,
            RGB_PIN_B1,
            RGB_PIN_B2,
            RGB_PIN_B3,
            RGB_PIN_B4,
            RGB_PIN_G0,
            RGB_PIN_G1,
            RGB_PIN_G2,
            RGB_PIN_G3,
            RGB_PIN_G4,
           RGB_PIN_G5,
            RGB_PIN_R0,
            RGB_PIN_R1,
            RGB_PIN_R2,
            RGB_PIN_R3,
            RGB_PIN_R4,
			// RGB_PIN_R5
        },

        .timings = {
            .pclk_hz = HDMI_BRIDGE_PCLK,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            // The following parameters should refer to LCD spec
            .hsync_back_porch = 80,
            .hsync_front_porch = 16,
            .hsync_pulse_width = 64,
            .vsync_back_porch = 13,
            .vsync_front_porch = 3,
            .vsync_pulse_width = 4,
            .flags.pclk_active_neg = false,
			.flags.vsync_idle_low = true,
			.flags.hsync_idle_low = true,
			.flags.de_idle_high = false,
			.flags.pclk_idle_high = false,

        },

		.flags.refresh_on_demand = refresh_on_demand,
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM

    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = OnVSYNCCallBack,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, xTaskGetCurrentTaskHandle()));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    return panel_handle;
}



lv_disp_t* HDMI_Initialize(void* context){
	HDMI_BRIDGE_Init();

	panelHandle = HDMI_InitRGBBus(NULL, false, OnVSYNCCallBack, context);

	TouchScreenUSB_Initialize();

	ESP_LOGI(TAG, "Initialize LVGL library");
	lv_init();

#if CONFIG_DOUBLE_FB
	ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
	ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panelHandle, 2, &buf1, &buf2));
	    // initialize LVGL draw buffers
	lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LCD_V_RES);
#else
	ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
	buf1 = malloc(LCD_H_RES * 100 * sizeof(lv_color_t));
	assert(buf1);
	buf2 = malloc(LCD_H_RES * 100 * sizeof(lv_color_t));
	assert(buf2);
	// initialize LVGL draw buffers
	lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 100);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

	ESP_LOGI(TAG, "Register display driver to LVGL");
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = LCD_H_RES;
	disp_drv.ver_res = LCD_V_RES;
	disp_drv.flush_cb = LVGL_FlushCallback;
	disp_drv.draw_buf = &disp_buf;
	disp_drv.user_data = panelHandle;
#if CONFIG_DOUBLE_FB
	disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
	disp = lv_disp_drv_register(&disp_drv);


	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = &touchscreen_read; // Substitua 'touchscreen_read' pela sua função de leitura de touchscreen
	lv_indev_drv_register(&indev_drv);

	ESP_LOGI(TAG, "Install LVGL tick timer");
	// Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
	const esp_timer_create_args_t lvgl_tick_timer_args = {
		.callback = &LVGL_IncTick,
		.name = "lvgl_tick"
	};
	esp_timer_handle_t lvgl_tick_timer = NULL;
	ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

	return disp;
}

void LVGL_Task(void* pvParameters){
	uint32_t time_till_next = 0;
	for(;;){
		time_till_next = lv_timer_handler();
		vTaskDelay(time_till_next);

	}
}
