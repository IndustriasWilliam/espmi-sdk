#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <freertos/queue.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ESPMI.h"
#include "ebyte_core.h"
#include "ebyte_port.h"
#include "ebyte_callback.h"
#include <usb/hid_host.h>
#include <usb/usb_host.h>
#include "HDMI.h"
#include "lvgl.h"
#include "ui.h"


static lv_disp_t* disp = NULL;

typedef struct {
	uint16_t temperature;
	uint16_t humidity;
	int16_t accX;
	int16_t accY;
	int16_t accZ;
}SensorBox_TypeDef;

SensorBox_TypeDef* sensors;
uint32_t packetsReceived = 0;
volatile static float temperature;
volatile static float humidity;
static float accX;
static float accY;
static float accZ;
static int8_t rssi;
static volatile int8_t rx_done = 0;

extern int x_coord;
extern int y_coord;
extern int touchscreen_interrupt;
extern int isPressed;
static const char TAG [] = "Main";

QueueHandle_t hid_host_event_queue;

/**
 * @brief HID Host event
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct {
    hid_host_device_handle_t hid_device_handle;
    hid_host_driver_event_t event;
    void *arg;
} hid_host_event_queue_t;


/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = {
    "NONE",
    "KEYBOARD",
    "MOUSE"
};


extern int8_t Ebyte_E220x_GetRssiInst( void );

float lis3dh_from_fs16_hr_to_mg(int16_t lsb){
	return ((float)lsb / 16.0f) * 12.0f;
}

void Ebyte_Port_TransmitCallback( uint16_t state ){

}
void Ebyte_Port_ReceiveCallback(  uint16_t state ,uint8_t *buffer, uint8_t length){
	if(state == 0x0002){
		sensors = (SensorBox_TypeDef*)buffer;
		temperature = -45.0f + 175.0f * (float)sensors->temperature/65535.0f;
		humidity = 100 * ((float)sensors->humidity/65535.0f);
		accX = lis3dh_from_fs16_hr_to_mg(sensors->accX)/100.0f;
		accY = lis3dh_from_fs16_hr_to_mg(sensors->accY)/100.0f;
		accZ = lis3dh_from_fs16_hr_to_mg(sensors->accZ)/100.0f;
		packetsReceived++;
		rx_done = 1;

	}

}

uint8_t Ebyte_BSP_SpiTransAndRecv(uint8_t data){
	spi_transaction_t spiTrans;
	const char* TAG = "E220 SPI transmitter";

	memset(&spiTrans, 0, sizeof(spi_transaction_t));

	spiTrans.length = 8;
	spiTrans.tx_data[0] = data;

	spiTrans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;

	if(spi_device_polling_transmit(ESPMI_GetSPIHandle(), &spiTrans) != ESP_OK){
		ESP_LOGE(TAG, "SPI transmission failed");
	}

	return spiTrans.rx_data[0];
}

void Ebyte_BSP_RfSpiUnselected( void ){
	gpio_set_level(ESPMI_E220_900MM22S_NSS, 1);
}
void Ebyte_BSP_RfSpiSelected( void ){
	gpio_set_level(ESPMI_E220_900MM22S_NSS, 0);
}
void Ebyte_BSP_RfResetIoHigh( void ){
	ESPMI_MCP23008_SetLevelGpioPin(ESPMI_E220_900MM22S_RST, True);

}
void Ebyte_BSP_RfResetIoLow( void ){
	ESPMI_MCP23008_SetLevelGpioPin(ESPMI_E220_900MM22S_RST, False);
}

void Ebyte_BSP_RfTxIoEnable( void ){
	ESPMI_MCP23008_SetLevelGpioPin(ESPMI_E220_900MM22S_TXEN, True);
}

void Ebyte_BSP_RfTxIoDisable( void ){
	ESPMI_MCP23008_SetLevelGpioPin(ESPMI_E220_900MM22S_TXEN, False);
}
void Ebyte_BSP_RfRxIoEnable( void ){
	ESPMI_MCP23008_SetLevelGpioPin(ESPMI_E220_900MM22S_RXEN, True);
}
void Ebyte_BSP_RfRxIoDisable( void ){
	ESPMI_MCP23008_SetLevelGpioPin(ESPMI_E220_900MM22S_RXEN, False);
}
uint8_t Ebyte_BSP_RfBusyIoRead( void ){
	Boolean state = False;
	ESPMI_MCP23008_ReadLevelPin(ESPMI_E220_900MM22S_BUSY, &state);

	return (uint8_t)state;
}

void Ebyte_Port_DelayMs( uint32_t time ){
	vTaskDelay(time/portTICK_PERIOD_MS);
}


inline int map_coord(int coordenada_touchscreen, int max_touchscreen, int max_tela) {
    return ((float)coordenada_touchscreen / max_touchscreen) * max_tela;
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data, const int length)
{
//	TouchInputReport_t input_report;
//	memcpy(&input_report, (const void *) data, sizeof(TouchInputReport_t));
	uint16_t x = ((uint8_t)data[3] << 8)| data[2];
	uint16_t y = ((uint8_t)data[5] << 8)| data[4];

	x_coord = map_coord(x, 16383, 640);
	y_coord = map_coord(y, 16282, 480);
	touchscreen_interrupt = 1;
	isPressed = data[1] >> 6;
//	printf("%i %i %i %i\r\n", data[2], data[3], data[4], data[5]);
//	printf("%i %i %i %i\r\n", input_report->data[0].x_lsb,
//			input_report->data[0].x_msb, input_report->data[0].y_lsb, input_report->data[0].y_msb);
	printf("%i %i\r\n", x_coord , y_coord);
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle, const hid_host_interface_event_t event, void *arg) {
    uint8_t data[64] = { 0 };
    size_t data_length = 0;
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK( hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        ESP_ERROR_CHECK( hid_host_device_get_raw_input_report_data(hid_device_handle,
                         data,
                         64,
                         &data_length));

        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
            if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
//                hid_host_keyboard_report_callback(data, data_length);
            } else if (HID_PROTOCOL_MOUSE == dev_params.proto) {
//                hid_host_mouse_report_callback(data, data_length);
            }
        } else {
            hid_host_generic_report_callback(data, data_length);
        }

        break;
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HID Device, protocol '%s' DISCONNECTED",
                 hid_proto_name_str[dev_params.proto]);
        ESP_ERROR_CHECK( hid_host_device_close(hid_device_handle) );
        break;
    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGI(TAG, "HID Device, protocol '%s' TRANSFER_ERROR",
                 hid_proto_name_str[dev_params.proto]);
        break;
    default:
        ESP_LOGE(TAG, "HID Device, protocol '%s' Unhandled event",
                 hid_proto_name_str[dev_params.proto]);
        break;
    }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle, const hid_host_driver_event_t event, void *arg) {
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK( hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event) {
		case HID_HOST_DRIVER_EVENT_CONNECTED:
			ESP_LOGI(TAG, "HID Device, protocol '%s' CONNECTED", hid_proto_name_str[dev_params.proto]);

			const hid_host_device_config_t dev_config = {
				.callback = hid_host_interface_callback,
				.callback_arg = NULL
			};

			ESP_ERROR_CHECK( hid_host_device_open(hid_device_handle, &dev_config) );
			if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
				ESP_ERROR_CHECK( hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT));
				if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
					ESP_ERROR_CHECK( hid_class_request_set_idle(hid_device_handle, 0, 0));
				}
			}
			ESP_ERROR_CHECK( hid_host_device_start(hid_device_handle) );
			break;
		default:
			break;
    }
}

/**
 * @brief Start USB Host install and handle common USB host library events while app pin not low
 *
 * @param[in] arg  Not used
 */
static void usb_lib_task(void *arg){

	const usb_host_config_t host_config = {
		.skip_phy_setup = false,
		.intr_flags = ESP_INTR_FLAG_LEVEL1,

	};

	ESP_ERROR_CHECK( usb_host_install(&host_config) );
	xTaskNotifyGive(arg);

	for(;;){
		uint32_t event_flags;
		usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

		// Release devices once all clients has deregistered
		if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
			usb_host_device_free_all();
			ESP_LOGI(TAG, "USB Event flags: NO_CLIENTS");
		}
		// All devices were removed
		if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
			ESP_LOGI(TAG, "USB Event flags: ALL_FREE");
		}
	}
}


/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event,
                              void *arg)
{
    const hid_host_event_queue_t evt_queue = {
        .hid_device_handle = hid_device_handle,
        .event = event,
        .arg = arg
    };
    xQueueSend(hid_host_event_queue, &evt_queue, 0);
}


/**
 * @brief HID Host main task
 *
 * Creates queue and get new event from the queue
 *
 * @param[in] pvParameters Not used
 */
void hid_host_task(void *pvParameters)
{
    hid_host_event_queue_t evt_queue;
    // Create queue
    hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));

    for(;;) {
        if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(50))) {
            hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event, evt_queue.arg);
        }
    }
}

#if 0

void RF_Task(void* pvParameters){
	char humidityValue [14];
	char temperatureValue [14];
	char receivedPackets[32];
	char loraRSSI[16];
	lv_coord_t ui_Chart2_series_1_array[10];
	lv_coord_t ui_Chart2_series_2_array[10];
	lv_coord_t ui_Chart2_series_3_array[10];
	uint8_t i = 0;

	extern lv_chart_series_t * ui_Chart2_series_1;
	extern lv_chart_series_t * ui_Chart2_series_2;
	extern lv_chart_series_t * ui_Chart2_series_3;

	for(;;){

		Ebyte_RF.StartPollTask();
		if(rx_done){
			rx_done = 0;
			rssi = Ebyte_E220x_GetRssiInst();
			sprintf(humidityValue,"%3.1f %c", humidity, '%');
			sprintf(temperatureValue,"%.1f °C", temperature);
			sprintf(receivedPackets,"Pacotes Recebidos: %li", packetsReceived);
			sprintf(loraRSSI,"LoRa RSSI: %i", rssi);

			if(i < 10){
				ui_Chart2_series_1_array[i] = (int16_t)accX;
				ui_Chart2_series_2_array[i] = (int16_t)accY;
				ui_Chart2_series_3_array[i] = (int16_t)accZ;
				i++;
			}
			else{
				i = 0;
				lv_chart_set_ext_y_array(ui_Chart2, ui_Chart2_series_1, ui_Chart2_series_1_array);
				lv_chart_set_ext_y_array(ui_Chart2, ui_Chart2_series_2, ui_Chart2_series_2_array);
				lv_chart_set_ext_y_array(ui_Chart2, ui_Chart2_series_3, ui_Chart2_series_3_array);
			}
			lv_label_set_text(ui_labelHum, humidityValue);
			lv_label_set_text(ui_labelTemp, temperatureValue);
			lv_label_set_text(ui_Label4, receivedPackets);
			lv_label_set_text(ui_Label5, loraRSSI);

		}
		vTaskDelay(250);
	}
}

#endif

void app_main(void)
{

#ifdef ESPMI_MINI
	gpio_config_t config;

	config.mode = GPIO_MODE_OUTPUT;
	config.pin_bit_mask = ((uint64_t)1 << ESPMI_ADV_PD);
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.pull_up_en = GPIO_PULLUP_DISABLE;

	gpio_config(&config);

	gpio_set_level(ESPMI_ADV_PD, 1);
	vTaskDelay(1000);

	gpio_set_level(ESPMI_ADV_PD, 0);
	vTaskDelay(10);
	gpio_set_level(ESPMI_ADV_PD, 1);
#endif

//	TaskHandle_t gui;


//	Ebyte_RF.Init();
//	Ebyte_RF.EnterReceiveMode(0);
//	xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096, xTaskGetCurrentTaskHandle(), 2, NULL, 0);
//
	ESPMI_ConfigI2C(ESPMI_I2C_MASTER_NUM);
//	ESPMI_ConfigSPI(SPI2_HOST);
//	ESPMI_ConfigGPIO();
//	ESPMI_MCP23008_ConfigGpioDir(0x71);
//
//	ulTaskNotifyTake(false, 1000);
//	/*
//	* HID host driver configuration
//	* - create background task for handling low level event inside the HID driver
//	* - provide the device callback to get new HID Device connection event
//	*/
//	const hid_host_driver_config_t hid_host_driver_config = {
//		.create_background_task = true,
//		.task_priority = 5,
//		.stack_size = 4096,
//		.core_id = 0,
//		.callback = hid_host_device_callback,
//		.callback_arg = NULL
//	};
//
//	ESP_ERROR_CHECK( hid_host_install(&hid_host_driver_config) );
//
//	/*
//	* Create HID Host task process for handle events
//	* IMPORTANT: Task is necessary here while there is no possibility to interact
//	* with USB device from the callback.
//	*/
//	xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2, NULL);
	disp = HDMI_Initialize(xTaskGetCurrentTaskHandle());
	ui_init();
//	xTaskCreatePinnedToCore(RF_Task, "rf_task", 1024*4, NULL, 5, NULL, 0);
//	xTaskCreatePinnedToCore(LVGL_Task, "guiTask", 1024*4, NULL, 4, &gui, 0);
    while (1) {
    	LVGL_Task(NULL);
//    	vTaskDelay(500);
    }
}
