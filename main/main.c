#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ESPMI.h"
#include "ebyte_core.h"
#include "ebyte_port.h"
#include "ebyte_callback.h"
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



void app_main(void)
{
	TaskHandle_t gui;
	ESPMI_ConfigI2C(ESPMI_I2C_MASTER_NUM);
	ESPMI_ConfigSPI(SPI2_HOST);
	ESPMI_ConfigGPIO();
	ESPMI_MCP23008_ConfigGpioDir(0x71);

	Ebyte_RF.Init();
	Ebyte_RF.EnterReceiveMode(0);
	disp = HDMI_Initialize(xTaskGetCurrentTaskHandle());
	ui_init();
	xTaskCreatePinnedToCore(RF_Task, "rf_task", 1024*4, NULL, 5, NULL, 0);
//	xTaskCreatePinnedToCore(LVGL_Task, "guiTask", 1024*4, NULL, 4, &gui, 0);

    while (1) {
    	LVGL_Task(NULL);
    }
}
