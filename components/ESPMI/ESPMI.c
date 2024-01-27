#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "include/ESPMI.h"
#include "esp_err.h"
#include "esp_log.h"


static spi_device_handle_t spiHandler = NULL;

esp_err_t ESPMI_I2CRead(uint8_t device_address, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(ESPMI_I2C_MASTER_NUM, device_address, &reg_addr, 1, data, len, ESPMI_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t ESPMI_I2CWriteByte(uint8_t device_address, uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(ESPMI_I2C_MASTER_NUM, device_address, write_buf, sizeof(write_buf), ESPMI_I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

spi_device_handle_t ESPMI_GetSPIHandle(void){
	return spiHandler;
}


esp_err_t ESPMI_ConfigI2C(i2c_port_t port){
	i2c_config_t i2cConfig = {0};

	i2cConfig.mode = I2C_MODE_MASTER;
	i2cConfig.scl_io_num = ESPMI_I2C_SCL;
	i2cConfig.sda_io_num = ESPMI_I2C_SDA;
	i2cConfig.master.clk_speed = ESPMI_I2C_FREQ;
	i2cConfig.scl_pullup_en = true;
	i2cConfig.sda_pullup_en = true;

	if(i2c_param_config(port, &i2cConfig) == ESP_OK){
		return i2c_driver_install(port, i2cConfig.mode, 0, 0, 0);
	}
	return ESP_FAIL;
}

esp_err_t ESPMI_ConfigSPI(spi_host_device_t device){
	esp_err_t ret;
	spi_bus_config_t buscfg = {
			.miso_io_num = ESPMI_E220_900MM22S_MISO,
			.mosi_io_num = ESPMI_E220_900MM22S_MOSI,
			.sclk_io_num = ESPMI_E220_900MM22S_SCK,
			.quadwp_io_num= -1,
			.quadhd_io_num= -1,
			.max_transfer_sz = sizeof(uint32_t)
	};
	spi_device_interface_config_t devcfg={
			.clock_speed_hz=10*1000*1000,
			.mode = 0,
			.spics_io_num = -1,
			.queue_size=7,
	};
	if(spi_bus_initialize(device, &buscfg, SPI_DMA_DISABLED) != ESP_FAIL){
		return spi_bus_add_device(device, &devcfg, &spiHandler);
	}

	return ESP_FAIL;
}

esp_err_t ESPMI_ConfigGPIO(void){
	gpio_config_t gpioConfig = {0};

	gpioConfig.intr_type = GPIO_INTR_DISABLE;
	gpioConfig.mode = GPIO_MODE_OUTPUT;
	gpioConfig.pin_bit_mask = ((uint64_t)1 << ESPMI_E220_900MM22S_NSS);
	gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;


	return gpio_config(&gpioConfig);
}


esp_err_t ESPMI_MCP23008_ConfigGpioDir(uint8_t iodir_mask){
	return ESPMI_I2CWriteByte(ESPMI_MCP23008_ADDRESS>>1, ESPMI_MCP23008_IODIR, iodir_mask);
}

esp_err_t ESPMI_MCP23008_SetLevelGpioPin(uint8_t gpio_pin, ESPMI_MCPGpioLevel level){

	const char* TAG = "MCP23008 GPIO Set Level";
	uint8_t reg_value = 0;

	if(gpio_pin > GPIO_NUM_7){
		ESP_LOGE(TAG, "GPIO pin out range");
		return ESP_FAIL;
	}

	if(ESPMI_I2CRead(ESPMI_MCP23008_ADDRESS >> 1, ESPMI_MCP23008_OLAT, &reg_value, 1) != ESP_OK){
		ESP_LOGE(TAG, "read OLAT register failed");
		return ESP_FAIL;
	}

	if(level == GPIO_LEVEL_SET){
		reg_value |= (1 << gpio_pin);
	}
	else if(level == GPIO_LEVEL_RESET){
		reg_value &= ~((uint8_t)1 << gpio_pin);
	}
	else{
		ESP_LOGE(TAG, "GPIO SET LEVEL VALUE ERROR");
		return ESP_FAIL;
	}

	return ESPMI_I2CWriteByte(ESPMI_MCP23008_ADDRESS>>1, ESPMI_MCP23008_GPIO, reg_value);
}

esp_err_t ESPMI_MCP23008_ReadLevelPin(uint8_t pin, Boolean* value){
	uint8_t reg_value = 0;
	const char* TAG = "MCP23008 GPIO Pin Read";

	if(ESPMI_I2CRead(ESPMI_MCP23008_ADDRESS >> 1, ESPMI_MCP23008_GPIO, &reg_value, 1) != ESP_OK){
		ESP_LOGE(TAG, "read GPIO register failed");
		return ESP_FAIL;
	}

	*value = (Boolean)(reg_value & (1 << pin)) >> pin;
	return ESP_OK;
}
