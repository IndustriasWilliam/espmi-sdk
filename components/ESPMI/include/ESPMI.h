#ifndef ESPMI_H_
#define ESPMI_H_


#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

// #define ESPMI_MINI


#ifndef ESPMI_MINI
#define ESPMI_I2C_SDA					GPIO_NUM_5
#define ESPMI_I2C_SCL					GPIO_NUM_4

#else
#define ESPMI_I2C_SDA					GPIO_NUM_2
#define ESPMI_I2C_SCL					GPIO_NUM_1
#define ESPMI_ADV_PD 					GPIO_NUM_7
#endif

#define ESPMI_I2C_FREQ					100000
#define ESPMI_I2C_MASTER_NUM 			0
#define ESPMI_I2C_MASTER_TIMEOUT_MS		1000

#define ESPMI_MCP23008_ADDRESS	0x40

#ifndef ESPMI_MINI
#define HDMI_ADDRESS			0x70
#else
#define HDMI_ADDRESS			0x7A
#endif

#define ESPMI_MCP23008_IODIR		0x00
#define ESPMI_MCP23008_IPOL			0x01
#define ESPMI_MCP23008_GPINTEN		0x02
#define ESPMI_MCP23008_DEFVAL		0x03
#define ESPMI_MCP23008_INTCON		0x04
#define ESPMI_MCP23008_IOCON		0x05
#define ESPMI_MCP23008_GPPU			0x06
#define ESPMI_MCP23008_INTF			0x07
#define ESPMI_MCP23008_INTCAP		0x08
#define ESPMI_MCP23008_GPIO			0x09
#define ESPMI_MCP23008_OLAT			0x0A


#define ESPMI_E220_900MM22S_BUSY	GPIO_NUM_0
#define ESPMI_E220_900MM22S_RST		GPIO_NUM_1
#define ESPMI_E220_900MM22S_RXEN	GPIO_NUM_2
#define ESPMI_E220_900MM22S_TXEN	GPIO_NUM_3
#define ESPMI_E220_900MM22S_DIO3	GPIO_NUM_4
#define ESPMI_E220_900MM22S_DIO2	GPIO_NUM_5
#define ESPMI_E220_900MM22S_DIO1	GPIO_NUM_6
#define ESPMI_LED_STATUS			GPIO_NUM_7

#define ESPMI_E220_900MM22S_NSS		GPIO_NUM_39
#define ESPMI_E220_900MM22S_MISO	GPIO_NUM_41
#define ESPMI_E220_900MM22S_MOSI	GPIO_NUM_40
#define ESPMI_E220_900MM22S_SCK		GPIO_NUM_38



typedef enum {
	GPIO_LEVEL_RESET,
	GPIO_LEVEL_SET,
}ESPMI_MCPGpioLevel;

typedef enum {
	False,
	True
}Boolean;

esp_err_t ESPMI_ConfigI2C(i2c_port_t port);
esp_err_t ESPMI_ConfigSPI(spi_host_device_t device);
esp_err_t ESPMI_ConfigGPIO(void);
esp_err_t ESPMI_I2CRead(uint8_t device_address, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t ESPMI_I2CWriteByte(uint8_t device_address, uint8_t reg_addr, uint8_t data);
esp_err_t ESPMI_MCP23008_ReadLevelPin(uint8_t pin, Boolean* value);
esp_err_t ESPMI_MCP23008_SetLevelGpioPin(uint8_t gpio_pin, ESPMI_MCPGpioLevel level);
esp_err_t ESPMI_MCP23008_ConfigGpioDir(uint8_t iodir_mask);
spi_device_handle_t ESPMI_GetSPIHandle(void);

#endif
