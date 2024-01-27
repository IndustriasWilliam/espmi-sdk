#ifndef HDMI_H_
#define HDMI_H_

#include "lvgl.h"

#define CONFIG_DOUBLE_FB 1

lv_disp_t* HDMI_Initialize(void* context);
void LVGL_Task(void* pvParameters);

// Atributo de alinhamento para garantir alinhamento adequado
#if defined(__GNUC__)
#define ATTRIBUTE_PACKED __attribute__((packed))
#elif defined(_MSC_VER)
#define ATTRIBUTE_PACKED
#pragma pack(push, 1)
#else
#error "Atributo de alinhamento não suportado para este compilador"
#endif

typedef struct {
	uint8_t status;
	uint8_t touchId;
	uint8_t x_lsb;
	uint8_t x_msb;
	uint8_t y_lsb;
	uint8_t y_msb;
}ATTRIBUTE_PACKED TouchData_t;


typedef struct {
	uint8_t reportId;
	TouchData_t data[10];
	uint8_t touch_cnt;
	uint8_t scanTimeL;
	uint8_t scanTimeH;
} ATTRIBUTE_PACKED  TouchInputReport_t;




#endif
