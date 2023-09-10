#ifndef HDMI_H_
#define HDMI_H_

#include "lvgl.h"

#define CONFIG_DOUBLE_FB 1

lv_disp_t* HDMI_Initialize(void* context);
void LVGL_Task(void* pvParameters);


#endif
