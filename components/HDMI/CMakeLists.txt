idf_component_register(SRCS
	 "HDMI.c"
	 
	 INCLUDE_DIRS 
	 "include" "."
	 REQUIRES driver esp_lcd lvgl esp_timer usb usb_host_hid
)

