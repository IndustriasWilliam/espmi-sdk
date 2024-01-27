// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: 3d_Printer_2

#include "../ui.h"

void ui_printing_screen_init(void)
{
    ui_printing = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_printing, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_img_src(ui_printing, &ui_img_bg1_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_header_group4 = lv_obj_create(ui_printing);
    lv_obj_set_height(ui_header_group4, 130);
    lv_obj_set_width(ui_header_group4, lv_pct(100));
    lv_obj_set_align(ui_header_group4, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_header_group4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_header_group4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_header_group4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_header_group4, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_header_group4, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_header_group4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_header_group4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label8 = lv_label_create(ui_header_group4);
    lv_obj_set_width(ui_Label8, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label8, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label8, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_Label8, "Printing");
    lv_obj_set_style_text_color(ui_Label8, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label8, &ui_font_H1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_pos_group1 = lv_obj_create(ui_header_group4);
    lv_obj_set_width(ui_pos_group1, lv_pct(50));
    lv_obj_set_height(ui_pos_group1, lv_pct(100));
    lv_obj_set_align(ui_pos_group1, LV_ALIGN_RIGHT_MID);
    lv_obj_set_flex_flow(ui_pos_group1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_pos_group1, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_pos_group1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_pos_group1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_pos_group1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_pos_group1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_pos_group1, 20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_posx1 = lv_obj_create(ui_pos_group1);
    lv_obj_set_height(ui_posx1, 50);
    lv_obj_set_width(ui_posx1, LV_SIZE_CONTENT);   /// 100
    lv_obj_set_align(ui_posx1, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_posx1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_posx1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_posx1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_posx1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_posx1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_posx1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_posx1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_label_x2 = lv_label_create(ui_posx1);
    lv_obj_set_width(ui_label_x2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_label_x2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_label_x2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_label_x2, "X");
    lv_obj_set_style_text_color(ui_label_x2, lv_color_hex(0x7B7D83), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_label_x2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_label_x2, &ui_font_Title, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_label_x3 = lv_label_create(ui_posx1);
    lv_obj_set_width(ui_label_x3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_label_x3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_label_x3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_label_x3, "130.0");
    lv_obj_set_style_text_color(ui_label_x3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_label_x3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_label_x3, &ui_font_Subtitle, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_posy1 = lv_obj_create(ui_pos_group1);
    lv_obj_set_height(ui_posy1, 50);
    lv_obj_set_width(ui_posy1, LV_SIZE_CONTENT);   /// 100
    lv_obj_set_align(ui_posy1, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_posy1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_posy1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_posy1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_posy1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_posy1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_posy1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_posy1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_label_y1 = lv_label_create(ui_posy1);
    lv_obj_set_width(ui_label_y1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_label_y1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_label_y1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_label_y1, "Y");
    lv_obj_set_style_text_color(ui_label_y1, lv_color_hex(0x7B7D83), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_label_y1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_label_y1, &ui_font_Title, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_label_y3 = lv_label_create(ui_posy1);
    lv_obj_set_width(ui_label_y3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_label_y3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_label_y3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_label_y3, "78.2");
    lv_obj_set_style_text_color(ui_label_y3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_label_y3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_label_y3, &ui_font_Subtitle, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_posz1 = lv_obj_create(ui_pos_group1);
    lv_obj_set_height(ui_posz1, 50);
    lv_obj_set_width(ui_posz1, LV_SIZE_CONTENT);   /// 100
    lv_obj_set_align(ui_posz1, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_posz1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_posz1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_posz1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_posz1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_posz1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(ui_posz1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(ui_posz1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_label_z3 = lv_label_create(ui_posz1);
    lv_obj_set_width(ui_label_z3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_label_z3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_label_z3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_label_z3, "X");
    lv_obj_set_style_text_color(ui_label_z3, lv_color_hex(0x7B7D83), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_label_z3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_label_z3, &ui_font_Title, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_label_z4 = lv_label_create(ui_posz1);
    lv_obj_set_width(ui_label_z4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_label_z4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_label_z4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_label_z4, "112.8");
    lv_obj_set_style_text_color(ui_label_z4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_label_z4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_label_z4, &ui_font_Subtitle, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_content_group4 = lv_obj_create(ui_printing);
    lv_obj_set_height(ui_content_group4, 466);
    lv_obj_set_width(ui_content_group4, lv_pct(100));
    lv_obj_set_align(ui_content_group4, LV_ALIGN_BOTTOM_MID);
    lv_obj_clear_flag(ui_content_group4, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_content_group4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_content_group4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_content_group4, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_content_group4, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_content_group4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_content_group4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_print_preview = lv_obj_create(ui_content_group4);
    lv_obj_set_width(ui_print_preview, 332);
    lv_obj_set_height(ui_print_preview, lv_pct(100));
    lv_obj_set_x(ui_print_preview, 90);
    lv_obj_set_y(ui_print_preview, 0);
    lv_obj_set_align(ui_print_preview, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_print_preview, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_print_preview, lv_color_hex(0x201F2D), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_print_preview, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_separetor_left = lv_img_create(ui_print_preview);
    lv_img_set_src(ui_separetor_left, &ui_img_separetor2_png);
    lv_obj_set_width(ui_separetor_left, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_separetor_left, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_separetor_left, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_add_flag(ui_separetor_left, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_separetor_left, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_separator_right = lv_img_create(ui_print_preview);
    lv_img_set_src(ui_separator_right, &ui_img_separetor2_png);
    lv_obj_set_width(ui_separator_right, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_separator_right, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_separator_right, LV_ALIGN_BOTTOM_RIGHT);
    lv_obj_add_flag(ui_separator_right, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_separator_right, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_printing_percent_group = lv_obj_create(ui_print_preview);
    lv_obj_set_height(ui_printing_percent_group, 106);
    lv_obj_set_width(ui_printing_percent_group, lv_pct(100));
    lv_obj_set_x(ui_printing_percent_group, 0);
    lv_obj_set_y(ui_printing_percent_group, -10);
    lv_obj_set_align(ui_printing_percent_group, LV_ALIGN_BOTTOM_MID);
    lv_obj_clear_flag(ui_printing_percent_group, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_printing_percent_group, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_printing_percent_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_printing_percent = lv_label_create(ui_printing_percent_group);
    lv_obj_set_width(ui_printing_percent, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_printing_percent, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_printing_percent, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_printing_percent, "84%");
    lv_obj_set_style_text_color(ui_printing_percent, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_printing_percent, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_printing_percent, &ui_font_H1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_printing_percent1 = lv_label_create(ui_printing_percent_group);
    lv_obj_set_width(ui_printing_percent1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_printing_percent1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_printing_percent1, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_printing_percent1, "Dino.gcode");
    lv_obj_set_style_text_color(ui_printing_percent1, lv_color_hex(0x83898B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_printing_percent1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_printing_percent1, &ui_font_Subtitle, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Slider2 = lv_slider_create(ui_print_preview);
    lv_slider_set_value(ui_Slider2, 84, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_Slider2) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_Slider2, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_Slider2, 288);
    lv_obj_set_height(ui_Slider2, 332);
    lv_obj_set_x(ui_Slider2, 0);
    lv_obj_set_y(ui_Slider2, -67);
    lv_obj_set_align(ui_Slider2, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_img_src(ui_Slider2, &ui_img_print_bar1_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_img_src(ui_Slider2, &ui_img_print_bar2_png, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Slider2, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Slider2, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_temp_group = lv_obj_create(ui_content_group4);
    lv_obj_set_width(ui_temp_group, 195);
    lv_obj_set_height(ui_temp_group, lv_pct(100));
    lv_obj_set_x(ui_temp_group, -10);
    lv_obj_set_y(ui_temp_group, 0);
    lv_obj_set_align(ui_temp_group, LV_ALIGN_BOTTOM_RIGHT);
    lv_obj_clear_flag(ui_temp_group, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_temp_group, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_temp_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_bar_group = ui_bar_group_create(ui_temp_group);
    lv_obj_set_x(ui_bar_group, 0);
    lv_obj_set_y(ui_bar_group, 0);
    lv_obj_set_align(ui_bar_group, LV_ALIGN_BOTTOM_RIGHT);





    ui_bar_group1 = ui_bar_group_create(ui_temp_group);
    lv_obj_set_x(ui_bar_group1, 0);
    lv_obj_set_y(ui_bar_group1, 0);
    lv_obj_set_align(ui_bar_group1, LV_ALIGN_BOTTOM_LEFT);



    lv_img_set_src(ui_comp_get_child(ui_bar_group1, UI_COMP_BAR_GROUP_BAR_TEXT), &ui_img_bedtemp_png);


    lv_label_set_text(ui_comp_get_child(ui_bar_group1, UI_COMP_BAR_GROUP_LABEL_BAR), "65°");

    ui_printing_group = lv_obj_create(ui_content_group4);
    lv_obj_set_width(ui_printing_group, 353);
    lv_obj_set_height(ui_printing_group, lv_pct(99));
    lv_obj_set_x(ui_printing_group, 15);
    lv_obj_set_y(ui_printing_group, 0);
    lv_obj_set_align(ui_printing_group, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_clear_flag(ui_printing_group, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_printing_group, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_printing_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_printing_group, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_printing_group, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_printing_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_printing_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_printing_btn_group = lv_obj_create(ui_printing_group);
    lv_obj_set_height(ui_printing_btn_group, 138);
    lv_obj_set_width(ui_printing_btn_group, lv_pct(100));
    lv_obj_set_align(ui_printing_btn_group, LV_ALIGN_BOTTOM_MID);
    lv_obj_clear_flag(ui_printing_btn_group, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_printing_btn_group, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_printing_btn_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_button_print = ui_button_print_create(ui_printing_btn_group);
    lv_obj_set_x(ui_button_print, 0);
    lv_obj_set_y(ui_button_print, 0);
    lv_obj_set_align(ui_button_print, LV_ALIGN_BOTTOM_LEFT);

    ui_button_print1 = ui_button_print_create(ui_printing_btn_group);
    lv_obj_set_x(ui_button_print1, 0);
    lv_obj_set_y(ui_button_print1, 0);
    lv_obj_set_align(ui_button_print1, LV_ALIGN_BOTTOM_RIGHT);

    lv_label_set_text(ui_comp_get_child(ui_button_print1, UI_COMP_BUTTON_PRINT_LABEL_BUTTON_PRINT), "Stop");

    ui_printing_params_group = lv_obj_create(ui_printing_group);
    lv_obj_set_height(ui_printing_params_group, 335);
    lv_obj_set_width(ui_printing_params_group, lv_pct(100));
    lv_obj_set_x(ui_printing_params_group, 0);
    lv_obj_set_y(ui_printing_params_group, -20);
    lv_obj_set_align(ui_printing_params_group, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_printing_params_group, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_printing_params_group, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_printing_params_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_printing_params_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_printing_params_group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_printing_params_group, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_printing_params_group, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_text_bg_group6 = ui_text_bg_group_create(ui_printing_params_group);
    lv_obj_set_height(ui_text_bg_group6, 138);
    lv_obj_set_width(ui_text_bg_group6, lv_pct(100));
    lv_obj_set_x(ui_text_bg_group6, 0);
    lv_obj_set_y(ui_text_bg_group6, 0);
    lv_obj_set_align(ui_text_bg_group6, LV_ALIGN_TOP_MID);






    lv_label_set_text(ui_comp_get_child(ui_text_bg_group6, UI_COMP_TEXT_BG_GROUP_TEXT_BG_TEXT_BG_CONTENT_TEXT_BG_LABEL1),
                      "54");

    lv_label_set_text(ui_comp_get_child(ui_text_bg_group6, UI_COMP_TEXT_BG_GROUP_TEXT_BG_TEXT_BG_CONTENT_TEXT_BG_LABEL2),
                      "min");

    lv_label_set_text(ui_comp_get_child(ui_text_bg_group6, UI_COMP_TEXT_BG_GROUP_TEXT_BG_LABEL3), "Printing time");

    ui_text_bg_group5 = ui_text_bg_group_create(ui_printing_params_group);
    lv_obj_set_height(ui_text_bg_group5, 138);
    lv_obj_set_width(ui_text_bg_group5, lv_pct(100));
    lv_obj_set_x(ui_text_bg_group5, 0);
    lv_obj_set_y(ui_text_bg_group5, 0);
    lv_obj_set_align(ui_text_bg_group5, LV_ALIGN_BOTTOM_MID);






    lv_label_set_text(ui_comp_get_child(ui_text_bg_group5, UI_COMP_TEXT_BG_GROUP_TEXT_BG_TEXT_BG_CONTENT_TEXT_BG_LABEL1),
                      "12");

    lv_label_set_text(ui_comp_get_child(ui_text_bg_group5, UI_COMP_TEXT_BG_GROUP_TEXT_BG_TEXT_BG_CONTENT_TEXT_BG_LABEL2),
                      "min");

    lv_label_set_text(ui_comp_get_child(ui_text_bg_group5, UI_COMP_TEXT_BG_GROUP_TEXT_BG_LABEL3), "Remain time");

    lv_obj_add_event_cb(ui_Slider2, ui_event_Slider2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_button_print1, ui_event_button_print1_button_print, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_printing, ui_event_printing, LV_EVENT_ALL, NULL);

}