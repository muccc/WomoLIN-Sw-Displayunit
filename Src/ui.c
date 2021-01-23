
#include <stdint.h>
#include "lvgl.h"

 void ui_init(){
 static lv_obj_t * tv;
  static lv_obj_t * t1;
  static lv_obj_t * t2;
  static lv_obj_t * t3;
  tv = lv_tabview_create(lv_scr_act(), NULL);
  t1 = lv_tabview_add_tab(tv, "IO test");
  t2 = lv_tabview_add_tab(tv, "Debug");
  t3 = lv_tabview_add_tab(tv, "Log");

  lv_obj_t * prev = NULL;
  for(int i = 0;i<20;i++){
    lv_obj_t *sw1 = lv_switch_create(t1, NULL);
    if(i == 0){
      lv_obj_align(sw1, prev, LV_ALIGN_IN_TOP_LEFT, 10, 10);
    }else{
      lv_obj_align(sw1, prev, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    }
    prev = sw1;
    lv_obj_t * label1 = lv_label_create(t1, NULL);
    char buf[10];
    lv_snprintf(buf,sizeof(buf),"output %i",i);
    lv_label_set_text(label1,buf);
    lv_obj_align(label1, sw1, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
  }

  // lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
  // lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
  // lv_obj_t * btn3 = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
  // lv_obj_t * btn4 = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/

  // lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
  // lv_obj_set_size(btn, 100, 50);                          /*Set its size*/
  // lv_obj_set_pos(btn2, 10, 100);                            /*Set its position*/
  // lv_obj_set_pos(btn3, 10, 200);                            /*Set its position*/
  // lv_obj_set_pos(btn4, 10, 300);                            /*Set its position*/

  // static lv_style_t style_btn_red;
  // lv_style_init(&style_btn_red);
  // lv_style_set_bg_color(&style_btn_red, LV_STATE_DEFAULT, LV_COLOR_MAKE(0xff, 0x00, 0x00));
  // lv_obj_add_style(btn2, LV_BTN_PART_MAIN, &style_btn_red);   /*Add the red style on top of the current */
  // static lv_style_t style_btn_green;
  // lv_style_init(&style_btn_green);
  // lv_style_set_bg_color(&style_btn_green, LV_STATE_DEFAULT, LV_COLOR_MAKE(0x00, 0xff, 0x00));
  // lv_obj_add_style(btn3, LV_BTN_PART_MAIN, &style_btn_green);   /*Add the red style on top of the current */
  // static lv_style_t style_btn_blue;
  // lv_style_init(&style_btn_blue);
  // lv_style_set_bg_color(&style_btn_blue, LV_STATE_DEFAULT, LV_COLOR_MAKE(0x00, 0x00, 0xff));
  // lv_obj_add_style(btn4, LV_BTN_PART_MAIN, &style_btn_blue);   /*Add the red style on top of the current */

  // //lv_obj_set_event_cb(btn, btn_event_cb);                 /*Assign a callback to the button*/

  // lv_obj_t * label = lv_label_create(btn, NULL);          /*Add a label to the button*/
  // lv_label_set_text(label, "Button");                     /*Set the labels text*/

  // lv_obj_t * clock = lv_label_create(lv_scr_act(), NULL);          /*Add a label to the button*/
  // lv_label_set_text(clock, "23:30:12");                     /*Set the labels text*/
  // lv_obj_set_pos(clock, 730, 10);                            /*Set its position*/
 }
 