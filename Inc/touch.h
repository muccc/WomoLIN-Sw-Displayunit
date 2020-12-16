#pragma once

#include "lvgl.h"

void touch_init();
bool touch_poll(lv_indev_drv_t * drv, lv_indev_data_t*data);
