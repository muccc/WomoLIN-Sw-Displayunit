#include "stm32l4xx_hal.h"
#include "touch.h"

I2C_HandleTypeDef hi2c2 = {0};

uint8_t touch_rx[4];
uint8_t touch_tx[] = {0x03};
extern I2C_HandleTypeDef hi2c2;

void touch_init(){
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touch_poll;
  lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

bool touch_poll(lv_indev_drv_t * drv, lv_indev_data_t*data){
    HAL_I2C_Master_Transmit(&hi2c2,0x70, touch_tx, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2,0x70, touch_rx, 4, 1);
    data->point.x = ((touch_rx[0] & 0xf) << 8) + touch_rx[1];
    data->point.y = ((touch_rx[2] & 0xf) << 8) + touch_rx[3];
    data->state = (touch_rx[0] & 0x40)?LV_INDEV_STATE_REL:LV_INDEV_STATE_PR;
    return false;
}
