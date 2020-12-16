#include "stm32l4xx_hal.h"
#include "touch.h"

I2C_HandleTypeDef hi2c2 = {0};

uint8_t touch_rx[4];
uint8_t touch_tx[] = {0x03};

void touch_init(){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB13     ------> I2C2_SCL
    PB14     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    hi2c2.Instance = I2C2;
    //hi2c2.Init.Timing = 0x307075B1; //100kHz
    hi2c2.Init.Timing = 0x00b04fdb; //400kHz
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c2);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
}

bool touch_poll(lv_indev_drv_t * drv, lv_indev_data_t*data){
    HAL_I2C_Master_Transmit(&hi2c2,0x70, touch_tx, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2,0x70, touch_rx, 4, 1);
    data->point.x = ((touch_rx[0] & 0xf) << 8) + touch_rx[1];
    data->point.y = ((touch_rx[2] & 0xf) << 8) + touch_rx[3];
    data->state = (touch_rx[0] & 0x40)?LV_INDEV_STATE_REL:LV_INDEV_STATE_PR;
    return false;
}
