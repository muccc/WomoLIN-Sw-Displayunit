/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lvgl.h"
#include "tft.h"
#include "touch.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LTDC_HandleTypeDef hltdc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t framebuffer[LV_HOR_RES_MAX*LV_VER_RES_MAX];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // for(int i = 0;i<50*50*3;i++){
  //   framebuffer[i] = 200;
  // }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LTDC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port,LCD_BL_CTRL_Pin,GPIO_PIN_SET);
   // LTDC->SRCR = LTDC_SRCR_IMR;     // immediate shadow registers reload

// LTDC->GCR &= ~LTDC_GCR_LTDCEN;       // enable LTDC
// LTDC_Layer1->CR &= ~LTDC_LxCR_LEN;   // enable layer1

// LTDC_Layer1->CLUTWR = 0x00000000; // 0 = black
// LTDC_Layer1->CLUTWR = 0x01FF0000; // 1 = red
// LTDC_Layer1->CLUTWR = 0x0200FF00; // 2 = green
// LTDC_Layer1->CLUTWR = 0x03FFFF00; // 3 = yellow
// LTDC_Layer1->CLUTWR = 0x040000FF; // 4 = blue
// LTDC_Layer1->CLUTWR = 0x05FF00FF; // 5 = purple
// LTDC_Layer1->CLUTWR = 0x0600FFFF; // 6 = cyan
// LTDC_Layer1->CLUTWR = 0x07FFFFFF; // 7 = white
// LTDC_Layer1->CR |= LTDC_LxCR_CLUTEN; // enable CLUT

// LTDC->GCR |= LTDC_GCR_LTDCEN;       // enable LTDC
// LTDC_Layer1->CR |= LTDC_LxCR_LEN;   // enable layer1

  lv_init();
  tft_init();
  // HAL_Delay(1000);
  // extern uint8_t womo_map[];
  // memset(womo_map,0x0ff,800*480);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
  lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
  lv_obj_t * btn3 = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
  lv_obj_t * btn4 = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/

  lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
  lv_obj_set_size(btn, 100, 50);                          /*Set its size*/
  lv_obj_set_pos(btn2, 10, 100);                            /*Set its position*/
  lv_obj_set_pos(btn3, 10, 200);                            /*Set its position*/
  lv_obj_set_pos(btn4, 10, 300);                            /*Set its position*/

  static lv_style_t style_btn_red;
  lv_style_init(&style_btn_red);
  lv_style_set_bg_color(&style_btn_red, LV_STATE_DEFAULT, LV_COLOR_MAKE(0xff, 0x00, 0x00));
  lv_obj_add_style(btn2, LV_BTN_PART_MAIN, &style_btn_red);   /*Add the red style on top of the current */
  static lv_style_t style_btn_green;
  lv_style_init(&style_btn_green);
  lv_style_set_bg_color(&style_btn_green, LV_STATE_DEFAULT, LV_COLOR_MAKE(0x00, 0xff, 0x00));
  lv_obj_add_style(btn3, LV_BTN_PART_MAIN, &style_btn_green);   /*Add the red style on top of the current */
  static lv_style_t style_btn_blue;
  lv_style_init(&style_btn_blue);
  lv_style_set_bg_color(&style_btn_blue, LV_STATE_DEFAULT, LV_COLOR_MAKE(0x00, 0x00, 0xff));
  lv_obj_add_style(btn4, LV_BTN_PART_MAIN, &style_btn_blue);   /*Add the red style on top of the current */

  //lv_obj_set_event_cb(btn, btn_event_cb);                 /*Assign a callback to the button*/

  lv_obj_t * label = lv_label_create(btn, NULL);          /*Add a label to the button*/
  lv_label_set_text(label, "Button");                     /*Set the labels text*/

  lv_obj_t * clock = lv_label_create(lv_scr_act(), NULL);          /*Add a label to the button*/
  lv_label_set_text(clock, "23:30:12");                     /*Set the labels text*/
  lv_obj_set_pos(clock, 730, 10);                            /*Set its position*/


  touch_init();
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touch_poll;
  lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
  while (1)
  {
    lv_task_handler();
    HAL_Delay(10);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInit.LtdcClockSelection = RCC_LTDCCLKSOURCE_PLLSAI2_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI2.PLLSAI2M = 2;
  PeriphClkInit.PLLSAI2.PLLSAI2N = 40;
  PeriphClkInit.PLLSAI2.PLLSAI2P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI2.PLLSAI2Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_LTDCCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */
extern uint8_t womo_map[];
  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 1;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 89;
  hltdc.Init.AccumulatedVBP = 35;
  hltdc.Init.AccumulatedActiveW = 889;
  hltdc.Init.AccumulatedActiveH = 515;
  hltdc.Init.TotalWidth = 928;
  hltdc.Init.TotalHeigh = 528;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 127;
  hltdc.Init.Backcolor.Red = 50;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 800;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 480;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_L8;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = (uint32_t)framebuffer;
  pLayerCfg.ImageWidth = 800;
  pLayerCfg.ImageHeight = 480;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */
  //converts RGB332 to RGB888, generated by clut.c
  const uint32_t table[] = {
    0x000000, //0,0,0 
    0x000055, //0,0,1 
    0x0000aa, //0,0,2 
    0x0000ff, //0,0,3 
    0x002400, //0,1,0 
    0x002455, //0,1,1 
    0x0024aa, //0,1,2 
    0x0024ff, //0,1,3 
    0x004800, //0,2,0 
    0x004855, //0,2,1 
    0x0048aa, //0,2,2 
    0x0048ff, //0,2,3 
    0x006d00, //0,3,0 
    0x006d55, //0,3,1 
    0x006daa, //0,3,2 
    0x006dff, //0,3,3 
    0x009100, //0,4,0 
    0x009155, //0,4,1 
    0x0091aa, //0,4,2 
    0x0091ff, //0,4,3 
    0x00b600, //0,5,0 
    0x00b655, //0,5,1 
    0x00b6aa, //0,5,2 
    0x00b6ff, //0,5,3 
    0x00da00, //0,6,0 
    0x00da55, //0,6,1 
    0x00daaa, //0,6,2 
    0x00daff, //0,6,3 
    0x00ff00, //0,7,0 
    0x00ff55, //0,7,1 
    0x00ffaa, //0,7,2 
    0x00ffff, //0,7,3 
    0x240000, //1,0,0 
    0x240055, //1,0,1 
    0x2400aa, //1,0,2 
    0x2400ff, //1,0,3 
    0x242400, //1,1,0 
    0x242455, //1,1,1 
    0x2424aa, //1,1,2 
    0x2424ff, //1,1,3 
    0x244800, //1,2,0 
    0x244855, //1,2,1 
    0x2448aa, //1,2,2 
    0x2448ff, //1,2,3 
    0x246d00, //1,3,0 
    0x246d55, //1,3,1 
    0x246daa, //1,3,2 
    0x246dff, //1,3,3 
    0x249100, //1,4,0 
    0x249155, //1,4,1 
    0x2491aa, //1,4,2 
    0x2491ff, //1,4,3 
    0x24b600, //1,5,0 
    0x24b655, //1,5,1 
    0x24b6aa, //1,5,2 
    0x24b6ff, //1,5,3 
    0x24da00, //1,6,0 
    0x24da55, //1,6,1 
    0x24daaa, //1,6,2 
    0x24daff, //1,6,3 
    0x24ff00, //1,7,0 
    0x24ff55, //1,7,1 
    0x24ffaa, //1,7,2 
    0x24ffff, //1,7,3 
    0x480000, //2,0,0 
    0x480055, //2,0,1 
    0x4800aa, //2,0,2 
    0x4800ff, //2,0,3 
    0x482400, //2,1,0 
    0x482455, //2,1,1 
    0x4824aa, //2,1,2 
    0x4824ff, //2,1,3 
    0x484800, //2,2,0 
    0x484855, //2,2,1 
    0x4848aa, //2,2,2 
    0x4848ff, //2,2,3 
    0x486d00, //2,3,0 
    0x486d55, //2,3,1 
    0x486daa, //2,3,2 
    0x486dff, //2,3,3 
    0x489100, //2,4,0 
    0x489155, //2,4,1 
    0x4891aa, //2,4,2 
    0x4891ff, //2,4,3 
    0x48b600, //2,5,0 
    0x48b655, //2,5,1 
    0x48b6aa, //2,5,2 
    0x48b6ff, //2,5,3 
    0x48da00, //2,6,0 
    0x48da55, //2,6,1 
    0x48daaa, //2,6,2 
    0x48daff, //2,6,3 
    0x48ff00, //2,7,0 
    0x48ff55, //2,7,1 
    0x48ffaa, //2,7,2 
    0x48ffff, //2,7,3 
    0x6d0000, //3,0,0 
    0x6d0055, //3,0,1 
    0x6d00aa, //3,0,2 
    0x6d00ff, //3,0,3 
    0x6d2400, //3,1,0 
    0x6d2455, //3,1,1 
    0x6d24aa, //3,1,2 
    0x6d24ff, //3,1,3 
    0x6d4800, //3,2,0 
    0x6d4855, //3,2,1 
    0x6d48aa, //3,2,2 
    0x6d48ff, //3,2,3 
    0x6d6d00, //3,3,0 
    0x6d6d55, //3,3,1 
    0x6d6daa, //3,3,2 
    0x6d6dff, //3,3,3 
    0x6d9100, //3,4,0 
    0x6d9155, //3,4,1 
    0x6d91aa, //3,4,2 
    0x6d91ff, //3,4,3 
    0x6db600, //3,5,0 
    0x6db655, //3,5,1 
    0x6db6aa, //3,5,2 
    0x6db6ff, //3,5,3 
    0x6dda00, //3,6,0 
    0x6dda55, //3,6,1 
    0x6ddaaa, //3,6,2 
    0x6ddaff, //3,6,3 
    0x6dff00, //3,7,0 
    0x6dff55, //3,7,1 
    0x6dffaa, //3,7,2 
    0x6dffff, //3,7,3 
    0x910000, //4,0,0 
    0x910055, //4,0,1 
    0x9100aa, //4,0,2 
    0x9100ff, //4,0,3 
    0x912400, //4,1,0 
    0x912455, //4,1,1 
    0x9124aa, //4,1,2 
    0x9124ff, //4,1,3 
    0x914800, //4,2,0 
    0x914855, //4,2,1 
    0x9148aa, //4,2,2 
    0x9148ff, //4,2,3 
    0x916d00, //4,3,0 
    0x916d55, //4,3,1 
    0x916daa, //4,3,2 
    0x916dff, //4,3,3 
    0x919100, //4,4,0 
    0x919155, //4,4,1 
    0x9191aa, //4,4,2 
    0x9191ff, //4,4,3 
    0x91b600, //4,5,0 
    0x91b655, //4,5,1 
    0x91b6aa, //4,5,2 
    0x91b6ff, //4,5,3 
    0x91da00, //4,6,0 
    0x91da55, //4,6,1 
    0x91daaa, //4,6,2 
    0x91daff, //4,6,3 
    0x91ff00, //4,7,0 
    0x91ff55, //4,7,1 
    0x91ffaa, //4,7,2 
    0x91ffff, //4,7,3 
    0xb60000, //5,0,0 
    0xb60055, //5,0,1 
    0xb600aa, //5,0,2 
    0xb600ff, //5,0,3 
    0xb62400, //5,1,0 
    0xb62455, //5,1,1 
    0xb624aa, //5,1,2 
    0xb624ff, //5,1,3 
    0xb64800, //5,2,0 
    0xb64855, //5,2,1 
    0xb648aa, //5,2,2 
    0xb648ff, //5,2,3 
    0xb66d00, //5,3,0 
    0xb66d55, //5,3,1 
    0xb66daa, //5,3,2 
    0xb66dff, //5,3,3 
    0xb69100, //5,4,0 
    0xb69155, //5,4,1 
    0xb691aa, //5,4,2 
    0xb691ff, //5,4,3 
    0xb6b600, //5,5,0 
    0xb6b655, //5,5,1 
    0xb6b6aa, //5,5,2 
    0xb6b6ff, //5,5,3 
    0xb6da00, //5,6,0 
    0xb6da55, //5,6,1 
    0xb6daaa, //5,6,2 
    0xb6daff, //5,6,3 
    0xb6ff00, //5,7,0 
    0xb6ff55, //5,7,1 
    0xb6ffaa, //5,7,2 
    0xb6ffff, //5,7,3 
    0xda0000, //6,0,0 
    0xda0055, //6,0,1 
    0xda00aa, //6,0,2 
    0xda00ff, //6,0,3 
    0xda2400, //6,1,0 
    0xda2455, //6,1,1 
    0xda24aa, //6,1,2 
    0xda24ff, //6,1,3 
    0xda4800, //6,2,0 
    0xda4855, //6,2,1 
    0xda48aa, //6,2,2 
    0xda48ff, //6,2,3 
    0xda6d00, //6,3,0 
    0xda6d55, //6,3,1 
    0xda6daa, //6,3,2 
    0xda6dff, //6,3,3 
    0xda9100, //6,4,0 
    0xda9155, //6,4,1 
    0xda91aa, //6,4,2 
    0xda91ff, //6,4,3 
    0xdab600, //6,5,0 
    0xdab655, //6,5,1 
    0xdab6aa, //6,5,2 
    0xdab6ff, //6,5,3 
    0xdada00, //6,6,0 
    0xdada55, //6,6,1 
    0xdadaaa, //6,6,2 
    0xdadaff, //6,6,3 
    0xdaff00, //6,7,0 
    0xdaff55, //6,7,1 
    0xdaffaa, //6,7,2 
    0xdaffff, //6,7,3 
    0xff0000, //7,0,0 
    0xff0055, //7,0,1 
    0xff00aa, //7,0,2 
    0xff00ff, //7,0,3 
    0xff2400, //7,1,0 
    0xff2455, //7,1,1 
    0xff24aa, //7,1,2 
    0xff24ff, //7,1,3 
    0xff4800, //7,2,0 
    0xff4855, //7,2,1 
    0xff48aa, //7,2,2 
    0xff48ff, //7,2,3 
    0xff6d00, //7,3,0 
    0xff6d55, //7,3,1 
    0xff6daa, //7,3,2 
    0xff6dff, //7,3,3 
    0xff9100, //7,4,0 
    0xff9155, //7,4,1 
    0xff91aa, //7,4,2 
    0xff91ff, //7,4,3 
    0xffb600, //7,5,0 
    0xffb655, //7,5,1 
    0xffb6aa, //7,5,2 
    0xffb6ff, //7,5,3 
    0xffda00, //7,6,0 
    0xffda55, //7,6,1 
    0xffdaaa, //7,6,2 
    0xffdaff, //7,6,3 
    0xffff00, //7,7,0 
    0xffff55, //7,7,1 
    0xffffaa, //7,7,2 
    0xffffff, //7,7,3 
  };
  HAL_LTDC_ConfigCLUT(&hltdc,table,256,LTDC_LAYER_1);
  HAL_LTDC_EnableCLUT(&hltdc,LTDC_LAYER_1);
  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
