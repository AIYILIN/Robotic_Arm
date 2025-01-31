#include "rgb_task.h"
#include "main.h"
#include "ws2812.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Header_RgbTask_Entry */
/**
* @brief Function implementing the RgbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RgbTask_Entry */

void RgbTask_Entry(void const * argument)
{
  /* USER CODE BEGIN RgbTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    // 初始化渐变的初始值
  uint8_t red = 0, green = 0, blue = 0;

  WS2812_Ctrl(0, 0, 0);
	osDelay(200);

  /* Infinite loop */
  for(;;)
  {
    // 从黑色渐变到红色
    for (red = 0; red < RGB_LIGHT_LIMIT; red++) {
      WS2812_Ctrl(red, 0, 0);  // 红色渐变
      osDelay(RGB_DELAY_TIME);  // 控制渐变速度
    }

    // 从红色渐变到绿色
    for (green = 0; green < RGB_LIGHT_LIMIT; green++) {
      WS2812_Ctrl(RGB_LIGHT_LIMIT-green, green, 0);  // 绿色渐变
      osDelay(RGB_DELAY_TIME);
    }

    // 从绿色渐变到蓝色
    for (blue = 0; blue < RGB_LIGHT_LIMIT; blue++) {
      WS2812_Ctrl(0, RGB_LIGHT_LIMIT-blue, blue);  // 蓝色渐变
      osDelay(RGB_DELAY_TIME);
    }

    // 从蓝色渐变回黑色
    for (blue = RGB_LIGHT_LIMIT; blue > 0; blue--) {
      WS2812_Ctrl(0, 0, blue);  // 蓝色消退
      osDelay(RGB_DELAY_TIME);
    }
  }
  }
  /* USER CODE END RgbTask_Entry */
}