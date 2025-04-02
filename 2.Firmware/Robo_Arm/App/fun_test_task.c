#include "fun_test_task.h"
#include "usart.h"
#include "main.h"
#include "tim.h"
#include "fdcan.h"
#include "ws2812.h"
#include "can_bsp.h"
#include <string.h>
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include "Emm_V5_can.h"
#include "bsp_buzzer.h"

#define U1_RX_LEN   5
#define U5_RX_LEN   5
#define U7_RX_LEN   5
#define U10_RX_LEN   5

uint8_t u1Rx[U1_RX_LEN];
uint8_t u1RxFlag = 0;

uint8_t u5Rx[U5_RX_LEN];
uint8_t u5RxFlag = 0;

uint8_t u7Rx[U7_RX_LEN];
uint8_t u7RxFlag = 0;

uint8_t u10Rx[U10_RX_LEN];
uint8_t u10RxFlag = 0;


uint8_t tx_data[8] = {0,1,2,3,4,5,6,7};


/* USER CODE BEGIN Header_FunTest_Entry */
/**
* @brief Function implementing the FunTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FunTest_Entry */
void FunTest_Entry(void const * argument)
{
    /* USER CODE BEGIN FunTest_Entry */
    DWT_Init();
    HAL_UART_Receive_IT(&huart1, u1Rx, U1_RX_LEN);
    HAL_UART_Receive_IT(&huart7, u7Rx, U7_RX_LEN);
    HAL_UART_Receive_IT(&huart10, u10Rx, U10_RX_LEN);
    osDelay(1000);
    
    /* Infinite loop */
    for(;;)
    {
//        if (u1RxFlag == 1)
//        {
            // HAL_UART_Transmit(&huart1, (const uint8_t *)"U1 Test!\n", sizeof("U1 Test!\n")-1, 0xFFFF);
            // DWT_DelayUS(1000000);
            // u1RxFlag = 0;
//        }
        
        // Emm_V5_Vel_Control_can(3, 1, 500, 0, 0);
        // uint32_t Send_ID;
        // uint8_t data[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
        // Send_ID = 0x00<<24|0x00<<8|0x00;
        // FDCAN3_Send_Msg(data,FDCAN_DLC_BYTES_8,Send_ID);
        
        // HAL_UART_Transmit(&huart1, (const uint8_t *)"U1 Test!\n", sizeof("U1 Test!\n")-1, 0xFFFF);
        osDelay(100);
    }
    /* USER CODE END FunTest_Entry */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        if (memcmp(u1Rx, "test\n", U1_RX_LEN) == 0)
        {
            u1RxFlag = 1;
        }
        HAL_UART_Receive_IT(&huart1, u1Rx, U1_RX_LEN);
    }
    else if (huart == &huart7)
    {
        if (memcmp(u7Rx, "test\n", U7_RX_LEN) == 0)
        {
            u7RxFlag = 1;
        }
        HAL_UART_Receive_IT(&huart7, u7Rx, U7_RX_LEN);
    }
    else if (huart == &huart10)
    {
        if (memcmp(u10Rx, "test\n", U10_RX_LEN) == 0)
        {
            u10RxFlag = 1;
        }
        HAL_UART_Receive_IT(&huart10, u10Rx, U10_RX_LEN);
    }
}


