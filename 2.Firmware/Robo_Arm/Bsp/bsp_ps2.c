#include "cmsis_os.h"
#include "main.h"
#include "bsp_ps2.h"
#include "BMI088Middleware.h"
#include "fun_test_task.h"
#include "usart.h"
#include "stdio.h"
#include <string.h>
#include "vofa.h"
#include "dwt.h"
//PS2手柄的输入输出口


JOYSTICK_TypeDef my_ps2;
const  uint8_t PS2_cmnd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};   //请求获取数据命令                         
uint8_t PS2_data[9] = {0};  //接收的数据


/**
  * @简  述  PS2数据读写函数
  * @参  数  data:要写入的数据
  * @返回值  读出数据
  */
char ps2_temp[100];
uint8_t ref,res;
static uint8_t PS2_ReadWriteData(uint8_t data)
{
	res = 0;
	for(ref = 0x01; ref > 0x00; ref <<= 1)
	{
		CLK_L();
		if(ref&data)
			CMD_H();
		else
			CMD_L();	
		
		DWT_DelayUS(6);
		
		CLK_H();

		if(DI())
			res |= ref; 
		
		DWT_DelayUS(6);	
		
	}
	
	CMD_H();

	// sprintf(ps2_temp,"%d\n",res);	
	// HAL_UART_Transmit(&huart1, (uint8_t *)ps2_temp, strlen(ps2_temp), 0xFFFF);

	//返回读出数据
    return res;	
}

/**
  * @简  述  PS2获取按键及摇杆数值。
  * @参  数  *JoystickStruct 手柄键值结构体
  * @返回值  无
  */
void PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct,uint8_t transmit_flag)
{
	uint8_t i;

	//使能手柄
	CS_L();
	
	//读取PS2数据
	for(i=0; i<9; i++)
	{
		PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
		DWT_DelayUS(3);
	}

	//关闭使能
	CS_H();

	//数值传递
	JoystickStruct->mode = PS2_data[1];
	JoystickStruct->btn1 = ~PS2_data[3];
	JoystickStruct->btn2 = ~PS2_data[4];
	JoystickStruct->RJoy_LR = PS2_data[5];
	JoystickStruct->RJoy_UD = PS2_data[6];
	JoystickStruct->LJoy_LR = PS2_data[7];
	JoystickStruct->LJoy_UD = PS2_data[8];

	JoystickStruct->select = (JoystickStruct->btn1 & 0x01) ? 1 : 0;
	JoystickStruct->joyL = (JoystickStruct->btn1 & 0x02) ? 1 : 0;
	JoystickStruct->joyR = (JoystickStruct->btn1 & 0x04) ? 1 : 0;
	JoystickStruct->start = (JoystickStruct->btn1 & 0x08)? 1 : 0;
	JoystickStruct->up = (JoystickStruct->btn1 & 0x10)? 1 : 0;
	JoystickStruct->right = (JoystickStruct->btn1 & 0x20)? 1 : 0;
	JoystickStruct->down = (JoystickStruct->btn1 & 0x40)? 1 : 0;
	JoystickStruct->left = (JoystickStruct->btn1 & 0x80)? 1 : 0;
	
	JoystickStruct->L2 = (JoystickStruct->btn2 & 0x01)? 1 : 0;
	JoystickStruct->R2 = (JoystickStruct->btn2 & 0x02)? 1 : 0;
	JoystickStruct->L1 = (JoystickStruct->btn2 & 0x04)? 1 : 0;
	JoystickStruct->R1 = (JoystickStruct->btn2 & 0x08)? 1 : 0;
	JoystickStruct->triangle = (JoystickStruct->btn2 & 0x10)? 1 : 0;
	JoystickStruct->circle = (JoystickStruct->btn2 & 0x20)? 1 : 0;
	JoystickStruct->fork = (JoystickStruct->btn2 & 0x40)? 1 : 0;
	JoystickStruct->rectangle = (JoystickStruct->btn2 & 0x80)? 1 : 0;
	
	// 转换坐标范围 0x00~0xFF -> -127~127
	JoystickStruct->RJoy_LR_0 = (int)((int)JoystickStruct->RJoy_LR - 128);
	JoystickStruct->RJoy_UD_0 = -(int)((int)JoystickStruct->RJoy_UD - 127);
	JoystickStruct->LJoy_LR_0 = (int)((int)JoystickStruct->LJoy_LR - 128);
	JoystickStruct->LJoy_UD_0 = -(int)((int)JoystickStruct->LJoy_UD - 127);
	

	// for(i=0; i<9; i++)
	// {
	// 	sprintf(ps2_temp,"i[%d]=%d\n",i,PS2_data[i]);	
	// 	HAL_UART_Transmit(&huart1, (uint8_t *)ps2_temp, strlen(ps2_temp), 0xFFFF);
	// 	osDelay(1);
	// }

	if(transmit_flag)
	{
		for(i=0; i<9; i++)
			{
				sprintf(ps2_temp, 
					"MODE: %02X BTN1: %02X BTN2: %02X RJOY_LR: %02X RJOY_UD: %02X LJOY_LR: %02X LJOY_UD: %02X\r\n", 
					JoystickStruct->mode,
					JoystickStruct->btn1,
					JoystickStruct->btn2,
					JoystickStruct->RJoy_LR,
					JoystickStruct->RJoy_UD,
					JoystickStruct->LJoy_LR,
					JoystickStruct->LJoy_UD);

				vofa_send_data(1, JoystickStruct->RJoy_LR);
				vofa_send_data(2, JoystickStruct->RJoy_UD);
				vofa_send_data(3, JoystickStruct->LJoy_LR);
				vofa_send_data(4, JoystickStruct->LJoy_UD);
				vofa_sendframetail();
				// 通过 UART 发送数据
				HAL_UART_Transmit(&huart1, (uint8_t *)ps2_temp, strlen(ps2_temp), 0xFFFF);
				osDelay(5);
			}
	}
	
	

}
