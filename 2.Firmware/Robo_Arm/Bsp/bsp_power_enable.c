#include "main.h"
#include "cmsis_os.h"
#include "bsp_power_enable.h"


void Power_Out_ctl(uint8_t p24v1_enable,uint8_t p24v2_enable,uint8_t p5v_enable)
{
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, p24v1_enable);
    HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, p24v2_enable);
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, p5v_enable);
}

void Power_Down(void)
{

    Power_Out_ctl(0,0,0);
}

void Power_Enable(void)
{
    
    Power_Out_ctl(1,1,1);
}

