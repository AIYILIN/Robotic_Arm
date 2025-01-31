#ifndef __BSP_POWER_ENABLE_H__
#define __BSP_POWER_ENABLE_H__
 
 
#include "main.h" 


#include "stdint.h"

extern void Power_Out_ctl(uint8_t p24v1_enable,uint8_t p24v2_enable,uint8_t p5v_enable);
extern void Power_Down(void);
extern void Power_Enable(void);


#endif
