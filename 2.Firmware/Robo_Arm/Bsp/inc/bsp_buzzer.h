#ifndef __BSP_BUZZER_H__
#define __BSP_BUZZER_H__

#include "tim.h"


extern void BSP_Buzzer_Init(void);
extern void BSP_Buzzer_On(void);
extern void BSP_Buzzer_Off(void);
extern void BSP_Buzzer_Toggle(void);
#endif
