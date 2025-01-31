#ifndef __FUN_TEST_TASK_H__
#define __FUN_TEST_TASK_H__

#include "cmsis_os.h"


extern void DWT_Init();
extern void DWT_DelayUS(uint32_t _ulDelayTime);
// extern void DWT_DelayMS(uint32_t _ulDelayTime);

#endif
