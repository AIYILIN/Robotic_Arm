
#ifndef __DWT_H
#define __DWT_H

#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"

extern void DWT_Init();
extern void DWT_DelayUS(uint32_t _ulDelayTime);

#endif
