#ifndef __KEY_TASK_H__
#define __KEY_TASK_H__

#define KEY_ON          1
#define KEY_OFF         0

#include "stdint.h"
#include "adc.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "bsp_user_key.h"
#include "bsp_buzzer.h"
#include "cybergear.h"
#include "ws2812.h"
#include "bsp_power_enable.h"
#include "motor_comm_task.h"
#include "bsp_ps2.h"
#include "usart.h"
#include "bsp_arm.h"

extern uint8_t key_status ;

#endif
