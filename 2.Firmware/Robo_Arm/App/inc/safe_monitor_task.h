#ifndef __SAFE_MONITOR_TASK_H__
#define __SAFE_MONITOR_TASK_H__

#define KEY_ON          1
#define KEY_OFF         0

#include "stdint.h"
#include "adc.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "bsp_user_key.h"
#include "bsp_buzzer.h"
#include "bsp_ps2.h"
#include "bsp_arm.h"

typedef struct 
{
    float joint_torque[4];

    float joint_torque_safelimit[4];


}SAFE_MONITOR;



extern SAFE_MONITOR joint_torque_limit;
extern uint8_t key_status ;

extern void motor_torque_Watchdog(SAFE_MONITOR *safe_monitor);

#endif
