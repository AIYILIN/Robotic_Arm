#ifndef __KINEMATIC_TASK_H__
#define __KINEMATIC_TASK_H__

#include "cmsis_os.h"
#include "fdcan.h"
#include "can_bsp.h"
#include "cybergear.h"
#include "motor_pid.h"
#include "bsp_user_key.h"
#include "imu_task.h"
#include "safe_monitor_task.h"
#include "user_lib.h"
#include "motor_comm_task.h"
#include "bsp_ps2.h"
#include "usart.h"
#include <string.h>
#include "stdio.h"
#include "vofa.h"
#include "bsp_arm.h"
#include "motor_control.h"
#include "arm_kinematics.h"


extern void joints_move_text(void);
extern void joints_precision_text(void);
extern void joints_euler_move_text(void);


#endif
