#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "cmsis_os.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2
#define INS_YAW_360               3
#define INS_YAW_last              4
#define INS_YAW_NORMALIZE         5

extern float imuAngle[6];

#endif
