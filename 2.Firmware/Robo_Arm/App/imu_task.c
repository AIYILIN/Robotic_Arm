#include "imu_task.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
#include "tim.h"
#include "vofa.h"
#include <math.h>

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

float gyro[3] = {0.0f};
float acc[3] = {0.0f};
static float temp = 0.0f;

float imuQuat[4] = {0.0f};
float imuAngle[6] = {0.0f};

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;


void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
int angle_yaw_flag = 0;
void NormalizeAngle(float* angle, float* angle_360, float* angle_last, float* angel_normalize)
{
    *angle_360 = *angle;

    if (*angle_360 > 100.0f && *angle_last < -100.0f) angle_yaw_flag++;
    else if (*angle_360 < -100.0f && *angle_last > 100.0f) angle_yaw_flag--;

    imuAngle[INS_YAW_NORMALIZE] = *angle_360 - angle_yaw_flag * 360.0f;    
    imuAngle[INS_YAW_last] = *angle_360;
}

void ImuTask_Entry(void const * argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
    // osDelay(500);
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    // while(BMI088_init())
    // {
    //     osDelay(100);
    // }
    
    // AHRS_init(imuQuat);
    // /* Infinite loop */
    for(;;)
    {
    //     BMI088_read(gyro, acc, &temp);
        
    //     AHRS_update(imuQuat, gyro, acc);
    //     GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);
            
    //     /* 将欧拉角从弧度转换为度数 */ 

    //     imuAngle[INS_YAW_ADDRESS_OFFSET]   *= (180.0f / 3.1415926f); // 偏航角
    //     imuAngle[INS_PITCH_ADDRESS_OFFSET] *= (180.0f / 3.1415926f); // 俯仰角
    //     imuAngle[INS_ROLL_ADDRESS_OFFSET]  *= (180.0f / 3.1415926f); // 滚动角

    //     /* 规范化偏航角，保持其在-180°到180°范围内 */
    //     NormalizeAngle(&imuAngle[INS_YAW_ADDRESS_OFFSET], &imuAngle[INS_YAW_360], &imuAngle[INS_YAW_last], &imuAngle[INS_YAW_NORMALIZE]);
        
    //     err_ll = err_l;
    //     err_l = err;
    //     err = DES_TEMP - temp;
    //     out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
    //     if (out > MAX_OUT) out = MAX_OUT;
    //     if (out < 0) out = 0.f;
    //     htim3.Instance->CCR4 = (uint16_t)out;
        
        // vofa_send_data(0, imuAngle[0]);
        // vofa_send_data(1, imuAngle[1]);
        // vofa_send_data(2, imuAngle[2]);
        // vofa_sendframetail();
        
        osDelay(2);
    }
    /* USER CODE END ImuTask_Entry */
}




