
#include "ctr_task.h"
#include "cmsis_os.h"
#include "ws2812.h"
#include "fdcan.h"
#include "can_bsp.h"
#include "string.h"
#include "can_bsp.h"
#include "cybergear.h"
#include "motor_pid.h"
#include "bsp_power_enable.h"
#include "imu_task.h"
#include "user_lib.h"
#include "key_task.h"
#include "bsp_ps2.h"
#include "BMI088Middleware.h"
#include "fun_test_task.h"
#include "bsp_arm.h"
#include "dma.h"
#include "bsp_buzzer.h"

// Ϊ6���ؽڸ��Դ��������Ĺ켣�滮��
T_Profile joint_T_profilel[6];



void joint_T_profilel_param_init(T_Profile *profile)
{
    for (int i = 0; i < 6; i++) 
    {
        joint_T_profilel[i].vel_max = 50;    
        joint_T_profilel[i].acc_max = 20;
        joint_T_profilel[i].vel_cur = 0;
        joint_T_profilel[i].vel_s_max = 0;
        joint_T_profilel[i].t_time = 0;
        joint_T_profilel[i].t_time_count = 0;
        
        
        joint_T_profilel[i].pos_current = 0;    //-3~3
        joint_T_profilel[i].pos_target_last = 0;    //-3~3
    }
        joint_T_profilel[0].pos_target = 0;    //-3~3
        joint_T_profilel[1].pos_target = 0;    //-8~8
        joint_T_profilel[2].pos_target = 0;    //-4~14
        joint_T_profilel[3].pos_target = 0;    //-2.7~2.7
}


void Update_T_Profile(T_Profile *profile, float dt) 
{

/**********************************************************Ŀ��λ�øı�,��ʼ���¹滮**********************************************************/
    if(profile->pos_target != profile->pos_target_last)
    {
        profile->pos_target_last = profile->pos_target;
        profile->pos_end = profile->pos_target;
        profile->pos_start = profile->pos_current;
        profile->remaining = profile->pos_end - profile->pos_start;

        profile->direction = (profile->pos_target > profile->pos_current)? 1.0f : -1.0f;    
      
        profile->t_time = (fabsf(profile->remaining)-(profile->vel_max * profile->vel_max /profile->acc_max))/profile->vel_max;

        if(profile->t_time >= 0)//����
        {
            profile->phase = PHASE_ACCEL_STEADY;//�����һ�׶Σ��ȼ���
            profile->t_time_count = 0;//ʱ���������
        }
        else//������
        {
            profile->phase = PHASE_ACCEL_STEADY;//�����һ�׶Σ��ȼ���
            profile->vel_s_max = sqrtf(fabsf(profile->remaining) * profile->acc_max) * profile->direction;
        }
    }
    /*******************************************************endĿ��λ�øı�,���¹滮���end*******************************************************/
        
        
    if(profile->t_time >= 0)//����
    {
        switch (profile->phase)
        {
        case PHASE_ACCEL_STEADY://��һ�׶Σ�1-�ȼ���
            {
                profile->vel_cur +=  profile->direction * profile->acc_max *  dt;
                if(fabs(profile->vel_cur)  >= profile->vel_max )
                {
                    profile->phase = PHASE_CRUISE;//����ڶ��׶Σ�3-����
                    profile->vel_cur = profile->vel_max * profile->direction;//����
                }
            }
            break;
        case PHASE_CRUISE://�ڶ��׶Σ�3-����
            {
                profile->t_time_count += dt;//ʱ������ۼ�
                if(profile->t_time_count >= profile->t_time)
                {
                    profile->phase = PHASE_DECEL_STEADY;//��������׶Σ�5-�ȼ���
                }
            }
            break;
        case PHASE_DECEL_STEADY://�����׶Σ�5-�ȼ���
            {
                profile->vel_cur -= profile->direction * profile->acc_max * dt;
                if(profile->vel_cur * profile->direction <= 0)
                {
                    profile->phase = PHASE_FINISHED;//������Ľ׶Σ�7-���
                    profile->vel_cur = 0;
                }
            }
            break;
        case PHASE_FINISHED://���Ľ׶Σ�7-���
            {
                
            }
        default:
            break;
        }
    }
    else//������
    {
        switch (profile->phase)
        {
            case PHASE_ACCEL_STEADY://��һ�׶Σ�1-�ȼ���
            {
                profile->vel_cur +=  profile->direction * profile->acc_max *  dt;
                if((profile->vel_cur * profile->direction) >= (profile->vel_s_max * profile->direction))
                {
                    profile->phase = PHASE_DECEL_STEADY;//����ڶ��׶Σ�5-�ȼ���
                    profile->vel_cur = profile->vel_s_max;//����
                }
            } 
            break;
            char debug_buffer[50];
            case PHASE_DECEL_STEADY://�ڶ��׶Σ�5-�ȼ���
            {              
                profile->vel_cur -= profile->direction * profile->acc_max * dt;
                if((profile->vel_cur * profile->direction) <= 0)
                {
                    profile->phase = PHASE_FINISHED;//������Ľ׶Σ�7-���
                    profile->vel_cur = 0;
                }
            }
            break;
            case PHASE_FINISHED://���Ľ׶Σ�7-���
            {
                
            }
            break;
            default:
                break;
        } 
    }
    profile->pos_current +=  profile->vel_cur * dt ;
    
}






int debug_count = 0;
void CtrTask_Entry(void const * argument)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buffer, RX_BUFFER_SIZE);
    Power_Enable(); 
    
    osDelay(500);

for (;;) 
{
     // �������йؽڵĹ켣
    for (int i = 0; i < 6; i++) 
    {
        Update_T_Profile(&joint_T_profilel[i], 0.003f);  // dt=5ms  
    }   

    // debug_count++;
    // if(debug_count % 1 == 0) 
    // {
    //     char debug_buffer[200];
    //     sprintf(debug_buffer, "phase=%2x,vel_cur=%4.2f,direction=%4.2f\r\n"
    //             ,joint_T_profilel[1].phase
    //             ,joint_T_profilel[1].vel_cur
    //             ,joint_T_profilel[1].direction
    //             // ,joint_T_profilel[1]
    //             // ,joint_T_profilel[1]
    //             // ,joint_T_profilel[1]
    //             // ,joint_T_profilel[1]
    //             // ,joint_T_profilel[1]
    //             // ,joint_T_profilel[1]
    //             // ,joint_T_profilel[1]
    //             // ,joint_T_profilel[1]
    //             // ,joint_T_profilel[1]
    //             // ,joint_T_profilel[1]
    //             );
    //     HAL_UART_Transmit(&huart1,(uint8_t*)debug_buffer,strlen(debug_buffer),100);
    //     debug_count = 0;
    // }
        

    for (int i = 0; i < 6; i++) 
    {
      arm_joint[i].pos = joint_T_profilel[i].pos_current;     // ʹ��ƽ�����λ��
    }
    
    Arm_joints_control(arm_joint);
    osDelay(1);
}

    /* USER CODE END CtrTask_Entry */
}




