
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
    joint_T_profilel[0].pos_current = JOINT_1_INIT_POS;
    joint_T_profilel[1].pos_current = JOINT_2_INIT_POS;
    joint_T_profilel[2].pos_current = JOINT_3_INIT_POS;
    joint_T_profilel[3].pos_current = JOINT_4_INIT_POS;

    joint_T_profilel[0].pos_target = JOINT_1_INIT_POS;    
    joint_T_profilel[1].pos_target = JOINT_2_INIT_POS;    
    joint_T_profilel[2].pos_target = JOINT_3_INIT_POS;    
    joint_T_profilel[3].pos_target = JOINT_4_INIT_POS;    

    for (int i = 0; i < 6; i++) 
    {
        joint_T_profilel[i].vel_max = 50;    
        joint_T_profilel[i].acc_max = 20;
        joint_T_profilel[i].vel_cur = 0;
        joint_T_profilel[i].vel_s_max = 0;
        joint_T_profilel[i].t_time = 0;
        joint_T_profilel[i].t_time_count = 0;
        joint_T_profilel[i].pos_target_last = 0;  
    }
        
}


void Update_T_Profile(T_Profile *profile, float dt) 
{

/**********************************************************Ŀ��λ�øı�,��ʼ���¹滮**********************************************************/
    if(profile->pos_target != profile->pos_target_last)
    {
        profile->vel_cur = 0;
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
                profile->pos_current = profile->pos_target;
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
                profile->pos_current = profile->pos_target;
            }
            break;
            default:
                break;
        } 
    }
    profile->pos_current +=  profile->vel_cur * dt ;
    
}




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


        for (int i = 0; i < 6; i++) 
        {
            arm_joint[i].pos = joint_T_profilel[i].pos_current;     // ʹ��ƽ�����λ��
        }
        
        Arm_joints_control(arm_joint);
        osDelay(1);
    }

    /* USER CODE END CtrTask_Entry */
}




