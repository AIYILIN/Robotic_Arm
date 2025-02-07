#include "bsp_arm.h"
#include "vofa.h"

ARM_JOINTS arm_joint[6];
ARM_JOINT_ID arm_joint_id;
uint8_t joints_init_flag = 0;
T_Profile joint_T_profilel[6];// 为6个关节各自创建独立的轨迹规划器


void Arm_joint_init_para_set(ARM_JOINTS arm_joint[6])
{
   arm_joint[0].id = ARM_JOINT_1;
   arm_joint[0].torque = 0;
   arm_joint[0].pos = 0;    //-3~3
   arm_joint[0].w = 0;
   arm_joint[0].kp = 0.3;
   arm_joint[0].kw = 0.3;

   arm_joint[1].id = ARM_JOINT_2;
   arm_joint[1].torque = 0;
   arm_joint[1].pos = 0;    //-8~8
   arm_joint[1].w = 0;
   arm_joint[1].kp = 0.3;
   arm_joint[1].kw = 0.22;

   arm_joint[2].id = ARM_JOINT_3;
   arm_joint[2].torque = 0;
   arm_joint[2].pos = 0;    //-4~14
   arm_joint[2].w = 0;
   arm_joint[2].kp = 0.3;
   arm_joint[2].kw = 0.22;

   arm_joint[3].id = ARM_JOINT_4;
   arm_joint[3].torque = 0;
   arm_joint[3].pos = 0;    //-2.7~2.7
   arm_joint[3].w = 0;
   arm_joint[3].kp = 0.3;
   arm_joint[3].kw = 0.22;


}


void Arm_joint_motion_para_set(ARM_JOINTS arm_joint[6])
{

   arm_joint[0].kp = 0;
   arm_joint[0].kw = 0;

   arm_joint[1].kp = 0;
   arm_joint[1].kw = 0;

   arm_joint[2].kp = 0;
   arm_joint[2].kw = 0;

   arm_joint[3].kp = 0;
   arm_joint[3].kw = 0;

//    arm_joint[0].kp = 20;
//    arm_joint[0].kw = 0.5;

//    arm_joint[1].kp = 6;
//    arm_joint[1].kw = 0.22;

//    arm_joint[2].kp = 6;
//    arm_joint[2].kw = 0.22;

//    arm_joint[3].kp = 20;
//    arm_joint[3].kw = 0.5;

}


HAL_StatusTypeDef Arm_joints_control(ARM_JOINTS arm_joint[6])
{
    for (int i = 0; i < 6; i++)
    {
        switch (arm_joint[i].id)
        {
        case 1:
            {
                Motion_Mode_Enable_Set(&Motors,3,arm_joint[i].torque,arm_joint[i].pos,arm_joint[i].w,arm_joint[i].kp,arm_joint[i].kw);
            }
            break;
        case 4:
            {
                Motion_Mode_Enable_Set(&Motors,1,arm_joint[i].torque,arm_joint[i].pos,arm_joint[i].w,arm_joint[i].kp,arm_joint[i].kw);
            }
            break;
        
        case 2:
            {
                go_cmd2.id = arm_joint[i].id;
                go_cmd2.mode = 1;
                go_cmd2.T = arm_joint[i].torque; 
                go_cmd2.W = arm_joint[i].w;
                go_cmd2.Pos = arm_joint[i].pos;
                go_cmd2.K_P = arm_joint[i].kp;
                go_cmd2.K_W = arm_joint[i].kw;
                SERVO_Send_recv(&go_cmd2,&go_rec2);


                // vofa_send_data(1,go_rec2.Pos);
                // vofa_send_data(2,go_rec2.W);
                // vofa_send_data(3,joint_T_profilel[1].pos_current);
                // vofa_send_data(4,joint_T_profilel[1].pos_target);
                // vofa_send_data(5,joint_T_profilel[1].vel_cur);
                // vofa_sendframetail();

            }
            break;   
        case 3:
            {
                go_cmd3.id = arm_joint[i].id;
                go_cmd3.mode = 1;
                go_cmd3.T = arm_joint[i].torque; 
                go_cmd3.W = arm_joint[i].w;
                go_cmd3.Pos = arm_joint[i].pos;
                go_cmd3.K_P = arm_joint[i].kp;
                go_cmd3.K_W = arm_joint[i].kw;
                SERVO_Send_recv(&go_cmd3,&go_rec3);
            }
            break;
        default:
            break;
        }
        
    }
    return HAL_OK;
    
}


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

/**********************************************************目标位置改变,开始重新规划**********************************************************/
    if(profile->pos_target != profile->pos_target_last)
    {
        profile->vel_cur = 0;
        profile->pos_target_last = profile->pos_target;
        profile->pos_end = profile->pos_target;
        profile->pos_start = profile->pos_current;
        profile->remaining = profile->pos_end - profile->pos_start;

        profile->direction = (profile->pos_target > profile->pos_current)? 1.0f : -1.0f;    
      
        profile->t_time = (fabsf(profile->remaining)-(profile->vel_max * profile->vel_max /profile->acc_max))/profile->vel_max;

        if(profile->t_time >= 0)//梯形
        {
            profile->phase = PHASE_ACCEL_STEADY;//进入第一阶段，匀加速
            profile->t_time_count = 0;//时间计数清零
        }
        else//三角形
        {
            profile->phase = PHASE_ACCEL_STEADY;//进入第一阶段，匀加速
            profile->vel_s_max = sqrtf(fabsf(profile->remaining) * profile->acc_max) * profile->direction;
        }
    }
    /*******************************************************end目标位置改变,重新规划完成end*******************************************************/
        
        
    if(profile->t_time >= 0)//梯形
    {
        switch (profile->phase)
        {
        case PHASE_ACCEL_STEADY://第一阶段，1-匀加速
            {
                profile->vel_cur +=  profile->direction * profile->acc_max *  dt;
                if(fabs(profile->vel_cur)  >= profile->vel_max )
                {
                    profile->phase = PHASE_CRUISE;//进入第二阶段，3-匀速
                    profile->vel_cur = profile->vel_max * profile->direction;//匀速
                }
            }
            break;
        case PHASE_CRUISE://第二阶段，3-匀速
            {
                profile->t_time_count += dt;//时间计数累加
                if(profile->t_time_count >= profile->t_time)
                {
                    profile->phase = PHASE_DECEL_STEADY;//进入第三阶段，5-匀减速
                }
            }
            break;
        case PHASE_DECEL_STEADY://第三阶段，5-匀减速
            {
                profile->vel_cur -= profile->direction * profile->acc_max * dt;
                if(profile->vel_cur * profile->direction <= 0)
                {
                    profile->phase = PHASE_FINISHED;//进入第四阶段，7-完成
                    profile->vel_cur = 0;
                }
            }
            break;
        case PHASE_FINISHED://第四阶段，7-完成
            {
                profile->pos_current = profile->pos_target;
            }
        default:
            break;
        }
    }
    else//三角形
    {
        switch (profile->phase)
        {
            case PHASE_ACCEL_STEADY://第一阶段，1-匀加速
            {
                profile->vel_cur +=  profile->direction * profile->acc_max *  dt;
                if((profile->vel_cur * profile->direction) >= (profile->vel_s_max * profile->direction))
                {
                    profile->phase = PHASE_DECEL_STEADY;//进入第二阶段，5-匀减速
                    profile->vel_cur = profile->vel_s_max;//匀速
                }
            } 
            break;
            char debug_buffer[50];
            case PHASE_DECEL_STEADY://第二阶段，5-匀减速
            {              
                profile->vel_cur -= profile->direction * profile->acc_max * dt;
                if((profile->vel_cur * profile->direction) <= 0)
                {
                    profile->phase = PHASE_FINISHED;//进入第四阶段，7-完成
                    profile->vel_cur = 0;
                }
            }
            break;
            case PHASE_FINISHED://第四阶段，7-完成
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