#include "bsp_arm.h"
#include "vofa.h"

ARM_JOINTS arm_joint[6];
ARM_JOINT_ID arm_joint_id;
uint8_t joints_init_flag = 0;

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


