
#include "kinematic_task.h"
#include "arm_kinematics.h"

// #include "6dof_kinematic.h"
// #include "dof6_c_interface.h"



// 计算结果存储
float fk_angle[6] = {0};
float fk_pos_eul[6] = {0};

void KinematicsTask_Entry(void const *argument)
{
  /* USER CODE BEGIN KinematicsTask_Entry */

  canfd_filter(&hfdcan1);
  canfd_filter(&hfdcan3);

  Init_Motor_All();

  Arm_joint_init_para_set(arm_joint);
  joint_T_profilel_param_init(joint_T_profilel);

  osDelay(1000);
  Arm_joint_motion_para_set(arm_joint);

  int court = 0;
  for (;;)
  {
    // 计算正运动学
    forward_kinematics(fk_angle, fk_pos_eul);

    Matrix4x4 target_pose = {
      .m = {{ 0.1428  ,  0.9885   , 0.0497   , -30.9827},
            { -0.8582  ,  0.1487  , -0.4912   , -5.4631},
            { -0.4930  ,  0.0275   ,0.8696    ,380.5556},
            {       0   ,      0   ,      0    ,1.0000}}
  };
  
   char num_sols = inverse_kinematics(&target_pose, links, ik_solutions);

        char debug_buf[1024];
        for (int i = 0; i < 8; i++) 
        {
            // 使用固定宽度+符号对齐格式
            snprintf(debug_buf, sizeof(debug_buf),
                "SOL:%02d | J1:%10.2f | J2:%10.2f | J3:%10.2f | J4:%10.2f | J5:%10.2f | J6:%10.2f | V:%d\r\n",
                i+1,
                ik_solutions[i].angles[0],
                ik_solutions[i].angles[1],
                ik_solutions[i].angles[2],
                ik_solutions[i].angles[3],
                ik_solutions[i].angles[4],
                ik_solutions[i].angles[5],
                ik_solutions[i].valid);

            HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, strlen(debug_buf), 10);
            osDelay(100);
        }

    // 更新所有关节的轨迹
    for (int i = 0; i < 6; i++) 
    {
        Update_T_Profile(&joint_T_profilel[i], 0.003f);  // dt=5ms  
    }   

    for (int i = 0; i < 6; i++) 
    {
        arm_joint[i].pos = joint_T_profilel[i].pos_current;     // 使用平滑后的位置
    }

    
    if(court%1000 == 0)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
      court = 0; 
    }
    court++;
    
//    joints_precision_text();

    osDelay(1);
  }
  /* USER CODE END KinematicsTask_Entry */
}





void joints_move_text(void)
{

  if (joints_init_flag == 0)
  {
    joint_T_profilel[0].pos_target = JOINT_1_INIT_POS - 30;
    joint_T_profilel[1].pos_target = JOINT_2_INIT_POS + 30;
    joint_T_profilel[2].pos_target = JOINT_3_INIT_POS - 30;
    joint_T_profilel[3].pos_target = JOINT_4_INIT_POS + 20;
    joint_T_profilel[4].pos_target = JOINT_5_INIT_POS + 30;
    joint_T_profilel[5].pos_target = JOINT_6_INIT_POS + 45;
  }
  osDelay(5500);
  if (joints_init_flag == 0)
  {
    joint_T_profilel[0].pos_target = JOINT_1_INIT_POS + 30;
    joint_T_profilel[1].pos_target = JOINT_2_INIT_POS - 30;
    joint_T_profilel[2].pos_target = JOINT_3_INIT_POS + 30;
    joint_T_profilel[3].pos_target = JOINT_4_INIT_POS - 20;
    joint_T_profilel[4].pos_target = JOINT_5_INIT_POS - 30;
    joint_T_profilel[5].pos_target = JOINT_6_INIT_POS - 45;
  }
  osDelay(5500);

  if (joints_init_flag == 1)
  {
    joint_T_profilel[0].pos_target = JOINT_1_INIT_POS;
    joint_T_profilel[1].pos_target = JOINT_2_INIT_POS;
    joint_T_profilel[2].pos_target = JOINT_3_INIT_POS;
    joint_T_profilel[3].pos_target = JOINT_4_INIT_POS;
    joint_T_profilel[4].pos_target = JOINT_5_INIT_POS;
    joint_T_profilel[5].pos_target = JOINT_6_INIT_POS;
  }
}

void joints_precision_text(void)
{

  if (joints_init_flag == 0)
  {

    joint_T_profilel[0].pos_target = JOINT_1_INIT_POS + 30;
    joint_T_profilel[1].pos_target = JOINT_2_INIT_POS - 60;
    joint_T_profilel[2].pos_target = JOINT_3_INIT_POS - 15;
    joint_T_profilel[3].pos_target = JOINT_4_INIT_POS ;
    joint_T_profilel[4].pos_target = JOINT_5_INIT_POS - 30;
    joint_T_profilel[5].pos_target = JOINT_6_INIT_POS ;

  }
  osDelay(6500);

  if (joints_init_flag == 0)
  {
    joint_T_profilel[4].pos_target = JOINT_5_INIT_POS - 28;
  }
  osDelay(4000);

  if (joints_init_flag == 0)
  {
    joint_T_profilel[4].pos_target = JOINT_5_INIT_POS -30;
  }
  osDelay(500);

  if (joints_init_flag == 0)
  {
    joint_T_profilel[0].pos_target = JOINT_1_INIT_POS - 30;
    joint_T_profilel[1].pos_target = JOINT_2_INIT_POS + 45;
    joint_T_profilel[2].pos_target = JOINT_3_INIT_POS + 65;
    joint_T_profilel[3].pos_target = JOINT_4_INIT_POS - 90;
    joint_T_profilel[4].pos_target = JOINT_5_INIT_POS - 45;
    joint_T_profilel[5].pos_target = JOINT_6_INIT_POS ;

  }
  osDelay(6500);

  if (joints_init_flag == 1)
  {
    joint_T_profilel[0].pos_target = JOINT_1_INIT_POS;
    joint_T_profilel[1].pos_target = JOINT_2_INIT_POS;
    joint_T_profilel[2].pos_target = JOINT_3_INIT_POS;
    joint_T_profilel[3].pos_target = JOINT_4_INIT_POS;
    joint_T_profilel[4].pos_target = JOINT_5_INIT_POS;
    joint_T_profilel[5].pos_target = JOINT_6_INIT_POS;
  }
}