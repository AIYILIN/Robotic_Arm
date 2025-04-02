
#include "kinematic_task.h"




// 计算结果存储
float angles[6] = {0};
float position[6] = {0};

void KinematicsTask_Entry(void const * argument)
{
  /* USER CODE BEGIN KinematicsTask_Entry */
	
	canfd_filter(&hfdcan1);
  canfd_filter(&hfdcan3);
  
  Init_Motor_All();

  Arm_joint_init_para_set(arm_joint);
  joint_T_profilel_param_init(joint_T_profilel);

  osDelay(1000);
  Arm_joint_motion_para_set(arm_joint);

  for(;;)
  {



    // 计算正运动学
    // forward_kinematics(angles, position);
    // inverse_kinematics(position[0],position[1],position[2],angles);
    // ik_iterative(position, angles, 100);


    if(joints_init_flag == 0)
    {
      joint_T_profilel[0].pos_target = JOINT_1_INIT_POS-45;    
      joint_T_profilel[1].pos_target = JOINT_2_INIT_POS+30;    
      joint_T_profilel[2].pos_target = JOINT_3_INIT_POS-30;    
      joint_T_profilel[3].pos_target = JOINT_4_INIT_POS+45;
      joint_T_profilel[4].pos_target = JOINT_5_INIT_POS+45;    
      joint_T_profilel[5].pos_target = JOINT_6_INIT_POS+45;
    }
      osDelay(5500);
    if(joints_init_flag == 0) 
    {
      joint_T_profilel[0].pos_target = JOINT_1_INIT_POS+45;    
      joint_T_profilel[1].pos_target = JOINT_2_INIT_POS-30;    
      joint_T_profilel[2].pos_target = JOINT_3_INIT_POS+30;    
      joint_T_profilel[3].pos_target = JOINT_4_INIT_POS-45;
      joint_T_profilel[4].pos_target = JOINT_5_INIT_POS-45; 
      joint_T_profilel[5].pos_target = JOINT_6_INIT_POS-45; 
    }
      osDelay(5500);


    
    if(joints_init_flag  == 1)
    {
      joint_T_profilel[0].pos_target = JOINT_1_INIT_POS;    
      joint_T_profilel[1].pos_target = JOINT_2_INIT_POS;   
      joint_T_profilel[2].pos_target = JOINT_3_INIT_POS;  
      joint_T_profilel[3].pos_target = JOINT_4_INIT_POS;  
      joint_T_profilel[4].pos_target = JOINT_5_INIT_POS;   
      joint_T_profilel[5].pos_target = JOINT_6_INIT_POS;  
    }

    osDelay(10);
  }
  /* USER CODE END KinematicsTask_Entry */
}
