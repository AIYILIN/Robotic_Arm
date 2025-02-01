#include "communication.h"
#include "vofa.h"
#include "bsp_arm.h"
#include "motor_control.h"

/* USER CODE BEGIN Header_CommunicationTask_Entry */
/**
* @brief Function implementing the Communication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommunicationTask_Entry */



void CommunicationTask_Entry(void const * argument)
{
  /* USER CODE BEGIN CommunicationTask_Entry */
	
	canfd_filter(&hfdcan1);
  Init_Motor_All();

  Arm_joint_init_para_set(arm_joint);
  joint_T_profilel_param_init(joint_T_profilel);


  // /* Infinite loop */
  osDelay(5000);
  Arm_joint_motion_para_set(arm_joint);

  for(;;)
  {
    if(joints_init_flag == 0)
    {
      joint_T_profilel[0].pos_target = JOINT_1_INIT_POS-45;    
      joint_T_profilel[1].pos_target = JOINT_2_INIT_POS+45;    
      joint_T_profilel[2].pos_target = JOINT_3_INIT_POS-45;    
      joint_T_profilel[3].pos_target = JOINT_4_INIT_POS+45;    
    }
      osDelay(5500);
    if(joints_init_flag == 0) 
    {
      joint_T_profilel[0].pos_target = JOINT_1_INIT_POS+45;    
      joint_T_profilel[1].pos_target = JOINT_2_INIT_POS-45;    
      joint_T_profilel[2].pos_target = JOINT_3_INIT_POS+45;    
      joint_T_profilel[3].pos_target = JOINT_4_INIT_POS-45;    
    }
      osDelay(5500);
    
    if(joints_init_flag  == 1)
    {
      joint_T_profilel[0].pos_target = JOINT_1_INIT_POS;    
      joint_T_profilel[1].pos_target = JOINT_2_INIT_POS;   
      joint_T_profilel[2].pos_target = JOINT_3_INIT_POS;    
      joint_T_profilel[3].pos_target = JOINT_4_INIT_POS;    
    }

    osDelay(10);
  }
  /* USER CODE END CommunicationTask_Entry */
}
