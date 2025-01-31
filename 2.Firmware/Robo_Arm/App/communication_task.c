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
  // joint_T_profilel_param_init(joint_T_profilel,5000,30,30);

  // /* Infinite loop */
    osDelay(5000);
    Arm_joint_motion_para_set(arm_joint);

  for(;;)
  {
    // if(joints_init_flag == 0)
    // {
    //   joint_S_profilel[0].target_pos = 10;    
    //   joint_S_profilel[1].target_pos = 50;    
    //   joint_S_profilel[2].target_pos = 70;    
    //   joint_S_profilel[3].target_pos = -30;    
    // }
    //   osDelay(5500);
    // if(joints_init_flag == 0) 
    // {
    //   joint_S_profilel[0].target_pos = -10;    
    //   joint_S_profilel[1].target_pos = -25;    
    //   joint_S_profilel[2].target_pos = 0;    
    //   joint_S_profilel[3].target_pos = 30;    

    //   osDelay(5500);
    // }
    if(joints_init_flag  == 1)
    {
      joint_T_profilel[0].pos_target = 0;    
      joint_T_profilel[1].pos_target = 0;   
      joint_T_profilel[2].pos_target = 0;    
      joint_T_profilel[3].pos_target = 0;    
    }

    osDelay(10);
  }
  /* USER CODE END CommunicationTask_Entry */
}
