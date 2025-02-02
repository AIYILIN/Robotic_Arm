
#include "motor_comm_task.h"






void MotorCommTask_Entry(void const * argument)
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

    /* USER CODE END MotorCommTask_Entry */
}




