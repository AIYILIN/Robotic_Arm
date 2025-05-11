
#include "motor_comm_task.h"
#include "Emm_V5_can.h"



void MotorCommTask_Entry(void const * argument)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buffer, RX_BUFFER_SIZE);
    Power_Enable(); 
    
    osDelay(500);

    for (;;) 
    {

        
        // Arm_joints_control(arm_joint);

        osDelay(1);
    }

    /* USER CODE END MotorCommTask_Entry */
}




