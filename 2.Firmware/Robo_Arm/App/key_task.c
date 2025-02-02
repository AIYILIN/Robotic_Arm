
#include "key_task.h"


float vbus = 0;
volatile uint16_t adc_val[2];
uint8_t key_status = 0;

void KeyTask_Entry(void const * argument)
{
    /* USER CODE BEGIN KeyTask_Entry */
    
    BSP_Buzzer_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val,2);
    osDelay(500);
    
    /* Infinite loop */
    for(;;)
    {
        if (BSP_UserKey_Detect() == BUTTON_PRESSED)
        {   
            BSP_Buzzer_Toggle();
            joints_init_flag = ! joints_init_flag;
            if(joints_init_flag)
            {
                joint_T_profilel[0].pos_target = JOINT_1_INIT_POS;    //-3~3
                joint_T_profilel[1].pos_target = JOINT_2_INIT_POS;    //-8~8
                joint_T_profilel[2].pos_target = JOINT_3_INIT_POS;    //-4~14
                joint_T_profilel[3].pos_target = JOINT_4_INIT_POS;    //-2.7~2.7
            }
        }
        vbus = (adc_val[0]*3.3f/65535)*11.0f;

        // PS2_ScanKey(&my_ps2,0);
        
        osDelay(10);
    }
    /* USER CODE END KeyTask_Entry */
}



