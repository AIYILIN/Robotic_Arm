#include "usart.h"
#include "motor_control.h"
#include "crc_ccitt.h"
#include "stdio.h"
#include "vofa.h"
#include <string.h>

#include "dma.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "projdefs.h"

MOTOR_send go_cmd2;
MOTOR_send go_cmd3;
MOTOR_recv go_rec2;
MOTOR_recv go_rec3;


uint8_t go_motor_send_data[17];

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
//		SATURATE(motor_s->id,   0,    15);
//		SATURATE(motor_s->mode, 0,    7);
		SATURATE(motor_s->K_P,  0.0f,   25.599f);
		SATURATE(motor_s->K_W,  0.0f,   25.599f);
		SATURATE(motor_s->T,   -127.99f,  127.99f);
		SATURATE(motor_s->W,   -804.00f,  804.00f);
		SATURATE(motor_s->Pos, -411774.0f,  411774.0f);

    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos  = motor_s->K_P/25.6f*32768;
    motor_s->motor_send_data.comd.k_spd  = motor_s->K_W/25.6f*32768;
    // motor_s->motor_send_data.comd.pos_des  = motor_s->Pos/6.2832f*32768;
    // motor_s->motor_send_data.comd.spd_des  = motor_s->W/6.2832f*256;
    motor_s->motor_send_data.comd.pos_des  = motor_s->Pos*3.1415926f/180.0f*32768;//去掉减速比,加上弧度到角度的运算,现在我发的命令是输出轴的角度
    motor_s->motor_send_data.comd.spd_des  = motor_s->W*3.1415926f/180.0f*256;    //去掉减速比,加上弧度到角度的运算,现在我发的命令是输出轴的角速度
    motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

int extract_data(MOTOR_recv *motor_r)
{
    if(motor_r->motor_recv_data.CRC16 !=
        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
        // printf("[WARNING] Receive data CRC error");
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        // motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
        motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)/3.1415926f*180.0f ;//去掉减速比,加上弧度到角度的运算,现在我收到的命令是输出轴的角速度
        motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
        // motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
        motor_r->Pos = ((float)motor_r->motor_recv_data.fbk.pos) / 32768.0f/3.1415926f*180.0f;       //去掉减速比,加上弧度到角度的运算,现在我收到的命令是输出轴的角度
				motor_r->footForce = motor_r->motor_recv_data.fbk.force;
				motor_r->correct = 1;

        return motor_r->correct;
    }
}


uint16_t *len = NULL;
// uint8_t *buffer = NULL;

HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)
{
    
    modify_data(pData);

    // 启动 DMA 发送 
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)pData, sizeof(pData->motor_send_data));
    if (status != HAL_OK) {
        return status;
    }

    // 等待发送完成（超时 100ms）
    if (xSemaphoreTake(uart_tx_semaphore, pdMS_TO_TICKS(1)) != pdTRUE) {
        return HAL_TIMEOUT;
    }

    // 等待接收完成（超时 100ms）
    if (xSemaphoreTake(uart_rx_semaphore, pdMS_TO_TICKS(1)) != pdTRUE) {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        
        xSemaphoreGiveFromISR(uart_tx_semaphore, NULL);
    }
}

uint16_t rx_count = 0;
uint8_t rData_Handle[32] = {0};
// 接收完成回调（空闲中断）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart == &huart2) {

        xSemaphoreGiveFromISR(uart_rx_semaphore, NULL); 
        // 重启接收（循环模式自动继续）
        
        memcpy(rData_Handle, dma_rx_buffer, 32);
        if(rData_Handle[0] == 0xFD && rData_Handle[1] == 0xEE && rData_Handle[16] == 0xFD && rData_Handle[17] == 0xEE)
        {
            if (rData_Handle[2]== 0x12)
            {
                memcpy(&go_rec2.motor_recv_data, &rData_Handle[0], sizeof(go_rec2.motor_recv_data));
                go_rec2.correct = 1;
                extract_data(&go_rec2);
            }
            if (rData_Handle[18]== 0x13)
            {
                memcpy(&go_rec3.motor_recv_data, &rData_Handle[16], sizeof(go_rec3.motor_recv_data));
                go_rec3.correct = 1;
                extract_data(&go_rec3);
            }
        }
    }
}
// 错误处理回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        // 处理错误（如 DMA 溢出）
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
        }
        HAL_UART_Abort(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_rx_buffer, RX_BUFFER_SIZE);
    }
}