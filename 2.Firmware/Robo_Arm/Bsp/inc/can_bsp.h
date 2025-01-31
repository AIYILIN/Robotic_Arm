#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__

#include "main.h"
#include "fdcan.h"

/**
************************************************************************
* @brief:      	FDCAN滤波器配置结构体
* @details:    	包含FDCAN滤波器的配置信息，包括滤波器ID、类型等
************************************************************************
**/
extern FDCAN_FilterTypeDef sFilterConfig;

/**
************************************************************************
* @brief:      	FDCAN发送头部结构体
* @details:    	包含发送消息的头部信息，例如标识符、数据长度等
************************************************************************
**/
extern FDCAN_TxHeaderTypeDef TxHeader;

/**
************************************************************************
* @brief:      	FDCAN接收头部结构体
* @details:    	包含接收消息的头部信息，例如标识符、数据长度等
************************************************************************
**/
extern FDCAN_RxHeaderTypeDef FDCan1_RxHeader;
extern FDCAN_RxHeaderTypeDef FDCan2_RxHeader;
extern FDCAN_RxHeaderTypeDef FDCan3_RxHeader;

/**
************************************************************************
* @brief:      	FDCAN接收数据缓冲区
* @details:    	存储接收到的数据
************************************************************************
**/
extern uint8_t FDCAN1_Rx_Data[8];
extern uint8_t FDCAN2_Rx_Data[8];
extern uint8_t FDCAN3_Rx_Data[8];

/**
************************************************************************
* @brief:      	FDCAN1发送消息
* @param:       msg：待发送的数据
* @param:       len：数据长度
* @param:       CAN_ID：FDCAN消息标识符
* @retval:     	0：成功；1：失败
* @details:    	向FDCAN1发送消息
************************************************************************
**/
uint8_t FDCAN1_Send_Msg(uint8_t* msg, uint32_t len, uint32_t CAN_ID);

/**
************************************************************************
* @brief:      	配置FDCAN滤波器
* @param:       fdcanhandle：FDCAN句柄
* @retval:     	void
* @details:    	初始化并配置FDCAN滤波器
************************************************************************
**/
void canfd_filter(FDCAN_HandleTypeDef* fdcanhandle);

/**
************************************************************************
* @brief:      	FDCAN接收消息
* @param:       buf：接收数据缓冲区
* @retval:     	接收的数据长度
* @details:    	从FDCAN接收消息并存储在buf中
************************************************************************
**/
uint8_t FDCAN1_Receive_Msg(FDCAN_RxHeaderTypeDef *header,uint8_t *data);
uint8_t FDCAN2_Receive_Msg(FDCAN_RxHeaderTypeDef *header,uint8_t *data);
uint8_t FDCAN3_Receive_Msg(FDCAN_RxHeaderTypeDef *header,uint8_t *data);

/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback
* @param:       hfdcan：FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的FDCAN中断回调函数，当FDCAN接收到新消息时被调用
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
************************************************************************
* @brief:      	fdcan_rx_callback
* @param:      	void
* @retval:     	void
* @details:    	供用户调用的FDCAN接收数据回调函数
************************************************************************
**/
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

#endif /* __CAN_BSP_H__ */
