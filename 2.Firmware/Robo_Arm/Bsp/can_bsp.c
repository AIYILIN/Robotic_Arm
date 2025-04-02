#include "can_bsp.h"
#include "cybergear.h"
#include "vofa.h"


FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;

FDCAN_RxHeaderTypeDef FDCan1_RxHeader;
FDCAN_RxHeaderTypeDef FDCan2_RxHeader;
FDCAN_RxHeaderTypeDef FDCan3_RxHeader;

uint8_t FDCAN1_Rx_Data[8] = {0};
uint8_t FDCAN2_Rx_Data[8] = {0};
uint8_t FDCAN3_Rx_Data[8] = {0};

/**
************************************************************************
* @brief:      	canfd_filter(FDCAN_HandleTypeDef* fdcanhandle)
* @param:       fdcanhandle：FDCAN句柄
* @retval:     	void
* @details:    	配置FDCAN滤波器，并启动FDCAN接口
************************************************************************
**/
void canfd_filter(FDCAN_HandleTypeDef* fdcanhandle)
{
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;  // 扩展ID
    sFilterConfig.FilterIndex = 0;  // 滤波器索引   
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;  // 滤波器类型
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 滤波器0关联到FIFO0  
    sFilterConfig.FilterID1 = 0x00000000;

    if(fdcanhandle == &hfdcan3) {
        sFilterConfig.FilterID2 = 0x00000000;
    }  
    else {
        sFilterConfig.FilterID2 = 0x1FFFFFFF;
    }

    TxHeader.IdType = FDCAN_EXTENDED_ID;  // 扩展ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;  // 数据帧
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;  // 关闭速率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // 传统的CAN模式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;  // 无发送事件
    TxHeader.MessageMarker = 0;  // 消息标记

    // 配置FDCAN滤波器
    if (HAL_FDCAN_ConfigFilter(fdcanhandle, &sFilterConfig) != HAL_OK)
        HAL_FDCAN_ConfigGlobalFilter(fdcanhandle, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE);  // FDCAN_ACCEPT_IN_RX_FIFO0
    
    HAL_FDCAN_ActivateNotification(fdcanhandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(fdcanhandle);  // 开启FDCAN
}

/**
************************************************************************
* @brief:      	FDCAN1_Send_Msg(uint8_t* msg, uint32_t len, uint32_t CAN_ID)
* @param:       msg：待发送的数据
* @param:       len：数据长度
* @param:       CAN_ID：FDCAN消息标识符
* @retval:     	0：成功；1：失败
* @details:    	向FDCAN1发送消息
************************************************************************
**/
uint8_t FDCAN1_Send_Msg(uint8_t* msg, uint32_t len, uint32_t CAN_ID)
{
    TxHeader.Identifier = CAN_ID;  // 32位ID
    TxHeader.DataLength = len;  // 数据长度
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, msg) != HAL_OK) 
    {
        return 1;  // 发送失败
    }

    return 0;  // 发送成功
}

/**
************************************************************************
* @brief:      	FDCAN2_Send_Msg(uint8_t* msg, uint32_t len, uint32_t CAN_ID)
* @param:       msg：待发送的数据
* @param:       len：数据长度
* @param:       CAN_ID：FDCAN消息标识符
* @retval:     	0：成功；1：失败
* @details:    	向FDCAN2发送消息
************************************************************************
**/
uint8_t FDCAN2_Send_Msg(uint8_t* msg, uint32_t len, uint32_t CAN_ID)
{
    TxHeader.Identifier = CAN_ID;  // 32位ID
    TxHeader.DataLength = len;  // 数据长度
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, msg) != HAL_OK)
    {
        return 1;  // 发送失败
    }

    return 0;  // 发送成功
}
/**
************************************************************************
* @brief:      	FDCAN3_Send_Msg(uint8_t* msg, uint32_t len, uint32_t CAN_ID)
* @param:       msg：待发送的数据
* @param:       len：数据长度
* @param:       CAN_ID：FDCAN消息标识符
* @retval:     	0：成功；1：失败
* @details:    	向FDCAN2发送消息
************************************************************************
**/
uint8_t FDCAN3_Send_Msg(uint8_t* msg, uint32_t len, uint32_t CAN_ID)
{
    TxHeader.Identifier = CAN_ID;  // 32位ID
    TxHeader.DataLength = len;  // 数据长度
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, msg) != HAL_OK) 
    {
        return 1;  // 发送失败
    }

    return 0;  // 发送成功
}
/**
************************************************************************
* @brief:      	FDCAN1_Receive_Msg(uint8_t *buf)
* @param:       1.header:接受数据位前面的标识符 2.buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	从FDCAN1接收数据
************************************************************************
**/
uint8_t FDCAN1_Receive_Msg(FDCAN_RxHeaderTypeDef *header,uint8_t *data)
{
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, header, data) != HAL_OK)
        return 0;  // 接收失败
    return header->DataLength >> 16;  // 返回数据长度
}

/**
************************************************************************
* @brief:      	FDCAN2_Receive_Msg(uint8_t *buf)
* @param:       1.header:接受数据位前面的标识符 2.buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	从FDCAN2接收数据
************************************************************************
**/
uint8_t FDCAN2_Receive_Msg(FDCAN_RxHeaderTypeDef *header,uint8_t *data)
{
    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, header, data) != HAL_OK)
        return 0;  // 接收失败
    return header->DataLength >> 16;  // 返回数据长度
}

/**
************************************************************************
* @brief:      	FDCAN3_Receive_Msg(uint8_t *buf)
* @param:       1.header:接受数据位前面的标识符 2.buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	从FDCAN3接收数据
************************************************************************
**/
uint8_t FDCAN3_Receive_Msg(FDCAN_RxHeaderTypeDef *header,uint8_t *data)
{
    if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, header, data) != HAL_OK)
        return 0;  // 接收失败
    return header->DataLength >> 16;  // 返回数据长度
}

/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan：FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的FDCAN中断回调函数
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (hfdcan == &hfdcan1)
        {
            fdcan1_rx_callback();  // FDCAN1接收回调
        }
        if (hfdcan == &hfdcan2)
        {
            fdcan2_rx_callback();  // FDCAN2接收回调
        }
        if (hfdcan == &hfdcan3)
        {
            fdcan3_rx_callback();  // FDCAN3接收回调
        }
    }
}

/**
************************************************************************
* @brief:      	fdcan1_rx_callback(void)
* @param:      	void
* @retval:     	void
* @details:    	供用户调用的FDCAN1接收数据回调函数
************************************************************************
**/
void fdcan1_rx_callback(void)
{
    FDCAN1_Receive_Msg(&FDCan1_RxHeader,FDCAN1_Rx_Data);  // 接收FDCAN1的数据
	// Motor_Data_Get(&Motors,&FDCan1_RxHeader,FDCAN1_Rx_Data);
    Motor_Data_Handler(&Motors,FDCan1_RxHeader.Identifier,FDCAN1_Rx_Data);
    // vofa_send_data(1,Motors.Motor3.Angle);
    // vofa_send_data(2,Motors.Motor3.Speed);
    // vofa_send_data(1,Motors.Motor3.Torque);
    // vofa_send_data(4,Motors.Motor3.Temp);
    // vofa_sendframetail();
}

/**
************************************************************************
* @brief:      	fdcan2_rx_callback(void)
* @param:      	void
* @retval:     	void
* @details:    	供用户调用的FDCAN2接收数据回调函数
************************************************************************
**/
void fdcan2_rx_callback(void)
{
    FDCAN2_Receive_Msg(&FDCan2_RxHeader,FDCAN2_Rx_Data);  // 接收FDCAN2的数据
}

/**
************************************************************************
* @brief:      	fdcan3_rx_callback(void)
* @param:      	void
* @retval:     	void
* @details:    	供用户调用的FDCAN3接收数据回调函数
************************************************************************
**/
void fdcan3_rx_callback(void)
{
    FDCAN3_Receive_Msg(&FDCan3_RxHeader,FDCAN3_Rx_Data);  // 接收FDCAN3的数据
}



/**
************************************************************************
* @brief:       Emm V5 步进电机驱动
************************************************************************
**/
// void USER_CAN3_Filter_Init(void)
// {
//     FDCAN_FilterTypeDef sFilterConfig;
    
//     // 配置全局过滤器
//     HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, 
//         FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    
//     // 配置标准ID过滤器
//     sFilterConfig.IdType = FDCAN_EXTENDED_ID;
//     sFilterConfig.FilterIndex = 0;
//     sFilterConfig.FilterType = FDCAN_FILTER_MASK;
//     sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//     sFilterConfig.FilterID1 = 0x00000000;  // ID掩码
//     sFilterConfig.FilterID2 = 0x00000000;  // 接收所有ID
    
//     HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig);
    
//     // 激活FIFO
//     HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
//     HAL_FDCAN_Start(&hfdcan3);
//     HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
// }

CAN_t emm_can;

/**
  * @brief  FDCAN发送多字节数据（适配H7）
  * @param  cmd: 数据指针，格式[ID高字节, ID低字节, data1, data2...]
  * @param  len: 数据总长度（包含ID字段）
  * @retval 无
  */
void Emm_can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    __IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
    
    // 数据长度计算（减去ID占用的2字节）
    j = len - 2;

    while(i < j)
    {
        // 初始化发送头（每次发送需重置）
        emm_can.CAN_TxMsg.Identifier = ((uint32_t)cmd[0] << 8) | packNum; // 组合ID与包序号
        emm_can.CAN_TxMsg.IdType = FDCAN_EXTENDED_ID;      // 扩展ID
        emm_can.CAN_TxMsg.TxFrameType = FDCAN_DATA_FRAME;  // 数据帧
        emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_8;  // 默认长度
        emm_can.CAN_TxMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        emm_can.CAN_TxMsg.BitRateSwitch = FDCAN_BRS_OFF;    // 关闭波特率切换
        emm_can.CAN_TxMsg.FDFormat = FDCAN_CLASSIC_CAN;     // 经典CAN模式
        emm_can.CAN_TxMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

        // 填充首字节数据
        emm_can.txData[0] = cmd[1];
        
        // 计算剩余数据长度
        k = j - i;

        // 数据分包处理
        if(k < 8)
        {
            // 填充剩余数据
            for(l = 0; l < k; l++, i++)
            {
                emm_can.txData[l + 1] = cmd[i + 2];
            }
             // 根据实际数据长度选择对应宏
                switch (k + 1)  // +1 是因为首字节 cmd[1] 占用了1字节
                {
                    case 1: emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_1; break;
                    case 2: emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_2; break;
                    case 3: emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_3; break;
                    case 4: emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_4; break;
                    case 5: emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_5; break;
                    case 6: emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_6; break;
                    case 7: emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_7; break;
                    case 8: emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_8; break;
                    default: break;
                }
        }
        else
        {
            // 填充7字节数据（保留1字节序号）
            for(l = 0; l < 7; l++, i++)
            {
                emm_can.txData[l + 1] = cmd[i + 2];
            }
            emm_can.CAN_TxMsg.DataLength = FDCAN_DLC_BYTES_8;
        }

        // 发送数据（H7使用FIFO队列）
        while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &emm_can.CAN_TxMsg, emm_can.txData) != HAL_OK);
        packNum++;
    }
}