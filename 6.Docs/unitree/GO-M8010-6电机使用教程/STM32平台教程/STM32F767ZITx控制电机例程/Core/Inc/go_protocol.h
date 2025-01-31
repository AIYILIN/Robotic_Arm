/**
 * @file go_protocol.h
 * @author Yichao Zhang (unitree@qq.com)
 * @brief Go1X/2关节电机驱动 通讯协议&数据包
 * @version 0.1
 * @date 2022-03-04
 * 
 * @copyright Copyright (c) unitree robotics .co.ltd. 2022
 */

#ifndef __GO_PROTOCOL_H
#define __GO_PROTOCOL_H

#include "ris_protocol.h"

#pragma pack(1)

/**
 * @brief 控制数据包格式
 * 
 */
//typedef struct
//{
//    uint8_t head[2];    // 包头         2Byte
//    RIS_Mode_t mode;    // 电机控制模式  1Byte
//    RIS_Comd_t comd;    // 电机期望数据 12Byte
//    uint16_t   CRC16;   // CRC          2Byte
//	
//} ControlData_t;    // 主机控制命令     17Byte

/**
 * @brief 电机反馈数据包格式
 * 
 */
//typedef struct
//{
//    uint8_t head[2];    // 包头         2Byte
//    RIS_Mode_t mode;    // 电机控制模式  1Byte
//    RIS_Fbk_t   fbk;    // 电机反馈数据 11Byte
//    uint16_t  CRC16;    // CRC          2Byte

//} MotorData_t;      // 电机返回数据     16Byte




#pragma pack()

#endif

/* 
 * Unitree robotics (c) .Co.Ltd. 2022 All Rights Reserved.
 */
