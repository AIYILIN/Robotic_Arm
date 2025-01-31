/**
 ******************************************************************************
* @file   
* @brief  
******************************************************************************
* @note

*/

/* Includes ------------------------------------------------------------------*/
#include "cybergear.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "string.h"
#define M_PI		3.1415926f
/* Definitions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t byte[4];
MI_Motor Motor1;
MI_Motor Motor2;
MI_Motor Motor3;
MI_Motor Motor4;
MI_Motor Motor5;
MI_Motor Motor6;

MotorGroup Motors;


/* Global variables ---------------------------------------------------------*/

/*Functions------------------------------------------------------------------*/

/*******************************************************************************
 * @function     : 浮点数转4字节函数
 * @param        : 浮点数
 * @return       : 4字节数组
 * @description  : IEEE 754 协议
 *******************************************************************************/
uint8_t* Float_to_Byte(float f)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;       
	byte[0] = (longdata & 0xFF000000) >> 24;
	byte[1] = (longdata & 0x00FF0000) >> 16;
	byte[2] = (longdata & 0x0000FF00) >> 8;
	byte[3] = (longdata & 0x000000FF);
	return byte;
}

/*******************************************************************************
 * @function     : 小米电机回文16位数据转浮点
 * @param        : 1. 16位回文 2.对应参数下限 3.对应参数上限 4. 参数位数
 * @return       : 参数对应浮点数
 * @description  : None
 *******************************************************************************/
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
	uint32_t span = (1 << bits) - 1;
	float offset = x_max - x_min;
	return offset * x / span + x_min;
}

/*******************************************************************************
 * @function     : 电机参数初始化
 * @param        : 1. 电机结构体 2.电机CANID 3.电机编号 4.电机工作模式（1.运控模式 2. 位置模式 3. 速度模式 4. 电流模式）
 * @return       : None
 * @description  : 负责初始化电机 CANID 电机编号 电机的工作模式
 *******************************************************************************/
void Init_Motor(MotorGroup *motors ,uint8_t Can_Id ,int motor_num,float mode)
{
	if(motor_num>=1 && motor_num<=6)
	{
		MI_Motor *motor = NULL;

		if(motor_num==1) motor = &motors->Motor1;
		if(motor_num==2) motor = &motors->Motor2;
		if(motor_num==3) motor = &motors->Motor3;
		if(motor_num==4) motor = &motors->Motor4;
		if(motor_num==5) motor = &motors->Motor5;
		if(motor_num==6) motor = &motors->Motor6;

		motor->CAN_ID=Can_Id;
		motor->Motor_Nbr=motor_num;

		Set_Mode(motors,motor_num,mode);
		osDelay(1);
		Enable_Motor(motors,motor_num);
		osDelay(1);

		Set_Motor_Parameter(motors,motor_num,Spd_Kp,2,'f');
		osDelay(1);
		Set_Motor_Parameter(motors,motor_num,Spd_Ki,0.2,'f');
		osDelay(1);
		Set_Motor_Parameter(motors,motor_num,Limit_Cur,5,'f');//电流滤波系数
		osDelay(1);

		// Set_Motor_Parameter(motors,motor_num,Spd_Kp,2,'f');		//spd_kp
		// Set_Motor_Parameter(motors,motor_num,Spd_Kp,0.5,'f');	//spd_ki
		// Set_Motor_Parameter(motors,motor_num,Spd_Ref,0,'f');	//设置速度值-30 -> 30 rad
	}
}

void Init_Motor_All(void)
{
	Init_Motor(&Motors,101,1,Speed_mode);
	// Init_Motor(&Motors,102,2,Speed_mode);
	Init_Motor(&Motors,103,3,Speed_mode);
	// Init_Motor(&Motors,104,4,Speed_mode);
	// Init_Motor(&Motors,105,5,Speed_mode);
	// Init_Motor(&Motors,106,6,Speed_mode);

	Set_ZeroPos(&Motors,1);
	Set_ZeroPos(&Motors,3);


}
	

/*******************************************************************************
 * @function     : 提取电机回复帧扩展ID中的电机CANID
 * @param        : 电机回复帧中的扩展CANID
 * @return       : 电机ID
 * @description  : 故障反馈帧（通信类型21）取0-7位，其他三种取扩展ID的8-15位,
 *******************************************************************************/
uint8_t Get_Motor_ID(uint32_t CAN_ID_Frame)
{
	if (((CAN_ID_Frame>>24)&0x000000ff) == Communication_Type_ErrorFeedback)	//故障反馈帧（通信类型21）
	{
		return (CAN_ID_Frame)&0x000000ff;
	}
	
	return (CAN_ID_Frame>>8)&0x000000ff;
}
/*******************************************************************************
 * @function     : 使能电机
 * @param        : 对应控制电机结构体
 * @return       : None
 * @description  : 使能电机
 *******************************************************************************/
void Enable_Motor(MotorGroup *motors,uint8_t motor_num)
{
	MI_Motor *motor = NULL;
	
	if(motor_num==1) motor = &motors->Motor1;
	if(motor_num==2) motor = &motors->Motor2;
	if(motor_num==3) motor = &motors->Motor3;
	if(motor_num==4) motor = &motors->Motor4;
	if(motor_num==5) motor = &motors->Motor5;
	if(motor_num==6) motor = &motors->Motor6;

	uint32_t Send_ID;
	uint8_t temp[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	Send_ID = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|motor->CAN_ID;
	FDCAN1_Send_Msg(temp,FDCAN_DLC_BYTES_8,Send_ID);
}

/*******************************************************************************
 * @function     : 停止电机
 * @param        : 1.对应控制电机结构体 2.清除错误位（0 不清除 1清除）
 * @return       : None
 * @description  : 使能电机
 *******************************************************************************/
void Stop_Motor(MotorGroup *motors,uint8_t motor_num,uint8_t clear_error)
{
	MI_Motor *motor = NULL;
	
	if(motor_num==1) motor = &motors->Motor1;
	if(motor_num==2) motor = &motors->Motor2;
	if(motor_num==3) motor = &motors->Motor3;
	if(motor_num==4) motor = &motors->Motor4;
	if(motor_num==5) motor = &motors->Motor5;
	if(motor_num==6) motor = &motors->Motor6;

	uint32_t Send_ID;
	uint8_t temp[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	temp[0]=clear_error;
	Send_ID = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|motor->CAN_ID;
	FDCAN1_Send_Msg(temp,FDCAN_DLC_BYTES_8,Send_ID);
}

/*******************************************************************************
 * @function     : 电机回复帧数据处理函数
 * @param        : 1.对应控制电机结构体 2.扩展ID帧 3.数据帧
 * @return       : None
 * @description  : 获取角度 速度 扭矩 温度 错误代码等等
 *******************************************************************************/
void Motor_Data_Handler(MotorGroup *motors,uint32_t IDFrame,uint8_t DataFrame[8])
{
	MI_Motor *motor_temp = NULL;

	switch (GetMotorFeedbackType(IDFrame))
	{
		case  Communication_Type_GetID ://0x00获取设备的ID和64位MCU唯一标识符
			{
				if(motors->Motor1.CAN_ID == Get_Motor_ID(IDFrame))	motors->Motor1.MCU_64bit_Id = get_motor_MCU_64bit_Id(DataFrame);
				if(motors->Motor2.CAN_ID == Get_Motor_ID(IDFrame))	motors->Motor2.MCU_64bit_Id = get_motor_MCU_64bit_Id(DataFrame);
				if(motors->Motor3.CAN_ID == Get_Motor_ID(IDFrame))	motors->Motor3.MCU_64bit_Id = get_motor_MCU_64bit_Id(DataFrame);
				if(motors->Motor4.CAN_ID == Get_Motor_ID(IDFrame))	motors->Motor4.MCU_64bit_Id = get_motor_MCU_64bit_Id(DataFrame);
				if(motors->Motor5.CAN_ID == Get_Motor_ID(IDFrame))	motors->Motor5.MCU_64bit_Id = get_motor_MCU_64bit_Id(DataFrame);
				if(motors->Motor6.CAN_ID == Get_Motor_ID(IDFrame))	motors->Motor6.MCU_64bit_Id = get_motor_MCU_64bit_Id(DataFrame);
			}
			break;
		case  Communication_Type_MotorRequest ://0x02用来向主机反馈电机运行状态
			{
				motor_temp = NULL;

				if(motors->Motor1.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor1;
				if(motors->Motor2.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor2;
				if(motors->Motor3.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor3;
				if(motors->Motor4.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor4;
				if(motors->Motor5.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor5;
				if(motors->Motor6.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor6;

				motor_temp->error_code=(IDFrame&0x1F0000)>>16;// 解析故障信息（bit21~16）
				// 各种故障类型的标志
				motor_temp->error_is_uncalibrated = (motor_temp->error_code & (1 << 21)) != 0;
				motor_temp->error_is_hall_fault =	 (motor_temp->error_code & (1 << 20)) != 0;
				motor_temp->error_is_mag_fault = 	(motor_temp->error_code & (1 << 19)) != 0;
				motor_temp->error_is_overtemp = 	(motor_temp->error_code & (1 << 18)) != 0;
				motor_temp->error_is_overcurrent = (motor_temp->error_code & (1 << 17)) != 0;
				motor_temp->error_is_undervoltage = (motor_temp->error_code & (1 << 16)) != 0;

				// 解析模式状态（bit22~23）
				motor_temp->mode_status = (IDFrame & 0x00C00000) >> 22;

				// 解析角度 速度 扭矩 温度（data-> Byte0~Byte7 ）
				motor_temp->Angle=uint16_to_float(DataFrame[0]<<8|DataFrame[1],MIN_P,MAX_P,16);		//Byte0~1:当前角度[0~65535]对应(-4π~4π) 
				motor_temp->Speed=uint16_to_float(DataFrame[2]<<8|DataFrame[3],MIN_S,MAX_S,16);		//Byte2~3:当前角速度[0~65535]对应(30rad/s~30rad/s)  
				motor_temp->Torque=uint16_to_float(DataFrame[4]<<8|DataFrame[5],MIN_T,MAX_T,16);	//Byte4~5:当前力矩 [0~65535]对应（-12Nm~12Nm）  				
				motor_temp->Temp=(DataFrame[6]<<8|DataFrame[7])*Temp_Gain;							//Byte6~7:当前温度：Temp(摄氏度）*10	
				
			}

			break;
		case  Communication_Type_GetSingleParameter ://0x11读取单个参数
			{
				motor_temp = NULL;

				if(motors->Motor1.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor1;
				if(motors->Motor2.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor2;
				if(motors->Motor3.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor3;
				if(motors->Motor4.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor4;
				if(motors->Motor5.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor5;
				if(motors->Motor6.CAN_ID == Get_Motor_ID(IDFrame))	motor_temp = &motors->Motor6;

				if((DataFrame[0]==0x05)&&(DataFrame[1]==0x70)&&(DataFrame[2]==0x00)&&(DataFrame[3]==0x00))
				motor_temp->run_mode = DataFrame[7];//貌似对了
				/*
				这里还有好多要写的
				*/
			}
			break;
		case  Communication_Type_ErrorFeedback ://0x15故障反馈帧
			{
				
				
			}
			break;
		default:
			break;
	}


	

}

/*******************************************************************************
 * @function     : 获取各个电机的反馈类型
 * @param        : 1.扩展ID帧
 * @return       : 1.0x00:获取设备的ID和64位MCU唯一标识符 2.0x02:用来向主机反馈电机运行状态 3.0x11:读取单个参数 4.0x15:故障反馈帧
 * @description  : 获取各个电机的反馈类型
 *******************************************************************************/
uint8_t GetMotorFeedbackType(uint32_t can_id_frame)
{
	return (can_id_frame>>24)&0x000000ff;
}

/*******************************************************************************
 * @function     : 获取各个电机的MCU_ID
 * @param        : 1.数据帧
 * @return       : 1.MCU ID
 * @description  : 获取各个电机的MCU_ID
 *******************************************************************************/
uint64_t get_motor_MCU_64bit_Id(uint8_t *DataFrame)
{
    return ((uint64_t)DataFrame[7] << 56) | ((uint64_t)DataFrame[6] << 48) | 
           ((uint64_t)DataFrame[5] << 40) | ((uint64_t)DataFrame[4] << 32) |
           ((uint64_t)DataFrame[3] << 24) | ((uint64_t)DataFrame[1] << 16) |
           ((uint64_t)DataFrame[1] << 8)  | (uint64_t)DataFrame[0];
}


/*******************************************************************************
 * @function     : 写入电机参数
 * @param        : 1.对应控制电机结构体 2.写入参数对应地址 3.写入参数值 4.写入参数数据类型
 * @return       : None
 * @description  : None
 *******************************************************************************/
void Set_Motor_Parameter(MotorGroup *motors ,uint8_t motor_num ,uint16_t Index ,float Value ,char Value_type)
{
	uint32_t Send_ID;
	uint8_t Send_Data[8];
	if(motor_num==1)Send_ID = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|motors->Motor1.CAN_ID;
	if(motor_num==2)Send_ID = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|motors->Motor2.CAN_ID;
	if(motor_num==3)Send_ID = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|motors->Motor3.CAN_ID;
	if(motor_num==4)Send_ID = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|motors->Motor4.CAN_ID;
	if(motor_num==5)Send_ID = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|motors->Motor5.CAN_ID;
	if(motor_num==6)Send_ID = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|motors->Motor6.CAN_ID;
	Send_Data[0]=Index;
	Send_Data[1]=Index>>8;
	Send_Data[2]=0x00;
	Send_Data[3]=0x00;
	if(Value_type == 'f'){
		Float_to_Byte(Value);
		Send_Data[4]=byte[3];
		Send_Data[5]=byte[2];
		Send_Data[6]=byte[1];
		Send_Data[7]=byte[0];		
	}
	else if(Value_type == 's'){
		Send_Data[4]=(uint8_t)Value;
		Send_Data[5]=0x00;
		Send_Data[6]=0x00;
		Send_Data[7]=0x00;				
	}
	FDCAN1_Send_Msg(Send_Data,FDCAN_DLC_BYTES_8,Send_ID);	
}

/*******************************************************************************
 * @function     : 设置电机控制模式
 * @param        : %2
 * @return       : %3
 * @description  : %4
 *******************************************************************************/
void Set_Mode(MotorGroup *motors,char motor_num,float Mode)
{
	Set_Motor_Parameter(motors,motor_num,Run_mode,Mode,'s');
}

/*******************************************************************************
 * @function     : 电流模式->使能->设置电流值
 * @param        : 1.电机结构体 2.电机序号 3.设定电流值
 * @return       : NULL
 * @description  : 电流模式->使能->设置电流值
 *******************************************************************************/
void CurrentMode_Enable_Set(MotorGroup *motors,uint8_t motor_num,float iq_ref)
{
	MI_Motor *motor = NULL;
	
	if(motor_num==1) motor = &motors->Motor1;
	if(motor_num==2) motor = &motors->Motor2;
	if(motor_num==3) motor = &motors->Motor3;
	if(motor_num==4) motor = &motors->Motor4;
	if(motor_num==5) motor = &motors->Motor5;
	if(motor_num==6) motor = &motors->Motor6;

	Set_Mode(motors,motor_num,Current_mode);					//设置电流模式
	Enable_Motor(motors,motor_num);			
	Set_Motor_Parameter(motors,motor_num,Iq_Ref,iq_ref,'f'); //设置电流值-23 -> 23 rad
}

/*******************************************************************************
 * @function     : 速度模式->使能->设置速度模式下最大电流值->设置速度
 * @param        : 1.电机结构体 2.电机序号 3.设定速度
 * @return       : NULL
 * @description  : 速度模式->使能->设置速度模式下最大电流值->设置速度
 *******************************************************************************/
void SpeedMode_Enable_Set(MotorGroup *motors,uint8_t motor_num,float limit_cur,float spd_ref)
{
	MI_Motor *motor_temp = NULL;
	
	if(motor_num==1) motor_temp = &motors->Motor1;
	if(motor_num==2) motor_temp = &motors->Motor2;
	if(motor_num==3) motor_temp = &motors->Motor3;
	if(motor_num==4) motor_temp = &motors->Motor4;
	if(motor_num==5) motor_temp = &motors->Motor5;
	if(motor_num==6) motor_temp = &motors->Motor6;

	Set_Mode(motors,motor_num,Speed_mode);					//设置速度模式
	Enable_Motor(motors,motor_num);							//电机使能
	Set_Motor_Parameter(motors,motor_num,Limit_Cur,limit_cur,'f');	//设置速度模式下最大电流值
	Set_Motor_Parameter(motors,motor_num,Spd_Ref,spd_ref,'f');	//设置速度值-30 -> 30 rad
	
}


/*******************************************************************************
 * @function     : 位置模式->使能->设置位置模式下最大速度值->设置位置
 * @param        : 1.电机结构体 2.电机序号 3.设定位置
 * @return       : NULL
 * @description  : 位置模式->使能->设置位置
 *******************************************************************************/
void PositionMode_Enable_Set(MotorGroup *motors,uint8_t motor_num,float limit_speed,float pos_ref)
{
	MI_Motor *motor_temp = NULL;
	
	if(motor_num==1) motor_temp = &motors->Motor1;
	if(motor_num==2) motor_temp = &motors->Motor2;
	if(motor_num==3) motor_temp = &motors->Motor3;
	if(motor_num==4) motor_temp = &motors->Motor4;
	if(motor_num==5) motor_temp = &motors->Motor5;
	if(motor_num==6) motor_temp = &motors->Motor6;

	Set_Mode(motors,motor_num,Position_mode);					//设置位置模式
	Enable_Motor(motors,motor_num);							//电机使能
	Set_Motor_Parameter(motors,motor_num,Limit_Spd,limit_speed,'f');//设置位置模式下最大速度值
	Set_Motor_Parameter(motors,motor_num,Loc_Ref,pos_ref*3.141592/180.0f,'f');	//设置位置（float型）
	
}

/*******************************************************************************
 * @function     : 运控模式->使能->发送运控模式电机控制指令->收到电机反馈帧
 * @param        : 1.电机结构体 2.电机序号 3.设定参数（力矩、位置(角度)、角速度(角度)、Kp、Kd）
 * @return       : NULL
 * @description  : 根据通信类型1协议发送控制指令
 *******************************************************************************/
void Motion_Mode_Enable_Set(MotorGroup *motors,uint8_t motor_num,float torque,float pos,float w,float kp,float kw)
{
	Set_Mode(motors,motor_num,Motion_mode);					//设置位置模式
	Enable_Motor(motors,motor_num);							//电机使能

	 // 校验电机序号有效性
    if (motor_num < 1 || motor_num > 6) return;

	// 获取目标电机的CAN ID
    uint8_t can_id;
    switch (motor_num) {
        case 1: can_id = motors->Motor1.CAN_ID; break;
        case 2: can_id = motors->Motor2.CAN_ID; break;
        case 3: can_id = motors->Motor3.CAN_ID; break;
        case 4: can_id = motors->Motor4.CAN_ID; break;
        case 5: can_id = motors->Motor5.CAN_ID; break;
        case 6: can_id = motors->Motor6.CAN_ID; break;
    }

	uint32_t Send_ID;
	uint8_t Send_Data[8];

	//力矩（-12Nm~12Nm → 0~65535）
    uint16_t torque_mapped = (uint16_t)((torque + 12) / 24 * 65535);
	Send_ID = Communication_Type_MotionControl<<24 | (torque_mapped>>8)<<16 | torque_mapped<<8 | can_id;
	
	uint16_t pos_mapped = (uint16_t)(((pos*M_PI/180.0f) + 4*M_PI) / (8*M_PI) * 65535);
	Send_Data[0] = pos_mapped >> 8;
	Send_Data[1] = pos_mapped;
	
 	uint16_t w_mapped = (uint16_t)(((w*M_PI/180.0f) + 30) / 60 * 65535);
	Send_Data[2]=w_mapped>>8;
	Send_Data[3]=w_mapped;

	uint16_t kp_mapped = (uint16_t)(kp / 500.0f * 65535);
	Send_Data[4]=kp_mapped>>8;
	Send_Data[5]=kp_mapped;		

	uint16_t kw_mapped = (uint16_t)(kw / 5.0f * 65535);
	Send_Data[6]=kw_mapped>>8;
	Send_Data[7]=kw_mapped;

	FDCAN1_Send_Msg(Send_Data,FDCAN_DLC_BYTES_8,Send_ID);	

}



/*******************************************************************************
 * @function     : 设置电机机械零点
 * @param        : %2
 * @return       : %3
 * @description  : %4
 *******************************************************************************/
void Set_ZeroPos(MotorGroup *motors,uint8_t motor_num)
{
	// SpeedMode_Enable_Set(motors,1,23,0);
	osDelay(1000);

	MI_Motor *motor = NULL;
	
	if(motor_num==1) motor = &motors->Motor1;
	if(motor_num==2) motor = &motors->Motor2;
	if(motor_num==3) motor = &motors->Motor3;
	if(motor_num==4) motor = &motors->Motor4;
	if(motor_num==5) motor = &motors->Motor5;
	if(motor_num==6) motor = &motors->Motor6;

	uint32_t Send_ID;
	uint8_t Send_Data[8]={0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	Send_ID = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|motor->CAN_ID;
	FDCAN1_Send_Msg(Send_Data,FDCAN_DLC_BYTES_8,Send_ID);		
}

/*******************************************************************************
 * @function     : 设置电机CANID
 * @param        : 1.对应控制电机结构体 2.要修改的电机序号 3.目标电机将要修改成的CAN_ID
 * @return       : %3
 * @description  : %4
 *******************************************************************************/
void Set_CANID(MotorGroup *motors,uint8_t motor_num,uint8_t can_id)
{

	// SpeedMode_Enable_Set(motors,1,23,0);
	osDelay(1000);


	MI_Motor *motor = NULL;
	
	if(motor_num==1) motor = &motors->Motor1;
	if(motor_num==2) motor = &motors->Motor2;
	if(motor_num==3) motor = &motors->Motor3;
	if(motor_num==4) motor = &motors->Motor4;
	if(motor_num==5) motor = &motors->Motor5;
	if(motor_num==6) motor = &motors->Motor6;

	uint32_t Send_ID;
	uint8_t Send_Data[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	// Send_ID = Communication_Type_CanID<<24|can_id<<16|Master_CAN_ID<<8|motor->CAN_ID;
	// Send_ID = Communication_Type_CanID<<24|can_id<<16|Master_CAN_ID<<8|0x7f;
	FDCAN1_Send_Msg(Send_Data,FDCAN_DLC_BYTES_8,Send_ID);
	osDelay(10);
}

/*******************************************************************************
 * @function     : 电机速度模式测试
 * @param        : 1.最大速度 2.速度增加梯度
 * @return       : None
 * @description  : 速度梯度增加/减少
 *******************************************************************************/
void Speed_gradient_test(MotorGroup *motors,uint8_t motor_num,float max_speed,float gradient)
{
	
	if(max_speed==0 && gradient==0)
	{
		max_speed = 10;
		gradient = 0.001;
	}
	for (float speed_ref= 0.0f; speed_ref < max_speed; speed_ref+=gradient)
	{
		SpeedMode_Enable_Set(motors,motor_num,23,speed_ref);
		osDelay(1);
		
	}
	for (float speed_ref = max_speed; speed_ref > -max_speed; speed_ref-=gradient)
	{
		SpeedMode_Enable_Set(motors,motor_num,23,speed_ref);
		osDelay(1);
		
	}
	for (float speed_ref = -max_speed; speed_ref < 0; speed_ref+=gradient)
	{
		SpeedMode_Enable_Set(motors,motor_num,23,speed_ref);
		osDelay(1);
	}
}


/* Exported function declarations --------------------------------------------*/


