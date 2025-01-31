 /**
   ******************************************************************************
   * @file   
   * @brief  
   ******************************************************************************
   * @note

  */

 #ifndef __MI_MOTOR_H
 #define __MI_MOTOR_H



 /* Includes ------------------------------------------------------------------*/
 // #include "System_Config.h"
 #include "FDCAN.h"
 #include "cybergear.h"
 #include "can_bsp.h"
 #include <stdbool.h>

 /* Definitions ---------------------------------------------------------*/
 


	
	
 typedef struct{

	uint64_t MCU_64bit_Id;//64位MCU唯一标识符
 	uint8_t CAN_ID;	//目标电机CAN_ID
 	int Motor_Nbr;	//目标电机序号
	
 	float Angle;	//当前角度[0~65535]对应(-4π~4π) 
 	float Speed;	//当前角速度[0~65535]对应(30rad/s~30rad/s)  
 	float Torque;	//当前力矩 [0~65535]对应（-12Nm~12Nm）  
 	float Temp;		//当前温度：Temp(摄氏度）*10
	
 	uint16_t set_current;
 	uint16_t SpeedMode_Enable_Set;
 	uint16_t set_position;
	
 	uint8_t error_code;	//故障信息（0无,1有）
	bool error_is_uncalibrated;//未标定 
	bool error_is_hall_fault;//HALL编码故障 
	bool error_is_mag_fault;//磁编码故障
	bool error_is_overtemp;//过温
	bool error_is_overcurrent;//过流
	bool error_is_undervoltage;//欠压故障
	uint8_t mode_status;//电机模式状态 (0: Reset[复位], 1: Cali[标定], 2: Motor[运行])
	
	//可读写单个参数列表 (7019-701D为最新版本固件可读)
	uint8_t run_mode;//0: 运控模式 1: 位置模式  2: 速度模式  3: 电流模式 
	float iq_ref;//电流模式Iq指令 (-23~23A )
	float spd_ref;//转速模式转速指令 (-30~30rad/s )
	float imit_torque;//转矩限制(0~12Nm)
	float cur_kp;//电流的Kp (默认值0.125)
	float cui_ki;//电流的Ki (默认值0.0158)

	float spd_kp_medium;//速度的Kp (默认值2)
	float spd_ki_medium;//速度的Ki (默认值0.021)
	float spd_kp_low;//速度的Kp (默认值2)
	float spd_ki_low;//速度的Ki (默认值0.5)


	float cur_filt_gain;//电流滤波系数(0~1.0，默认值0.1)
	float loc_ref;//位置模式角度指令 (单位rad)
	float limit_spd;//位置模式速度限制 (0~30rad/s)
	float limit_cur;//速度位置模式电流限制(0~23A)
	float mechPos;//负载端计圈机械角度(单位rad)
	float iqf;//iq滤波值(-23~23A )
	float mechVel;//负载端转速(-30~30rad/s )
	float VBUS;//母线电压 (单位:V)
	int16_t rotation;//圈数




 	float Angle_Bias;

	// uint32_t Id_Frame;
	// uint8_t  Data_Frame[8];
	
	
 }MI_Motor;
 

extern MI_Motor Motor1;
extern MI_Motor Motor2;
extern MI_Motor Motor3;
extern MI_Motor Motor4;
extern MI_Motor Motor5;
extern MI_Motor Motor6;
 

typedef struct 
{
	MI_Motor Motor1;
	MI_Motor Motor2;
	MI_Motor Motor3;
	MI_Motor Motor4;
	MI_Motor Motor5;
	MI_Motor Motor6;

}MotorGroup;

extern MotorGroup Motors;

#define Master_CAN_ID 0x00

#define Communication_Type_GetID 0x00     //获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01 	//用来电机主机发送控制指令
#define Communication_Type_MotorRequest 0x02	//用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03	//电机使能运行
#define Communication_Type_MotorStop 0x04	//电机停止运行
#define Communication_Type_SetPosZero 0x06	//设置电机机械零位
#define Communication_Type_CanID 0x07	//更改当前电机CAN_ID

#define Communication_Type_GetSingleParameter 0x11	//读取单个参数

#define Communication_Type_Control_Mode 0x12
#define Communication_Type_SetSingleParameter 0x12	//设定单个参数

#define Communication_Type_ErrorFeedback 0x15	//故障反馈帧

#define Run_mode 0x7005	
#define Iq_Ref 0x7006
#define Spd_Ref 0x700A
#define Limit_Torque 0x700B
#define Cur_Kp 0x7010
#define Cur_Ki 0x7011

#define Spd_Kp 0x2014
#define Spd_Ki 0x2015

#define Cur_Filt_Gain 0x7014
#define Loc_Ref 0x7016
#define Limit_Spd 0x7017
#define Limit_Cur 0x7018

#define Gain_Angle 720/32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30/32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12/32767.0
#define Bias_Torque 0x8000
#define Temp_Gain 0.1

#define MAX_P 720
#define MIN_P -720
#define MAX_S 30
#define MIN_S -30
#define MAX_T 12
#define MIN_T -12

#define Motor_Error 0x00
#define Motor_OK 0X01

#define Current_mode 3
#define Position_mode 1
#define Motion_mode 0
#define Speed_mode 2
 /* Private variables ---------------------------------------------------------*/
 
 /* Global variables ---------------------------------------------------------*/

 /*Functions------------------------------------------------------------------*/
extern void Init_Motor(MotorGroup *motors,uint8_t Can_Id,int Motor_Num,float mode);
extern void Init_Motor_All(void);
extern void Enable_Motor(MotorGroup *motors,uint8_t motor_num);
extern void Stop_Motor(MotorGroup *motors,uint8_t motor_num,uint8_t clear_error);
extern void Motor_Data_Handler(MotorGroup *motors,uint32_t IDFrame,uint8_t DataFrame[8]);
// extern void Motor_Data_Get(MotorGroup *motors,FDCAN_RxHeaderTypeDef *header ,uint8_t *DataFrame);
extern uint8_t Get_Motor_ID(uint32_t CAN_ID_Frame);
extern uint8_t GetMotorFeedbackType(uint32_t can_id_frame);
extern uint64_t get_motor_MCU_64bit_Id(uint8_t *DataFrame);
extern float uint16_to_float(uint16_t x,float x_min,float x_max,int bits);
extern void Set_Mode(MotorGroup *motors,char motor_num,float Mode);
extern void CurrentMode_Enable_Set(MotorGroup *motors,uint8_t motor_num,float iq_ref);
extern void SpeedMode_Enable_Set(MotorGroup *motors,uint8_t motor_num,float limit_cur,float spd_ref);
extern void PositionMode_Enable_Set(MotorGroup *motors,uint8_t motor_num,float limit_speed,float pos_ref);
extern void Set_Motor_Parameter(MotorGroup *motors ,uint8_t motor_num ,uint16_t Index ,float Value ,char Value_type);
extern void Set_ZeroPos(MotorGroup *motors,uint8_t motor_num);
extern void Set_CANID(MotorGroup *motors,uint8_t motor_num,uint8_t can_id);
extern void Speed_gradient_test(MotorGroup *motors,uint8_t motor_num,float max_speed,float gradient);
extern void Motion_Mode_Enable_Set(MotorGroup *motors,uint8_t motor_num,float torque,float pos,float w,float kp,float kw);
 /* Exported function declarations --------------------------------------------*/

#endif
