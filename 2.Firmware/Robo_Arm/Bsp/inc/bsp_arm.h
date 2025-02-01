#ifndef __BSP_ARM_H
#define __BSP_ARM_H

#include "cybergear.h"
#include "motor_control.h"
#include "communication.h"
#include "can_bsp.h"

#define JOINT_1_INIT_POS 0
#define JOINT_2_INIT_POS 30
#define JOINT_3_INIT_POS 45
#define JOINT_4_INIT_POS 0

typedef enum
{
    ARM_JOINT_1 = 1,
    ARM_JOINT_2,
    ARM_JOINT_3,
    ARM_JOINT_4,
    ARM_JOINT_5,
    ARM_JOINT_6,
}ARM_JOINT_ID;

typedef struct
{
    ARM_JOINT_ID id;
    float torque;
    float pos;
    float w;
    float kp;
    float kw;

}ARM_JOINTS;


extern ARM_JOINT_ID arm_joint_id;
extern ARM_JOINTS arm_joint[6];
extern uint8_t joints_init_flag;

extern HAL_StatusTypeDef Arm_joint_control(ARM_JOINT_ID id, float torque,float pos , float w, float kp, float kw);
extern HAL_StatusTypeDef Arm_joints_control(ARM_JOINTS arm_joint[6]);
extern void Arm_joint_init_para_set(ARM_JOINTS arm_joint[6]);
extern void Arm_joint_motion_para_set(ARM_JOINTS arm_joint[6]);

#endif
