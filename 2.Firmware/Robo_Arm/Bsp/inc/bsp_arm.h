#ifndef __BSP_ARM_H
#define __BSP_ARM_H

#include "cybergear.h"
#include "motor_control.h"
#include "motor_comm_task.h"
#include "kinematic_task.h"
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

typedef enum {
    PHASE_ACCEL_UP,     // 0-�Ӽ���
    PHASE_ACCEL_STEADY, // 1-�ȼ���
    PHASE_ACCEL_DOWN,   // 2-������
    PHASE_CRUISE,       // 3-����
    PHASE_DECEL_UP,     // 4-�Ӽ���
    PHASE_DECEL_STEADY, // 5-�ȼ���
    PHASE_DECEL_DOWN,   // 6-������
    PHASE_FINISHED,     // 7-���
    PHASE_START         // 8-��ʼ
} Phase;


typedef struct 
{
    float pos_target;   // Ŀ��λ��
    float pos_current;  // ��ǰλ��
    float pos_target_last;     // ��һ��λ��

    float pos_start;    // ��ʼλ��
    float pos_end;      // ����λ��

    float vel_max;      // ����ٶ�
    float vel_cur;      // ��ǰ�ٶ�
    float vel_s_max;    // ����������ٶ�

    float acc_max;      // �����ٶ�

    float remaining;    // ʣ�����
    float direction;    // �˶�����
    float t_time;       // ����ʱ��
    float t_time_count; // ʱ�����

    Phase phase;        // �׶�
     
} T_Profile;




typedef struct {
    float current_pos;      // ��ǰλ��
    float current_vel;      // ��ǰ�ٶ�
    float current_acc;      // ��ǰ���ٶ�
    float target_pos;       // Ŀ��λ��
    float last_target_pos;  // ��һ��Ŀ��λ�ã����ڼ��仯��
    float distance;         // ����Ŀ��λ�õľ���
    uint8_t distance_updata_flag;    // ����Ŀ��λ��λ�ø��±�־λ

    float remaining;
    float direction;

    float max_vel;          // ����ٶȣ�rad/s��
    float max_acc;          // �����ٶȣ�rad/s?��
    float max_jerk;         // ���Ӽ��ٶȣ�rad/s?��
    float jerk;
    Phase phase;      // ��ǰ�׶�
} S_Profile;




extern HAL_StatusTypeDef Arm_joints_control(ARM_JOINTS arm_joint[6]);

extern void Arm_joint_init_para_set(ARM_JOINTS arm_joint[6]);
extern void Arm_joint_motion_para_set(ARM_JOINTS arm_joint[6]);

extern void joint_T_profilel_param_init(T_Profile *profile);
extern void Update_T_Profile(T_Profile *profile, float dt);


extern ARM_JOINT_ID arm_joint_id;
extern ARM_JOINTS arm_joint[6];
extern uint8_t joints_init_flag;
extern T_Profile joint_T_profilel[6];

#endif
