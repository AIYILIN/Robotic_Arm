#ifndef __CTR_TASK_H__
#define __CTR_TASK_H__


#include "motor_pid.h"
#include "bsp_ps2.h"

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




extern void joint_T_profilel_param_init(T_Profile *profile);
extern T_Profile joint_T_profilel[6];



#endif
