#ifndef __CTR_TASK_H__
#define __CTR_TASK_H__


#include "motor_pid.h"
#include "bsp_ps2.h"

typedef enum {
    PHASE_ACCEL_UP,     // 0-加加速
    PHASE_ACCEL_STEADY, // 1-匀加速
    PHASE_ACCEL_DOWN,   // 2-减加速
    PHASE_CRUISE,       // 3-匀速
    PHASE_DECEL_UP,     // 4-加减速
    PHASE_DECEL_STEADY, // 5-匀减速
    PHASE_DECEL_DOWN,   // 6-减减速
    PHASE_FINISHED,     // 7-完成
    PHASE_START         // 8-开始
} Phase;


typedef struct 
{
    float pos_target;   // 目标位置
    float pos_current;  // 当前位置
    float pos_target_last;     // 上一次位置

    float pos_start;    // 起始位置
    float pos_end;      // 结束位置

    float vel_max;      // 最大速度
    float vel_cur;      // 当前速度
    float vel_s_max;    // 三角形最大速度

    float acc_max;      // 最大加速度

    float remaining;    // 剩余距离
    float direction;    // 运动方向
    float t_time;       // 匀速时间
    float t_time_count; // 时间计数

    Phase phase;        // 阶段
     
} T_Profile;




typedef struct {
    float current_pos;      // 当前位置
    float current_vel;      // 当前速度
    float current_acc;      // 当前加速度
    float target_pos;       // 目标位置
    float last_target_pos;  // 上一次目标位置（用于检测变化）
    float distance;         // 距离目标位置的距离
    uint8_t distance_updata_flag;    // 距离目标位置位置更新标志位

    float remaining;
    float direction;

    float max_vel;          // 最大速度（rad/s）
    float max_acc;          // 最大加速度（rad/s?）
    float max_jerk;         // 最大加加速度（rad/s?）
    float jerk;
    Phase phase;      // 当前阶段
} S_Profile;




extern void joint_T_profilel_param_init(T_Profile *profile);
extern T_Profile joint_T_profilel[6];



#endif
