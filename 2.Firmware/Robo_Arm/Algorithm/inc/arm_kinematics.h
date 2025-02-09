
#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define M_PI_2 (M_PI/2.0f)


#define ARM_MATH_MATRIX_CHECK  // 在包含arm_math.h前定义
#include "arm_math.h"


// 定义齐次变换矩阵结构体
typedef struct 
{
    float m[4][4];
} Matrix4x4;

// 定义DH参数结构体
typedef struct 
{
    float d;      // 连杆偏距
    float a;      // 连杆长度
    float alpha;  // 连杆扭转角
    float offset; // 关节初始偏移
} DH_Link;

// 初始化DH参数
static const DH_Link links[] = 
{
    {   50.0f,    50.0f,   -M_PI_2,     0.0f }, // L1
    {    0.0f,   100.0f,      0.0f,  -M_PI_2 }, // L2
    {    0.0f,    50.0f,   -M_PI_2,     0.0f }, // L3
    {  100.0f,     0.0f,    M_PI_2,     0.0f },  // L4
    {    0.0f,     0.0f,   -M_PI_2,     0.0f }, // L5
    {  100.0f,     0.0f,      0.0f,     0.0f } // L6
};


extern void forward_kinematics(const float *joint_angles, float *xyz) ;// 正运动学求解函数
// extern int inverse_kinematics(float x, float y, float z, float theta[6]);// 逆运动学求解函数
// extern void ik_iterative(float target[3], float joint_angles[6], int max_iter);

extern float proj_x ; // 根据末端工具坐标修正
extern float proj_y ;

#endif