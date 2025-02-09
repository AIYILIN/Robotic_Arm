#include "arm_kinematics.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
#include <string.h>
#include "usart.h"
#include "stdio.h"


// 矩阵乘法函数（A * B）
static void matrix_multiply(Matrix4x4 *result, const Matrix4x4 *a, const Matrix4x4 *b) 
{
    for (int i = 0; i < 4; ++i) 
    {
        for (int j = 0; j < 4; ++j) 
        {
            result->m[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k) 
            {
                result->m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }
}

// DH变换矩阵生成函数
static Matrix4x4 dh_transform(float theta, const DH_Link *link) 
{
    Matrix4x4 T;
    float ct = cosf(theta + link->offset); // 加入关节偏移
    float st = sinf(theta + link->offset);
    float ca = cosf(link->alpha);
    float sa = sinf(link->alpha);
    
    // 变换矩阵
    T.m[0][0] = ct;        T.m[0][1] = -st*ca;     T.m[0][2] = st*sa;      T.m[0][3] = link->a * ct;
    T.m[1][0] = st;        T.m[1][1] = ct*ca;      T.m[1][2] = -ct*sa;     T.m[1][3] = link->a * st;
    T.m[2][0] = 0.0f;      T.m[2][1] = sa;         T.m[2][2] = ca;         T.m[2][3] = link->d;
    T.m[3][0] = 0.0f;      T.m[3][1] = 0.0f;       T.m[3][2] = 0.0f;       T.m[3][3] = 1.0f;
    
    return T;
}
// ZYX欧拉角解算（Yaw-Pitch-Roll）
static void rotation_to_zyx_euler(const Matrix4x4 *T, float* euler_deg) 
{
    /* 
    输入：齐次变换矩阵（行主序）
    输出：欧拉角数组[yaw, pitch, roll]（单位：度）
    公式：R = Rz(yaw) * Ry(pitch) * Rx(roll)
    */
    
    // 提取旋转矩阵元素
    const float R11 = T->m[0][0], R12 = T->m[0][1], R13 = T->m[0][2];
    const float R21 = T->m[1][0], R22 = T->m[1][1], R23 = T->m[1][2];
    const float R31 = T->m[2][0], R32 = T->m[2][1], R33 = T->m[2][2];

    // 计算Pitch（θ）并检测奇异点
    const float pitch_rad = asinf(-R31); // pitch ∈ [-π/2, π/2]
    const float cos_pitch = cosf(pitch_rad);
    const float threshold = 1e-6f;

    // 角度计算结果（弧度）
    float yaw_rad, roll_rad;

    if (fabsf(cos_pitch) > threshold) {
        // 非奇异情况
        yaw_rad = atan2f(R21, R11); // yaw ∈ [-π, π]
        roll_rad = atan2f(R32, R33); // roll ∈ [-π, π]
    } else {
        // 奇异情况（cos_pitch ≈ 0，即pitch ≈ ±90°）
        yaw_rad = atan2f(-R12, R22); 
        roll_rad = 0.0f; // 固定Roll为0
    }

    // 转换为角度
    const float RAD2DEG = 180.0f / M_PI;
    euler_deg[0] = yaw_rad * RAD2DEG;   // Yaw
    euler_deg[1] = pitch_rad * RAD2DEG; // Pitch
    euler_deg[2] = roll_rad * RAD2DEG;  // Roll

    // 角度规范化到[-180°, 180°]
    for(int i=0; i<3; i++){
        euler_deg[i] = fmodf(euler_deg[i] + 180.0f, 360.0f) - 180.0f;
        if(fabsf(euler_deg[i]) < 0.001f) euler_deg[i] = 0.0f; // 消除-0.0
    }
}

// 正运动学计算函数（输入关节角度数组，输出末端xyz）
void forward_kinematics(const float *joint_angles, float *xyzypr) 
{
    Matrix4x4 T_total = {{
        {1,0,0,0},  // 初始化为单位矩阵
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}
    }};
    
    char debug_buff[200];
    // 逐关节计算变换矩阵
    for (int i = 0; i < 6; ++i)
    {
        Matrix4x4 T = dh_transform(joint_angles[i]/180.0f*M_PI, &links[i]);

        // sprintf(debug_buff,"Joint %d Transform Matrix:\n %4.2f %4.2f %4.2f %4.2f\n %4.2f %4.2f %4.2f %4.2f\n %4.2f %4.2f %4.2f %4.2f\n %4.2f %4.2f %4.2f %4.2f\n\n ", i+1,T.m[0][0],T.m[0][1],T.m[0][2],T.m[0][3],T.m[1][0],T.m[1][1],T.m[1][2],T.m[1][3],T.m[2][0],T.m[2][1],T.m[2][2],T.m[2][3],T.m[3][0],T.m[3][1],T.m[3][2],T.m[3][3]);
        // HAL_UART_Transmit(&huart1, (uint8_t*)debug_buff, strlen(debug_buff), HAL_MAX_DELAY);

        Matrix4x4 temp;
        matrix_multiply(&temp, &T_total, &T);
        T_total = temp; // 更新累积变换矩阵
    }
    
    // 提取末端位置（最后一列的前三个元素）
    xyzypr[0] = T_total.m[0][3];
    xyzypr[1] = T_total.m[1][3];
    xyzypr[2] = T_total.m[2][3];

    // 计算欧拉角（弧度）
    float euler_rad[3];
    rotation_to_zyx_euler(&T_total, euler_rad);

    // 转换为角度并输出
    const float RAD2DEG = 180.0f / M_PI;
    xyzypr[3] = euler_rad[0]  ;  // Yaw
    xyzypr[4] = euler_rad[1] ;  // Pitch
    xyzypr[5] = euler_rad[2] ;  // Roll
}
