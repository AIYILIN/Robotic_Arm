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

// 正运动学计算函数（输入关节角度数组，输出末端xyz）
void forward_kinematics(const float *joint_angles, float *xyz) 
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
    xyz[0] = T_total.m[0][3];
    xyz[1] = T_total.m[1][3];
    xyz[2] = T_total.m[2][3];
}





// 逆运动学求解函数
// 输入：目标位置(x, y, z)（单位：毫米）
// 输出：关节角度theta[4]（单位：弧度，已包含offset）
// 返回值：0-成功，-1-无解
float proj_x = 0; // 根据末端工具坐标修正
float proj_y = 0;
int inverse_kinematics(float x, float y, float z, float theta[6]) 
{
    //--- 步骤1：计算关节1（theta1）---
    // 投影到基座XY平面
    proj_x = x - links[3].a * cosf(theta[3]); // 根据末端工具坐标修正
    proj_y = y - links[3].a * sinf(theta[3]);
    theta[0] = atan2f(proj_y, proj_x) - links[0].offset;

    //--- 步骤2：计算关节2和3（theta2, theta3）---
    // 转换为L2坐标系下的坐标
    float x2 = proj_x * cosf(theta[0]) + proj_y * sinf(theta[0]) - links[1].a;
    float z2 = z - links[0].d - links[1].d;
    
    // 解三角形（L2和L3构成平面二连杆）
    float l2 = links[2].a;
    float l3 = links[3].a;
    float D = (x2*x2 + z2*z2 - l2*l2 - l3*l3) / (2*l2*l3);
    
    // 检查解的存在性
    if (fabsf(D) > 1.0f) return -1; // 无解
    
    theta[2] = atan2f(sqrtf(1 - D*D), D); // 解1（肘部向上）
    // theta[2] = atan2f(-sqrtf(1 - D*D), D); // 解2（肘部向下）
    
    theta[1] = atan2f(z2, x2) - atan2f(l3*sinf(theta[2]), l2 + l3*cosf(theta[2]));
    theta[1] -= links[1].offset;

    //--- 步骤3：计算关节4（theta4）---
    // 根据姿态需求调整（此处假设末端垂直）
    // theta[3] = - (theta[1] + theta[2]) - links[3].offset;
    theta[3] = 0;

    for (int i = 0; i < 4; i++)
    {
        theta[i] = theta[i] * 180.0f / M_PI;
    }
    

    return 0;
}

