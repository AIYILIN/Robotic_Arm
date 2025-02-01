#include "arm_kinematics.h"
#include "math.h"
#include <stdint.h>

// 矩阵乘法函数（A * B）
static void matrix_multiply(Matrix4x4 *result, const Matrix4x4 *a, const Matrix4x4 *b) 
{
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result->m[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k) {
                result->m[i][j] += a->m[i][k] * b->m[k][j];
            }
        }
    }
}

// DH变换矩阵生成函数
static Matrix4x4 dh_transform(float theta, const DH_Link *link) {
    Matrix4x4 T;
    float ct = cosf(theta + link->offset); // 加入关节偏移
    float st = sinf(theta + link->offset);
    float ca = cosf(link->alpha);
    float sa = sinf(link->alpha);
    
    // 填充变换矩阵（行优先存储）
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
    
    // 逐关节计算变换矩阵
    for (int i = 0; i < 4; ++i) {
        Matrix4x4 T = dh_transform(joint_angles[i], &links[i]);
        Matrix4x4 temp;
        matrix_multiply(&temp, &T_total, &T);
        T_total = temp; // 更新累积变换矩阵
    }
    
    // 提取末端位置（最后一列的前三个元素）
    xyz[0] = T_total.m[0][3];
    xyz[1] = T_total.m[1][3];
    xyz[2] = T_total.m[2][3];
}

/***************************** 使用示例 *******************************
#include "stm32f4xx_hal.h"

int main(void) {
    // 关节角度数组（单位：弧度）
    float angles[4] = {0.0f, M_PI_2, -M_PI_4, 0.0f}; 
    
    // 计算结果存储
    float position[3];
    
    // 计算正运动学
    forward_kinematics(angles, position);
    
    // 输出结果（通过UART或调试接口）
    printf("End Effector Position: [%.2f, %.2f, %.2f] mm\r\n", 
           position[0], position[1], position[2]);
    
    while(1);
}
**********************************************************************/

/***************************** 优化建议 *******************************
1. 使用CMSIS-DSP库加速三角函数计算：
   #include "arm_math.h"
   float ct = arm_cos_f32(theta + link->offset);
   
2. 对于固定DH参数，可以预先计算sin/cos值存储为常量

3. 使用定点数运算提升性能（需损失一定精度）

4. 启用STM32硬件FPU加速浮点运算：
   - 在IDE中启用浮点单元
   - 添加编译选项：-mfpu=fpv4-sp-d16 -mfloat-abi=hard
**********************************************************************/