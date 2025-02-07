#include "arm_kinematics.h"
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
    for (int i = 0; i < 4; ++i)
    {
        Matrix4x4 T = dh_transform(joint_angles[i], &links[i]);

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
int inverse_kinematics(float x, float y, float z, float theta[4]) 
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

#include <math.h>
#include "arm_math.h"

// 角度约束宏
#define ANGLE_MOD(angle) fmodf((angle), 2*M_PI)
// 角度约束函数（保持角度在[-π, π]）
static void constrain_angles(float *angles) {
    for (int i = 0; i < 4; ++i) {
        while (angles[i] > M_PI) angles[i] -= 2*M_PI;
        while (angles[i] < -M_PI) angles[i] += 2*M_PI;
    }
}

void ik_iterative(float target[3], float joint_angles[4], int max_iter) {
    const float initial_alpha = 0.1f;     // 增大初始学习率
    const float alpha_decay = 0.98f;      // 调整衰减率
    const float tolerance = 1e-4f;        // 收敛阈值
    const float lambda = 0.01f;           // 减小阻尼系数
    
    float alpha = initial_alpha;
    float last_error = INFINITY;

    // 矩阵初始化（修正维度）
    arm_matrix_instance_f32 J_mat, J_T_mat, error_mat, delta_theta_mat;
    float J[12] = {0};          // 正确维度：3x4矩阵（行优先存储）
    float J_T[12] = {0};        // 转置后维度：4x3
    float error[3] = {0};
    float delta_theta[4] = {0};

    arm_mat_init_f32(&J_mat, 3, 4, J);       // 修正矩阵维度声明
    arm_mat_init_f32(&J_T_mat, 4, 3, J_T);
    arm_mat_init_f32(&error_mat, 3, 1, error);
    arm_mat_init_f32(&delta_theta_mat, 4, 1, delta_theta);

    for (int iter = 0; iter < max_iter; iter++) {
        float current[3];
        forward_kinematics(joint_angles, current);

        // 计算误差向量
        error[0] = target[0] - current[0];
        error[1] = target[1] - current[1];
        error[2] = target[2] - current[2];
        
        // 计算误差范数
        float error_sq = error[0]*error[0] + error[1]*error[1] + error[2]*error[2];
        float error_norm;
        arm_sqrt_f32(error_sq, &error_norm);

        if (error_norm < tolerance) {
            char debug_buff[100];
            sprintf(debug_buff,"Converged at iter %d: error=%.4f\n", iter, error_norm);
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_buff, strlen(debug_buff), HAL_MAX_DELAY);
            break;
        }

        // 动态学习率调整
        if (error_norm > last_error) {
            alpha *= 0.5f;  // 误差增大时快速降低学习率
        }
        last_error = error_norm;
        alpha *= alpha_decay;

        // 数值法计算雅可比矩阵（修正索引顺序）
        const float delta = 1e-3f;
        for (int i = 0; i < 4; i++) {
            float angles_plus[4], pos_plus[3];
            arm_copy_f32(joint_angles, angles_plus, 4);
            angles_plus[i] += delta;
            
            forward_kinematics(angles_plus, pos_plus);
            
            // 行优先填充3x4矩阵
            J[i*3 + 0] = (pos_plus[0] - current[0]) / delta;  // 第0行，第i列
            J[i*3 + 1] = (pos_plus[1] - current[1]) / delta;  // 第1行，第i列
            J[i*3 + 2] = (pos_plus[2] - current[2]) / delta;  // 第2行，第i列
        }

        // 转置雅可比矩阵（3x4 -> 4x3）
        arm_mat_trans_f32(&J_mat, &J_T_mat);

        // 计算阻尼最小二乘解：delta_theta = (J^T*J + λI)^-1 * J^T * error
        // 步骤1：计算J^T*J + λI
        float JTJ[16] = {0}; // 4x4矩阵
        arm_mat_mult_f32(&J_T_mat, &J_mat, &(arm_matrix_instance_f32){4,4,JTJ});
        
        // 添加阻尼项到对角线
        JTJ[0]  += lambda;  // [0][0]
        JTJ[5]  += lambda;  // [1][1]
        JTJ[10] += lambda;  // [2][2]
        JTJ[15] += lambda;  // [3][3]

        // 步骤2：求逆矩阵
        arm_matrix_instance_f32 JTJ_inv_mat;
        float JTJ_inv[16];
        arm_mat_init_f32(&JTJ_inv_mat, 4, 4, JTJ_inv);
        arm_status inv_status = arm_mat_inverse_f32(&(arm_matrix_instance_f32){4,4,JTJ}, &JTJ_inv_mat);
        
        // 处理奇异矩阵情况
        if (inv_status != ARM_MATH_SUCCESS) {
            // 对角线添加微小扰动后重新求逆
            JTJ[0] += 1e-4f; JTJ[5] += 1e-4f; 
            JTJ[10] += 1e-4f; JTJ[15] += 1e-4f;
            arm_mat_inverse_f32(&(arm_matrix_instance_f32){4,4,JTJ}, &JTJ_inv_mat);
        }

        // 步骤3：计算 delta_theta = JTJ_inv * J^T * error
        float temp[3] = {0};
        arm_matrix_instance_f32 temp_mat;
        arm_mat_init_f32(&temp_mat, 4, 3, temp);
        
        arm_mat_mult_f32(&JTJ_inv_mat, &J_T_mat, &temp_mat);
        arm_mat_mult_f32(&temp_mat, &error_mat, &delta_theta_mat);

        // 更新关节角度（应用约束）
        for (int i = 0; i < 4; i++) {
            joint_angles[i] += alpha * delta_theta[i];
        }
        constrain_angles(joint_angles);

        // 调试输出（每10次迭代）
        if (iter % 10 == 0) {
            char debug_buff[200];
            sprintf(debug_buff,"Iter %3d: error=%.4f, angles=[%7.2f, %7.2f, %7.2f, %7.2f] deg\n",
                  iter, error_norm,
                  joint_angles[0]*180/M_PI, 
                  joint_angles[1]*180/M_PI,
                  joint_angles[2]*180/M_PI,
                  joint_angles[3]*180/M_PI);
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_buff, strlen(debug_buff), HAL_MAX_DELAY);
            
            // 首次迭代打印雅可比矩阵
            if (iter == 0) {
                sprintf(debug_buff, "Jacobian:\n");
                for(int row=0; row<3; row++){
                    sprintf(debug_buff+strlen(debug_buff), "[");
                    for(int col=0; col<4; col++){
                        sprintf(debug_buff+strlen(debug_buff), "%7.2f ", J[col*3 + row]);
                    }
                    sprintf(debug_buff+strlen(debug_buff), "]\n");
                }
                HAL_UART_Transmit(&huart1, (uint8_t*)debug_buff, strlen(debug_buff), HAL_MAX_DELAY);
            }
        }
    }
}
