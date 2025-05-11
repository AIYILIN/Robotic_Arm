#include "arm_kinematics.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
#include <string.h>
#include "usart.h"
#include "stdio.h"
#include "cmsis_os.h"



// 逆运动8组解声明
IKSolution ik_solutions[MAX_IK_SOLUTIONS];
//x,y,z,yaw, pitch, roll
// XYZ_EulerAngles XYZ_euler_angles={-203.2699 ,-247.9015 , 271.3051, 101.8249 ,  11.6415 ,  37.6341};//20,40,60,80,100,120
XYZ_EulerAngles XYZ_euler_angles={-22.608, -88.241, 527.093, 29.5, 2.8488  ,-81.7780};//10,20,30,40,50,60


ToolParams default_tool = {
  .offset = {0.0f, 0.0f, 168.51f},  // 工具坐标系下腕部中心的偏移量(mm)
  .mass = 210.0f                 // 工具质量（g）
};

//10 20 30 40 50 60
// Matrix4x4 target_pose = {
//       .m = {{ 0.1428  ,  0.9885   , 0.0497   , -30.9827},
//             { -0.8582  ,  0.1487  , -0.4912   , -5.4631},
//             { -0.4930  ,  0.0275   ,0.8696    ,380.5556 + 500},
//             {       0   ,      0   ,      0    ,1.0000}}
//   };


  //10 20 30 40 50 60 加d
// Matrix4x4 target_pose = {
//       .m = {{ 0.1428  ,  0.9885   , 0.0497   , -6.1327},
//             { -0.8582  ,  0.1487  , -0.4912   , -251.0814},
//             { -0.4930  ,  0.0275   ,0.8696    , 815.3592},
//             {       0   ,      0   ,      0    ,1.0000}}
//   };

// 0 0 0 0 0 0
Matrix4x4 target_pose = {
      .m = {{ 0.0000   ,-0.0000   , 1.0000 , 0},
            { 0.0000   ,-1.0000   ,-0.0000  , 0.0000},
            { 1.0000   , 0.0000   ,-0.0000 , 0},
            {      0        , 0      ,   0   ,1.0000}}
  };


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

// 3x3矩阵乘法函数
static void matrix_multiply_3x3(Matrix3x3 *result, const Matrix3x3 *a, const Matrix3x3 *b)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result->m3x3[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                result->m3x3[i][j] += a->m3x3[i][k] * b->m3x3[k][j];
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

    // 改进型型变换矩阵
    T.m[0][0] = ct;         T.m[0][1] = -st;        T.m[0][2] = 0;      T.m[0][3] = link->a;
    T.m[1][0] = st * ca;    T.m[1][1] = ct * ca;    T.m[1][2] = -sa;    T.m[1][3] = -link->d * sa;
    T.m[2][0] = st * sa;    T.m[2][1] = ct * sa;    T.m[2][2] = ca;     T.m[2][3] = link->d * ca;
    T.m[3][0] = 0.0f;       T.m[3][1] = 0.0f;       T.m[3][2] = 0.0f;   T.m[3][3] = 1.0f;

    // // 标准型变换矩阵
    // T.m[0][0] = ct;     T.m[0][1] = -st * ca;       T.m[0][2] = st * sa;    T.m[0][3] = link->a * ct;
    // T.m[1][0] = st;     T.m[1][1] = ct * ca;        T.m[1][2] = -ct * sa;   T.m[1][3] = link->a * st;
    // T.m[2][0] = 0.0f;   T.m[2][1] = sa;             T.m[2][2] = ca;         T.m[2][3] = link->d;
    // T.m[3][0] = 0.0f;   T.m[3][1] = 0.0f;           T.m[3][2] = 0.0f;       T.m[3][3] = 1.0f;

    return T;
}


/*
输入：齐次变换矩阵（行主序）
输出：欧拉角数组[roll, pitch, yaw]（单位：度）
公式：R = Rx(roll) * Ry(pitch) * Rz(yaw)
*/
static void rotation_to_xyz_euler(const Matrix4x4 *T, float *euler_deg)
{
    // 提取旋转矩阵元素
    const float R11 = T->m[0][0], R12 = T->m[0][1], R13 = T->m[0][2];
    const float R21 = T->m[1][0], R22 = T->m[1][1], R23 = T->m[1][2];
    const float R31 = T->m[2][0], R32 = T->m[2][1], R33 = T->m[2][2];

    // 计算Pitch（θ）并检测奇异点
    const float pitch_rad = asinf(R13); // pitch ∈ [-π/2, π/2]
    const float cos_pitch = cosf(pitch_rad);
    const float threshold = 1e-6f;

    // 角度计算结果（弧度）
    float roll_rad, yaw_rad;

    if (fabsf(cos_pitch) > threshold)
    {
        // 非奇异情况
        yaw_rad = atan2f(-R12, R11);  // yaw ∈ [-π, π]
        roll_rad = atan2f(-R23, R33); // roll ∈ [-π, π]
    }
    else
    {
        // 奇异情况（cos_pitch ≈ 0，即pitch ≈ ±90°）
        yaw_rad = atan2f(R21, R22); // 计算yaw与roll的和
        roll_rad = 0.0f;            // 固定Roll为0
    }

    // 转换为角度
    const float RAD2DEG = 180.0f / M_PI;
    euler_deg[0] = roll_rad * RAD2DEG;  // Roll (X轴)
    euler_deg[1] = pitch_rad * RAD2DEG; // Pitch (Y轴)
    euler_deg[2] = yaw_rad * RAD2DEG;   // Yaw (Z轴)

    // 角度规范化到[-180°, 180°]
    for (int i = 0; i < 3; i++)
    {
        euler_deg[i] = fmodf(euler_deg[i] + 180.0f, 360.0f) - 180.0f;
        if (fabsf(euler_deg[i]) < 0.001f)
            euler_deg[i] = 0.0f; // 消除-0.0
    }
}

/*
输入：齐次变换矩阵（行主序）
输出：欧拉角数组[yaw, pitch, roll]（单位：度）
公式：R = Rz(yaw) * Ry(pitch) * Rx(roll)
*/
static void rotation_to_zyx_euler(const Matrix4x4 *T, float *euler_deg)
{


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

    if (fabsf(cos_pitch) > threshold)
    {
        // 非奇异情况
        yaw_rad = atan2f(R21, R11);  // yaw ∈ [-π, π]
        roll_rad = atan2f(R32, R33); // roll ∈ [-π, π]
    }
    else
    {
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
    for (int i = 0; i < 3; i++)
    {
        euler_deg[i] = fmodf(euler_deg[i] + 180.0f, 360.0f) - 180.0f;
        if (fabsf(euler_deg[i]) < 0.001f)
            euler_deg[i] = 0.0f; // 消除-0.0
    }
}



// 从4x4齐次矩阵中提取3x3旋转部分
static void extract_rotation_matrix(Matrix3x3 *dst, const Matrix4x4 *src)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            dst->m3x3[i][j] = src->m[i][j];
        }
    }
}



// 正运动学计算函数（输入关节角度数组，输出末端xyz）
void forward_kinematics(const float *joint_angles, float *xyzypr)
{
    Matrix4x4 T_total = {{{1, 0, 0, 0}, // 初始化为单位矩阵
                          {0, 1, 0, 0},
                          {0, 0, 1, 0},
                          {0, 0, 0, 1}}};

    char debug_buff[200];
    // 逐关节计算变换矩阵
    for (int i = 0; i < 6; ++i)
    {
        Matrix4x4 T = dh_transform(joint_angles[i] / 180.0f * M_PI, &links[i]);

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
    // rotation_to_zyx_euler(&T_total, euler_rad);
    rotation_to_xyz_euler(&T_total, euler_rad);

    // 转换为角度并输出
    const float RAD2DEG = 180.0f / M_PI;
    xyzypr[3] = euler_rad[0]; // Yaw
    xyzypr[4] = euler_rad[1]; // Pitch
    xyzypr[5] = euler_rad[2]; // Roll
}




// 逆运动私有函数声明
static void calculate_theta3(const float p[3], const DH_Link *links, float theta3[2]);
static void calculate_theta2(float g3, const float f[3], float alpha2, float theta2[2]);
static void equation_cal(float m, float n, float e, float solutions[2][2]);
static float calculate_theta1(const float p[3], const float f[3], float theta2, float alpha2);
static void calculate_theta456(float theta1, float theta2, float theta3, const Matrix3x3 *R06, const DH_Link *links, float theta456[2][3], uint8_t *num_sols);
static Matrix3x3 dh_rotation_matrix_3x3(float theta, float alpha);


Vector3 wrist_center;
Matrix3x3 R_tool;

// 逆运动学主函数
uint8_t inverse_kinematics(const Matrix4x4 *T_target, const ToolParams *tool, const DH_Link *links, IKSolution *solutions)
{


    // Step 1: 计算腕部中心位置（考虑工具偏移）
    Vector3 tool_center = {T_target->m[0][3], T_target->m[1][3], T_target->m[2][3]};

    
    extract_rotation_matrix(&R_tool, T_target);
    calculate_wrist_center(&tool_center, &R_tool, tool, &wrist_center);

    float p[3] = {wrist_center.x, wrist_center.y, wrist_center.z};


    // float p[3] = {T_target->m[0][3], T_target->m[1][3], T_target->m[2][3]};
    float theta3[2];
    calculate_theta3(p, links, theta3);

    // 计算前三个关节的可能解
    float T1[4][3];
    float f1[3], f2[3];

    // 计算f1和theta2_1
    f1[0] = links[3].a * cosf(theta3[0]) + links[3].d * sinf(links[3].alpha) * sinf(theta3[0]) + links[2].a;
    f1[1] = links[3].a * cosf(links[2].alpha) * sinf(theta3[0]) - links[3].d * sinf(links[3].alpha) * cosf(links[2].alpha) * cosf(theta3[0]) - links[3].d * sinf(links[2].alpha) * cosf(links[3].alpha) - links[2].d * sinf(links[2].alpha);

    float theta2_1[2];
    calculate_theta2(p[2], f1, links[1].alpha, theta2_1);

    // 计算f2和theta2_2
    f2[0] = links[3].a * cosf(theta3[1]) + links[3].d * sinf(links[3].alpha) * sinf(theta3[1]) + links[2].a;
    f2[1] = links[3].a * cosf(links[2].alpha) * sinf(theta3[1]) - links[3].d * sinf(links[3].alpha) * cosf(links[2].alpha) * cosf(theta3[1]) - links[3].d * sinf(links[2].alpha) * cosf(links[3].alpha) - links[2].d * sinf(links[2].alpha);

    float theta2_2[2];
    calculate_theta2(p[2], f2, links[1].alpha, theta2_2);

    // 计算f1[3]和f2[3]
    f1[2] = links[3].a * sinf(links[2].alpha) * sinf(theta3[0]) - links[3].d * sinf(links[3].alpha) * sinf(links[2].alpha) * cosf(theta3[0]) + links[3].d * cosf(links[2].alpha) * cosf(links[3].alpha) + links[2].d * cosf(links[2].alpha);

    f2[2] = links[3].a * sinf(links[2].alpha) * sinf(theta3[1]) - links[3].d * sinf(links[3].alpha) * sinf(links[2].alpha) * cosf(theta3[1]) + links[3].d * cosf(links[2].alpha) * cosf(links[3].alpha) + links[2].d * cosf(links[2].alpha);

    // 计算关节1角度
    T1[0][0] = calculate_theta1(p, f1, theta2_1[0], links[1].alpha);
    T1[0][1] = theta2_1[0];
    T1[0][2] = theta3[0];

    T1[1][0] = calculate_theta1(p, f1, theta2_1[1], links[1].alpha);
    T1[1][1] = theta2_1[1];
    T1[1][2] = theta3[0];

    T1[2][0] = calculate_theta1(p, f2, theta2_2[0], links[1].alpha);
    T1[2][1] = theta2_2[0];
    T1[2][2] = theta3[1];

    T1[3][0] = calculate_theta1(p, f2, theta2_2[1], links[1].alpha);
    T1[3][1] = theta2_2[1];
    T1[3][2] = theta3[1];

    // 提取目标旋转矩阵
    Matrix3x3 R06;
    extract_rotation_matrix(&R06, T_target);

    // 计算所有可能的解
    uint8_t solution_count = 0;
    for (int i = 0; i < 4; i++)
    {
        float theta456[2][3];
        uint8_t num_sols;

        char buf[20];
        sprintf(buf, "\r\n Matrix%d:\r\n", i + 1);
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);
        osDelay(100);

        calculate_theta456(T1[i][0], T1[i][1], T1[i][2], &R06, links, theta456, &num_sols);

        for (int j = 0; j < num_sols; j++)
        {
            // 应用关节偏移
            for (int k = 0; k < 6; k++)
            {
                float offset = (k == 1) ? (M_PI_2) : 0.0f; // 关节2偏移90度
                solutions[solution_count].angles[k] =
                    (k < 3) ? (T1[i][k] - offset) : (theta456[j][k - 3] - offset);

                // 角度归一化
                solutions[solution_count].angles[k] =
                    fmodf(solutions[solution_count].angles[k] + M_PI, 2 * M_PI) - M_PI;

                solutions[solution_count].angles[k] = solutions[solution_count].angles[k] * 180.0f / M_PI;
            }
            solutions[solution_count].valid = 1;
            solution_count++;
            if (solution_count >= MAX_IK_SOLUTIONS)
                break;
        }
        if (solution_count >= MAX_IK_SOLUTIONS)
            break;
    }

    return solution_count;
}

// 计算theta3（两个解）
static void calculate_theta3(const float p[3], const DH_Link *links, float theta3[2])
{
    const float r = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
    const float m = 2 * links[2].a * links[3].d * sinf(links[3].alpha);
    const float n = 2 * links[2].a * links[3].a;
    const float e = r - (powf(links[3].a, 2) + powf(links[3].d, 2) + powf(links[2].d, 2) + powf(links[2].a, 2) + 2 * links[3].d * links[2].d * cosf(links[3].alpha));

    float solutions[2][2];
    equation_cal(m, n, e, solutions);

    theta3[0] = atan2f(solutions[0][0], solutions[0][1]);
    theta3[1] = atan2f(solutions[1][0], solutions[1][1]);
}

// 计算theta2（两个解）
static void calculate_theta2(float g3, const float f[3], float alpha2, float theta2[2])
{
    const float e = g3;
    const float m = f[0] * sinf(alpha2);
    const float n = f[1] * sinf(alpha2);

    float solutions[2][2];
    equation_cal(m, n, e, solutions);

    theta2[0] = atan2f(solutions[0][0], solutions[0][1]);
    theta2[1] = atan2f(solutions[1][0], solutions[1][1]);
}

// 通用三角方程求解器
static void equation_cal(float m, float n, float e, float solutions[2][2])
{
    const float denominator = m * m + n * n;
    const float sqrt_term = sqrtf(m * m + n * n - e * e);

    // 第一组解
    solutions[0][0] = (e - (n * (e * n + m * sqrt_term)) / denominator) / m; // sinθ
    solutions[0][1] = (e * n + m * sqrt_term) / denominator;                 // cosθ

    // 第二组解
    solutions[1][0] = (e - (n * (e * n - m * sqrt_term)) / denominator) / m; // sinθ
    solutions[1][1] = (e * n - m * sqrt_term) / denominator;                 // cosθ
}

// 计算theta1
static float calculate_theta1(const float p[3], const float f[3], float theta2, float alpha2)
{
    const float g1 = cosf(theta2) * f[0] - sinf(theta2) * f[1];
    const float g2 = -sinf(alpha2) * f[2];
    const float denominator = g1 * g1 + g2 * g2;

    const float sin_theta1 = (g1 * p[1] - g2 * p[0]) / denominator;
    const float cos_theta1 = (g1 * p[0] + g2 * p[1]) / denominator;
    return atan2f(sin_theta1, cos_theta1);
}



static void calculate_theta456(float theta1, float theta2, float theta3,
                               const Matrix3x3 *R06, const DH_Link *links,
                               float theta456[2][3], uint8_t *num_sols)
{
    // 1. 计算各关节的3x3旋转矩阵
    Matrix3x3 R01 = dh_rotation_matrix_3x3(theta1, links[0].alpha);
    Matrix3x3 R12 = dh_rotation_matrix_3x3(theta2, links[1].alpha);
    Matrix3x3 R23 = dh_rotation_matrix_3x3(theta3, links[2].alpha);
    Matrix3x3 R34 = dh_rotation_matrix_3x3(0.0f, links[3].alpha); // theta4=0

        
    // 2. 按顺序计算矩阵链：R34' * R23' * R12' * R01' * R06
    Matrix3x3 temp, temp2;

    // 第一步：R01' * R06
    Matrix3x3 R01_T;
    matrix_transpose_3x3(&R01_T, &R01);
    matrix_multiply_3x3(&temp, &R01_T, R06); // temp = R01' * R06

    // 第二步：R12' * (R01' * R06)
    Matrix3x3 R12_T;
    matrix_transpose_3x3(&R12_T, &R12);
    matrix_multiply_3x3(&temp2, &R12_T, &temp); // temp2 = R12' * temp

    // 第三步：R23' * (R12' * R01' * R06)
    Matrix3x3 R23_T;
    matrix_transpose_3x3(&R23_T, &R23);
    matrix_multiply_3x3(&temp, &R23_T, &temp2); // temp = R23' * temp2

    // 第四步：R34' * (R23' * R12' * R01' * R06)
    Matrix3x3 R34_T;
    matrix_transpose_3x3(&R34_T, &R34);
    Matrix3x3 R46;
    matrix_multiply_3x3(&R46, &R34_T, &temp); // R46 = R34' * temp

        // print_matrix_3x3("R01", &R01);
        // print_matrix_3x3("R12", &R12);
        // print_matrix_3x3("R23", &R23);
        // print_matrix_3x3("R34", &R34);
        // print_matrix_3x3("R46", &R46);



    // 4. 严格对应Matlab的奇异值判断逻辑
    const float R33 = R46.m3x3[2][2]; // 对应Matlab的R46(3,3)

    if (fabsf(R33 - 1.0f) < 1e-6f)
    { // 奇异情况1
        theta456[0][0] = 0.0f;
        theta456[0][1] = 0.0f;
        theta456[0][2] = atan2f(-R46.m3x3[0][1], R46.m3x3[0][0]);
        *num_sols = 1;
    }
    else if (fabsf(R33 + 1.0f) < 1e-6f)
    { // 奇异情况2
        theta456[0][0] = 0.0f;
        theta456[0][1] = M_PI;
        theta456[0][2] = atan2f(R46.m3x3[0][1], -R46.m3x3[0][0]);
        *num_sols = 1;
    }
    else
    { // 常规情况
        // 第一组解
        const float theta5 = atan2f(sqrtf(R46.m3x3[2][0] * R46.m3x3[2][0] + R46.m3x3[2][1] * R46.m3x3[2][1]), R46.m3x3[2][2]);
        const float sin_theta5 = sinf(theta5);

        theta456[0][0] = atan2f(R46.m3x3[1][2] / sin_theta5, R46.m3x3[0][2] / sin_theta5);
        theta456[0][1] = theta5;
        theta456[0][2] = atan2f(R46.m3x3[2][1] / sin_theta5, -R46.m3x3[2][0] / sin_theta5);

        // 第二组解（theta5取负）
        const float theta5_neg = -theta5;
        const float sin_theta5_neg = sinf(theta5_neg);

        theta456[1][0] = atan2f(R46.m3x3[1][2] / sin_theta5_neg, R46.m3x3[0][2] / sin_theta5_neg);
        theta456[1][1] = theta5_neg;
        theta456[1][2] = atan2f(R46.m3x3[2][1] / sin_theta5_neg, -R46.m3x3[2][0] / sin_theta5_neg);

        *num_sols = 2;
    }

    // 5. 角度规范化处理（与Matlab一致）
    for (int i = 0; i < *num_sols; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            // 映射到[-π, π]范围
            theta456[i][j] = fmodf(theta456[i][j] + M_PI, 2 * M_PI) - M_PI;
            // 消除-0.0
            if (fabsf(theta456[i][j]) < 1e-6f)
            {
                theta456[i][j] = 0.0f;
            }
        }
    }
}

static Matrix3x3 dh_rotation_matrix_3x3(float theta, float alpha)
{
    Matrix3x3 R;
    float ct = cosf(theta);
    float st = sinf(theta);
    float ca = cosf(alpha);
    float sa = sinf(alpha);

    R.m3x3[0][0] = ct;          R.m3x3[0][1] = -st;         R.m3x3[0][2] = 0.0f;
    R.m3x3[1][0] = st * ca;     R.m3x3[1][1] = ct * ca;     R.m3x3[1][2] = -sa;
    R.m3x3[2][0] = st * sa;     R.m3x3[2][1] = ct * sa;     R.m3x3[2][2] = ca;

    return R;
}



// 打印3x3矩阵的函数实现
void print_matrix_3x3(const char* name, const Matrix3x3* mat) 
{
    char buf[64];
    snprintf(buf, sizeof(buf), "\r\n%s:\r\n", name);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    
    for(int i=0; i<3; i++) {
        snprintf(buf, sizeof(buf), "%+8.4f %+8.4f %+8.4f\r\n",
                mat->m3x3[i][0], mat->m3x3[i][1], mat->m3x3[i][2]);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    }
}


// 新的3x3矩阵转置函数
void matrix_transpose_3x3(Matrix3x3* result, const Matrix3x3* src) 
{
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            result->m3x3[i][j] = src->m3x3[j][i];
        }
    }
}


/**
 * @brief 计算腕部中心点位置（考虑工具偏移）
 * @param end_effector_pos 末端执行器的目标位置（基坐标系）
 * @param R_end_effector 末端执行器的旋转矩阵（基坐标系）
 * @param tool 工具参数（包含工具坐标系下的偏移量）
 * @param wrist_center 输出的腕部中心位置（基坐标系）
 */
void calculate_wrist_center(const Vector3 *end_effector_pos, const Matrix3x3 *R_end_effector, const ToolParams *tool, Vector3 *wrist_center) 
{
    // 将工具偏移量从工具坐标系转换到基坐标系
    Vector3 tool_offset_base = 
    {
        R_end_effector->m3x3[0][0] * tool->offset.x + R_end_effector->m3x3[0][1] * tool->offset.y + R_end_effector->m3x3[0][2] * tool->offset.z,
        R_end_effector->m3x3[1][0] * tool->offset.x + R_end_effector->m3x3[1][1] * tool->offset.y + R_end_effector->m3x3[1][2] * tool->offset.z,
        R_end_effector->m3x3[2][0] * tool->offset.x + R_end_effector->m3x3[2][1] * tool->offset.y + R_end_effector->m3x3[2][2] * tool->offset.z
    };

    // 计算腕部中心位置
    wrist_center->x = end_effector_pos->x - tool_offset_base.x;
    wrist_center->y = end_effector_pos->y - tool_offset_base.y;
    wrist_center->z = end_effector_pos->z - tool_offset_base.z;
}


Matrix4x4* pose_and_xyzEulerAngles_to_matrix(XYZ_EulerAngles *euler, Matrix4x4 *T)//注意：这里的欧拉角转换公式有问题！！！，后续换一步到位的矩阵！！！
{
    // Step 1: 初始化矩阵为单位矩阵
    memset(T, 0, sizeof(Matrix4x4));
    T->m[3][3] = 1.0f;

    // Step 2: 角度转弧度
    const float roll_rad  = euler->yaw * M_PI / 180.0f;             //注意：这里的欧拉角转换公式有问题！！！，后续换一步到位的矩阵！！！
    const float pitch_rad = euler->pitch * M_PI / 180.0f;           //注意：这里的欧拉角转换公式有问题！！！，后续换一步到位的矩阵！！！
    const float yaw_rad   = euler->roll   * M_PI / 180.0f;          //注意：这里的欧拉角转换公式有问题！！！，后续换一步到位的矩阵！！！

    // Step 3: 计算各轴旋转矩阵
    // X轴旋转矩阵（Roll）
    const float cr = cosf(roll_rad), sr = sinf(roll_rad);
    const float Rx[3][3] = {
        {1,  0,    0   },
        {0,  cr, -sr  },
        {0,  sr,  cr  }
    };

    // Y轴旋转矩阵（Pitch）
    const float cp = cosf(pitch_rad), sp = sinf(pitch_rad);
    const float Ry[3][3] = {
        { cp,  0,  sp },
        { 0,   1,  0 },
        {-sp,  0,  cp }
    };

    // Z轴旋转矩阵（Yaw）
    const float cy = cosf(yaw_rad), sy = sinf(yaw_rad);
    const float Rz[3][3] = {
        {cy, -sy, 0 },
        {sy,  cy, 0 },
        {0,   0,  1 }
    };

    // Step 4: 合并旋转矩阵 R = Rx * Ry * Rz
    float temp[3][3], R[3][3];
    
    // 先计算Ry * Rz
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            temp[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k) {
                temp[i][j] += Ry[i][k] * Rz[k][j];
            }
        }
    }
    
    // 再计算Rx * (Ry * Rz)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k) {
                R[i][j] += Rx[i][k] * temp[k][j];
            }
        }
    }

    // Step 5: 填充矩阵
    // 旋转部分
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            T->m[i][j] = R[i][j];
        }
    }
    
    // 平移部分
    T->m[0][3] = euler->x;
    T->m[1][3] = euler->y;
    T->m[2][3] = euler->z;

    return T;
}