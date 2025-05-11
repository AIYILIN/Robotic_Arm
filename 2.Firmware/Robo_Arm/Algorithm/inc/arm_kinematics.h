
#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define M_PI_2 (M_PI/2.0f)


#define ARM_MATH_MATRIX_CHECK  // 在包含arm_math.h前定义
#include "arm_math.h"


// 定义齐次变换矩阵结构体

typedef struct {
    float m[4][4];
} Matrix4x4;


// 修改后的3x3矩阵结构体
typedef struct {
    float m3x3[3][3];
} Matrix3x3;


// 定义DH参数结构体
typedef struct 
{
    float d;      // 连杆偏距
    float a;      // 连杆长度
    float alpha;  // 连杆扭转角
    float offset; // 关节初始偏移
} DH_Link;


static const DH_Link links[] = 
{
    {    0.0f,     0.0f,        0.0f,     0.0f   }, // L1
    {    0.0f,     0.0f,      M_PI_2,     M_PI_2 }, // L2
    {    0.0f,  199.54f,        0.0f,     0.0f   }, // L3
    { 171.53f,   95.91f,      M_PI_2,     0.0f   },  // L4
    {    0.0f,     0.0f,     -M_PI_2,     0.0f   }, // L5
    {    0.0f,     0.0f,      M_PI_2,     0.0f   } // L6
};

extern void forward_kinematics(const float *joint_angles, float *xyzypr);




#define MAX_IK_SOLUTIONS 8

typedef struct {
    float angles[6];     // 关节角度（弧度）
    uint8_t valid;       // 解是否有效
} IKSolution;


// 新的3x3矩阵转置函数
static void matrix_transpose_3x3(Matrix3x3* result, const Matrix3x3* src) {
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            result->m3x3[i][j] = src->m3x3[j][i];
        }
    }
}

extern IKSolution ik_solutions[MAX_IK_SOLUTIONS];
// 新增逆运动学函数声明
uint8_t inverse_kinematics(const Matrix4x4* T_target, const DH_Link* links, IKSolution* solutions);

extern void print_matrix_3x3(const char* name, const Matrix3x3* mat);

#endif


