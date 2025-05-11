
#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define M_PI_2 (M_PI/2.0f)

#define ARM_MATH_MATRIX_CHECK  // 在包含arm_math.h前定义
#include "arm_math.h"

#define MAX_IK_SOLUTIONS 8

// 定义齐次变换矩阵结构体4x4
typedef struct 
{
    float m[4][4];
} Matrix4x4;

// 3x3矩阵结构体
typedef struct 
{
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

// DH参数结构体
static const DH_Link links[] = 
{
    {    0.0f,     0.0f,        0.0f,     0.0f   }, // L1
    {    0.0f,     0.0f,      M_PI_2,     M_PI_2 }, // L2
    {    0.0f,  199.54f,        0.0f,     0.0f   }, // L3
    { 171.53f,   95.91f,      M_PI_2,     0.0f   },  // L4
    {    0.0f,     0.0f,     -M_PI_2,     0.0f   }, // L5
    {    0.0f,     0.0f,      M_PI_2,     0.0f   } // L6
};


typedef struct 
{
    float angles[6];     // 关节角度（弧度）
    uint8_t valid;       // 解是否有效
} IKSolution;

// arm_kinematics.h
typedef struct {
    float x;
    float y;
    float z;
} Vector3;

// arm_kinematics.h
// 新增工具参数结构体
typedef struct {
    Vector3 offset;     // 工具坐标系下腕部中心的偏移量
    float mass;         // 工具质量（可选）
} ToolParams;

// 定义XYZ欧拉角+位置结构体
typedef struct {
    float x, y, z;       // 位置
    float yaw, pitch, roll; // X-Y-Z欧拉角（单位：度）
} XYZ_EulerAngles;

extern ToolParams default_tool;
extern IKSolution ik_solutions[MAX_IK_SOLUTIONS];
extern Matrix4x4 target_pose;
extern XYZ_EulerAngles XYZ_euler_angles;

extern Vector3 wrist_center;
extern Matrix3x3 R_tool;

//运动学函数声明
extern void print_matrix_3x3(const char* name, const Matrix3x3* mat);
extern void matrix_transpose_3x3(Matrix3x3* result, const Matrix3x3* src);
extern void forward_kinematics(const float *joint_angles, float *xyzypr);
extern uint8_t inverse_kinematics(const Matrix4x4 *T_target, const ToolParams *tool, const DH_Link *links, IKSolution *solutions);
extern void calculate_wrist_center(const Vector3 *end_effector_pos, const Matrix3x3 *R_end_effector, const ToolParams *tool, Vector3 *wrist_center) ;
extern Matrix4x4* pose_and_xyzEulerAngles_to_matrix(XYZ_EulerAngles *euler, Matrix4x4 *T);

#endif


