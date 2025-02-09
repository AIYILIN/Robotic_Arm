
#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define M_PI_2 (M_PI/2.0f)


#define ARM_MATH_MATRIX_CHECK  // �ڰ���arm_math.hǰ����
#include "arm_math.h"


// ������α任����ṹ��
typedef struct 
{
    float m[4][4];
} Matrix4x4;

// ����DH�����ṹ��
typedef struct 
{
    float d;      // ����ƫ��
    float a;      // ���˳���
    float alpha;  // ����Ťת��
    float offset; // �ؽڳ�ʼƫ��
} DH_Link;

// ��ʼ��DH����
static const DH_Link links[] = 
{
    {   50.0f,    50.0f,   -M_PI_2,     0.0f }, // L1
    {    0.0f,   100.0f,      0.0f,  -M_PI_2 }, // L2
    {    0.0f,    50.0f,   -M_PI_2,     0.0f }, // L3
    {  100.0f,     0.0f,    M_PI_2,     0.0f },  // L4
    {    0.0f,     0.0f,   -M_PI_2,     0.0f }, // L5
    {  100.0f,     0.0f,      0.0f,     0.0f } // L6
};


extern void forward_kinematics(const float *joint_angles, float *xyz) ;// ���˶�ѧ��⺯��
// extern int inverse_kinematics(float x, float y, float z, float theta[6]);// ���˶�ѧ��⺯��
// extern void ik_iterative(float target[3], float joint_angles[6], int max_iter);

extern float proj_x ; // ����ĩ�˹�����������
extern float proj_y ;

#endif