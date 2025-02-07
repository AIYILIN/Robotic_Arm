
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
    { 0.0f,    0.0f,     M_PI_2,  0.0f    }, // L1
    { 16.982f, 202.596f, 0.0f,    M_PI_2  }, // L2
    { -33.7f,  211.366f, 0.0f,    0.0f    }, // L3
    { 27.7f,   215.389f, M_PI_2,  0.0f    }  // L4
};


extern void forward_kinematics(const float *joint_angles, float *end_effector_pose);// ���˶�ѧ���㺯��
extern int inverse_kinematics(float x, float y, float z, float theta[4]);// ���˶�ѧ��⺯��
extern void ik_iterative(float target[3], float joint_angles[4], int max_iter);

extern float proj_x ; // ����ĩ�˹�����������
extern float proj_y ;

#endif

