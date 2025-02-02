
#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define M_PI_2 (M_PI/2.0f)




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

// ���˶�ѧ���㺯��
void forward_kinematics(const float *joint_angles, float *end_effector_pose);

#endif

