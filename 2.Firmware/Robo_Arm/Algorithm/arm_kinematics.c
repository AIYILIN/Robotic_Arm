#include "arm_kinematics.h"
#include "arm_math.h"
#include "math.h"
#include <stdint.h>
#include <string.h>
#include "usart.h"
#include "stdio.h"


// ����˷�������A * B��
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

// DH�任�������ɺ���
static Matrix4x4 dh_transform(float theta, const DH_Link *link) 
{
    Matrix4x4 T;
    float ct = cosf(theta + link->offset); // ����ؽ�ƫ��
    float st = sinf(theta + link->offset);
    float ca = cosf(link->alpha);
    float sa = sinf(link->alpha);
    
    // �任����
    T.m[0][0] = ct;        T.m[0][1] = -st*ca;     T.m[0][2] = st*sa;      T.m[0][3] = link->a * ct;
    T.m[1][0] = st;        T.m[1][1] = ct*ca;      T.m[1][2] = -ct*sa;     T.m[1][3] = link->a * st;
    T.m[2][0] = 0.0f;      T.m[2][1] = sa;         T.m[2][2] = ca;         T.m[2][3] = link->d;
    T.m[3][0] = 0.0f;      T.m[3][1] = 0.0f;       T.m[3][2] = 0.0f;       T.m[3][3] = 1.0f;
    
    return T;
}
// ZYXŷ���ǽ��㣨Yaw-Pitch-Roll��
static void rotation_to_zyx_euler(const Matrix4x4 *T, float* euler_deg) 
{
    /* 
    ���룺��α任����������
    �����ŷ��������[yaw, pitch, roll]����λ���ȣ�
    ��ʽ��R = Rz(yaw) * Ry(pitch) * Rx(roll)
    */
    
    // ��ȡ��ת����Ԫ��
    const float R11 = T->m[0][0], R12 = T->m[0][1], R13 = T->m[0][2];
    const float R21 = T->m[1][0], R22 = T->m[1][1], R23 = T->m[1][2];
    const float R31 = T->m[2][0], R32 = T->m[2][1], R33 = T->m[2][2];

    // ����Pitch���ȣ�����������
    const float pitch_rad = asinf(-R31); // pitch �� [-��/2, ��/2]
    const float cos_pitch = cosf(pitch_rad);
    const float threshold = 1e-6f;

    // �Ƕȼ����������ȣ�
    float yaw_rad, roll_rad;

    if (fabsf(cos_pitch) > threshold) {
        // ���������
        yaw_rad = atan2f(R21, R11); // yaw �� [-��, ��]
        roll_rad = atan2f(R32, R33); // roll �� [-��, ��]
    } else {
        // ���������cos_pitch �� 0����pitch �� ��90�㣩
        yaw_rad = atan2f(-R12, R22); 
        roll_rad = 0.0f; // �̶�RollΪ0
    }

    // ת��Ϊ�Ƕ�
    const float RAD2DEG = 180.0f / M_PI;
    euler_deg[0] = yaw_rad * RAD2DEG;   // Yaw
    euler_deg[1] = pitch_rad * RAD2DEG; // Pitch
    euler_deg[2] = roll_rad * RAD2DEG;  // Roll

    // �Ƕȹ淶����[-180��, 180��]
    for(int i=0; i<3; i++){
        euler_deg[i] = fmodf(euler_deg[i] + 180.0f, 360.0f) - 180.0f;
        if(fabsf(euler_deg[i]) < 0.001f) euler_deg[i] = 0.0f; // ����-0.0
    }
}

// ���˶�ѧ���㺯��������ؽڽǶ����飬���ĩ��xyz��
void forward_kinematics(const float *joint_angles, float *xyzypr) 
{
    Matrix4x4 T_total = {{
        {1,0,0,0},  // ��ʼ��Ϊ��λ����
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}
    }};
    
    char debug_buff[200];
    // ��ؽڼ���任����
    for (int i = 0; i < 6; ++i)
    {
        Matrix4x4 T = dh_transform(joint_angles[i]/180.0f*M_PI, &links[i]);

        // sprintf(debug_buff,"Joint %d Transform Matrix:\n %4.2f %4.2f %4.2f %4.2f\n %4.2f %4.2f %4.2f %4.2f\n %4.2f %4.2f %4.2f %4.2f\n %4.2f %4.2f %4.2f %4.2f\n\n ", i+1,T.m[0][0],T.m[0][1],T.m[0][2],T.m[0][3],T.m[1][0],T.m[1][1],T.m[1][2],T.m[1][3],T.m[2][0],T.m[2][1],T.m[2][2],T.m[2][3],T.m[3][0],T.m[3][1],T.m[3][2],T.m[3][3]);
        // HAL_UART_Transmit(&huart1, (uint8_t*)debug_buff, strlen(debug_buff), HAL_MAX_DELAY);

        Matrix4x4 temp;
        matrix_multiply(&temp, &T_total, &T);
        T_total = temp; // �����ۻ��任����
    }
    
    // ��ȡĩ��λ�ã����һ�е�ǰ����Ԫ�أ�
    xyzypr[0] = T_total.m[0][3];
    xyzypr[1] = T_total.m[1][3];
    xyzypr[2] = T_total.m[2][3];

    // ����ŷ���ǣ����ȣ�
    float euler_rad[3];
    rotation_to_zyx_euler(&T_total, euler_rad);

    // ת��Ϊ�ǶȲ����
    const float RAD2DEG = 180.0f / M_PI;
    xyzypr[3] = euler_rad[0]  ;  // Yaw
    xyzypr[4] = euler_rad[1] ;  // Pitch
    xyzypr[5] = euler_rad[2] ;  // Roll
}
