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

// ���˶�ѧ���㺯��������ؽڽǶ����飬���ĩ��xyz��
void forward_kinematics(const float *joint_angles, float *xyz) 
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
    xyz[0] = T_total.m[0][3];
    xyz[1] = T_total.m[1][3];
    xyz[2] = T_total.m[2][3];
}





// ���˶�ѧ��⺯��
// ���룺Ŀ��λ��(x, y, z)����λ�����ף�
// ������ؽڽǶ�theta[4]����λ�����ȣ��Ѱ���offset��
// ����ֵ��0-�ɹ���-1-�޽�
float proj_x = 0; // ����ĩ�˹�����������
float proj_y = 0;
int inverse_kinematics(float x, float y, float z, float theta[6]) 
{
    //--- ����1������ؽ�1��theta1��---
    // ͶӰ������XYƽ��
    proj_x = x - links[3].a * cosf(theta[3]); // ����ĩ�˹�����������
    proj_y = y - links[3].a * sinf(theta[3]);
    theta[0] = atan2f(proj_y, proj_x) - links[0].offset;

    //--- ����2������ؽ�2��3��theta2, theta3��---
    // ת��ΪL2����ϵ�µ�����
    float x2 = proj_x * cosf(theta[0]) + proj_y * sinf(theta[0]) - links[1].a;
    float z2 = z - links[0].d - links[1].d;
    
    // �������Σ�L2��L3����ƽ������ˣ�
    float l2 = links[2].a;
    float l3 = links[3].a;
    float D = (x2*x2 + z2*z2 - l2*l2 - l3*l3) / (2*l2*l3);
    
    // ����Ĵ�����
    if (fabsf(D) > 1.0f) return -1; // �޽�
    
    theta[2] = atan2f(sqrtf(1 - D*D), D); // ��1���ⲿ���ϣ�
    // theta[2] = atan2f(-sqrtf(1 - D*D), D); // ��2���ⲿ���£�
    
    theta[1] = atan2f(z2, x2) - atan2f(l3*sinf(theta[2]), l2 + l3*cosf(theta[2]));
    theta[1] -= links[1].offset;

    //--- ����3������ؽ�4��theta4��---
    // ������̬����������˴�����ĩ�˴�ֱ��
    // theta[3] = - (theta[1] + theta[2]) - links[3].offset;
    theta[3] = 0;

    for (int i = 0; i < 4; i++)
    {
        theta[i] = theta[i] * 180.0f / M_PI;
    }
    

    return 0;
}

