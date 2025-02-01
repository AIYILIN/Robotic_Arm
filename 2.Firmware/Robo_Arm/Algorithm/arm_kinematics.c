#include "arm_kinematics.h"
#include "math.h"
#include <stdint.h>

// ����˷�������A * B��
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

// DH�任�������ɺ���
static Matrix4x4 dh_transform(float theta, const DH_Link *link) {
    Matrix4x4 T;
    float ct = cosf(theta + link->offset); // ����ؽ�ƫ��
    float st = sinf(theta + link->offset);
    float ca = cosf(link->alpha);
    float sa = sinf(link->alpha);
    
    // ���任���������ȴ洢��
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
    
    // ��ؽڼ���任����
    for (int i = 0; i < 4; ++i) {
        Matrix4x4 T = dh_transform(joint_angles[i], &links[i]);
        Matrix4x4 temp;
        matrix_multiply(&temp, &T_total, &T);
        T_total = temp; // �����ۻ��任����
    }
    
    // ��ȡĩ��λ�ã����һ�е�ǰ����Ԫ�أ�
    xyz[0] = T_total.m[0][3];
    xyz[1] = T_total.m[1][3];
    xyz[2] = T_total.m[2][3];
}

/***************************** ʹ��ʾ�� *******************************
#include "stm32f4xx_hal.h"

int main(void) {
    // �ؽڽǶ����飨��λ�����ȣ�
    float angles[4] = {0.0f, M_PI_2, -M_PI_4, 0.0f}; 
    
    // �������洢
    float position[3];
    
    // �������˶�ѧ
    forward_kinematics(angles, position);
    
    // ��������ͨ��UART����Խӿڣ�
    printf("End Effector Position: [%.2f, %.2f, %.2f] mm\r\n", 
           position[0], position[1], position[2]);
    
    while(1);
}
**********************************************************************/

/***************************** �Ż����� *******************************
1. ʹ��CMSIS-DSP��������Ǻ������㣺
   #include "arm_math.h"
   float ct = arm_cos_f32(theta + link->offset);
   
2. ���ڹ̶�DH����������Ԥ�ȼ���sin/cosֵ�洢Ϊ����

3. ʹ�ö����������������ܣ�����ʧһ�����ȣ�

4. ����STM32Ӳ��FPU���ٸ������㣺
   - ��IDE�����ø��㵥Ԫ
   - ��ӱ���ѡ�-mfpu=fpv4-sp-d16 -mfloat-abi=hard
**********************************************************************/