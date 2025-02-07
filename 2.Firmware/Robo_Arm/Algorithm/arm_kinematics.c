#include "arm_kinematics.h"
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
    for (int i = 0; i < 4; ++i)
    {
        Matrix4x4 T = dh_transform(joint_angles[i], &links[i]);

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
int inverse_kinematics(float x, float y, float z, float theta[4]) 
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

#include <math.h>
#include "arm_math.h"

// �Ƕ�Լ����
#define ANGLE_MOD(angle) fmodf((angle), 2*M_PI)
// �Ƕ�Լ�����������ֽǶ���[-��, ��]��
static void constrain_angles(float *angles) {
    for (int i = 0; i < 4; ++i) {
        while (angles[i] > M_PI) angles[i] -= 2*M_PI;
        while (angles[i] < -M_PI) angles[i] += 2*M_PI;
    }
}

void ik_iterative(float target[3], float joint_angles[4], int max_iter) {
    const float initial_alpha = 0.1f;     // �����ʼѧϰ��
    const float alpha_decay = 0.98f;      // ����˥����
    const float tolerance = 1e-4f;        // ������ֵ
    const float lambda = 0.01f;           // ��С����ϵ��
    
    float alpha = initial_alpha;
    float last_error = INFINITY;

    // �����ʼ��������ά�ȣ�
    arm_matrix_instance_f32 J_mat, J_T_mat, error_mat, delta_theta_mat;
    float J[12] = {0};          // ��ȷά�ȣ�3x4���������ȴ洢��
    float J_T[12] = {0};        // ת�ú�ά�ȣ�4x3
    float error[3] = {0};
    float delta_theta[4] = {0};

    arm_mat_init_f32(&J_mat, 3, 4, J);       // ��������ά������
    arm_mat_init_f32(&J_T_mat, 4, 3, J_T);
    arm_mat_init_f32(&error_mat, 3, 1, error);
    arm_mat_init_f32(&delta_theta_mat, 4, 1, delta_theta);

    for (int iter = 0; iter < max_iter; iter++) {
        float current[3];
        forward_kinematics(joint_angles, current);

        // �����������
        error[0] = target[0] - current[0];
        error[1] = target[1] - current[1];
        error[2] = target[2] - current[2];
        
        // ��������
        float error_sq = error[0]*error[0] + error[1]*error[1] + error[2]*error[2];
        float error_norm;
        arm_sqrt_f32(error_sq, &error_norm);

        if (error_norm < tolerance) {
            char debug_buff[100];
            sprintf(debug_buff,"Converged at iter %d: error=%.4f\n", iter, error_norm);
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_buff, strlen(debug_buff), HAL_MAX_DELAY);
            break;
        }

        // ��̬ѧϰ�ʵ���
        if (error_norm > last_error) {
            alpha *= 0.5f;  // �������ʱ���ٽ���ѧϰ��
        }
        last_error = error_norm;
        alpha *= alpha_decay;

        // ��ֵ�������ſɱȾ�����������˳��
        const float delta = 1e-3f;
        for (int i = 0; i < 4; i++) {
            float angles_plus[4], pos_plus[3];
            arm_copy_f32(joint_angles, angles_plus, 4);
            angles_plus[i] += delta;
            
            forward_kinematics(angles_plus, pos_plus);
            
            // ���������3x4����
            J[i*3 + 0] = (pos_plus[0] - current[0]) / delta;  // ��0�У���i��
            J[i*3 + 1] = (pos_plus[1] - current[1]) / delta;  // ��1�У���i��
            J[i*3 + 2] = (pos_plus[2] - current[2]) / delta;  // ��2�У���i��
        }

        // ת���ſɱȾ���3x4 -> 4x3��
        arm_mat_trans_f32(&J_mat, &J_T_mat);

        // ����������С���˽⣺delta_theta = (J^T*J + ��I)^-1 * J^T * error
        // ����1������J^T*J + ��I
        float JTJ[16] = {0}; // 4x4����
        arm_mat_mult_f32(&J_T_mat, &J_mat, &(arm_matrix_instance_f32){4,4,JTJ});
        
        // ���������Խ���
        JTJ[0]  += lambda;  // [0][0]
        JTJ[5]  += lambda;  // [1][1]
        JTJ[10] += lambda;  // [2][2]
        JTJ[15] += lambda;  // [3][3]

        // ����2���������
        arm_matrix_instance_f32 JTJ_inv_mat;
        float JTJ_inv[16];
        arm_mat_init_f32(&JTJ_inv_mat, 4, 4, JTJ_inv);
        arm_status inv_status = arm_mat_inverse_f32(&(arm_matrix_instance_f32){4,4,JTJ}, &JTJ_inv_mat);
        
        // ��������������
        if (inv_status != ARM_MATH_SUCCESS) {
            // �Խ������΢С�Ŷ�����������
            JTJ[0] += 1e-4f; JTJ[5] += 1e-4f; 
            JTJ[10] += 1e-4f; JTJ[15] += 1e-4f;
            arm_mat_inverse_f32(&(arm_matrix_instance_f32){4,4,JTJ}, &JTJ_inv_mat);
        }

        // ����3������ delta_theta = JTJ_inv * J^T * error
        float temp[3] = {0};
        arm_matrix_instance_f32 temp_mat;
        arm_mat_init_f32(&temp_mat, 4, 3, temp);
        
        arm_mat_mult_f32(&JTJ_inv_mat, &J_T_mat, &temp_mat);
        arm_mat_mult_f32(&temp_mat, &error_mat, &delta_theta_mat);

        // ���¹ؽڽǶȣ�Ӧ��Լ����
        for (int i = 0; i < 4; i++) {
            joint_angles[i] += alpha * delta_theta[i];
        }
        constrain_angles(joint_angles);

        // ���������ÿ10�ε�����
        if (iter % 10 == 0) {
            char debug_buff[200];
            sprintf(debug_buff,"Iter %3d: error=%.4f, angles=[%7.2f, %7.2f, %7.2f, %7.2f] deg\n",
                  iter, error_norm,
                  joint_angles[0]*180/M_PI, 
                  joint_angles[1]*180/M_PI,
                  joint_angles[2]*180/M_PI,
                  joint_angles[3]*180/M_PI);
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_buff, strlen(debug_buff), HAL_MAX_DELAY);
            
            // �״ε�����ӡ�ſɱȾ���
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
