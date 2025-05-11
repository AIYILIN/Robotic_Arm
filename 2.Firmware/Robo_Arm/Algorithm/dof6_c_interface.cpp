// // File: dof6_c_interface.cpp
// #include "6dof_kinematic.h"
// #include <string.h>
// #include "usart.h"
// #include "stdio.h"

// // 定义C接口结构体（不透明指针）
// typedef void* DOF6_Handle;

// // 创建对象实例
// extern "C" DOF6_Handle DOF6_Create(float L_BS, float D_BS, float L_AM, 
//                                   float L_FA, float D_EW, float L_WT) {
//     return new DOF6Kinematic(L_BS, D_BS, L_AM, L_FA, D_EW, L_WT);
// }

// // 销毁对象实例
// extern "C" void DOF6_Destroy(DOF6_Handle handle) {
//     delete static_cast<DOF6Kinematic*>(handle);
// }

// // 正运动学解算（C风格接口）
// extern "C" int DOF6_SolveFK(DOF6_Handle handle, 
//                            const float joints[6], 
//                            float pose[6]) {
//     DOF6Kinematic::Joint6D_t input;
//     // 确保输入关节角被正确赋值
//     for(int i=0; i<6; i++) 
//         input.a[i] = joints[i]; // 单位为度（假设SolveFK内部会转为弧度）
    

//     // char dbg[100];
//     // sprintf(dbg, "Input joints: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
//     // HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 1000);


//         DOF6Kinematic::Pose6D_t output;
//         bool ret = static_cast<DOF6Kinematic*>(handle)->SolveFK(input, output);
    
//     // 输出单位检查（确认是否需要转为毫米）
//     pose[0] = output.X * 1000.0f;  // 如果SolveFK输出单位为米，转为毫米
//     pose[1] = output.Y * 1000.0f;
//     pose[2] = output.Z * 1000.0f;
//     pose[3] = output.A;  // 角度单位保持度
//     pose[4] = output.B;
//     pose[5] = output.C;
    
//     return ret ? 0 : -1; // 返回0表示成功
// }



