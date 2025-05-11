// File: dof6_c_interface.h
#ifdef __cplusplus
extern "C" {
#endif

typedef void* DOF6_Handle;

DOF6_Handle DOF6_Create(float L_BS, float D_BS, float L_AM,
                       float L_FA, float D_EW, float L_WT);
                       
void DOF6_Destroy(DOF6_Handle handle);

int DOF6_SolveFK(DOF6_Handle handle, 
                const float joints[6], 
                float pose[6]);

#ifdef __cplusplus
}
#endif