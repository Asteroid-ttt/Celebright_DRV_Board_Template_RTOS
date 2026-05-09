#ifndef __IMU_H
#define __IMU_H

#include <math.h>
#define M_PI  (float)3.1415926535
typedef struct
{
    float x;
    float y;
    float z;
} xyz_f_t;
extern xyz_f_t north,west;
extern volatile float yaw[5];   //�����������ֵ
extern float motion6[7];
extern float imu_g_z;//z轴角速度
//Mini IMU AHRS �����API
void IMU_init(void);
void IMU_getYawPitchRoll(float * ypr); //������̬
void IMU_TT_getgyro(float * zsjganda);
//uint32_t micros(void);	//��ȡϵͳ�ϵ���ʱ��  ��λ us 
void MPU6050_InitAng_Offset(void);
#endif

//------------------End of File----------------------------
