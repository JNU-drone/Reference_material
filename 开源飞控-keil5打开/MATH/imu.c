/******************************************************************************************************
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 *******************************************************************************************************/	
#include "imu.h"
#include "myMath.h"
#include <math.h>


static float NormAcc;

//这部分程序摘自网上，本人也不是很了解，所以需要使用者自己根据资料去学习。
//这部分很难，新手可以做到知道输入是六轴传感器数据，输出是三轴角度即可。

 
typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;
 Quaternion NumQ = {1, 0, 0, 0};
 
	 struct V{
				float x;
				float y;
				float z;
				};
	
volatile	 struct V GyroIntegError = {0}; 
	 
void imu_rest(void)
{
	NumQ.q0 =1;
	NumQ.q1 = 0;
	NumQ.q2 = 0;
	NumQ.q3 = 0;	
	GyroIntegError.x = 0;
	GyroIntegError.y = 0;
	GyroIntegError.z = 0;
	Angle.pitch = 0;
	Angle.roll = 0;
}


void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt) 
{		
volatile struct V Gravity,Acc,Gyro,AccGravity;


	static  float KpDef = 0.5f ;
	static  float KiDef = 0.0003f;
//		static  float KiDef = 0.00001f;
	
	float q0_t,q1_t,q2_t,q3_t;
  //float NormAcc;	
	float NormQuat; 
	float HalfTime = dt * 0.5f;

	

	// 提取等效旋转矩阵中的重力分量 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	// 加速度归一化
 NormAcc = Q_rsqrt(squa(MPU6050.accX)+ squa(MPU6050.accY) +squa(MPU6050.accZ));
	
    Acc.x = pMpu->accX * NormAcc;
    Acc.y = pMpu->accY * NormAcc;
    Acc.z = pMpu->accZ * NormAcc;	
 	//向量差乘得出的值
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	//再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
	//角速度融合加速度积分补偿值
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		
	// 一阶龙格库塔法, 更新四元数

	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// 四元数归一化
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	
	

		// 四元数转欧拉角
	{
		 
			#ifdef	YAW_GYRO
			*(
		float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
			#else
				float yaw_G = pMpu->gyroZ * Gyro_G;
				if((yaw_G > 1.0f) || (yaw_G < -1.0f)) //数据太小可以认为是干扰，不是偏航动作
				{
					pAngE->yaw  += yaw_G * dt;			
				}
			#endif
			pAngE->pitch  =  asin(2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;						
		
			pAngE->roll	= atan2(2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA;	//PITCH 			
	}
}


/***************************************************END OF FILE***************************************************/




