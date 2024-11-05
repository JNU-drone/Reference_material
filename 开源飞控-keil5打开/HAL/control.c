/*******************************************************************
 *MPU6050

 ******************************************************************/
#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1 
#undef EMERGENT
#define EMERGENT 0
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //结构体数组，将每一个数组放一个pid结构体，这样就可以批量操作各个PID的数据了  比如解锁时批量复位pid控制数据，新手明白这句话的作用就可以了
};
/**************************************************************
 *  flight control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void FlightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //等待解锁
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //准备进入控制
			pidRest(pPidObject,6); //批量复位PID数据，防止上次遗留的数据影响本次控制

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //锁定偏航角
		
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //正式进入控制
			if(Angle.pitch<-50||Angle.pitch>50||Angle.roll<-50||Angle.roll>50)//倾斜检测，大角度判定为意外情况，则紧急上锁	
					if(Remote.thr>1200)//当油门的很低时不做倾斜检测
						ALL_flag.unlock = EMERGENT;//打入紧急情况
			
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //内环测量值 角度/秒
			pidRateY.measured = MPU6050.gyroY * Gyro_G;
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
		
			pidPitch.measured = Angle.pitch; //外环测量值 单位：角度
		  pidRoll.measured = Angle.roll;
			pidYaw.measured = Angle.yaw;
		
		 	pidUpdate(&pidRoll,dt);    //调用PID处理函数来处理外环	横滚角PID		
			pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
			pidUpdate(&pidRateX,dt);  //再调用内环

		 	pidUpdate(&pidPitch,dt);    //调用PID处理函数来处理外环	俯仰角PID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //再调用内环

			CascadePID(&pidRateZ,&pidYaw,dt);	//也可以直接调用串级PID函数来处理
			break;
		case EXIT_255:  //退出控制
			pidRest(pPidObject,6);
			status = WAITING_1;//返回等待解锁
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
}


#define MOTOR1 motor_PWM_Value[0] 
#define MOTOR2 motor_PWM_Value[1] 
#define MOTOR3 motor_PWM_Value[2] 
#define MOTOR4 motor_PWM_Value[3] 
uint16_t low_thr_cnt_quiet;
uint16_t low_thr_cnt;
void MotorControl(void)
{	
	volatile static uint8_t status=WAITING_1;
	
	
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: //等待解锁	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2: //解锁完成后判断使用者是否开始拨动遥杆进行飞行控制
			if(Remote.thr>1100)
			{
				low_thr_cnt_quiet=0;
				low_thr_cnt=0;
				pidRest(pPidObject,6);
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			{
				int16_t temp,thr;
				temp = Remote.thr -1000; //油门+定高输出值
				//油门比例规划
				thr = 100+0.55f * temp;
			
				if(temp<10) //自动关停判断
				{
					if(low_thr_cnt<1500)
							low_thr_cnt++;
					thr = thr-(low_thr_cnt*0.6);//油门摇杆值慢慢降为0 
					if(MPU6050.accZ<8500&&MPU6050.accZ>7800)
					{
						low_thr_cnt++;
						if(low_thr_cnt>600)//1800ms
						{
							thr = 0;
							
							pidRest(pPidObject,6);
							MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 =0;
							status = WAITING_2;
							
							break;
						}
					}
				}
				else low_thr_cnt=0;
				MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr,0,700); //留100给姿态控制
					
//以下输出的脉冲分配取决于电机PWM分布与飞控坐标体系。请看飞控坐标体系图解，与四个电机PWM分布分布	
//           机头      
//   PWM3     ♂       PWM1
//      *           *
//      	*       *
//    		  *   *
//      			*  
//    		  *   *
//      	*       *
//      *           *
//    PWM4           PWM2			
//		pidRateX.out 横滚角串级PID输出 控制左右，可以看出1 2和3 4，左右两组电机同增同减
//    pidRateY.out 俯仰角串级PID输出 控制前后，可以看出2 3和1 4，前后两组电机同增同减
//		pidRateZ.out 横滚角串级PID输出 控制旋转，可以看出2 4和1 3，两组对角线电机同增同减	

// 正负号取决于算法输出 比如输出是正的话  往前飞必然是尾巴两个电机增加,往右飞必然是左边两个电机增加		

				MOTOR1 +=    + pidRateX.out + pidRateY.out + pidRateZ.out;//; 姿态输出分配给各个电机的控制量
				MOTOR2 +=    + pidRateX.out - pidRateY.out - pidRateZ.out ;//;
				MOTOR3 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
				MOTOR4 +=    - pidRateX.out - pidRateY.out + pidRateZ.out;//;
			}	
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
			status = WAITING_1;	
			break;
		default:
			break;
	}
	
	
	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);  //更新PWM
	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);
	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);
	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);
} 

/************************************END OF FILE********************************************/ 
