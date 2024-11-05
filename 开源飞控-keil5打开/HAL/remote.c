/*******************************************************************
 *@title Remote Control;
 *@brief remote data process on polling every 10ms.
 *@brief remote control on interrupt every 24ms
 ******************************************************************/
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "mpu6050.h"

#define SUCCESS 0
#undef FAILED
#define FAILED  1
/*****************************************************************************************
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
	uint8_t RC_rxData[32];
void remote_unlock(void);	

void RC_Analy(void)  
{
		static uint16_t cnt;
/*             Receive  and check RC data                               */	
	if(NRF24L01_RxPacket(RC_rxData)==SUCCESS)
	{ 	
		
		uint8_t i;
		uint8_t CheckSum=0;

		cnt = 0;
		
		for(i=0;i<31;i++)
		{
			CheckSum +=  RC_rxData[i];
		}
		if(RC_rxData[31]==CheckSum && RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)  //如果接收到的遥控数据正确
		{
			  Remote.roll = ((uint16_t)RC_rxData[4]<<8) | RC_rxData[5];  //通道1
				Remote.roll = LIMIT(Remote.roll,1000,2000);
				Remote.pitch = ((uint16_t)RC_rxData[6]<<8) | RC_rxData[7];  //通道2
				Remote.pitch = LIMIT(Remote.pitch,1000,2000);
				Remote.thr = 	((uint16_t)RC_rxData[8]<<8) | RC_rxData[9];   //通道3
				Remote.thr = 	LIMIT(Remote.thr,1000,2000);
				Remote.yaw =  ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //通道4
				Remote.yaw =  LIMIT(Remote.yaw,1000,2000);
				Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //通道5  左上角按键都属于通道5  
				Remote.AUX1 =  LIMIT(Remote.AUX1,1000,2000);
				Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //通道6  右上角按键都属于通道6 
				Remote.AUX2 =  LIMIT(Remote.AUX2,1000,2000);
				Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //通道7  左下边按键都属于通道7 
				Remote.AUX3 =  LIMIT(Remote.AUX3,1000,2000);
				Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //通道8  右下边按键都属于通道6  
				Remote.AUX4 = LIMIT(Remote.AUX4,1000,4000);	
				


				
				{
							const float roll_pitch_ratio = 0.04f;
							const float yaw_ratio =   0.3f;	
					
							pidPitch.desired =-(Remote.pitch-1500)*roll_pitch_ratio;	 //将遥杆值作为飞行角度的期望值
							pidRoll.desired = -(Remote.roll-1500)*roll_pitch_ratio;
					    if(Remote.yaw>1820)
							{
								pidYaw.desired -= yaw_ratio;	
							}
							else if(Remote.yaw <1180)
							{
								pidYaw.desired += yaw_ratio;	
							}						
				}
				remote_unlock();
			
		}
  }
//如果3秒没收到遥控数据，则判断遥控信号丢失，飞控在任何时候停止飞行，避免伤人。
//意外情况，使用者可紧急关闭遥控电源，飞行器会在3秒后立即关闭，避免伤人。
//立即关闭遥控，如果在飞行中会直接掉落，可能会损坏飞行器。
	else
	{
	
		
		cnt++;
		if(cnt>500)
		{
			cnt = 0;
			ALL_flag.unlock = 0; 
			NRF24L01_init();
		}
	}
}

/*****************************************************************************************
 *  解锁判断
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
#include "imu.h"
void remote_unlock(void)
{
	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0;

	if(Remote.thr<1050 &&Remote.yaw<1200)                         //油门遥杆左下角锁定
	{
		status = EXIT_255;
	}
	
	switch(status)
	{
		case WAITING_1://等待解锁
			if(Remote.thr<1150)           //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁
			{			 
					 status = WAITING_2;				 
			}		
			break;
		case WAITING_2:
			if(Remote.thr>1600)          
			{		
						static uint8_t cnt = 0;
					 	cnt++;		
						if(cnt>5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
						{	
								cnt=0;
								status = WAITING_3;
						}
			}			
			break;
		case WAITING_3:
			if(Remote.thr<1100)          
			{			 
					 status = WAITING_4;				 
			}			
			break;			
		case WAITING_4:	//解锁前准备	               
				ALL_flag.unlock = 1;
				status = PROCESS_31;
				LED.status = AlwaysOn;	
			  imu_rest();		
				 break;		
		case PROCESS_31:	//进入解锁状态
				if(Remote.thr<1020)
				{
					if(cnt++ > 3000)                                     // 油门遥杆处于最低9S自动上锁
					{								
						status = EXIT_255;								
					}
				}
				else if(!ALL_flag.unlock)                           //Other conditions lock 
				{
					status = EXIT_255;				
				}
				else					
					cnt = 0;
			break;
		case EXIT_255: //进入锁定
			LED.status = AllFlashLight;	                                 //exit
			cnt = 0;
			LED.FlashTime = 100; //100*3ms		
			ALL_flag.unlock = 0;
			status = WAITING_1;
			break;
		default:
			status = EXIT_255;
			break;
	}
}
/***********************END OF FILE*************************************/







