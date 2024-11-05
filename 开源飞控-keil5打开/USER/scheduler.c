#include "ALL_DEFINE.h"
#include "scheduler.h"



void TIM1_UP_IRQHandler(void)   //TIM3中断 3ms
{
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		MpuGetData();
		GetAngle(&MPU6050,&Angle,0.003f);
		RC_Analy();			
		FlightPidControl(0.003f);
		MotorControl();
	
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update );  //清除TIMx的中断待处理位:TIM 中断源 
	}
}


