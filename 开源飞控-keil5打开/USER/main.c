/*******************************************************************
 *程序入口
 *@brief 主初始化函数。轮询调用的函数
 *@brief 
 *@time  2021.1.8
 *@editor zin小店
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
 ******************************************************************/
#include "ALL_DEFINE.h"


//特别声明：请在无人空旷的地带或者室内进行飞行。遇到非常紧急的情况，可紧急关闭遥控。

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
int main(void)
{	
	cycleCounterInit();  //得到系统每个us的系统CLK个数，为以后延时函数，和得到精准的当前执行时间使用
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //2个bit的抢占优先级，2个bit的子优先级
	SysTick_Config(SystemCoreClock / 1000);	//系统滴答时钟

	ALL_Init();//系统初始化
	/*
	 while循环里做一些比较不重要的事情，比如上位机和LED控制。
	其余功能皆安排在中断，整个工程只有一个3ms更新一次的中断，请见scheduler.c里面的工作
	*/
	while(1)
	{
			ANTO_polling(); //匿名上位机发送数据
	//		Usb_Hid_Receive();//USB接收到匿名上位机的数据
			PilotLED(); //LED刷新
	}
}










