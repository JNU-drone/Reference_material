/**************************************************************
 * 
 ***************************************************************/
#include "stm32f10x.h"
#include "USB_HID.h"
#include "USB_config.h"
//#include "USB_init.h" 
#include "usb_lib.h"
#include "delay.h"
#include "ANO_DT.h"


u8 USB_ReceiveFlg = 0;
u8 Hid_RxData[64];
u8 HID_SEND_TIMEOUT = 5;			//hid发送不足一帧时，等待HID_SEND_TIMEOUT周期进行发送
u8 hid_datatemp[256];					//hid环形缓冲区
u8 hid_datatemp_begin = 0;		//环形缓冲区数据指针，指向应当发送的数据
u8 hid_datatemp_end = 0;			//环形缓冲区数据结尾
//bool USB_ReceiveFlg;
void USB_HID_Init(void)
{
//	USB_GPIO_Configuration();

	Set_USBClock();     
	USB_Init();
	USB_ReceiveFlg = 0;
	USB_Connect(0);//软件或者硬件重启usb
	delay_ms(100);
	USB_Connect(1);
		USB_Interrupts_Config();
}

void Usb_Hid_Adddata(u8 *dataToSend , u8 length)
{
	u8 i;
	for(i=0; i<length; i++)
	{
		hid_datatemp[hid_datatemp_end++] = dataToSend[i];
	}
}

void Usb_Hid_Receive()//不断查询
{
	if (USB_ReceiveFlg)
	{
		if(Hid_RxData[0] < 33 && Hid_RxData[1]==0xaa)
		{
				//Hid_RxData[0]长度，从Hid_RxData[1]开始为一帧完整的数据
					   if(Hid_RxData[1] == 0xaa && Hid_RxData[2] == 0xaF ) //帧头正确	
						 {    
										u8 i;
										u8 check_sum = 0;
										for(i=1;i<Hid_RxData[0];i++)  //buf[0] 为收到PC来的数据总长度
										{
											check_sum+= Hid_RxData[i];
										}
										if(check_sum == Hid_RxData[Hid_RxData[0]]) //buf[0] 为收到PC来的数据总长度 最后一个buf为PC发来的本次数据校验和，与本次收到计算出的校验和做比较
										{
												if(Hid_RxData[4] >= 0x10 && Hid_RxData[5]<=0x15) //如果收到的是PID数据则要返回校验值
												{	
														checkPID = Hid_RxData[4]<<8 | check_sum;  //返回PID校验数据													
														ANTO_Send(ANTO_CHECK); //收到HID发来的PID则要马上返回校验值给上位机
												}	
												ANO_Recive((int8_t*)(Hid_RxData+1));		//hid接收到数据会调用此函数	
										}
										Hid_RxData[1] = 0;//帧头清0
						 }	
		}
		USB_ReceiveFlg = 0;	
	}

}

void Usb_Hid_Send (void)
{
	static u8 notfull_timeout=0;
	u8 i;
	if(hid_datatemp_end > hid_datatemp_begin)
	{
		if((hid_datatemp_end - hid_datatemp_begin) >= 63)
		{
			notfull_timeout = 0;
			Transi_Buffer[0] = 63;
			for( i=0; i<63; i++)
			{
				Transi_Buffer[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			UserToPMABufferCopy(Transi_Buffer, ENDP2_TXADDR, 64);
			SetEPTxValid(ENDP2);
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				notfull_timeout = 0;
				Transi_Buffer[0] = hid_datatemp_end - hid_datatemp_begin;
				for( i=0; i<63; i++)
				{
					if(i<hid_datatemp_end - hid_datatemp_begin)
						Transi_Buffer[i+1] = hid_datatemp[hid_datatemp_begin+i];
					else
						Transi_Buffer[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				UserToPMABufferCopy(Transi_Buffer, ENDP2_TXADDR, 64);
				SetEPTxValid(ENDP2);
			}
		}
	}
	else if(hid_datatemp_end < hid_datatemp_begin)
	{
		if((256 - hid_datatemp_begin + hid_datatemp_end) >= 63)
		{
			notfull_timeout = 0;
			Transi_Buffer[0] = 63;
			for( i=0; i<63; i++)
			{
				Transi_Buffer[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			UserToPMABufferCopy(Transi_Buffer, ENDP2_TXADDR, 64);
			SetEPTxValid(ENDP2);
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				notfull_timeout = 0;
				Transi_Buffer[0] = 256 - hid_datatemp_begin + hid_datatemp_end;
				for( i=0; i<63; i++)
				{
					if(i<256 - hid_datatemp_begin + hid_datatemp_end)
						Transi_Buffer[i+1] = hid_datatemp[(u8)(hid_datatemp_begin+i)];
					else
						Transi_Buffer[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				UserToPMABufferCopy(Transi_Buffer, ENDP2_TXADDR, 64);
				SetEPTxValid(ENDP2);
			}
		}
	}
}







/*********************END OF FILE******************************************************/
















