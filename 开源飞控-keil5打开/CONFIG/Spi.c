/*******************************************************************

 ******************************************************************/
#include "spi.h"

#define  SCK_H  GPIOB->BSRR=GPIO_Pin_13  //SCK拉高
#define  SCK_L  GPIOB->BRR=GPIO_Pin_13  //SCK拉高
#define  MOSI_H  GPIOB->BSRR=GPIO_Pin_15  //MOSI拉高
#define  MOSI_L  GPIOB->BRR=GPIO_Pin_15  //MOSI拉高
#define  MISO  ((GPIOB->IDR&GPIO_Pin_14)?1:0)  //读取MISO

//#define  SCK_H  GPIO_SetBits(GPIOB, GPIO_Pin_13)  //SCK拉高
//#define  SCK_L  GPIO_ResetBits(GPIOB, GPIO_Pin_13)  //SCK拉高

//#define  MOSI_H  GPIO_SetBits(GPIOB, GPIO_Pin_15)  //MOSI拉高
//#define  MOSI_L  GPIO_ResetBits(GPIOB, GPIO_Pin_15)  //MOSI拉高

//#define  MISO   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)  //读取MISO

void SPI_Config(void)//io初始化配置
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_SetBits(GPIOB, GPIO_Pin_12); //NRF_CS预置为高 
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//SCK
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//MOSI
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;//MISO
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //MOSI要用模拟输入
		GPIO_Init(GPIOB,&GPIO_InitStructure);
		SCK_L;
		MOSI_H;
}




u8 SPI_RW(u8 byte)
{
		uint8_t i;

		u8 Temp=0x00;

		for (i = 0; i < 8; i++)

		{

				SCK_L;//sclk = 0;//先将时钟拉高

				if (byte&0x80)
				{
						MOSI_H; // //SO=1
				}
				else
				{
						MOSI_L;// //SO=0
				}

				byte <<= 1;
				Temp<<=1;
				
				SCK_H; //拉低时钟
				
				if(MISO) //读到1时
				{
					Temp++;
				}
				SCK_L;	
		}
		return Temp; //返回读到miso输入的值

}


