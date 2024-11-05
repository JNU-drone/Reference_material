/*******************************************************************
 *MPU6050

 ******************************************************************/

#include "usart.h"
#include "misc.h"
#include "stdio.h"
#include "delay.h"
/*
 * 函数名：USART1_Config
 * 描述  ：USART1 GPIO 配置,工作模式配置。
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
 void DMA_Uart_Init(void)
{

    DMA_InitTypeDef DMA_InitStructure;
    /* DMA clock enable */

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 开启DMA1时钟
//=DMA_Configuration==============================================================================//

/*--- LUMMOD_UART_Tx_DMA_Channel DMA Config ---*/

    DMA_Cmd(DMA1_Channel4, DISABLE);                           // 关DMA通道  USART1_TX   DMA1_Channel4

    DMA_DeInit(DMA1_Channel4);                                 // 恢复缺省值

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);// 设置串口发送数据寄存器

    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;         // 设置发送缓冲区首地址

    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // 设置外设位目标，内存缓冲区 -> 外设寄存器

    DMA_InitStructure.DMA_BufferSize = 0;                     // 需要发送的字节数，这里其实可以设置为0，因为在实际要发送的时候，会重新设置次值

    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的

    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节

    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节

    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式

    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置

    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式

    DMA_Init(DMA1_Channel4, &DMA_InitStructure);               // 写入配置

    DMA_ClearFlag( DMA1_FLAG_GL4/*DMA1_FLAG_TC4 | DMA1_FLAG_TE4 */);                                 // 清除DMA所有标志

		DMA_Cmd(DMA1_Channel4, DISABLE); // 关闭DMA

    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);            // 开启发送DMA通道中断
}
uint8_t datatest[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26};
void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* 配置串口1 （USART1） 时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* Configure the NVIC Preemption Priority Bits */  

	
	/* 使能串口1中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	//USART1  串口1全局中断 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级1
	/*IRQ通道使能*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1*/
	NVIC_Init(&NVIC_InitStructure);
	
		 NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;   // 发送DMA通道的中断配置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // 优先级设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	

	/*串口GPIO端口配置*/
  /* 配置串口1 （USART1 Tx (PA.09)）*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	/* 配置串口1 USART1 Rx (PA.10)*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = 230400;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
	
	
	DMA_Uart_Init();

	USART_Cmd(USART1, ENABLE); //使能串口 
	 USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送

//	while(1)//测试发送
//	{
//		while (!(USART1->SR & USART_FLAG_TXE));	 // while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
//		USART_SendData(USART1,0xAA);	 	
//		delay_ms(10);
//	}
//	while(1)//测试发送
//	{
//		  DMA_ClearFlag( DMA1_FLAG_GL4/*DMA1_FLAG_TC4 | DMA1_FLAG_TE4 */);   
//		Uart_Start_DMA_Tx(datatest,25);
//		delay_ms(12);
//		
//	}
}

void DMA1_Channel4_IRQHandler(void)
{

    if(DMA_GetITStatus(DMA1_FLAG_TC4))
    {
				DMA_ClearFlag( DMA1_FLAG_GL4/*DMA1_FLAG_TC4 | DMA1_FLAG_TE4 */);

				DMA_Cmd(DMA1_Channel4, DISABLE);   // 关闭DMA通道
    }

}


void Uart_Start_DMA_Tx(const int8_t *data ,uint8_t size)
{
//		DMA1_Channel4->CCR &= ~DMA_CCR1_EN;
//delay_ms(1);

	DMA1_Channel4->CPAR = (uint32_t)(&USART1->DR);
	DMA1_Channel4->CMAR = (uint32_t)data;
	
    DMA1_Channel4->CNDTR = (uint16_t)size; // 设置要发送的字节数目

  //  DMA_Cmd(DMA1_Channel4, ENABLE);        //开始DMA发送
	DMA1_Channel4->CCR |= DMA_CCR1_EN;
}



void USART1_SendByte(const int8_t *Data,uint8_t len)
{ 
	uint8_t i;
	
	for(i=0;i<len;i++)
	{
		while (!(USART1->SR & USART_FLAG_TXE));	 // while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		USART_SendData(USART1,*(Data+i));	 
	}
}




int8_t CheckSend[7]={0xAA,0XAA,0xEF,2,0,0,0};

void USART1_setBaudRate(uint32_t baudRate)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate =  baudRate;
	USART_Init(USART1, &USART_InitStructure);
}






/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
	/* 将Printf内容发往串口 */
	USART_SendData(USART1, (unsigned char) ch);
	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	
	return (ch);
}

