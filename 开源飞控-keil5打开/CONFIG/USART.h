#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

extern void USART1_SendByte(const int8_t *Data,uint8_t len);
void Uart_Start_DMA_Tx(const int8_t *data ,uint8_t size);
extern void USART1_Config(void);




#endif /* __USART1_H */

