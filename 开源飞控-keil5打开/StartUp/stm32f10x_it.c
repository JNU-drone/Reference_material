/**
  ******************************************************************************
  * @file    ADC/ADC1_DMA/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "ALL_DATA.h"

#include "usb_istr.h"
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC_ADC1_DMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}


/**
  * @brief  This function handles SysTick_Handler exception.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	SysTick_count++;
}

/**
  * @brief  
STM32的USB模块可以产生三种中断：USB唤醒中断、USB高优先级中断和USB低优先级中断，在STM32的参考手册中没有详细说明这三种中断对应哪些事件，现说明如下： 
1）USB唤醒中断：在中断向量表中的位置是42。这个中断在USB设备从暂停模式唤醒时产生，唤醒事件由USB_ISTR寄存器的WKUP位标识。 
2）USB高优先级中断：在中断向量表中的位置是19。这个中断仅由USB同步(Isochronous)模式传输或双缓冲块(Bulk)传输模式下的正确传输事件产生，正确传输事件由USB_ISTR寄存器的CTR位标识。 
3）USB低优先级中断：在中断向量表中的位置是20。这个中断由所有其它的USB事件产生，例如正确传输(不包括同步模式和双缓冲块模式)、USB复位等，事件标志位在USB_ISTR寄存器中。 
在STM32的USB开发包的例子中包含了上述中断的处理，例如在USB扬声器的例子中，CTR_HP函数处理USB高优先级中断；在所有例子中都有USB_Istr()函数处理USB低优先级中断。

*/
//void USB_HP_CAN1_TX_IRQHandler(void)//USB_HP_CAN1_TX_IRQHandle
//{
//}

void USB_LP_CAN1_RX0_IRQHandler(void)//USB_LP_CAN1_RX0_IRQHandler //USB收发中断
{
    USB_Istr();
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
