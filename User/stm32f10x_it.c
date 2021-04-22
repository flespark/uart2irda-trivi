/**
  ******************************************************************************
  * @file    ./User/stm32f10x_it.c 
  * @author  Flespark
  * @version V1.0.5
  * @date    4-June-2019
	* @coding  UTF-8 whitout signature
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "functions.h"



/** @addtogroup STM32F10x_StdPeriph_Template
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
/*****************内核程序异常中断配置部分(维持默认设置不变）******************/

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
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
/**************************外设中断配置部分************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)//判断是否为PA0的输入中断
	{	

	}
	EXTI_ClearITPendingBit(EXTI_Line0);//清除中断挂起位；
}

void HOST_USART_IRQHandler(void)
{
    uint8_t ucTemp;
    if( USART_GetITStatus(USART2, USART_IT_RXNE) == SET ){
        ucTemp = USART_ReceiveData(USART2);
        if( buffer_index < IR_VALUE_SIZE ){
            ir_value_buffer[buffer_index++] = ucTemp;
        }else{
            //add some error echo
        }
        HUT_count_down_val = HOST_USART_LAPSE;
    }
    //USART非异常中断都可以通过读或写数据寄存器清零
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET){//计时器产生溢出中断

        if( buffer_index >= IR_VALUE_SIZE ){
            //Add some error info
            buffer_index=0;
        }else if(buffer_index == 0){
            //Enter standby
        }else{
            
            //直接指定NEC模式，待修改；
            os.sys_stat = MODE_NEC;
            os.ir_stat = NEC_PRE;
        }
        
        TIM_ClearITPendingBit(TIM3 , TIM_IT_Update);//清零计时器溢出标志位
    }
}
 */

void IR_TIM_IRQHandler(void)
{
	if ( TIM_GetITStatus(IR_TIM, TIM_IT_Update) == SET ) //判断是否为计时器溢出中断
	{
        if(--ir_wave_table[buffer_index]){
            TIM_SetCounter(IR_TIM,1);
        }else if(ir_wave_table[++buffer_index]){
            GPIO_ToggleBit(IR_OUT_GPIO_PORT,IR_OUT_GPIO_PIN);
            TIM_SetCounter(IR_TIM,1);
        }else{
            buffer_index = 0;
            GPIO_ToggleBit(IR_OUT_GPIO_PORT,IR_OUT_GPIO_PIN);
            TIM_Cmd(IR_TIM,DISABLE);
            os.task_stat = NEC_ECHO;
        }
		TIM_ClearITPendingBit(IR_TIM , TIM_IT_Update);//清零计时器溢出标志位
	}
}


/***********供电电压过低ADC中断函数************************/
void ADC_IRQHandler(void)
{

}


/******************* (C) COPYRIGHT BY CC 4.0 *****END OF FILE****/
