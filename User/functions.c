#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "functions.h"
#include <stdlib.h>
#include <stdint.h>



volatile char ir_value_buffer[IR_VALUE_SIZE];
volatile char ir_string_buffer[IR_STRING_SIZE];
volatile int HUT_count_down_val;
int buffer_index;
int ir_wave_table[IR_WAVE_SIZE];

volatile consolo os;

void system_flag_init(void)
{
    os.sys_stat = MODE_WAIT;
    
}

void system_active(void)
{
    if(buffer_index != 0)
        os.sys_stat = MODE_RX;
}

void led_pin_config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;//定义GPIO配置结构体
	RCC_APB2PeriphClockCmd(LED_GPIO_CLK, ENABLE);//开启LED的GPIO时钟
	
	GPIO_InitStruct.GPIO_Pin = LED_GPIO_PIN;//选择要初始化的引脚
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//选择为推挽输出模式
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;//选择输出信号的最大频率
	
	GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);//初始化LED的引脚配置
}

void host_usart_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(HOST_USART_CLK, ENABLE);//Attention:USART2 attach to APB1
	RCC_APB2PeriphClockCmd(HOST_RX_GPIO_CLK, ENABLE);
    
    //先初始化GPIO以防对USART产生干扰
	GPIO_InitStructure.GPIO_Pin = HOST_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(HOST_TX_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = HOST_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(HOST_RX_GPIO_PORT, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = HOST_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(HOST_USART, &USART_InitStructure);
	USART_Cmd(HOST_USART, ENABLE);
    
}

//************未启用******************/
void host_usart_dma_config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
    
    DMA_InitStructure.DMA_PeripheralBaseAddr = HOST_TXDR_ADDRESS;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ir_string_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = IR_STRING_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    
    DMA_Init(HOST_USART_DMA_CHANNEL,&DMA_InitStructure);
    //DMA_Cmd(HOST_USART_DMA_CHANNEL,ENABLE);
    
}

/**************配置接收中断******************/
void host_usart_nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = HOST_USART_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(HOST_USART, USART_IT_RXNE, ENABLE);
}

//用来判断链接主机的串口是否发送完毕,可用普通延时替换（未使用）
void usart_terminal_tim_config(void)
{
	RCC_APB1PeriphClockCmd(TERMINAL_TIM_CLK,ENABLE);
	TIM_TimeBaseInitTypeDef  TIM_InitStructure;
    
	TIM_InitStructure.TIM_Period = TERMINAL_TIM_PRD;	
	TIM_InitStructure.TIM_Prescaler = TERMINAL_TIM_PSC;	
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TERMINAL_TIM, &TIM_InitStructure);
	TIM_ClearFlag(TERMINAL_TIM, TIM_FLAG_Update);
	TIM_ITConfig(TERMINAL_TIM, TIM_IT_Update,ENABLE);
    
    //TIM_Cmd(TERMINAL_TIM,ENABLE);
}

/************发送一个字节信号**************/
void usart_sendbyte(USART_TypeDef* pUSARTx, uint8_t data)
{
	USART_SendData(pUSARTx, data);//调用STM32标准库函数，发送一个字节信号，进行传输时USART_FLAG_TXE会自动置位，传输完成自动清零
	while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET );//等待字节发送完成
}

/***********发送一个字符串************/
void usart_sendstr(USART_TypeDef* pUSARTx, char *str)
{
	uint8_t i=0;
	do//逐个字节发送
	{
		usart_sendbyte(pUSARTx, *(str+i));
		i++;
	}while(*(str+i) != '\0');//判断字符串结尾
	while( USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET );
}

void host_usart_terminal_check(void)
{
    if(--HUT_count_down_val == 0){
        os.sys_stat = MODE_NEC;
        os.task_stat = NEC_PRE;
    }
}

void host_usart_int(void)
{
    host_usart_config();
    host_usart_nvic_config();
}

/****************************************************/
/****************************************************/
/****************************************************/

void ir_output_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(IR_OUT_GPIO_CLK, ENABLE);
    
	GPIO_InitStructure.GPIO_Pin = IR_OUT_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(IR_OUT_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = IR_GND_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(HOST_TX_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_ResetBits(IR_OUT_GPIO_PORT,IR_OUT_GPIO_PIN);
    GPIO_ResetBits(IR_GND_GPIO_PORT,IR_GND_GPIO_PIN);
}
//红外信号延时计时器
void ir_generator_tim_config(void)
{
	RCC_APB1PeriphClockCmd(IR_TIM_CLK,ENABLE);
	TIM_TimeBaseInitTypeDef  TIM_InitStructure;
    
	TIM_InitStructure.TIM_Period = IR_TIM_PRD;	
	TIM_InitStructure.TIM_Prescaler = IR_TIM_PSC;	
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//忘了设置会变成随机数哦
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(IR_TIM, &TIM_InitStructure);
	TIM_ClearFlag(IR_TIM,TIM_IT_Update);
	TIM_ITConfig(IR_TIM,TIM_IT_Update,ENABLE);
}

void ir_generator_nvic_condig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	
	NVIC_InitStructure.NVIC_IRQChannel = IR_TIM_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void ir_generator_init()
{
    ir_output_gpio_config();
    ir_generator_tim_config();
    ir_generator_nvic_condig();
}
    


int str_to_nec_code(const char *int_str, int *nec_code)
{
    uint32_t digital_val;
    uint32_t pulse_pos = 2;
    nec_code[0]=16;
    nec_code[1]=8;
    digital_val = (uint32_t)strtoul(int_str,NULL,16);
    for(int byte_pos=24;byte_pos>=0;byte_pos-=8){
        uint8_t sigbyte = (digital_val>>byte_pos)&0xff;
        for(int bit_pos=1;bit_pos<=0x80;bit_pos<<=1){
            nec_code[pulse_pos++] = 1;
            if(sigbyte&bit_pos)
                nec_code[pulse_pos++] = 3;
            else
                nec_code[pulse_pos++] = 1;
        }
    }
    nec_code[pulse_pos]=1;
    return pulse_pos;
}

void ir_nec_code_paser(void)
{
    ir_value_buffer[buffer_index] = '\0';
    if( ir_value_buffer[0]=='0' && (ir_value_buffer[1]=='x' || ir_value_buffer[2]=='X') ){
        str_to_nec_code((char *)ir_value_buffer, ir_wave_table);//add else
        GPIO_SetBits(IR_OUT_GPIO_PORT,IR_OUT_GPIO_PIN);
        TIM_Cmd(IR_TIM,ENABLE);
        os.task_stat = NEC_TX;
    }else{
        os.sys_stat = MODE_WAIT;
        usart_sendstr(HOST_USART,"Key value MUST start with 0x !\n");
    }
    buffer_index = 0;//不好的设计
}


void ir_echo_info(void)
{
    usart_sendstr(HOST_USART,"Sended!\n");
    os.sys_stat = MODE_WAIT;
}
