#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdint.h>
#include "stm32f10x.h"

#define NEC_PRE                      0x00
#define NEC_TX                       0x01
#define NEC_ECHO                     0x02


#define MODE_WAIT                    0x00
#define MODE_RX                      0x01
#define MODE_NEC                     0x02
#define MODE_TX                      0x04
#define MODE_FLUSH                   0x08

#define LED_GPIO_CLK                 RCC_APB2Periph_GPIOC
#define LED_GPIO_PORT                GPIOC
#define LED_GPIO_PIN                 GPIO_Pin_13
#define LED_ON                       GPIO_ResetBits(LED_GPIO_PORT,LED_GPIO_PIN)
#define LED_OFF                      GPIO_SetBits(LED_GPIO_PORT,LED_GPIO_PIN)

#define IR_VALUE_SIZE                10
#define IR_STRING_SIZE               128
#define IR_WAVE_SIZE                 200

#define HOST_USART                   USART2
#define HOST_USART_CLK               RCC_APB1Periph_USART2
#define HOST_BAUDRATE                115200
#define HOST_USART_IRQ               USART2_IRQn
#define HOST_USART_IRQHandler        USART2_IRQHandler
#define HOST_RX_GPIO_CLK             RCC_APB2Periph_GPIOA
#define HOST_RX_GPIO_PORT            GPIOA
#define HOST_RX_GPIO_PIN             GPIO_Pin_3
#define HOST_TX_GPIO_CLK             RCC_APB2Periph_GPIOA
#define HOST_TX_GPIO_PORT            GPIOA
#define HOST_TX_GPIO_PIN             GPIO_Pin_2
#define HOST_TXDR_ADDRESS            (USART2_BASE+0x04)
#define HOST_USART_DMA_CHANNEL       DMA1_Channel7
#define HOST_USART_LAPSE             1000

#define IR_TIM				         TIM2
#define IR_TIM_CLK			         RCC_APB1Periph_TIM2
#define IR_TIM_PRD			         (1125-1)
#define IR_TIM_PSC			         (36-1)
#define IR_TIM_IRQ                   TIM2_IRQn
#define IR_TIM_IRQHandler            TIM2_IRQHandler

#define IR_OUT_GPIO_CLK              RCC_APB2Periph_GPIOB
#define IR_OUT_GPIO_PORT             GPIOB
#define IR_OUT_GPIO_PIN              GPIO_Pin_13
#define IR_GND_GPIO_CLK              RCC_APB2Periph_GPIOB
#define IR_GND_GPIO_PORT             GPIOB
#define IR_GND_GPIO_PIN              GPIO_Pin_12

#define TERMINAL_TIM				 TIM3
#define TERMINAL_TIM_CLK			 RCC_APB1Periph_TIM3
#define TERMINAL_TIM_PRD			 (5000-1)
#define TERMINAL_TIM_PSC			 (72-1)
#define TERMINAL_IRQ                 TIM3_IRQn

typedef struct {
    uint8_t sys_stat;
    uint8_t task_stat;
}consolo;

extern volatile consolo os;
extern volatile char ir_value_buffer[IR_VALUE_SIZE];
extern volatile char ir_string_buffer[IR_STRING_SIZE];
extern volatile int HUT_count_down_val;
extern int buffer_index;
extern int ir_wave_table[IR_WAVE_SIZE];

static inline void GPIO_ToggleBit(GPIO_TypeDef *port,uint16_t pin)
{
    port->ODR^=pin;
}

//main progress control
void system_flag_init(void);
void system_active(void);
void host_usart_terminal_check(void);

//led init
void led_pin_config(void);

//usart conneted to host serial port
void host_usart_config(void);
void host_usart_dma_config(void);
void host_usart_nvic_config(void);
void usart_terminal_tim_config(void);
void usart_sendbyte(USART_TypeDef* pUSARTx, uint8_t data);
void usart_sendstr(USART_TypeDef* pUSARTx, char *str);
void ir_echo_info(void);
void host_usart_int(void);

//gpio transmit infrared signal
void ir_output_gpio_config(void);
void ir_generator_tim_config(void);
void ir_generator_nvic_condig(void);
void ir_generator_init(void);
void ir_nec_code_paser(void);

//data transform api
int str_to_nec_code(const char *int_str, int *nec_code);

    
#endif

