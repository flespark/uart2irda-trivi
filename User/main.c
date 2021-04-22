
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "functions.h"

int main()
{
    system_flag_init();
    host_usart_int();
    ir_generator_init();
    led_pin_config();
    LED_ON;
    start:
    while(1){
        switch(os.sys_stat){
            case MODE_WAIT:
                goto sys_wait;
            case MODE_RX:
                goto receive;
            case MODE_NEC:
                goto nec;
            case MODE_FLUSH:
                goto flush;
            default:
                __nop();
        }
    }
    
    while(0){
        //Add system reset here;
        sys_wait:
        system_active();
        break;
        
        receive:
        host_usart_terminal_check();
        break;
        
        nec:
        switch(os.task_stat){
            case NEC_PRE:
                ir_nec_code_paser();
                break;
            case NEC_TX:
                __nop();
                break;
            case NEC_ECHO:
                ir_echo_info();
                break;
        }
        break;
        
        flush:
        break;
    }
    goto start;
}
