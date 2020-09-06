/*
 * uart-tx-int.c
 *
 * author: Furkan Cayci
 * description:
 *   UART example with tx interrupt
 *   uses USART2 PA2/PA3 pins to transmit data
 *   connect a Serial to USB adapter to see the
 *   data on PC
 *
 * setup:
 *   1. enable usart clock from RCC
 *   2. enable gpioa clock
 *   3. set PA2 and PA3 as af7
 *   4. set uart word length and parity
 *   5. enable transmit and receive (TE/RE bits)
 *   6. calculate baud rate and set BRR
 *   7. enable uart
 *   8. setup uart handler to send out a given buffer
 *   9. enable uart interrupt from NVIC
 *   10.enable tx interrupt and disable when the buffer
 *      transmission is complete
 */

#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "stdbool.h"
#include <stdint.h>
#include "stdio.h"
#include "stdlib.h"
#include "debug.h"


/*************************************************
* function declarations
*************************************************/
int main(void);
void log_message(char* ch){
    while(*ch)
        ITM_SendChar(*ch++);
}


static uint8_t* data;
static uint32_t data_size;
volatile uint32_t next_byte_pos;

static volatile bool uart1_transfer_complete = false;

static uint16_t get_bbr_for_speed(uint32_t speed, uint32_t fck){
    // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)

    static const float over8 = 1.0f;
    float usartDiv = ((float)fck/(float)speed)/(8.0f *(2.0f-over8));

    uint32_t mantisa = (uint32_t)usartDiv;
    uint16_t fraction = (uint16_t)(16*(usartDiv-(float)mantisa));
    uint16_t BRR = (uint16_t)((mantisa << 4U ) + fraction);
    return BRR;
}


void USART1_IRQHandler(void)
{
    // check if the source is transmit interrupt
    if (USART1->SR & USART_SR_TXE_Msk) {

        if (next_byte_pos == data_size) {
            uart1_transfer_complete = true;
            USART1->CR1 &= ~ USART_CR1_TXEIE;
        }
        else {
            // flush ot the next char in the buffer
            USART1->DR = data[next_byte_pos++];
        }
    }
}

void configure_usart1(uint32_t baudrate){


    // enable USART2 clock, bit 17 on APB1ENR
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // set pin modes as alternate mode 7 (pins 2 and 3)
    // USART2 TX and RX pins are PA2 and PA3 respectively
    GPIOA->MODER &= ~(0xFU << (2*9)); // Reset bits  PA9 and PA10
    GPIOA->MODER |=  (0xAU << (2*9)); // Set   bits  PA9 and PA10 to alternate mode (10)

    // choose AF7 for USART2 in Alternate Function registers
    GPIOA->AFR[1] |= (0x7 << (9-8)*4); // for pin A9
    GPIOA->AFR[1] |= (0x7 << (10-8)*4); // for pin A10

    // USART2 word length M, bit 12
    //USART2->CR1 |= (0 << 12); // 0 - 1,8,n

    // USART2 parity control, bit 9
    //USART2->CR1 |= (0 << 9); // 0 - no parity

    // USART2 TX enable, TE bit 3
    USART1->CR1 |= USART_CR1_TE;

    // USART2 RX enable, RE bit 2
    //USART2->CR1 |= (1 << 2);

    
    USART1->CR1 |= USART_CR1_OVER8;

    USART1->BRR = get_bbr_for_speed(baudrate,84000000);
    // enable usart1 - UE, bit 13
    USART1->CR1 |= USART_CR1_UE;

    NVIC_SetPriority(USART1_IRQn, 1); // Priority level 1
    NVIC_EnableIRQ(USART1_IRQn);

}
/**
 * @brief send the data to uart1 using interrupt to push data
 * 
 * @param buffer pointer to data to send, it has to be valid until the data has been send 
 * @param size amount of data to send.
 */
void start_transfer_usart1(uint8_t* buffer, uint32_t size){
    data = buffer;
    data_size = size;
    next_byte_pos = 0;
    uart1_transfer_complete = false;
    USART1->CR1 |= USART_CR1_TXEIE ;
}


uint32_t common_speeds[] = {
    4800, 
    9600, 
    19200, 
    38400, 
    57600, 
    115200, 
    230400, 
    460800, 
    921600,
    1312500,
    2625000,
    5250000,
    10500000, /* MAX Suppoprted Speed */
    };

#define COMMON_SPEEDS_SIZE sizeof(common_speeds)/sizeof(*common_speeds)
#define REPS 5U

void randomize_payload(uint8_t* data, uint32_t size){
    for(uint32_t i = 0; i< size; i++){
        data[i] = (uint8_t)rand();
    }
}

void test(){
    static uint8_t data[500];
    char log_buffer[128];
    for(uint32_t boudrate_id = 0; boudrate_id < COMMON_SPEEDS_SIZE; boudrate_id++){
        for(uint8_t retry = 0; retry<REPS;retry++){
            init_cycles();
            uint32_t counter = 0;
            configure_usart1(common_speeds[boudrate_id]);
            randomize_payload(data, sizeof(data));
            start_transfer_usart1(data, sizeof(data));
            GPIOA->ODR |= 1<<4;
            
            uint32_t begin_it = read_cycles_it();
            uint32_t begin = read_cycles();
            
            while(!uart1_transfer_complete){
                counter++;
            }
            uint32_t end = read_cycles();
            uint32_t end_it = read_cycles_it();
            GPIOA->ODR &= (uint32_t)~(1<<4);
            sprintf(log_buffer,"%lu,%lu,%lu,%lu\n",common_speeds[boudrate_id], counter, end-begin, end_it-begin_it);
            log_message(log_buffer);
        }
    }
}
/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();
    RCC->AHB1ENR |=RCC_AHB1RSTR_GPIOARST;
    GPIOA->MODER &= ~(3U << 2*4);
    GPIOA->MODER |= (1U << 2*4);
    GPIOA->ODR &= ~(1<<4);
    
    test();

    log_message("program end\n");
    while(1);

    return 0;
}
