/*
 * uart-dma.c
 *
 * author: Furkan Cayci
 * description:
 *   UART example with DMA transfer on TX
 *   uses USART2 PA2/PA3 pins to transmit data
 *   connect a Serial to USB adapter to see the
 *   data on PC
 *
 * setup:
 *   1. enable usart and associated dma clocks from RCC
 *   2. setup usart and enable dma transfer / receive bits
 *   3. setup memory-to-peripheral or peripheral-to-memory mode for DMA
 *   4. setup source and destination addresses, as well as the number of bytes to be transferred
 *   5. enable completion interrupt and enable both peripherals
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

/*************************************************
* function declarations
*************************************************/
int main(void);
void log_message(char* ch){
    while(*ch)
        ITM_SendChar(*ch++);
}


volatile uint8_t msg[] = "https://furkan.space/\n\r";

static volatile bool usart1_transfer_complete = false;

void DMA2_Stream7_IRQHandler(void)
{
    // clear stream transfer complete interrupt
    if (DMA2->HISR & DMA_HISR_TCIF7) {
        // clear interrupt
        DMA2->HIFCR |= DMA_HISR_TCIF7;
        usart1_transfer_complete = true;
    }
}

static uint16_t get_bbr_for_speed(uint32_t speed, uint32_t fck){
    // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)

    static const float over8 = 1.0f;
    float usartDiv = ((float)fck/(float)speed)/(8.0f *(2.0f-over8));

    uint32_t mantisa = (uint32_t)usartDiv;
    uint16_t fraction = (uint16_t)(16*(usartDiv-(float)mantisa));
    uint16_t BRR = (uint16_t)((mantisa << 4U ) + fraction);
    return BRR;
}


void configure_usart1(uint32_t baudrate){


    // enable USART1 clock
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

    USART1->CR3 |= USART_CR3_DMAT;

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

    USART1->SR &= ~USART_SR_TC;

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    // clear control register
    DMA2_Stream7->CR = 0;
    // wait until DMA is disabled
    while(DMA2_Stream7->CR & (1 << 0));


    // channel select
    DMA2_Stream7->CR |= (0x4 << 25); // channel4

    // enable transfer complete interrupt
    DMA2_Stream7->CR |= DMA_SxCR_TCIE; // (1 << 4);

    // peripheral data size already 00 for byte
    // memory increment mode
    DMA2_Stream7->CR |= DMA_SxCR_MINC; // (1 << 10);

    // Priority level
    DMA2_Stream7->CR |= (0x2 << 16); // high - 10

    // DIR bits should be 01 for memory-to-peripheral
    //   source is SxM0AR, dest is SxPAR
    DMA2_Stream7->CR |= (0x1 << 6);


    NVIC_SetPriority(DMA2_Stream7_IRQn, 3); // Priority level 3
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

void transfer_usart1_dma(uint8_t* data, uint32_t size){
    usart1_transfer_complete = false;
    // source memory address
    DMA2_Stream7->M0AR = (uint32_t)data;
    // destination memory address
    DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);
    // number of items to be transferred
    DMA2_Stream7->NDTR = size;
    // enable DMA
    DMA2_Stream7->CR |= DMA_SxCR_EN; // (1 << 0);
}

uint32_t common_speeds[] = {
    4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
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

            uint32_t counter = 0;
            configure_usart1(common_speeds[boudrate_id]);
            randomize_payload(data, sizeof(data));
            transfer_usart1_dma(data, sizeof(data));
            GPIOA->ODR |= 1<<4;
            while(!usart1_transfer_complete){
                counter++;
            }
            GPIOA->ODR &= (uint32_t)~(1<<4);
            sprintf(log_buffer,"%lu,%lu\n",common_speeds[boudrate_id], counter);
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
    while(1)
    {

    }

    return 0;
}
