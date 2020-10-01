// File: uart_int.c
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "uart_int.h"

#include "stdbool.h"
#include "stm32f407xx.h"
#include "stm32f4xx.h"


static uint8_t *data;
static uint32_t data_size;
volatile uint32_t next_byte_pos;

extern volatile bool UART_TransferComplete;

static uint16_t get_bbr_for_speed(uint32_t speed, uint32_t fck) {
  // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)

  static const float over8 = 1.0f;
  float usartDiv = ((float)fck / (float)speed) / (8.0f * (2.0f - over8));

  uint32_t mantisa = (uint32_t)usartDiv;
  uint16_t fraction = (uint16_t)(16 * (usartDiv - (float)mantisa));
  uint16_t BRR = (uint16_t)((mantisa << 4U) + fraction);
  return BRR;
}

void USART1_IRQHandler(void) {
  // check if the source is transmit interrupt
  if (USART1->SR & USART_SR_TXE_Msk) {

    if (next_byte_pos == data_size) {
      UART_TransferComplete = true;
      USART1->CR1 &= ~USART_CR1_TXEIE;
    } else {
      // flush ot the next char in the buffer
      USART1->DR = data[next_byte_pos++];
    }
  }
}

void configure_usart1(uint32_t baudrate) {

  // enable USART2 clock, bit 17 on APB1ENR
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  // enable GPIOA clock, bit 0 on AHB1ENR
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // set pin modes as alternate mode 7 (pins 2 and 3)
  // USART2 TX and RX pins are PA2 and PA3 respectively
  GPIOA->MODER &= ~(0xFU << (2 * 9)); // Reset bits  PA9 and PA10
  GPIOA->MODER |=
      (0xAU << (2 * 9)); // Set   bits  PA9 and PA10 to alternate mode (10)

  // choose AF7 for USART2 in Alternate Function registers
  GPIOA->AFR[1] |= (0x7 << (9 - 8) * 4);  // for pin A9
  GPIOA->AFR[1] |= (0x7 << (10 - 8) * 4); // for pin A10

  // USART2 word length M, bit 12
  // USART2->CR1 |= (0 << 12); // 0 - 1,8,n

  // USART2 parity control, bit 9
  // USART2->CR1 |= (0 << 9); // 0 - no parity

  // USART2 TX enable, TE bit 3
  USART1->CR1 |= USART_CR1_TE;

  // USART2 RX enable, RE bit 2
  // USART2->CR1 |= (1 << 2);

  USART1->CR1 |= USART_CR1_OVER8;

  USART1->BRR = get_bbr_for_speed(baudrate, 84000000);
  // enable usart1 - UE, bit 13
  USART1->CR1 |= USART_CR1_UE;

  NVIC_SetPriority(USART1_IRQn, 1); // Priority level 1
  NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief send the data to uart1 using interrupt to push data
 *
 * @param buffer pointer to data to send, it has to be valid until the data has
 * been send
 * @param size amount of data to send.
 */
void start_transfer_usart1(uint8_t *buffer, uint32_t size) {
  data = buffer;
  data_size = size;
  next_byte_pos = 0;
  UART_TransferComplete = false;
  USART1->CR1 |= USART_CR1_TXEIE;
}

void Unintialize(void)
{
  USART1->CR1 &= ~USART_CR1_UE;

  NVIC_DisableIRQ(USART1_IRQn);
}