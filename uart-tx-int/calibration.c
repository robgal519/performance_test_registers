// File: calibration.c
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "calibration.h"
#include "stm32f4xx.h"
#include "stdbool.h"

extern volatile bool uart1_transfer_complete;

void TIM2_IRQHandler(void) {
  // clear interrupt status
  if (TIM2->DIER & 0x01) {
    if (TIM2->SR & 0x01) {
      TIM2->SR &= (uint32_t) ~(1U << 0);
    }
  }

  uart1_transfer_complete = true;
}

void configure_timer(uint32_t baudrate) {
  (void)baudrate;
  RCC->APB1ENR |= (1 << 0);

  // Timer clock runs at ABP1 * 2
  //   since ABP1 is set to /4 of fCLK
  //   thus 168M/4 * 2 = 84Mhz
  // set prescaler to 83999
  //   it will increment counter every prescalar cycles
  // fCK_PSC / (PSC[15:0] + 1)
  // 84 Mhz / 8399 + 1 = 10 khz timer clock speed
  TIM2->PSC = 0;

  // Set the auto-reload value to 10000
  //   which should give 1 second timer interrupts
  TIM2->ARR = 84000000;

  // Update Interrupt Enable
  TIM2->DIER |= (1 << 0);

  NVIC_SetPriority(TIM2_IRQn, 2); // Priority level 2
  // enable TIM2 IRQ from NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
}

void start_calibration(uint8_t *data, uint32_t size) {
  (void)data;
  (void)size;
  TIM2->CR1 |= (1 << 0);
}
