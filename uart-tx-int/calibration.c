// File: calibration.h
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "calibration.h"

#include <stdbool.h>

extern volatile bool UART_TransferComplete;

void TIM2_IRQHandler(void) {
  // clear interrupt status
  if (TIM2->DIER & 0x01) {
    if (TIM2->SR & 0x01) {
      TIM2->SR &= (uint32_t) ~(1U << 0);
    }
  }
  TIM2->CR1 &= (~((uint16_t)TIM_CR1_CEN));
  UART_TransferComplete = true;
  NVIC_DisableIRQ(TIM2_IRQn);
}

void setup_timer(void **internal, uint32_t baudrate) {
  (void)internal;
  (void)baudrate;
  RCC->APB1ENR |= (1 << 0);

  // Timer clock runs at ABP1 * 2
  //   since ABP1 is set to /4 of fCLK
  //   thus 168M/4 * 2 = 84Mhz
  // set prescaler to 0
  TIM2->PSC = 0;

  // Set the auto-reload value to 84000000
  //   which should give 1 second timer interrupts
  TIM2->ARR = 84000000;

  // Update Interrupt Enable
  TIM2->DIER |= (1 << 0);

  NVIC_SetPriority(TIM2_IRQn, 2); // Priority level 2
  // enable TIM2 IRQ from NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
}

bool start_timer(void *internal, uint8_t *data, uint16_t size) {
  (void)internal;
  (void)data;
  (void)size;

  TIM2->CR1 |= (1 << 0);
  return true;
}

bool deinit_timer(void *internal) {
  (void)internal;
  TIM2->CR1 &= (~((uint16_t)TIM_CR1_CEN));
  NVIC_DisableIRQ(TIM2_IRQn);
  return true;
}