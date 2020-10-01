// File: main.c
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "calibration.h"
#include "stdbool.h"
#include "stddef.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "uart_int.h"

#define COMMON_SPEEDS_SIZE sizeof(common_speeds) / sizeof(*common_speeds)
#define REPS 5U

int main(void);

struct test_ctx {
  void (*configure)(uint32_t baudrate);
  void (*transfer)(uint8_t *data, uint32_t size);
  void (*deinit)(void);
};

volatile bool UART_TransferComplete = false;

uint32_t common_speeds[] = {
    4800,   9600,   19200,   38400,   57600,   115200,   230400,
    460800, 921600, 1312500, 2625000, 5250000, 10500000, /* MAX Suppoprted Speed
                                                          */
};

static void log_message(char *ch) {
  while (*ch)
    ITM_SendChar(*ch++);
}

void randomize_payload(uint8_t *data, uint32_t size) {
  for (uint32_t i = 0; i < size; i++) {
    data[i] = (uint8_t)rand();
  }
}

struct test_ctx calibration = {.configure = configure_timer,
                               .transfer = start_calibration,
                               .deinit = diable_timer};

struct test_ctx testing_dma = {.configure = configure_usart1,
                               .transfer = start_transfer_usart1,
                               .deinit = Unintialize};

bool test_performance(struct test_ctx *ctx, uint32_t baud, uint32_t *counter) {

  static uint8_t data[500];
  uint32_t cnt = 0;

  if (ctx == NULL)
    return false;

  if (ctx->configure == NULL)
    return false;
  ctx->configure(baud);

  randomize_payload(data, sizeof(data));

  if (ctx->transfer == NULL)
    return false;

  UART_TransferComplete = false;

  ctx->transfer(data, sizeof(data));
  GPIOA->ODR |= 1 << 4;
  while (!UART_TransferComplete) {
    cnt++;
  }
  GPIOA->ODR &= (uint32_t) ~(1 << 4);

  if (ctx->deinit == NULL)
    return false;

  ctx->deinit();

  *counter = cnt;
  return true;
}

void test() {
  char log_buffer[128];
  uint32_t calibration_counter;
  test_performance(&calibration, 0, &calibration_counter);
  sprintf(log_buffer, "Calibration: %lu\n", calibration_counter);
  log_message(log_buffer);

  for (uint32_t boudrate_id = 0; boudrate_id < COMMON_SPEEDS_SIZE;
       boudrate_id++) {
    for (uint8_t retry = 0; retry < REPS; retry++) {
      uint32_t counter = 0;
      test_performance(&testing_dma, common_speeds[boudrate_id], &counter);
      sprintf(log_buffer, "%lu,%lu\n", common_speeds[boudrate_id], counter);
      log_message(log_buffer);
    }
  }
}

/*************************************************
 * main code starts from here
 *************************************************/
int main(void) {
  /* set system clock to 168 Mhz */
  set_sysclk_to_168();
  RCC->AHB1ENR |= RCC_AHB1RSTR_GPIOARST;
  GPIOA->MODER &= ~(3U << 2 * 4);
  GPIOA->MODER |= (1U << 2 * 4);
  GPIOA->ODR &= (uint32_t) ~(1 << 4);

  test();
  while (1) {
  }

  return 0;
}
