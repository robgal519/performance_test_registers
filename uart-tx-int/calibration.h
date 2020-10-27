// File: calibration.h
// Author: Robert Gałat
// Email: robgal519@gmail.com

#include "stdint.h"
#include "stm32f4xx.h"
#include <stdbool.h>

void setup_timer(void **internal, uint32_t baudrate);
bool start_timer(void *internal, uint8_t *data, uint16_t size);
bool deinit_timer(void *internal);