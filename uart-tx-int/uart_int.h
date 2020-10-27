// File: uart_int.h
// Author: Robert Gałat
// Email: robgal519@gmail.com


#ifndef UART_INT_H_
#define UART_INT_H_

#include <stdint.h>

void configure_usart1(void **internal, uint32_t baudrate);
bool start_transfer_usart1(void *internal, uint8_t *data, uint16_t size);
bool Unintialize(void *internal);
#endif // UART_INT_H_
