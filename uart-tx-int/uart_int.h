// File: uart_int.h
// Author: Robert Gałat
// Email: robgal519@gmail.com


#ifndef UART_INT_H_
#define UART_INT_H_

#include <stdint.h>

void configure_usart1(uint32_t baudrate);
void start_transfer_usart1(uint8_t* buffer, uint32_t size);

#endif // UART_INT_H_
