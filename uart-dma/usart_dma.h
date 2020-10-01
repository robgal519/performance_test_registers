// File: usart_dma.h
// Author: Robert Gałat
// Email: robgal519@gmail.com


#ifndef USART_DMA_H_
#define USART_DMA_H_

#include <stdint.h>

void configure_usart1(uint32_t baudrate);
void transfer_usart1_dma(uint8_t *data, uint32_t size);
void Unintialize(void);
#endif // USART_DMA_H_
