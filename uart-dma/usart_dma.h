// File: usart_dma.h
// Author: Robert Gałat
// Email: robgal519@gmail.com


#ifndef USART_DMA_H_
#define USART_DMA_H_

#include <stdint.h>
#include <stdbool.h>

void configure_usart1(void **internal,uint32_t baudrate);
bool transfer_usart1_dma(void *internal, uint8_t *data, uint32_t size);
bool Unintialize(void *internal);
#endif // USART_DMA_H_
