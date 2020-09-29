// File: calibration.h
// Author: Robert Gałat
// Email: robgal519@gmail.com

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <stdint.h>

void configure_timer(uint32_t baudrate);
void start_calibration(uint8_t *data, uint32_t size);

#endif // CALIBRATION_H_