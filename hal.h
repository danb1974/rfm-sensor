#ifndef __HAL_H__
#define __HAL_H__

#include <Arduino.h>
#ifndef SENSOR_NO_DEFAULT_SPI
void spi_Transfer(uint8_t *data, uint8_t len);
#endif

#endif