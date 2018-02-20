#include "hal.h"

void spi_Transfer(uint8_t *data, uint8_t len)
{
    digitalWrite(SS, LOW);
    while (len--)
    {
        *data = SPI.transfer(*data);
        data++;
    }
    digitalWrite(SS, HIGH);
}
