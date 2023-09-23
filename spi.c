#include <avr/io.h>
#include "spi.h"

/*******************************************************************************
*   spi.c v1 - 11/02/2018
        Initial definitions
*   spi.c v2 - 11/07/2018
        Updated initialization functions
*******************************************************************************/

void SPI_init(uint16_t initParams)
{
    // set CS, MOSI and SCK to output
    DDR_SPI |= (1 << CS) | (1 << MOSI) | (1 << SCK);

    // enable pull up resistor in MISO
    DDR_SPI |= (1 << MISO);

    // set SPI params
    SPCR |= ((uint8_t) (initParams >> 8)) | (1 << SPE);
    SPSR |= ((uint8_t) initParams);
}

uint8_t SPI_transfer(uint8_t data)
{
    // load data into register
    SPDR = data;

    // Wait for transmission complete
    while(!(SPSR & (1 << SPIF)));

    // return SPDR
    return SPDR;
}
