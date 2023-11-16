#ifndef TLX493D_SPI_USING_SPICLASS_HPP
#define TLX493D_SPI_USING_SPICLASS_HPP


// std includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Arduino includes
#include <Arduino.h>
// #include <SPI.h>

// project cpp includes
#include "SPIClassWrapper.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"


extern "C" bool tlx493d_transferSPI(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);
bool tlx493d_initCommunication(TLx493D_t *sensor, SPIClassWrapper<SPIClass> &spi);
bool tlx493d_initCommunication(TLx493D_t *sensor, SPIClass &spi);


#endif // TLX493D_SPI_USING_SPICLASS_HPP
