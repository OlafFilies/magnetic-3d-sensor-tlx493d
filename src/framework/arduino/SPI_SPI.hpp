#ifndef TLX493D_SPI_SPI_HPP
#define TLX493D_SPI_SPI_HPP


// std includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Arduino includes
#include <Arduino.h>
// #include <SPI.h>

// project cpp includes
#include "SPILib.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"


extern "C" bool TLx493D_transferSPI(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);
bool TLx493D_initCommunication(TLx493D_t *sensor, SPILib<SPIClass> &spi);
bool TLx493D_initCommunication(TLx493D_t *sensor, SPIClass &spi);


#endif // TLX493D_SPI_SPI_HPP
