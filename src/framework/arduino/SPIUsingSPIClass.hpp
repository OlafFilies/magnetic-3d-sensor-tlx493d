#ifndef TLX493D_SPI_USING_SPICLASS_HPP
#define TLX493D_SPI_USING_SPICLASS_HPP


// std includes
#include <cstdbool>

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "SPIClassWrapper.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"


namespace ifx {
    namespace tlx493d {
        // bool initCommunication(TLx493D_t *sensor, SPIClassWrapper &spi, bool executeInit = false, uint32_t clockFreq = 200000, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE2);
        bool initCommunication(TLx493D_t *sensor, SPIClass &spi, bool executeInit = false, uint32_t clockFreq = 200000, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE2);
    }
}


#endif // TLX493D_SPI_USING_SPICLASS_HPP
