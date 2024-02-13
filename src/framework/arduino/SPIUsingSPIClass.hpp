#ifndef TLX493D_SPI_USING_SPICLASS_HPP
#define TLX493D_SPI_USING_SPICLASS_HPP


// std includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "SPIClassWrapper.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"


namespace ifx {
    namespace tlx493d {
        bool initCommunication(TLx493D_t *sensor, SPIClassWrapper &spi);
        bool initCommunication(TLx493D_t *sensor, SPIClass &spi);
    }
}


#endif // TLX493D_SPI_USING_SPICLASS_HPP
