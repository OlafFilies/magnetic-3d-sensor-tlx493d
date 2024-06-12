#ifndef TLX493D_ARDUINO_TYPES_H
#define TLX493D_ARDUINO_TYPES_H

/** Arduino includes. */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

/** project cpp includes. */
#include "Kit2GoBoardSupport.hpp"
#include "TLx493D.hpp"

/** project c includes. */
/** common to all sensors. */
#include "tlx493d_types.h"

/**
 * @brief Structure to hold the I2C-object, needed for communication.
 */
using TLx493D_I2CObject_t = struct TLx493D_I2CObject_t {
    ifx::tlx493d::TwoWireWrapper wire;

    // ifx::tlx493d::TwoWireWrapper *wire;
    // bool                          isToBeDeleted;
};

/**
 * @brief Structure to hold the SPI-object, needed for communication.
 */
using TLx493D_SPIObject_t = struct TLx493D_SPIObject_t {
    ifx::tlx493d::SPIClassWrapper spi;

    // ifx::tlx493d::SPIClassWrapper *spi;
    // bool                           isToBeDeleted;
};

/**
 * @brief Structure to hold the Kit2GoBoardSupportObject.
*/
using TLx493D_Kit2GoBoardSupportObject_t = struct TLx493D_Kit2GoBoardSupportObject_t {
    ifx::tlx493d::Kit2GoBoardSupport  *k2go;
    // bool                               isToBeDeleted;
};


namespace ifx {
    namespace tlx493d {
        /** Definiton of the specific sensor constructors. This makes instantiation of the sensors easier for the user,
         *  since they can use a "regular" constructor call, instead of using the template notation. 
         */
        using TLx493D_A1B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_A1B6_e>;
        using TLx493D_A2B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_A2B6_e>;
        using TLx493D_P2B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_P2B6_e>;
        using TLx493D_W2B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_W2B6_e>;
        using TLx493D_W2BW = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_W2BW_e>;
        using TLx493D_P3B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_P3B6_e>;

        // using TLx493D_P3I8 = TLx493D<Kit2GoBoardSupport, SPIClass, TLx493D_P3I8_e>;
        using TLx493D_P3I8 = TLx493D<Kit2GoBoardSupport, SPIClassWrapper, TLx493D_P3I8_e>;
    }
}

#endif // TLX493D_ARDUINO_TYPES_H
