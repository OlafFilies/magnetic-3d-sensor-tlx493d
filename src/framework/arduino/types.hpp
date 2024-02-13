#ifndef TLX493D_ARDUINO_TYPES_H
#define TLX493D_ARDUINO_TYPES_H


// std includes

// Arduino includes
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// project cpp includes
#include "Kit2GoBoardSupport.hpp"
#include "TLx493D.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"


// typedef struct TLx493D_I2CObject_t {
//     TwoWireWrapper *wire;
//     bool            isToBeDeleted;
// } TLx493D_I2CObject_t;

using TLx493D_I2CObject_t = struct TLx493D_I2CObject_t {
    ifx::tlx493d::TwoWireWrapper *wire;
    bool                          isToBeDeleted;
};


// typedef struct TLx493D_SPIObject_t {
//     SPIClassWrapper *spi;
//     bool             isToBeDeleted;
// } TLx493D_SPIObject_t;
using TLx493D_SPIObject_t = struct TLx493D_SPIObject_t {
    ifx::tlx493d::SPIClassWrapper *spi;
    bool                           isToBeDeleted;
};


// typedef struct TLx493D_Kit2GoBoardSupportObject_t {
//     Kit2GoBoardSupport  *k2go;
//     bool                 isToBeDeleted;
// } TLx493D_Kit2GoBoardSupportObject_t;
using TLx493D_Kit2GoBoardSupportObject_t = struct TLx493D_Kit2GoBoardSupportObject_t {
    ifx::tlx493d::Kit2GoBoardSupport  *k2go;
    bool                               isToBeDeleted;
};


namespace ifx {
    namespace tlx493d {
        // typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_A1B6_e>  TLx493D_A1B6;
        // typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_A2B6_e>  TLx493D_A2B6;
        // typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_P2B6_e>  TLx493D_P2B6;
        // typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_W2B6_e>  TLx493D_W2B6;
        // typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_W2BW_e>  TLx493D_W2BW;
        // typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_P3B6_e>  TLx493D_P3B6;
        using TLx493D_A1B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_A1B6_e>;
        using TLx493D_A2B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_A2B6_e>;
        using TLx493D_P2B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_P2B6_e>;
        using TLx493D_W2B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_W2B6_e>;
        using TLx493D_W2BW = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_W2BW_e>;
        using TLx493D_P3B6 = TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_P3B6_e>;

        // typedef TLx493D<Kit2GoBoardSupport, SPIClassWrapper, TLx493D_P3I8_e> TLx493D_P3I8;
        using TLx493D_P3I8 = TLx493D<Kit2GoBoardSupport, SPIClassWrapper, TLx493D_P3I8_e>;
    }
}

#endif // TLX493D_ARDUINO_TYPES_H
