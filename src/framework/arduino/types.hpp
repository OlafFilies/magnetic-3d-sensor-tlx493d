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


typedef struct TLx493D_I2CObject_t {
    TwoWireWrapper *wire;
    bool            isToBeDeleted;
} TLx493D_I2CObject_t;


typedef struct TLx493D_SPIObject_t {
    SPIClassWrapper *spi;
    bool             isToBeDeleted;
} TLx493D_SPIObject_t;


typedef struct TLx493D_BoardSupportObject_t {
    Kit2GoBoardSupport  *bsc;
} TLx493D_BoardSupportObject_t;


typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_A1B6_e>  TLx493D_A1B6;
typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_A2B6_e>  TLx493D_A2B6;
typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_P2B6_e>  TLx493D_P2B6;
typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_W2B6_e>  TLx493D_W2B6;
typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_W2BW_e>  TLx493D_W2BW;
typedef TLx493D<Kit2GoBoardSupport, TwoWireWrapper, TLx493D_P3B6_e>  TLx493D_P3B6;

typedef TLx493D<Kit2GoBoardSupport, SPIClassWrapper, TLx493D_P3I8_e> TLx493D_P3I8;


#endif // TLX493D_ARDUINO_TYPES_H
