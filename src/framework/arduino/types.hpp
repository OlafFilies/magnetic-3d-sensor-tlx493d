#ifndef TLX493D_ARDUINO_DEFINES_H
#define TLX493D_ARDUINO_DEFINES_H


// std includes

// Arduino includes
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// project cpp includes
#include "S2GoTemplateArduino.hpp"
// #include "SPIClassWrapper.hpp"
#include "TLx493D.hpp"
// #include "TwoWireWrapper.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"


template<typename TW> class TwoWireWrapper;
template<typename SP> class SPIClassWrapper;


typedef struct TLx493D_I2CObject_t {
    TwoWireWrapper<TwoWire> *wire;
} TLx493D_I2CObject_t;


typedef struct TLx493D_SPIObject_t {
    SPIClassWrapper<SPIClass> *spi;
} TLx493D_SPIObject_t;



typedef S2GoTemplate<pinCtrl>  S2GoTemplateArduino;

typedef TLx493D<S2GoTemplateArduino, TwoWireWrapper, TwoWire, TLx493D_A1B6_e, TLx493D_I2C_e> TLx493D_A1B6;
typedef TLx493D<S2GoTemplateArduino, TwoWireWrapper, TwoWire, TLx493D_A2B6_e, TLx493D_I2C_e> TLx493D_A2B6;
typedef TLx493D<S2GoTemplateArduino, TwoWireWrapper, TwoWire, TLx493D_P2B6_e, TLx493D_I2C_e> TLx493D_P2B6;
typedef TLx493D<S2GoTemplateArduino, TwoWireWrapper, TwoWire, TLx493D_W2B6_e, TLx493D_I2C_e> TLx493D_W2B6;
typedef TLx493D<S2GoTemplateArduino, TwoWireWrapper, TwoWire, TLx493D_W2BW_e, TLx493D_I2C_e> TLx493D_W2BW;
typedef TLx493D<S2GoTemplateArduino, TwoWireWrapper, TwoWire, TLx493D_P3B6_e, TLx493D_I2C_e> TLx493D_P3B6;

typedef TLx493D<S2GoTemplateArduino, SPIClassWrapper, SPIClass, TLx493D_P3I8_e, TLx493D_SPI_e> TLx493D_P3I8;


#endif // TLX493D_ARDUINO_DEFINES_H
