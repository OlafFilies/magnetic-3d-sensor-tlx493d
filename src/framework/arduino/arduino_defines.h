#ifndef TLX493D_ARDUINO_DEFINES_H
#define TLX493D_ARDUINO_DEFINES_H


// std includes

// Arduino includes
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// project cpp includes
#include "TLx493D.hpp"
#include "S2GoTemplateArduino.hpp"
#include "SPI_Lib.hpp"
#include "TwoWire_Lib.hpp"
#include "Logger.h"


template<typename TW> class TwoWire_Lib;
template<typename SP> class SPI_Lib;


typedef struct TLx493D_I2CObject_ts {
    TwoWire_Lib<TwoWire> *wire;
} TLx493D_I2CObject_ts;


typedef struct TLx493D_SPIObject_ts {
    SPI_Lib<SPIClass> *spi;
} TLx493D_SPIObject_ts;


typedef struct TLx493D_ts  TLx493D_ts;


bool TLx493D_initCommunication(TLx493D_ts *sensor, TwoWire_Lib<TwoWire> &tw);
bool TLx493D_initCommunication(TLx493D_ts *sensor, SPI_Lib<SPIClass> &spi);


bool TLx493D_initCommunication(TLx493D_ts *sensor, TwoWire &iic);
bool TLx493D_initCommunication(TLx493D_ts *sensor, SPIClass &spi);


typedef TLx493D<S2GoTemplateArduino, TwoWire_Lib, TwoWire, TLx493D_A1B6_e, TLx493D_I2C_e> TLx493D_A1B6;
typedef TLx493D<S2GoTemplateArduino, TwoWire_Lib, TwoWire, TLx493D_A2B6_e, TLx493D_I2C_e> TLx493D_A2B6;
typedef TLx493D<S2GoTemplateArduino, TwoWire_Lib, TwoWire, TLx493D_P2B6_e, TLx493D_I2C_e> TLx493D_P2B6;
typedef TLx493D<S2GoTemplateArduino, TwoWire_Lib, TwoWire, TLx493D_W2B6_e, TLx493D_I2C_e> TLx493D_W2B6;
typedef TLx493D<S2GoTemplateArduino, TwoWire_Lib, TwoWire, TLx493D_W2BW_e, TLx493D_I2C_e> TLx493D_W2BW;
typedef TLx493D<S2GoTemplateArduino, TwoWire_Lib, TwoWire, TLx493D_P3B6_e, TLx493D_I2C_e> TLx493D_P3B6;

typedef TLx493D<S2GoTemplateArduino, SPI_Lib, SPIClass, TLx493D_P3I8_e, TLx493D_SPI_e> TLx493D_P3I8;


#endif // TLX493D_ARDUINO_DEFINES_H
