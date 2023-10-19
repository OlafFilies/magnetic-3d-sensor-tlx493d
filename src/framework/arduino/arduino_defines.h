#ifndef ARDUINO_DEFINES_H
#define ARDUINO_DEFINES_H


// std includes

// Arduino includes
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>


#include "Logger.h"


template<typename TW> class TwoWire_Lib;
template<typename SP> class SPI_Lib;


typedef struct I2CObject_ts {
    TwoWire_Lib<TwoWire> *wire;
} I2CObject_ts;


typedef struct SPIObject_ts {
    SPI_Lib<SPIClass> *spi;
} SPIObject_ts;


typedef struct Sensor_ts Sensor_ts;


bool initI2CComLibIF(Sensor_ts *sensor, TwoWire_Lib<TwoWire> &tw);
bool initSPIComLibIF(Sensor_ts *sensor, SPI_Lib<SPIClass> &spi);


extern "C" {
    bool initI2CComLibIF(Sensor_ts *sensor, TwoWire &iic);
    bool initSPIComLibIF(Sensor_ts *sensor, SPIClass &spi);
}


#endif // ARDUINO_DEFINES_H
