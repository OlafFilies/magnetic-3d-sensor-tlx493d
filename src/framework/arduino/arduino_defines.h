#ifndef ARDUINO_DEFINES_H
#define ARDUINO_DEFINES_H


// std includes

// Arduino includes
#include <Arduino.h>
#include <Wire.h>


#define log  (Serial.print)


template<typename TW> class TwoWire_Lib;


typedef struct I2CObject_ts {
    TwoWire_Lib<TwoWire> *wire;
} I2CObject_ts;


typedef struct Sensor_ts Sensor_ts;


bool initI2CComLibIF(Sensor_ts *sensor, TwoWire_Lib<TwoWire> &tw);


extern "C" {
    bool initI2CComLibIF(Sensor_ts *sensor, TwoWire &iic);
    // void myDebug(char *msg);
}


#endif // ARDUINO_DEFINES_H
