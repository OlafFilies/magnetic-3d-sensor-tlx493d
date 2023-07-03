#ifndef ARDUINO_DEFINES_H
#define ARDUINO_DEFINES_H


// std includes

// Arduino includes
#include <Arduino.h>
#include <Wire.h>


#define log  (Serial.print)


typedef struct I2CObject_ts {
    TwoWire *wire;
} I2CObject_ts;


typedef struct Sensor_ts Sensor_ts;


extern "C" {
    void initI2CComLibIF(Sensor_ts *sensor, TwoWire &iic);
}


#endif // ARDUINO_DEFINES_H
