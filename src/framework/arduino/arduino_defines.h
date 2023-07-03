#ifndef ARDUINO_DEFINES_H
#define ARDUINO_DEFINES_H


// std includes

// Arduino includes
#include <Arduino.h>
#include <Wire.h>


#define log  (Serial.print)


typedef struct Sensor_ts Sensor_ts;


typedef struct ComLibraryObject_ts {
    TwoWire *wire;
} ComLibraryObject_ts;


extern "C" {
    void initComLibIF(Sensor_ts *sensor, TwoWire &iic);
}


#endif // ARDUINO_DEFINES_H
