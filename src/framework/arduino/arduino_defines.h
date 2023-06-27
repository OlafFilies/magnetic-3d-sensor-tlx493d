#ifndef ARDUINO_DEFINES_H
#define ARDUINO_DEFINES_H


// std includes

// Arduino includes
#include <Arduino.h>
#include <Wire.h>


#define log  (Serial.print)


typedef struct Sensor_ts Sensor_ts;


struct I2C_t {
    TwoWire *wire;
};


extern "C" {
    void initComLibIF(Sensor_ts *sensor, struct I2C_t &iic);
}


#endif // ARDUINO_DEFINES_H
