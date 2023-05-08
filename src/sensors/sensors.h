// #ifndef SENSORS_H
// #define SENSORS_H
#pragma once


#include <stdbool.h>
#include <stdint.h>


#include "sensor_types.h"


// In case a specific sensor shall be instantiated bypassing the interface
void *createSensor(sensorTypes_t sensorType);


// #endif // SENSORS_H
