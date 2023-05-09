#ifndef SENSORS_H
#define SENSORS_H

// no std feature
// #pragma once


#include <stdbool.h>
#include <stdint.h>


#include "sensor_types.h"


// In case a specific sensor shall be instantiated bypassing the interface
void *createSensor(sensorTypes_t sensorType);


sensorTypes_t getSensorType(void *sensor);


#endif // SENSORS_H
