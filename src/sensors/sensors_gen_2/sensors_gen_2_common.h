#ifndef SENSORS_GEN_2_COMMON_H
#define SENSORS_GEN_2_COMMON_H


// std includes
#include <stdbool.h>


// project c includes
// common to all sensors
#include "sensor_types.h"


/*
    Utility to concatenate MSB and LSB of field values
*/
void concatBytes(Sensor_ts *sensor, Register_ts *msb, Register_ts *lsb, int16_t *result);

void getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
void setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);


#endif // SENSORS_GEN_2_COMMON_H
