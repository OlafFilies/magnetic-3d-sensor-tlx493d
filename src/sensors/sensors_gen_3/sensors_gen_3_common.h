#ifndef SENSORS_GEN_3_COMMON_H
#define SENSORS_GEN_3_COMMON_H


#include "sensor_types.h"


void gen_3_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);


bool gen_3_writeRegisterSPI(Sensor_ts* sensor, uint8_t bitField);
bool gen_3_readRegistersSPI(Sensor_ts *sensor);


#endif // SENSORS_GEN_3_COMMON_H
