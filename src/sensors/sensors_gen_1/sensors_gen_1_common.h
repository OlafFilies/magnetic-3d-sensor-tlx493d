#ifndef SENSORS_GEN_1_COMMON_H
#define SENSORS_GEN_1_COMMON_H

// std includes
#include <stdbool.h>

// project c includes
// common to all sensors
#include "sensor_types.h"

void gen_1_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
uint8_t gen_1_returnBitfield(Sensor_ts *sensor, uint8_t bitField);
void gen_1_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool gen_1_writeRegister(Sensor_ts* sensor, uint8_t bitField);

bool gen_1_readRegisters(Sensor_ts *sensor);

bool gen_1_hasValidFuseParity(Sensor_ts *sensor);
bool gen_1_isFunctional(Sensor_ts *sensor);
bool gen_1_hasValidTBit(Sensor_ts *sensor);
bool gen_1_hasValidPDBit(Sensor_ts *sensor);
bool gen_1_hasValidData(Sensor_ts *sensor);

#endif // SENSORS_GEN_1_COMMON_H
