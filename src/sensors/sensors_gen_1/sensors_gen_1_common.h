#ifndef SENSORS_GEN_1_COMMON_H
#define SENSORS_GEN_1_COMMON_H

// std includes
#include <stdbool.h>

// project c includes
// common to all sensors
#include "sensor_types.h"

#define TLx493D_A1B6_READ_REGISTERS_MAX_COUNT          10

void gen_1_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
uint8_t gen_1_returnBitfield(Sensor_ts *sensor, uint8_t bitField);
void gen_1_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool gen_1_writeRegister(Sensor_ts* sensor, uint8_t registerAddr);

bool gen_1_readRegisters(Sensor_ts *sensor);

#endif // SENSORS_GEN_1_COMMON_H
