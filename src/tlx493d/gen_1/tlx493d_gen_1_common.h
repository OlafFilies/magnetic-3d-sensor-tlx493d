#ifndef TLX493D_GEN_1_COMMON_H
#define TLX493D_GEN_1_COMMON_H

// std includes
#include <stdbool.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"

void tlx493d_gen_1_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
uint8_t tlx493d_gen_1_returnBitfield(Sensor_ts *sensor, uint8_t bitField);
void tlx493d_gen_1_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool tlx493d_gen_1_writeRegister(Sensor_ts* sensor, uint8_t bitField);

bool tlx493d_gen_1_readRegisters(Sensor_ts *sensor);

bool tlx493d_gen_1_hasValidFuseParity(Sensor_ts *sensor, uint8_t ffBF);
bool tlx493d_gen_1_isFunctional(Sensor_ts *sensor, uint8_t ffBF);
bool tlx493d_gen_1_hasValidTBit(Sensor_ts *sensor, uint8_t tBF);
bool tlx493d_gen_1_hasValidPDBit(Sensor_ts *sensor, uint8_t pdBF);
bool tlx493d_gen_1_hasValidData(Sensor_ts *sensor, uint8_t tBF, uint8_t pdBF);

#endif // TLX493D_GEN_1_COMMON_H
