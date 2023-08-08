#ifndef SENSORS_GEN_1_UTILS_H_
#define SENSORS_GEN_1_UTILS_H_

#include "stddef.h"

#include "sensors_config_common.h"
#include "sensors_common.h"
#include "sensor_types.h"

void gen_1_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
uint8_t gen_1_returnBitfield(Sensor_ts *sensor, uint8_t bitField);
void gen_1_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool gen_1_writeRegister(Sensor_ts* sensor, uint8_t registerAddr);

int16_t gen_1_concat_values(uint8_t MSB, uint8_t LSB, bool IsMSBinOneRegsiter);

#endif /* SENSORS_GEN_1_UTILS_H_ */