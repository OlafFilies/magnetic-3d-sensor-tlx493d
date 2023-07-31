#ifndef SENSORS_GEN_2_UTILS_H_
#define SENSORS_GEN_2_UTILS_H_

#include "stddef.h"

#include "sensors_config_common.h"
#include "sensors_common.h"
#include "sensor_types.h"

void getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
void setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool writeRegister(Sensor_ts* sensor, uint8_t registerAddr);

#endif /* SENSORS_GEN_2_UTILS_H_ */