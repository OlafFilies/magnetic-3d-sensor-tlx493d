#ifndef TLE493D_A2B6_H
#define TLE493D_A2B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_config_common.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_config_common.h"
#include "sensors_gen_2_common.h"

// sensor specicifc includes
#include "TLE493D_A2B6_config.h"


// common functions
bool TLE493D_A2B6_init(Sensor_ts *sensor);
bool TLE493D_A2B6_deinit(Sensor_ts *sensor);

bool TLE493D_A2B6_getTemperature(Sensor_ts *sensor, float *temp);
bool TLE493D_A2B6_updateGetTemperature(Sensor_ts *sensor, float *temp);

bool TLE493D_A2B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
bool TLE493D_A2B6_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

bool TLE493D_A2B6_reset(Sensor_ts *sensor);
bool TLE493D_A2B6_getDiagnosis(Sensor_ts *sensor);
bool TLE493D_A2B6_calculateParity(Sensor_ts *sensor);

bool TLE493D_A2B6_setDefaultConfig(Sensor_ts *sensor);
bool TLE493D_A2B6_updateRegisterMap(Sensor_ts *sensor);


// individual functions
void TLE493D_A2B6_get1ByteModeBuffer(uint8_t *buf, uint8_t *bufLen);
void TLE493D_A2B6_getTemperatureMeasurementsBuffer(uint8_t *regMap, uint8_t *buf, uint8_t *bufLen);


// utility functions
void TLE493D_A2B6_concatBytes(Sensor_ts *sensor, Register_ts *msb, Register_ts *lsb, int16_t *result);


#endif /** TLE493D_A2B6_H */