#ifndef TLE493D_A2B6_H
#define TLE493D_A2B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_common.h"

// sensor specicifc includes


// common functions
bool TLE493D_A2B6_init(Sensor_ts *sensor);
bool TLE493D_A2B6_deinit(Sensor_ts *sensor);

void TLE493D_A2B6_calculateTemperature(Sensor_ts *sensor, float *temp);
bool TLE493D_A2B6_getTemperature(Sensor_ts *sensor, float *temp);

void TLE493D_A2B6_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
bool TLE493D_A2B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

void TLE493D_A2B6_calculateSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);
bool TLE493D_A2B6_getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);

// bool TLE493D_A2B6_hasValidData(Sensor_ts *sensor);
// bool TLE493D_A2B6_isFunctional(Sensor_ts *sensor);

bool TLE493D_A2B6_setDefaultConfig(Sensor_ts *sensor);
bool TLE493D_A2B6_transferRegisterMap(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len);

// utility functions


#endif /** TLE493D_A2B6_H */