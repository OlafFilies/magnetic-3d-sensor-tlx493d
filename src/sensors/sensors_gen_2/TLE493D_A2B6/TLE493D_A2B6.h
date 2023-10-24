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

// sensor specific includes
// #include "TLE493D_A2B6_defines.h"


// common functions
bool TLE493D_A2B6_init(Sensor_ts *sensor);
bool TLE493D_A2B6_deinit(Sensor_ts *sensor);

void TLE493D_A2B6_calculateTemperature(Sensor_ts *sensor, double *temp);
bool TLE493D_A2B6_getTemperature(Sensor_ts *sensor, double *temp);

void TLE493D_A2B6_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);
bool TLE493D_A2B6_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);

void TLE493D_A2B6_calculateMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);
bool TLE493D_A2B6_getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);

// bool TLE493D_A2B6_hasValidData(Sensor_ts *sensor);
// bool TLE493D_A2B6_isFunctional(Sensor_ts *sensor);

bool TLE493D_A2B6_setDefaultConfig(Sensor_ts *sensor);
bool TLE493D_A2B6_transferRegisterMap(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len);

// utility functions
bool TLE493D_A2B6_hasValidIICadr(Sensor_ts *sensor);
bool TLE493D_A2B6_hasWakeup(Sensor_ts *sensor);


bool TLE493D_A2B6_enableCollisionAvoidance(Sensor_ts *sensor);
bool TLE493D_A2B6_disableCollisionAvoidance(Sensor_ts *sensor);

#endif /** TLE493D_A2B6_H */