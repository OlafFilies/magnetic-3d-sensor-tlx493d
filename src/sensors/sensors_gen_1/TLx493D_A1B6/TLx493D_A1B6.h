#ifndef TLx493D_A1B6_H
#define TLx493D_A1B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_1_common.h"

// sensor specicifc includes


// common functions
bool TLx493D_A1B6_init(Sensor_ts *sensor);
bool TLx493D_A1B6_deinit(Sensor_ts *sensor);

void TLx493D_A1B6_calculateTemperature(Sensor_ts *sensor, float *temp);
bool TLx493D_A1B6_getTemperature(Sensor_ts *sensor, float *temp);

void TLx493D_A1B6_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
bool TLx493D_A1B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

// bool TLx493D_A1B6_reset(Sensor_ts *sensor);
// bool TLx493D_A1B6_getDiagnosis(Sensor_ts *sensor);
void TLx493D_A1B6_calculateParity(Sensor_ts *sensor);

bool TLx493D_A1B6_setDefaultConfig(Sensor_ts *sensor);

void TLx493D_A1B6_setReservedRegisterValues(Sensor_ts *senor);

bool TLx493D_A1B6_enableTemperatureMeasurements(Sensor_ts *sensor);
bool TLx493D_A1B6_disableTemperatureMeasurements(Sensor_ts *sensor);

bool TLx493D_A1B6_transferWriteRegisters(Sensor_ts *sensor);

void TLx493D_A1B6_setPowerDownMode(Sensor_ts *sensor);
void TLx493D_A1B6_setMasterControlledMode(Sensor_ts *sensor);

bool TLx493D_A1B6_enableParityTest(Sensor_ts *sensor);
bool TLx493D_A1B6_disableParityTest(Sensor_ts *sensor);

void TLE493D_A2B6_calculateSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);
bool TLE493D_A2B6_getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);

#endif // TLx493D_A1B6_H
