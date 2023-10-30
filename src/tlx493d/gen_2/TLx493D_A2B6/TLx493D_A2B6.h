#ifndef TLX493D_A2B6_H
#define TLX493D_A2B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common.h"

// common to same generation of sensors
#include "tlx493d_gen_2_common.h"

// sensor specific includes
// #include "TLx493D_A2B6_defines.h"


// common functions
bool TLx493D_A2B6_init(Sensor_ts *sensor);
bool TLx493D_A2B6_deinit(Sensor_ts *sensor);

bool TLx493D_A2B6_setPowerMode(Sensor_ts *sensor, uint8_t mode);
bool TLx493D_A2B6_setIICAddress(Sensor_ts *sensor, StandardIICAddresses_te address);

void TLx493D_A2B6_calculateTemperature(Sensor_ts *sensor, double *temp);
bool TLx493D_A2B6_getTemperature(Sensor_ts *sensor, double *temp);

void TLx493D_A2B6_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);
bool TLx493D_A2B6_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);

void TLx493D_A2B6_calculateMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_A2B6_getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);

uint8_t TLx493D_A2B6_calculateFuseParity(Sensor_ts *sensor);
uint8_t TLx493D_A2B6_calculateBusParity(Sensor_ts *sensor);
uint8_t TLx493D_A2B6_calculateConfigurationParity(Sensor_ts *sensor);

bool TLx493D_A2B6_enableAngularMeasurement(Sensor_ts *sensor);
bool TLx493D_A2B6_disableAngularMeasurement(Sensor_ts *sensor);

bool TLx493D_A2B6_enableTemperatureMeasurement(Sensor_ts *sensor);
bool TLx493D_A2B6_disableTemperatureMeasurement(Sensor_ts *sensor);

bool TLx493D_A2B6_enable1ByteReadMode(Sensor_ts *sensor);
bool TLx493D_A2B6_disable1ByteReadMode(Sensor_ts *sensor);

// bool TLx493D_A2B6_enableCollisionAvoidance(Sensor_ts *sensor);
// bool TLx493D_A2B6_disableCollisionAvoidance(Sensor_ts *sensor);

bool TLx493D_A2B6_setDefaultConfig(Sensor_ts *sensor);

// utility functions
bool TLx493D_A2B6_hasValidFuseParity(Sensor_ts *sensor);
bool TLx493D_A2B6_hasValidBusParity(Sensor_ts *sensor);
bool TLx493D_A2B6_hasValidConfigurationParity(Sensor_ts *sensor);

bool TLx493D_A2B6_hasValidIICadr(Sensor_ts *sensor);
bool TLx493D_A2B6_hasWakeup(Sensor_ts *sensor);

bool TLx493D_A2B6_hasValidData(Sensor_ts *sensor);
bool TLx493D_A2B6_hasValidTemperatureData(Sensor_ts *sensor);
bool TLx493D_A2B6_hasValidMagneticFieldData(Sensor_ts *sensor);
bool TLx493D_A2B6_hasValidTBit(Sensor_ts *sensor);
bool TLx493D_A2B6_hasValidPD0Bit(Sensor_ts *sensor);
bool TLx493D_A2B6_hasValidPD3Bit(Sensor_ts *sensor);

bool TLx493D_A2B6_isFunctional(Sensor_ts *sensor);


#endif /** TLX493D_A2B6_H */