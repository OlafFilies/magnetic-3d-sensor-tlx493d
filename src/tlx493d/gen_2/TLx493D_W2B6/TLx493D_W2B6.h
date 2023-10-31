/**
 * @file TLV493D_W2B6.h
 * @author Infineon Technologies AG
 * @brief Definiton of the complete sensor functionality
 * @copyright Copyright (c) 2023 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TLX493D_W2B6_H
#define TLX493D_W2B6_H


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


bool TLx493D_W2B6_init(TLx493D_ts *sensor);
bool TLx493D_W2B6_deinit(TLx493D_ts *sensor);

bool TLx493D_W2B6_setPowerMode(TLx493D_ts *sensor, uint8_t mode);
bool TLx493D_W2B6_setIICAddress(TLx493D_ts *sensor, TLx493D_IICAddresses_te address);

void TLx493D_W2B6_calculateTemperature(TLx493D_ts *sensor, double *temp);
bool TLx493D_W2B6_getTemperature(TLx493D_ts *sensor, double *temp);

void TLx493D_W2B6_calculateMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z);
bool TLx493D_W2B6_getMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z);

void TLx493D_W2B6_calculateMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_W2B6_getMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp);

uint8_t TLx493D_W2B6_calculateFuseParity(TLx493D_ts *sensor);
uint8_t TLx493D_W2B6_calculateBusParity(TLx493D_ts *sensor);
uint8_t TLx493D_W2B6_calculateConfigurationParity(TLx493D_ts *sensor);

bool TLx493D_W2B6_enableAngularMeasurement(TLx493D_ts *sensor);
bool TLx493D_W2B6_disableAngularMeasurement(TLx493D_ts *sensor);

bool TLx493D_W2B6_enableTemperatureMeasurement(TLx493D_ts *sensor);
bool TLx493D_W2B6_disableTemperatureMeasurement(TLx493D_ts *sensor);

bool TLx493D_W2B6_enable1ByteReadMode(TLx493D_ts *sensor);
bool TLx493D_W2B6_disable1ByteReadMode(TLx493D_ts *sensor);

// bool TLx493D_W2B6_enableCollisionAvoidance(TLx493D_ts *sensor);
// bool TLx493D_W2B6_disableCollisionAvoidance(TLx493D_ts *sensor);

bool TLx493D_W2B6_setDefaultConfig(TLx493D_ts *sensor);

// utility functions
bool TLx493D_W2B6_hasValidFuseParity(TLx493D_ts *sensor);
bool TLx493D_W2B6_hasValidBusParity(TLx493D_ts *sensor);
bool TLx493D_W2B6_hasValidConfigurationParity(TLx493D_ts *sensor);

bool TLx493D_W2B6_hasValidIICadr(TLx493D_ts *sensor);
bool TLx493D_W2B6_hasWakeup(TLx493D_ts *sensor);

bool TLx493D_W2B6_hasValidData(TLx493D_ts *sensor);
bool TLx493D_W2B6_hasValidTemperatureData(TLx493D_ts *sensor);
bool TLx493D_W2B6_hasValidMagneticFieldData(TLx493D_ts *sensor);
bool TLx493D_W2B6_hasValidTBit(TLx493D_ts *sensor);
bool TLx493D_W2B6_hasValidPD0Bit(TLx493D_ts *sensor);
bool TLx493D_W2B6_hasValidPD3Bit(TLx493D_ts *sensor);

bool TLx493D_W2B6_isFunctional(TLx493D_ts *sensor);


#endif // TLX493D_W2B6_H
