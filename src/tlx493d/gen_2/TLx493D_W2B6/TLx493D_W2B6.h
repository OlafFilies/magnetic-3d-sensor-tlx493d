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
// #include "TLx493D_W2B6_defines.h"


// /**
//  * @brief Initializes the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It initializes the sensor structure and sets the I2C communication protocol
//  *
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
// bool TLx493D_W2B6_init(Sensor_ts *sensor);


// /**
//  * @brief De-Initializes the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It frees the corresponding pointers of the sensor structure and sets the pointers to NULL
//  *
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
// bool TLx493D_W2B6_deinit(Sensor_ts *sensor);


// /**
//  * @brief Retrieves the temperature value of the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It reads out the required registers to calculate the temperature value.
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @param[in, out] temp Retrieved temperature value of the sensor
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
// bool TLx493D_W2B6_getTemperature(Sensor_ts *sensor, double *temp);


// /**
//  * @brief Retrieves the magnetic field values of the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It reads out the required registers to calculate the magnetic field value for the X, Y, Z-Axis
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @param[in, out] x Retrieved magnetic value of the X-Axis
//  * @param[in, out] y Retrieved magnetic value of the Y-Axis
//  * @param[in, out] z Retrieved magnetic value of the Z-Axis
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
// void TLx493D_W2B6_calculateFieldValues(Sensor_ts *sensor, double *x, double *y, double *z);

// /**
//  * @brief Updates the required registers to read the magnetic field values of the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It updates the required registers and then calls the calculateFieldValues() function of the sensor
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @param[in, out] x Retrieved magnetic value of the X-Axis
//  * @param[in, out] y Retrieved magnetic value of the Y-Axis
//  * @param[in, out] z Retrieved magnetic value of the Z-Axis
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
// bool TLx493D_W2B6_getFieldValues(Sensor_ts *sensor, double *x, double *y, double *z);

// /**
//  * @brief Sets the default configuration for the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It sets the sensor into 1-Byte-Mode and also enables the temperature measurement
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @return true - If successful
//  * @return false - If unsuccessful 
//  */
// bool TLx493D_W2B6_setDefaultConfig(Sensor_ts *sensor);


// // /**
// //  * @brief Updates all registers of the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
// //  * 
// //  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
// //  * @return true - If successful
// //  * @return false - If unsuccessful
// //  */
// // bool TLx493D_W2B6_updateRegisterMap(Sensor_ts *sensor);


bool TLx493D_W2B6_init(Sensor_ts *sensor);
bool TLx493D_W2B6_deinit(Sensor_ts *sensor);

bool TLx493D_W2B6_setPowerMode(Sensor_ts *sensor, uint8_t mode);
bool TLx493D_W2B6_setIICAddress(Sensor_ts *sensor, StandardIICAddresses_te address);

void TLx493D_W2B6_calculateTemperature(Sensor_ts *sensor, double *temp);
bool TLx493D_W2B6_getTemperature(Sensor_ts *sensor, double *temp);

void TLx493D_W2B6_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);
bool TLx493D_W2B6_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);

void TLx493D_W2B6_calculateMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_W2B6_getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);

uint8_t TLx493D_W2B6_calculateFuseParity(Sensor_ts *sensor);
uint8_t TLx493D_W2B6_calculateBusParity(Sensor_ts *sensor);
uint8_t TLx493D_W2B6_calculateConfigurationParity(Sensor_ts *sensor);

bool TLx493D_W2B6_enableAngularMeasurement(Sensor_ts *sensor);
bool TLx493D_W2B6_disableAngularMeasurement(Sensor_ts *sensor);

bool TLx493D_W2B6_enableTemperatureMeasurement(Sensor_ts *sensor);
bool TLx493D_W2B6_disableTemperatureMeasurement(Sensor_ts *sensor);

bool TLx493D_W2B6_enable1ByteReadMode(Sensor_ts *sensor);
bool TLx493D_W2B6_disable1ByteReadMode(Sensor_ts *sensor);

// bool TLx493D_W2B6_enableCollisionAvoidance(Sensor_ts *sensor);
// bool TLx493D_W2B6_disableCollisionAvoidance(Sensor_ts *sensor);

bool TLx493D_W2B6_setDefaultConfig(Sensor_ts *sensor);

// utility functions
bool TLx493D_W2B6_hasValidFuseParity(Sensor_ts *sensor);
bool TLx493D_W2B6_hasValidBusParity(Sensor_ts *sensor);
bool TLx493D_W2B6_hasValidConfigurationParity(Sensor_ts *sensor);

bool TLx493D_W2B6_hasValidIICadr(Sensor_ts *sensor);
bool TLx493D_W2B6_hasWakeup(Sensor_ts *sensor);

bool TLx493D_W2B6_hasValidData(Sensor_ts *sensor);
bool TLx493D_W2B6_hasValidTemperatureData(Sensor_ts *sensor);
bool TLx493D_W2B6_hasValidMagneticFieldData(Sensor_ts *sensor);
bool TLx493D_W2B6_hasValidTBit(Sensor_ts *sensor);
bool TLx493D_W2B6_hasValidPD0Bit(Sensor_ts *sensor);
bool TLx493D_W2B6_hasValidPD3Bit(Sensor_ts *sensor);

bool TLx493D_W2B6_isFunctional(Sensor_ts *sensor);


#endif // TLX493D_W2B6_H
