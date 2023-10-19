/**
 * @file TLV493D_P3B6.h
 * @author Infineon Technologies AG
 * @brief Definiton of the complete sensor functionality
 * @copyright Copyright (c) 2023 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TLE493D_P3B6_H
#define TLE493D_P3B6_H

/** std includes*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
// #include "sensors_common_defines.h"
#include "sensors_common.h"

/** Common to the same generation of senors */
// #include "sensors_gen_3_common_defines.h"
#include "sensors_gen_3_common.h"

// sensor specicifc includes
// #include "TLE493D_P3B6_defines.h"


/**
 * @brief Initializes the XENSIV™ TLE493D-P3B6 magnetic 3D sensor
 * It initializes the sensor structure and sets the I2C communication protocol
 *
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-P3B6 magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_P3B6_init(Sensor_ts *sensor);


/**
 * @brief De-Initializes the XENSIV™ TLE493D-P3B6 magnetic 3D sensor
 * It frees the corresponding pointers of the sensor structure and sets the pointers to NULL
 *
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-P3B6 magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_P3B6_deinit(Sensor_ts *sensor);


/**
 * @brief Retrieves the temperature value of the XENSIV™ TLE493D-P3B6 magnetic 3D sensor
 * It reads out the required registers to calculate the temperature value.
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-P3B6 magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_P3B6_getTemperature(Sensor_ts *sensor, float *temp);


/**
 * @brief Retrieves the magnetic field values of the XENSIV™ TLE493D-P3B6 magnetic 3D sensor
 * It reads out the required registers to calculate the magnetic field value for the X, Y, Z-Axis
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-P3B6 magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLE493D_P3B6_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

/**
 * @brief Updates the required registers to read the magnetic field values of the XENSIV™ TLE493D-P3B6 magnetic 3D sensor
 * It updates the required registers and then calls the calculateFieldValues() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-P3B6 magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_P3B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

/**
 * @brief Sets the default configuration for the XENSIV™ TLE493D-P3B6 magnetic 3D sensor
 * It sets the sensor into 1-Byte-Mode and also enables the temperature measurement
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-P3B6 magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful 
 */
bool TLE493D_P3B6_setDefaultConfig(Sensor_ts *sensor);


// /**
//  * @brief Updates all registers of the XENSIV™ TLE493D-P3B6 magnetic 3D sensor
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLE493D-P3B6 magnetic 3D sensor structure
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
// bool TLE493D_P3B6_updateRegisterMap(Sensor_ts *sensor);

#endif // TLE493D_P3B6_H
