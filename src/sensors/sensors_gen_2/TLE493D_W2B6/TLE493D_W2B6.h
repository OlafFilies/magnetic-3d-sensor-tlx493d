/**
 * @file TLV493D_W2B6.h
 * @author Infineon Technologies AG
 * @brief Definiton of the complete sensor functionality
 * @copyright Copyright (c) 2023 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TLE493D_W2B6_H
#define TLE493D_W2B6_H

/** std includes*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_config_common.h"
#include "sensors_common.h"

/** Common to the same generation of senors */
#include "sensors_gen_2_config_common.h"
#include "sensors_gen_2_common.h"
#include "sensors_gen_2_utils.h"

// sensor specicifc includes
#include "TLE493D_W2B6_config.h"

/**
 * @brief Initializes the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * It initializes the sensor structure and checks if the right communication protocol is provided
 *
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @param[in] comLibIF Communication interface type of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_init(Sensor_ts *sensor);

/**
 * @brief De-Initializes the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * It frees the corresponding pointers of the sensor structure and sets the pointers to NULL
 *
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_deinit(Sensor_ts *sensor);
/**
 * @brief Retrieves the temperature value of the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * It reads out the required registers to calculate the temperature value.
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_getTemperature(Sensor_ts *sensor, float *temp);

/**
 * @brief Updates the required registers to read the temperature value of the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * It updates the required registers and then calls the getTemperature() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_updateGetTemperature(Sensor_ts *sensor, float *temp);

/**
 * @brief Retrieves the magnetic field values of the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * It reads out the required registers to calculate the magnetic field value for the X, Y, Z-Axis
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

/**
 * @brief Updates the required registers to read the magnetic field values of the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * It updates the required registers and then calls the getFieldValues() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

/**
 * @brief Sets the default configuration for the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * It sets the sensor into 1-Byte-Mode and also enables the temperature measurement
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful 
 */

bool TLE493D_W2B6_setDefaultConfig(Sensor_ts *sensor);

/**
 * @brief Calculates the parity and sets the corresponding parity flags for the necessary registers f the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLE493D_W2B6_calculateParity(Sensor_ts *sensor);

/**
 * @brief Updates all registers of the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_updateRegisterMap(Sensor_ts *sensor);

/**
 * @brief Enables the temperature measurement of the XENSIV™ TLE493D-W2B6 magnetic 3D sensor 
 * 
 * @param[in] sensor Pointer to the XENSIV™ TEV493D-W2B6 magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_enableTemperature(Sensor_ts* sensor);

/**
 * @brief Disable the temperature measurement of the XENSIV™ TLE493D-W2B6 magnetic 3D sensor 
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLE493D-W2B6 magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_disableTemperature(Sensor_ts *sensor);

#endif // TLE493D_W2B6_H
