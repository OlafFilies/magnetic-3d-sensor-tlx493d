/**
 * @file TLV493D_A2BW.h
 * @author Infineon Technologies AG
 * @brief Definiton of the complete sensor functionality
 * @copyright Copyright (c) 2023 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TLV493D_A2BW_H_
#define TLV493D_A2BW_H_

/** std includes*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/** Common to all sensors */
#include "sensor_types.h"
#include "sensors_common_defines.h"
#include "sensors_common.h"

/** Common to the same generation of senors */
#include "sensors_gen_2_common_defines.h"
#include "sensors_gen_2_common.h"

/** Sensor specific includes */
#include "TLV493D_A2BW_defines.h"

/**
 * @brief Initializes the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * It initializes the sensor structure and sets the I2C communication protocol
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLV493D_A2BW_init(Sensor_ts *sensor);

/**
 * @brief De-Initializes the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * It frees the corresponding pointers of the sensor structure and sets the pointers to NULL
 *
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLV493D_A2BW_deinit(Sensor_ts *sensor);

/**
 * @brief Updates the required registers to read the temperature value of the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * It updates the required registers and then calls the getTemperature() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLV493D_A2BW_calculateTemperature(Sensor_ts *sensor, float *temp);

/**
 * @brief Retrieves the temperature value of the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * It reads out the required registers to calculate the temperature value.
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLV493D_A2BW_getTemperature(Sensor_ts *sensor, float *temp);

/**
 * @brief Updates the required registers to read the magnetic field values of the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * It updates the required registers and then calls the calculateFieldValues() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLV493D_A2BW_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

/**
 * @brief Retrieves the magnetic field values of the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * It reads out the required registers to calculate the magnetic field value for the X, Y, Z-Axis
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLV493D_A2BW_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

bool TLV493D_A2BW_getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);

bool TLV493D_A2BW_reset(Sensor_ts *sensor);
bool TLV493D_A2BW_getDiagnosis(Sensor_ts *sensor);

/**
 * @brief Calculates the parity and sets the corresponding parity flags for the necessary registers f the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLV493D_A2BW_calculateParity(Sensor_ts *sensor);

/**
 * @brief Sets the default configuration for the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * It sets the sensor into 1-Byte-Mode and also enables the temperature measurement
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful 
 */
bool TLV493D_A2BW_setDefaultConfig(Sensor_ts *sensor);

/**
 * @brief Enables the temperature measurement of the XENSIV™ TLV493D-A2BW magnetic 3D sensor 
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLV493D_A2BW_enableTemperature(Sensor_ts* sensor);

/**
 * @brief Disable the temperature measurement of the XENSIV™ TLV493D-A2BW magnetic 3D sensor 
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLV493D_A2BW_disableTemperature(Sensor_ts *sensor);

/**
 * @brief Enables the interrupt after measurement completion of the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLV493D_A2BW_enableInterrupt(Sensor_ts *sensor);

/**
 * @brief Disables the interrupt after measurement completion of the XENSIV™ TLV493D-A2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-A2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLV493D_A2BW_disableInterrupt(Sensor_ts *sensor);

#endif /** TLV493D_A2BW_H */