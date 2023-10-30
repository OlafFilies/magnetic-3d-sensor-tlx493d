/**
 * @file TLx493D_W2BW.h
 * @author Infineon Technologies AG
 * @brief Definiton of the complete sensor functionality
 * @copyright Copyright (c) 2023 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TLX493D_W2BW_H_
#define TLX493D_W2BW_H_

/** std includes*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Common to all sensors */
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

/** Common to the same generation of senors */
#include "tlx493d_gen_2_common_defines.h"
#include "tlx493d_gen_2_common.h"

/** Sensor specific includes */
#include "TLx493D_W2BW_defines.h"

/**
 * @brief Initializes the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It initializes the sensor structure and sets the I2C communication protocol
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_init(Sensor_ts *sensor);

/**
 * @brief De-Initializes the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It frees the corresponding pointers of the sensor structure and sets the pointers to NULL
 *
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_deinit(Sensor_ts *sensor);

/**
 * @brief Updates the required registers to read the temperature value of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It updates the required registers and then calls the getTemperature() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLx493D_W2BW_calculateTemperature(Sensor_ts *sensor, double *temp);

/**
 * @brief Retrieves the temperature value of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It reads out the required registers to calculate the temperature value.
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_getTemperature(Sensor_ts *sensor, double *temp);

/**
 * @brief Updates the required registers to read the magnetic field values of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It updates the required registers and then calls the calculateFieldValues() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLx493D_W2BW_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);

/**
 * @brief Retrieves the magnetic field values of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It reads out the required registers to calculate the magnetic field value for the X, Y, Z-Axis
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);

// bool TLx493D_W2BW_getSensorValues(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);

uint8_t TLx493D_W2BW_calculateConfigurationParityBit(Sensor_ts *sensor);

bool TLx493D_W2BW_reset(Sensor_ts *sensor);
bool TLx493D_W2BW_getDiagnosis(Sensor_ts *sensor);

/**
 * @brief Calculates the parity and sets the corresponding parity flags for the necessary registers f the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLx493D_W2BW_calculateParity(Sensor_ts *sensor);

/**
 * @brief Sets the default configuration for the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It sets the sensor into 1-Byte-Mode and also enables the temperature measurement
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful 
 */
bool TLx493D_W2BW_setDefaultConfig(Sensor_ts *sensor);


bool TLx493D_W2BW_setPowerMode(Sensor_ts *sensor, uint8_t mode);
bool TLx493D_W2BW_setIICAddress(Sensor_ts *sensor, StandardIICAddresses_te address);
bool TLx493D_W2BW_setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits);

// /**
//  * @brief 
//  * 
//  * @param sensor 
//  * @param tx_buffer 
//  * @param tx_len 
//  * @param rx_buffer 
//  * @param rx_len 
//  * @return true 
//  * @return false 
//  */
// bool TLx493D_W2BW_transferRegisterMap(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len);

/**
 * @brief Enables the temperature measurement of the XENSIV™ TLx493D-W2BW magnetic 3D sensor 
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_enableTemperatureMeasurement(Sensor_ts* sensor);

/**
 * @brief Disable the temperature measurement of the XENSIV™ TLx493D-W2BW magnetic 3D sensor 
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_disableTemperatureMeasurement(Sensor_ts *sensor);

/**
 * @brief Enables the interrupt after measurement completion of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_enableInterrupt(Sensor_ts *sensor);

/**
 * @brief Disables the interrupt after measurement completion of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_disableInterrupt(Sensor_ts *sensor);


/**
 * @brief Sets the update rate for the measurements of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in] bit Value which is set to the PRD-Bitfield
 * @return true - If successful
 * @return false - If unsuccessful 
 */
bool TLx493D_W2BW_setUpdateRate(Sensor_ts *sensor, uint8_t bit);

/**
 * @brief Checks if the wake up functionality is enabled
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If wake up is enabled
 * @return false - If wake up is disabled
 */
bool TLx493D_W2BW_isWakeUpActive(Sensor_ts *sensor);

bool TLx493D_W2BW_enableWakeUpMode(Sensor_ts *sensor);
bool TLx493D_W2BW_disableWakeUpMode(Sensor_ts *sensor);

bool TLx493D_W2BW_setLowerWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold);
bool TLx493D_W2BW_setLowerWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold); 
bool TLx493D_W2BW_setLowerWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold);

bool TLx493D_W2BW_setUpperWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold);
bool TLx493D_W2BW_setUpperWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold);
bool TLx493D_W2BW_setUpperWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold);

bool TLx493D_W2BW_setWakeUpThresholds(Sensor_ts *sensor, int16_t xh_th, int16_t xl_th, int16_t yh_th, int16_t yl_th, int16_t zh_th, int16_t zl_th);


#endif /** TLX493D_W2BW_H */