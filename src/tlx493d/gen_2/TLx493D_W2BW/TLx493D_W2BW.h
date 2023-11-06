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
#include <stdbool.h>
#include <stdint.h>

/** Common to all sensors */
#include "tlx493d_types.h"
#include "tlx493d_common.h"

/** Common to the same generation of senors */
#include "tlx493d_gen_2_common.h"

/** Sensor specific includes */


/**
 * @brief Initializes the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It initializes the sensor structure and sets the I2C communication protocol
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_init(TLx493D_ts *sensor);

/**
 * @brief De-Initializes the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It frees the corresponding pointers of the sensor structure and sets the pointers to NULL
 *
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_deinit(TLx493D_ts *sensor);

/**
 * @brief Updates the required registers to read the temperature value of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It updates the required registers and then calls the getTemperature() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLx493D_W2BW_calculateTemperature(TLx493D_ts *sensor, double *temp);

/**
 * @brief Retrieves the temperature value of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It reads out the required registers to calculate the temperature value.
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_getTemperature(TLx493D_ts *sensor, double *temp);

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
void TLx493D_W2BW_calculateMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z);

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
bool TLx493D_W2BW_getMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z);


void TLx493D_W2BW_calculateMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_W2BW_getMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp);

bool TLx493D_W2BW_setTrigger(TLx493D_ts *sensor, uint8_t triggerBits);


/**
 * @brief Sets the default configuration for the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It sets the sensor into 1-Byte-Mode and also enables the temperature measurement
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful 
 */
bool TLx493D_W2BW_setDefaultConfig(TLx493D_ts *sensor);
bool TLx493D_W2BW_setIICAddress(TLx493D_ts *sensor, TLx493D_IICAddressType_te address);

/**
 * @brief Enables the interrupt after measurement completion of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_enableInterrupt(TLx493D_ts *sensor);

/**
 * @brief Disables the interrupt after measurement completion of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_disableInterrupt(TLx493D_ts *sensor);


bool TLx493D_W2BW_setPowerMode(TLx493D_ts *sensor, uint8_t mode);


/**
 * @brief Sets the update rate for the measurements of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in] bit Value which is set to the PRD-Bitfield
 * @return true - If successful
 * @return false - If unsuccessful 
 */
bool TLx493D_W2BW_setUpdateRate(TLx493D_ts *sensor, uint8_t bit);


bool TLx493D_W2BW_hasValidData(TLx493D_ts *sensor);
bool TLx493D_W2BW_isFunctional(TLx493D_ts *sensor);

/**
 * @brief Checks if the wake up functionality is enabled
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If wake up is enabled
 * @return false - If wake up is disabled
 */
bool TLx493D_W2BW_isWakeUpActive(TLx493D_ts *sensor);

bool TLx493D_W2BW_enableWakeUpMode(TLx493D_ts *sensor);
bool TLx493D_W2BW_disableWakeUpMode(TLx493D_ts *sensor);

bool TLx493D_W2BW_setLowerWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold);
bool TLx493D_W2BW_setLowerWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold); 
bool TLx493D_W2BW_setLowerWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold);

bool TLx493D_W2BW_setUpperWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold);
bool TLx493D_W2BW_setUpperWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold);
bool TLx493D_W2BW_setUpperWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold);

bool TLx493D_W2BW_setWakeUpThresholdsAsInteger(TLx493D_ts *sensor, int16_t xh_th, int16_t xl_th, int16_t yh_th, int16_t yl_th, int16_t zh_th, int16_t zl_th);
// thesholds im mT, to be converted to proper format
bool TLx493D_W2BW_setWakeUpThresholds(TLx493D_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);



bool TLx493D_W2BW_softReset(TLx493D_ts *sensor);


/**
 * @brief Enables the temperature measurement of the XENSIV™ TLx493D-W2BW magnetic 3D sensor 
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_enableTemperatureMeasurement(TLx493D_ts* sensor);

/**
 * @brief Disable the temperature measurement of the XENSIV™ TLx493D-W2BW magnetic 3D sensor 
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_disableTemperatureMeasurement(TLx493D_ts *sensor);


bool TLx493D_W2BW_enable1ByteMode(TLx493D_ts *sensor);

// /**
//  * @brief Calculates the parity and sets the corresponding parity flags for the necessary registers f the XENSIV™ TLx493D-W2BW magnetic 3D sensor
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
// void TLx493D_W2BW_calculateParity(TLx493D_ts *sensor);
uint8_t TLx493D_W2BW_calculateConfigurationParityBit(TLx493D_ts *sensor);

void TLx493D_W2BW_setResetValues(TLx493D_ts *sensor);


#endif /** TLX493D_W2BW_H */