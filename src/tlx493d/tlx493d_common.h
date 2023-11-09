#ifndef TLX493D_COMMON_H
#define TLX493D_COMMON_H

// project c includes
// common to all sensors
#include "tlx493d_types.h"


// /**
//  * @brief Initializes the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It initializes the sensor structure and sets the I2C communication protocol
//  *
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
bool tlx493d_common_init(TLx493D_ts *sensor, uint8_t regMapSize, TLx493D_Register_ts *regDef, TLx493D_CommonFunctions_ts *commonFuncs, TLx493D_SupportedSensorType_te sensorType, TLx493D_SupportedComLibraryInterfaceType_te comIFType);


// /**
//  * @brief De-Initializes the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It frees the corresponding pointers of the sensor structure and sets the pointers to NULL
//  *
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
bool tlx493d_common_deinit(TLx493D_ts *sensor);


void tlx493d_common_getBitfield(TLx493D_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
void tlx493d_common_setBitfield(TLx493D_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool tlx493d_common_writeRegister(TLx493D_ts* sensor, uint8_t bitField);
bool tlx493d_common_readRegisters(TLx493D_ts *sensor);


// /**
//  * @brief Retrieves the temperature value of the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It reads out the required registers to calculate the temperature value.
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @param[in, out] temp Retrieved temperature value of the sensor
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
bool tlx493d_common_getTemperature(TLx493D_ts *sensor, double *temp);


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
bool tlx493d_common_getMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z);


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
bool tlx493d_common_getMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp);


// utilities
uint8_t tlx493d_common_calculateParity(uint8_t data);
uint8_t tlx493d_common_getOddParity(uint8_t parity);
uint8_t tlx493d_common_getEvenParity(uint8_t parity);

void tlx493d_common_concatBytes(TLx493D_ts *sensor, uint8_t msbBitfield, uint8_t lsbBitfield, int16_t *result);

const char *tlx493d_common_getTypeAsString(TLx493D_ts *sensor);

void warnFeatureNotAvailableForSensorType(TLx493D_ts *sensor, const char *featureName);
void warnSelectionNotSupportedForSensorType(TLx493D_ts *sensor, uint8_t sel, const char *selType);


#endif // TLX493D_COMMON_H
