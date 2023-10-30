#ifndef TLX493D_COMMON_H
#define TLX493D_COMMON_H

// project c includes
// common to all sensors
#include "tlx493d_types.h"

// functions common to all sensors
bool tlx493d_init(Sensor_ts *sensor, uint8_t regMapSize, Register_ts *regDef, CommonFunctions_ts *commonFuncs, SupportedSensorTypes_te sensorType, SupportedComLibraryInterfaceTypes_te comIFType);
bool tlx493d_deinit(Sensor_ts *sensor);

void tlx493d_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
void tlx493d_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool tlx493d_writeRegister(Sensor_ts* sensor, uint8_t bitField);

bool tlx493d_readRegisters(Sensor_ts *sensor);
// bool tlx493d_setDefaultConfig(Sensor_ts *sensor);
// bool tlx493d_setPowerMode(Sensor_ts *sensor, uint8_t mode);
// bool tlx493d_setIICAddress(Sensor_ts *sensor, uint8_t addr);

bool tlx493d_getTemperature(Sensor_ts *sensor, double *temp);
bool tlx493d_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);
bool tlx493d_getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);

// // bool tlx493d_selectMeasuredValues(Sensor_ts *sensor, enum <possible combinations> mVals); // Bx/By/Bz, Bx/By, Bx/By/Temp, ...
// // or do it separately ? Like :
// bool tlx493d_enableTemperatureMeasurement(Sensor_ts *sensor);
// bool tlx493d_disableTemperatureMeasurement(Sensor_ts *sensor);
// bool tlx493d_enableAngularMeasurement(Sensor_ts *sensor);
// bool tlx493d_disableAngularMeasurement(Sensor_ts *sensor);

// // value of update rate is sensor / generation specific !
// // bool tlx493d_setUpdateRate(Sensor_ts *sensor, enum <possible combinations> rate);
// bool tlx493d_setUpdateRate(Sensor_ts *sensor, uint8_t bit);

// // bool tlx493d_setRange(Sensor_ts *sensor, enum <possible combinations> range); // full, short, extreme short : whatever is supported

// // bool tlx493d_setInterruptAndCollisionAvoidance(Sensor_ts *sensor, enum <possible combinations> eVal);
// bool tlx493d_enableInterrupt(Sensor_ts *sensor);
// bool tlx493d_disableInterrupt(Sensor_ts *sensor);
// // bool tlx493d_enableCollisionAvoidance(Sensor_ts *sensor);
// // bool tlx493d_disableCollisionAvoidance(Sensor_ts *sensor);


// // set register bits
// // bool tlx493d_setTrigger(Sensor_ts *sensor, uint8_t trigger);
// // trigger bits shall be ORed to register address always by shifting left by 5 -> default is 0b000
// bool tlx493d_setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits);


// bool tlx493d_isWakeUpActive(Sensor_ts *sensor);
// bool tlx493d_enableWakeUpMode(Sensor_ts *sensor);
// bool tlx493d_disableWakeUpMode(Sensor_ts *sensor);
// bool tlx493d_enableWakeup(Sensor_ts *sensor);
// bool tlx493d_disableWakeup(Sensor_ts *sensor);

// bool tlx493d_setLowerWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold);
// bool tlx493d_setLowerWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold);
// bool tlx493d_setLowerWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold);

// bool tlx493d_setUpperWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold);
// bool tlx493d_setUpperWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold);
// bool tlx493d_setUpperWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold);

// bool tlx493d_setWakeUpThresholds(Sensor_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);
// // thesholds im mT, to be converted to proper format
// bool tlx493d_setWakeupThesholds(Sensor_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);

// bool tlx493d_softReset(Sensor_ts *sensor) {
//    return sensor->functions->softReset(sensor);
// }


// utilities
uint8_t tlx493d_calculateParity(uint8_t data);
uint8_t tlx493d_getOddParity(uint8_t parity);
uint8_t tlx493d_getEvenParity(uint8_t parity);

void tlx493d_concatBytes(Sensor_ts *sensor, uint8_t msbBitfield, uint8_t lsbBitfield, int16_t *result);


// functions available only to a subset of sensors


#endif // TLX493D_COMMON_H
