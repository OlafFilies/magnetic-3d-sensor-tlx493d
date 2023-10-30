#ifndef C_INTERFACE_H
#define C_INTERFACE_H

// project c includes
// common to all sensors
#include "tlx493d_types.h"


#ifdef __cplusplus

extern "C" {

#endif


bool init(Sensor_ts *sensor, SupportedSensorTypes_te sensorType);
bool deinit(Sensor_ts *sensor);

bool readRegisters(Sensor_ts *sensor);
bool setDefaultConfig(Sensor_ts *sensor);

bool setPowerMode(Sensor_ts *sensor, uint8_t mode);
// bool setPowerMode(Sensor_ts *sensor, enum <possible combinations> mode);  // value of mode is sensor / generation specific !

bool setIICAddress(Sensor_ts *sensor, uint8_t addr);
// bool setIICAddress(Sensor_ts *sensor, enum <possible combinations> addr); // Gen. 1 and 2

bool getTemperature(Sensor_ts *sensor, double *temp);
bool getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);
bool getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);

// bool selectMeasuredValues(Sensor_ts *sensor, enum <possible combinations> mVals); // Bx/By/Bz, Bx/By, Bx/By/Temp, ...
// or do it separately ? Like :
bool enableTemperatureMeasurement(Sensor_ts *sensor);
bool disableTemperatureMeasurement(Sensor_ts *sensor);
bool enableAngularMeasurement(Sensor_ts *sensor);
bool disableAngularMeasurement(Sensor_ts *sensor);

// value of update rate is sensor / generation specific !
// bool setUpdateRate(Sensor_ts *sensor, enum <possible combinations> rate);
bool setUpdateRate(Sensor_ts* sensor, uint8_t bit);


// bool setRange(Sensor_ts *sensor, enum <possible combinations> range); // full, short, extreme short : whatever is supported

// bool setInterruptAndCollisionAvoidance(Sensor_ts *sensor, enum <possible combinations> eVal);
bool enableInterrupt(Sensor_ts *sensor);
bool disableInterrupt(Sensor_ts *sensor);
// bool enableCollisionAvoidance(Sensor_ts *sensor);
// bool disableCollisionAvoidance(Sensor_ts *sensor);

bool isWakeUpActive(Sensor_ts *sensor);
bool enableWakeUpMode(Sensor_ts *sensor);
bool disableWakeUpMode(Sensor_ts *sensor);

bool setLowerWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold);
bool setLowerWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold);
bool setLowerWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold);

bool setUpperWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold);
bool setUpperWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold);
bool setUpperWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold);

bool setWakeUpThresholds(Sensor_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);
// thesholds im mT, to be converted to proper format
bool setWakeupThesholds(Sensor_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);

bool softReset(Sensor_ts *sensor);

// Severin : nice to have, set proper defaults
// set register bits
// bool setTrigger(Sensor_ts *sensor, uint8_t trigger);
// trigger bits shall be ORed to register address always by shifting left by 5 -> default is 0b000
// bool setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits);

// diagnosis functions
bool hasValidData(Sensor_ts *sensor);
bool hasValidTemperatureData(Sensor_ts *sensor);
bool hasValidMagneticFieldData(Sensor_ts *sensor);

bool isFunctional(Sensor_ts *sensor);


// utilities
const char *getTypeAsString(SupportedSensorTypes_te sensorType);


// functions available only to a subset of sensors


#ifdef __cplusplus

}

#endif


#endif // C_INTERFACE_H
