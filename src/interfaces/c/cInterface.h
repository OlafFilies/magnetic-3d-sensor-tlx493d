#ifndef C_INTERFACE_H
#define C_INTERFACE_H

// project c includes
// common to all sensors
#include "sensor_types.h"


// functions common to all sensors
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

bool hasValidData(Sensor_ts *sensor);
bool hasValidDataTemperatureData(Sensor_ts *sensor);
bool hasValidDataMagneticFieldData(Sensor_ts *sensor);

bool isFunctional(Sensor_ts *sensor);

// bool selectMeasuredValues(Sensor_ts *sensor, enum <possible combinations> mVals); // Bx/By/Bz, Bx/By, Bx/By/Temp, ...
bool enableTemperatureMeasurement(Sensor_ts *sensor);
bool disableTemperatureMeasurement(Sensor_ts *sensor);

bool enableAngularMeasurement(Sensor_ts *sensor);
bool disableAngularMeasurement(Sensor_ts *sensor);

// value of update rate is sensor / generation specific !
// bool setUpdateRate(Sensor_ts *sensor, enum <possible combinations> rate);
bool setUpdateRate(Sensor_ts* sensor, uint8_t bit);

// bool setSensorRange(Sensor_ts *sensor, enum <possible combinations> range); // full, short, extreme short : whatever is supported

// bool setInterruptAndCollisionAvoidance(Sensor_ts *sensor, enum <possible combinations> eVal);
bool enableInterrupt(Sensor_ts *sensor);
bool disableInterrupt(Sensor_ts *sensor);

bool enableWakeup(Sensor_ts *sensor);
bool disableWakeup(Sensor_ts *sensor);
// thesholds im mT, to be converted to proper format
bool setWakeupThesholds(Sensor_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);

bool softReset(Sensor_ts *sensor);

// Severin : nice to have, set proper defaults
// set register bits
// bool setTrigger(Sensor_ts *sensor, uint8_t trigger);
// trigger bits shall be ORed to register address always by shifting left by 5 -> default is 0b000
// bool setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits);


// utilities
const char *getTypeAsString(SupportedSensorTypes_te sensorType);


// functions available only to a subset of sensors


#endif // C_INTERFACE_H
