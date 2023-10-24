#ifndef SENSORS_COMMON_H
#define SENSORS_COMMON_H

// project c includes
// common to all sensors
#include "sensor_types.h"

// functions common to all sensors
bool init(Sensor_ts *sensor, SupportedSensorTypes_te sensorType);
bool initSensor(Sensor_ts *sensor, uint8_t regMapSize, Register_ts *regDef, CommonFunctions_ts *commonFuncs, SupportedSensorTypes_te sensorType, SupportedComLibraryInterfaceTypes_te comIFType);

bool deinit(Sensor_ts *sensor);
bool deinitSensor(Sensor_ts *sensor);

// bool getSensorTemperature(Sensor_ts *sensor, double *temp);
// bool getSensorMagneticField(Sensor_ts *sensor, double *x, double *y, double *z );
// bool getSensorMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);

// // void calculateTemperature(Sensor_ts *sensor, float *temp);
bool getTemperature(Sensor_ts *sensor, double *temp);

// // void calculateMagneticField(Sensor_ts *sensor, float *x, float *y, float *z);
bool getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);

bool getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);


bool hasValidData(Sensor_ts *sensor);

bool hasValidTemperatureData(Sensor_ts *sensor);
bool hasValidMagneticFieldData(Sensor_ts *sensor);

bool isFunctional(Sensor_ts *sensor);


bool setDefaultConfig(Sensor_ts *sensor);
bool readRegisters(Sensor_ts *sensor);

// bool enableTemperatureMeasurements(Sensor_ts *sensor);
// bool disableTemperatureMeasurements(Sensor_ts *sensor);
bool enableTemperatureMeasurement(Sensor_ts *sensor);
bool disableTemperatureMeasurement(Sensor_ts *sensor);

bool enableInterrupt(Sensor_ts *sensor);
bool disableInterrupt(Sensor_ts *sensor);

bool setPowerMode(Sensor_ts *sensor, uint8_t mode);

bool setIICAddress(Sensor_ts *sensor, uint8_t addr);

bool enableAngularMeasurement(Sensor_ts *sensor);
bool disableAngularMeasurement(Sensor_ts *sensor);

bool setTriggerBits(Sensor_ts* sensor, uint8_t bits);
bool setUpdateRate(Sensor_ts* sensor, uint8_t bit);

// utilities
const char *getTypeAsString(SupportedSensorTypes_te sensorType);

uint8_t calculateParity(uint8_t data);
uint8_t getOddParity(uint8_t parity);
uint8_t getEvenParity(uint8_t parity);

// void concatBytes(Sensor_ts *sensor, Register_ts *msb, Register_ts *lsb, int16_t *result);
void concatBytes(Sensor_ts *sensor, uint8_t msbBitfield, uint8_t lsbBitfield, int16_t *result);


// functions available only to a subset of sensors


#endif // SENSORS_COMMON_H
