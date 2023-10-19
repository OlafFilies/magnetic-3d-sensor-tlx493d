#ifndef SENSORS_COMMON_H
#define SENSORS_COMMON_H

// project c includes
// common to all sensors
#include "sensor_types.h"

// functions common to all sensors
bool init(Sensor_ts *sensor, SupportedSensorTypes_te sensorType);
bool deinit(Sensor_ts *sensor);

// void calculateTemperature(Sensor_ts *sensor, float *temp);
bool getTemperature(Sensor_ts *sensor, float *temp);

// void calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
bool getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

// bool getFieldAndTemperatureValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);
bool getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);


bool hasValidData(Sensor_ts *sensor);

bool hasValidDataTemperatureData(Sensor_ts *sensor);
bool hasValidDataFieldValueData(Sensor_ts *sensor);

bool isFunctional(Sensor_ts *sensor);


bool setDefaultConfig(Sensor_ts *sensor);
bool readRegisters(Sensor_ts *sensor);

// bool enableTemperatureMeasurements(Sensor_ts *sensor);
// bool disableTemperatureMeasurements(Sensor_ts *sensor);
bool enableTemperature(Sensor_ts *sensor);
bool disableTemperature(Sensor_ts *sensor);

bool enableInterrupt(Sensor_ts *sensor);
bool disableInterrupt(Sensor_ts *sensor);

bool setPowerMode(Sensor_ts *sensor, uint8_t mode);

bool setIICAddress(Sensor_ts *sensor, uint8_t addr);

// void setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits);


// utilities
const char *getTypeAsString(SupportedSensorTypes_te sensorType);

uint8_t calculateParity(uint8_t data);
uint8_t getOddParity(uint8_t parity);
uint8_t getEvenParity(uint8_t parity);

void concatBytes(Sensor_ts *sensor, Register_ts *msb, Register_ts *lsb, int16_t *result);


// functions available only to a subset of sensors


#endif // SENSORS_COMMON_H
