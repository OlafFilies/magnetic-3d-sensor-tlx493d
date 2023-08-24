#ifndef SENSORS_COMMON_H
#define SENSORS_COMMON_H


// project c includes
// common to all sensors
#include "sensor_types.h"


// functions common to all sensors
bool init(Sensor_ts *sensor, SupportedSensorTypes_te sensorType);
bool deinit(Sensor_ts *sensor);

void calculateTemperature(Sensor_ts *sensor, float *temp);
bool getTemperature(Sensor_ts *sensor, float *temp);

void calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
bool getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

bool getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);

// bool reset(Sensor_ts *sensor);

bool hasValidData(Sensor_ts *sensor);
bool isFunctional(Sensor_ts *sensor);

bool setDefaultConfig(Sensor_ts *sensor);
bool updateRegisterMap(Sensor_ts *sensor);


// utilities
const char *getTypeAsString(SupportedSensorTypes_te sensorType);

uint8_t calculateParity(uint8_t data);
uint8_t getOddParity(uint8_t parity);
uint8_t getEvenParity(uint8_t parity);


// functions available only to a subset of sensors


#endif // SENSORS_COMMON_H
