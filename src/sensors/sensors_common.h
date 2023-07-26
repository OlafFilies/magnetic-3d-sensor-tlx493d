#ifndef SENSORS_COMMON_H
#define SENSORS_COMMON_H


// project c includes
// common to all sensors
#include "sensor_types.h"


// functions common to all sensors
bool init(Sensor_ts *sensor, SupportedSensorTypes_te sensorType);
bool deinit(Sensor_ts *sensor);

bool getTemperature(Sensor_ts *sensor, float *temp);
bool updateGetTemperature(Sensor_ts *sensor, float *temp);

bool getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
bool updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

bool reset(Sensor_ts *sensor);
bool getDiagnosis(Sensor_ts *sensor);
void calculateParity(Sensor_ts *sensor);

bool setDefaultConfig(Sensor_ts *sensor);
bool updateRegisterMap(Sensor_ts *sensor);

bool enableTemperature(Sensor_ts *sensor);
bool disableTemperature(Sensor_ts *sensor);

// functions available only to a subset of sensors


// utility
const char *getTypeAsString(SupportedSensorTypes_te sensorType);



#endif // SENSORS_COMMON_H
