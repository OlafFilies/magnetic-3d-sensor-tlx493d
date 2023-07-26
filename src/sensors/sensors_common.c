// std includes
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_config_common.h"
#include "sensors_common.h"

// sensor specific includes
#include "TLE493D_A1B6.h"
#include "TLV493D_A1B6.h"
#include "TLE493D_A2B6.h"
#include "TLE493D_P2B6.h"
#include "TLE493D_W2B6.h"
#include "TLV493D_A2BW.h"

// functions common to all sensors
bool init(Sensor_ts *sensor, SupportedSensorTypes_te sensorType) {
    switch(sensorType) {
        case TLE493D_A2B6_e : return TLE493D_A2B6_init(sensor);
                              break;

      case TLE493D_A1B6_e : return TLE493D_A1B6_init(sensor, comLibIF);
                              break;                              

      case TLV493D_A2BW_e : return TLV493D_A2BW_init(sensor, comLibIF);
                              break;

      case TLE493D_W2B6_e : return TLE493D_W2B6_init(sensor, comLibIF);
                              break;

      default : return false;
   }
}

bool deinit(Sensor_ts *sensor) {
   return sensor->functions->deinit(sensor);
}


bool getTemperature(Sensor_ts *sensor, float *temp) {
   return sensor->functions->getTemperature(sensor, temp);
}


bool updateGetTemperature(Sensor_ts *sensor, float *temp) {
   return sensor->functions->updateGetTemperature(sensor, temp);
}


bool getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
   return sensor->functions->getFieldValues(sensor, x, y, z);
}


bool updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
   return sensor->functions->updateGetFieldValues(sensor, x, y, z);
 }


bool reset(Sensor_ts *sensor) {
   return sensor->functions->reset(sensor);
}


bool getDiagnosis(Sensor_ts *sensor) {
   return sensor->functions->getDiagnosis(sensor);
}


void calculateParity(Sensor_ts *sensor) {
   return sensor->functions->calculateParity(sensor);
}


bool setDefaultConfig(Sensor_ts *sensor) {
   return sensor->functions->setDefaultConfig(sensor);
}


bool updateRegisterMap(Sensor_ts *sensor) {
   return sensor->functions->updateRegisterMap(sensor);
}

bool enableTemperature(Sensor_ts *sensor) {
   return sensor->functions->enableTemperature(sensor);
}

bool disableTemperature(Sensor_ts *sensor) {
   return sensor->functions->disableTemperature(sensor);
}

// utility function
const char *getTypeAsString(SupportedSensorTypes_te sensorType) {
   switch(sensorType) {
      case TLE493D_A1B6_e : return "TLE493D_A1B6";
                           break;

      case TLV493D_A1B6_e : return "TLV493D_A1B6";
                           break;

      case TLE493D_A2B6_e : return "TLE493D_A2B6";
                           break;

      case TLE493D_P2B6_e : return "TLE493D_P2B6";
                           break;

      case TLE493D_W2B6_e : return "TLE493D_W2B6";
                           break;

      case TLV493D_A2BW_e : return "TLV493D_A2BW";
                           break;

      default : return "ERROR : Unknown sensorType !";
               break;
   }
}
