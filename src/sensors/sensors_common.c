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

      case TLE493D_A1B6_e : return TLE493D_A1B6_init(sensor);
                              break;                              

      case TLV493D_A2BW_e : return TLV493D_A2BW_init(sensor);
                              break;

      // case TLE493D_W2B6_e : return TLE493D_W2B6_init(sensor);
      //                         break;

      default : return false;
   }
}

bool deinit(Sensor_ts *sensor) {
   return sensor->functions->deinit(sensor);
}


bool getTemperature(Sensor_ts *sensor, float *temp) {
   return sensor->functions->getTemperature(sensor, temp);
}

void calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
   sensor->functions->calculateFieldValues(sensor, x, y, z);
}

bool getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
   return sensor->functions->getFieldValues(sensor, x, y, z);
}


bool getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp) {
   return sensor->functions->getSensorValues(sensor, x, y, z, temp);
 }


// bool reset(Sensor_ts *sensor) {
//    return sensor->functions->reset(sensor);
// }


bool hasValidData(Sensor_ts *sensor) {
   return sensor->functions->hasValidData(sensor);
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

bool enableInterrupt(Sensor_ts *sensor) {
   return sensor->functions->enableInterrupt(sensor);
}

bool disableInterrupt(Sensor_ts *sensor) {
   return sensor->functions->disableInterrupt(sensor);
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

      // case TLE493D_W2B6_e : return "TLE493D_W2B6";
      //                      break;

      case TLV493D_A2BW_e : return "TLV493D_A2BW";
                           break;

      default : return "ERROR : Unknown sensorType !";
               break;
   }
}


uint8_t calculateParity(uint8_t data) {
	data ^= data >> 4;
	data ^= data >> 2;
	data ^= data >> 1;
	return data & 1U;
}


uint8_t getOddParity(uint8_t parity) {
    return (parity ^ 1U) & 1U;
}


uint8_t getEvenParity(uint8_t parity) {
    return parity & 1U;
}
