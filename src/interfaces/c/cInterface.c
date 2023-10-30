// std includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

// sensor specific includes
// #include "TLx493D_A1B6.h"
#include "TLx493D_A2B6.h"
// #include "TLx493D_P2B6.h"
#include "TLx493D_W2B6.h"
// #include "TLx493D_A2BW.h"
// // #include "TLx493D_W2BW.h"
// #include "TLx493D_P3B6.h"
// #include "TLx493D_P3I8.h"


// functions common to all sensors
bool init(Sensor_ts *sensor, SupportedSensorTypes_te sensorType) {
   switch(sensorType) {
      // case TLx493D_A1B6_e : return TLx493D_A1B6_init(sensor);
      //                       break;                              

     case TLx493D_A2B6_e : return TLx493D_A2B6_init(sensor);
                           break;

     // case TLx493D_A2BW_e : return TLx493D_A2BW_init(sensor);
     //                       break;

     // case TLx493D_P2B6_e : return TLx493D_P2B6_init(sensor);
     //                       break;

     case TLx493D_W2B6_e : return TLx493D_W2B6_init(sensor);
                           break;

     // // case TLx493D_W2BW_e : return TLx493D_W2BW_init(sensor);
     // //                       break;

     // case TLx493D_P3B6_e : return TLx493D_P3B6_init(sensor);
     //                       break;

     // case TLx493D_P3I8_e : return TLx493D_P3I8_init(sensor);
     //                       break;

      default : return false;
   }
}


bool deinit(Sensor_ts *sensor) {
   return sensor->functions->deinit(sensor);
}


/***
 * 
*/
bool readRegisters(Sensor_ts *sensor) {
   // return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
   return sensor->functions->readRegisters(sensor);
}


bool setDefaultConfig(Sensor_ts *sensor) {
   return sensor->functions->setDefaultConfig(sensor);
}


// bool setPowerMode(Sensor_ts *sensor, enum <possible combinations> mode);  // value of mode is sensor / generation specific !
bool setPowerMode(Sensor_ts *sensor, uint8_t mode) {
   return sensor->functions->setPowerMode(sensor, mode);
}


// bool setIICAddress(Sensor_ts *sensor, enum <possible combinations> addr); // Gen. 1 and 2
bool setIICAddress(Sensor_ts *sensor, uint8_t addr) {
   return sensor->functions->setIICAddress(sensor, addr);
}


/***
 * 
*/
bool getTemperature(Sensor_ts *sensor, double *temp) {
   tlx493d_getTemperature(sensor, temp);
   // return sensor->functions->getTemperature(sensor, temp);
}


/***
 * 
*/
bool getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z ) {
   tlx493d_getMagneticField(sensor, x, y, z);
   // return sensor->functions->getMagneticField(sensor, x, y, z);
}


/***
 * 
*/
bool getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
   tlx493d_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
   // return sensor->functions->getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


// bool selectMeasuredValues(Sensor_ts *sensor, enum <possible combinations> mVals); // Bx/By/Bz, Bx/By, Bx/By/Temp, ...
bool enableTemperatureMeasurement(Sensor_ts *sensor) {
   return sensor->functions->enableTemperatureMeasurement(sensor);
}
bool disableTemperatureMeasurement(Sensor_ts *sensor) {
   return sensor->functions->disableTemperatureMeasurement(sensor);
}
bool enableAngularMeasurement(Sensor_ts *sensor) {
   return sensor->functions->enableAngularMeasurement(sensor);
}
bool disableAngularMeasurement(Sensor_ts *sensor) {
   return sensor->functions->disableAngularMeasurement(sensor);
}


// bool setUpdateRate(Sensor_ts *sensor, enum <possible combinations> rate);
bool setUpdateRate(Sensor_ts *sensor, uint8_t bit) {
   return sensor->functions->setUpdateRate(sensor, bit);
}


// bool setRange(Sensor_ts *sensor, enum <possible combinations> range); // full, short, extreme short : whatever is supported

// bool setInterruptAndCollisionAvoidance(Sensor_ts *sensor, enum <possible combinations> eVal);
bool enableInterrupt(Sensor_ts *sensor) {
   return sensor->functions->enableInterrupt(sensor);
}
bool disableInterrupt(Sensor_ts *sensor) {
   return sensor->functions->disableInterrupt(sensor);
}
// bool enableCollisionAvoidance(Sensor_ts *sensor);
// bool disableCollisionAvoidance(Sensor_ts *sensor);


bool isWakeUpActive(Sensor_ts *sensor) {
   return sensor->functions->isWakeUpActive(sensor);
}

bool enableWakeUpMode(Sensor_ts *sensor) {
   return sensor->functions->enableWakeUpMode(sensor);
}

bool disableWakeUpMode(Sensor_ts *sensor) {
   return sensor->functions->disableWakeUpMode(sensor);
}

bool setUpperWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold) {
   return sensor->functions->setUpperWakeUpThresholdX(sensor, threshold);
}

bool setUpperWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold) {
   return sensor->functions->setUpperWakeUpThresholdY(sensor, threshold);
}

bool setUpperWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold) {
   return sensor->functions->setUpperWakeUpThresholdZ(sensor, threshold);
}

bool setLowerWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold) {
   return sensor->functions->setLowerWakeUpThresholdX(sensor, threshold);
}

bool setLowerWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold) {
   return sensor->functions->setLowerWakeUpThresholdY(sensor, threshold);
}

bool setLowerWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold) {
   return sensor->functions->setLowerWakeUpThresholdZ(sensor, threshold);
}

bool setWakeUpThresholds(Sensor_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
   return sensor->functions->setWakeUpThresholds(sensor, xl_th, xh_th, yl_th, yh_th, zl_th, zh_th);
}

// thesholds im mT, to be converted to proper format
bool setWakeupThesholds(Sensor_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh) {
   return sensor->functions->setWakeUpThresholds(sensor, xLow, xHigh, yLow, yHigh, zLow, zHigh);
}

// bool softReset(Sensor_ts *sensor) {
//    return sensor->functions->reset(sensor);
// }


// Severin : nice to have, set proper defaults
// set register bits
// bool setTrigger(Sensor_ts *sensor, uint8_t trigger);
// trigger bits shall be ORed to register address always by shifting left by 5 -> default is 0b000
// bool setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits) {
//    return sensor->functions->setTriggerBits(sensor, triggerBits);
// }


// diagnosis functions
bool hasValidData(Sensor_ts *sensor) {
   return sensor->functions->hasValidData(sensor);
}

bool hasValidTemperatureData(Sensor_ts *sensor) {
   return sensor->functions->hasValidTemperatureData(sensor);
}

bool hasValidMagneticFieldData(Sensor_ts *sensor) {
   return sensor->functions->hasValidMagneticFieldData(sensor);
}

bool isFunctional(Sensor_ts *sensor) {
   return sensor->functions->isFunctional(sensor);
}


// utility function
const char *getTypeAsString(SupportedSensorTypes_te sensorType) {
   switch(sensorType) {
      // case TLx493D_A1B6_e : return "TLx493D_A1B6";
      //                      break;

      case TLx493D_A2B6_e : return "TLx493D_A2B6";
                            break;

      // case TLx493D_A2BW_e : return "TLx493D_A2BW";
      //                       break;

      // case TLx493D_P2B6_e : return "TLx493D_P2B6";
      //                       break;

      case TLx493D_W2B6_e : return "TLx493D_W2B6";
                            break;

      // // case TLx493D_W2BW_e : return "TLx493D_W2BW";
      // //                      break;

      // case TLx493D_P3B6_e : return "TLx493D_P3B6";
      //                       break;

      // case TLx493D_P3I8_e : return "TLx493D_P3I8";
      //                       break;

      default : return "ERROR : Unknown sensorType !";
               break;
   }
}
