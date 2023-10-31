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
#include "TLx493D_A1B6.h"
#include "TLx493D_A2B6.h"
#include "TLx493D_P2B6.h"
#include "TLx493D_W2B6.h"
#include "TLx493D_W2BW.h"
#include "TLx493D_P3B6.h"
#include "TLx493D_P3I8.h"


// functions common to all sensors
bool tlx493d_init(TLx493D_ts *sensor, TLx493D_SupportedSensorTypes_te sensorType) {
   switch(sensorType) {
      case TLx493D_A1B6_e : return TLx493D_A1B6_init(sensor);
                            break;                              

      case TLx493D_A2B6_e : return TLx493D_A2B6_init(sensor);
                            break;

      case TLx493D_P2B6_e : return TLx493D_P2B6_init(sensor);
                            break;

      case TLx493D_W2B6_e : return TLx493D_W2B6_init(sensor);
                            break;

      case TLx493D_W2BW_e : return TLx493D_W2BW_init(sensor);
                            break;

      case TLx493D_P3B6_e : return TLx493D_P3B6_init(sensor);
                            break;

      case TLx493D_P3I8_e : return TLx493D_P3I8_init(sensor);
                            break;

      default : return false;
   }
}


bool tlx493d_deinit(TLx493D_ts *sensor) {
   return sensor->functions->deinit(sensor);
}


/***
 * 
*/
bool tlx493d_readRegisters(TLx493D_ts *sensor) {
   // return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
   return sensor->functions->readRegisters(sensor);
}


/***
 * 
*/
bool tlx493d_getTemperature(TLx493D_ts *sensor, double *temp) {
   tlx493d_common_getTemperature(sensor, temp);
   // return sensor->functions->getTemperature(sensor, temp);
}


/***
 * 
*/
bool tlx493d_getMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z ) {
   tlx493d_common_getMagneticField(sensor, x, y, z);
   // return sensor->functions->getMagneticField(sensor, x, y, z);
}


/***
 * 
*/
bool tlx493d_getMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp) {
   tlx493d_common_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
   // return sensor->functions->getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


// bool selectMeasuredValues(TLx493D_ts *sensor, enum <possible combinations> mVals); // Bx/By/Bz, Bx/By, Bx/By/Temp, ...
bool tlx493d_enableTemperatureMeasurement(TLx493D_ts *sensor) {
   return sensor->functions->enableTemperatureMeasurement(sensor);
}
bool tlx493d_disableTemperatureMeasurement(TLx493D_ts *sensor) {
   return sensor->functions->disableTemperatureMeasurement(sensor);
}
bool tlx493d_enableAngularMeasurement(TLx493D_ts *sensor) {
   return sensor->functions->enableAngularMeasurement(sensor);
}
bool tlx493d_disableAngularMeasurement(TLx493D_ts *sensor) {
   return sensor->functions->disableAngularMeasurement(sensor);
}


bool tlx493d_setTrigger(TLx493D_ts *sensor, uint8_t trigger) {
   return sensor->functions->setTrigger(sensor, trigger);
}


bool tlx493d_setRange(TLx493D_ts *sensor, TLx493D_RangeTypes_te range) {
   return sensor->functions->setRange(sensor, range);
}


bool tlx493d_setDefaultConfig(TLx493D_ts *sensor) {
   return sensor->functions->setDefaultConfig(sensor);
}


bool setIICAddress(TLx493D_ts *sensor, TLx493D_IICAddresses_te addr) {
   return sensor->functions->setIICAddress(sensor, addr);
}
// bool tlx493d_setIICAddress(TLx493D_ts *sensor, uint8_t addr) {
//    return sensor->functions->setIICAddress(sensor, addr);
// }


bool tlx493d_enableInterrupt(TLx493D_ts *sensor) {
   return sensor->functions->enableInterrupt(sensor);
}
bool tlx493d_disableInterrupt(TLx493D_ts *sensor) {
   return sensor->functions->disableInterrupt(sensor);
}
bool tlx493d_enableCollisionAvoidance(TLx493D_ts *sensor) {
   return sensor->functions->enableCollisionAvoidance(sensor);
}
bool tlx493d_disableCollisionAvoidance(TLx493D_ts *sensor) {
   return sensor->functions->disableCollisionAvoidance(sensor);
}


// bool tlx493d_setPowerMode(TLx493D_ts *sensor, enum <possible combinations> mode);  // value of mode is sensor / generation specific !
bool tlx493d_setPowerMode(TLx493D_ts *sensor, uint8_t mode) {
   return sensor->functions->setPowerMode(sensor, mode);
}


// bool tlx493d_setUpdateRate(TLx493D_ts *sensor, enum <possible combinations> rate);
bool tlx493d_setUpdateRate(TLx493D_ts *sensor, uint8_t bit) {
   return sensor->functions->setUpdateRate(sensor, bit);
}


bool tlx493d_hasValidData(TLx493D_ts *sensor) {
   return sensor->functions->hasValidData(sensor);
}

bool tlx493d_hasValidTemperatureData(TLx493D_ts *sensor) {
   return sensor->functions->hasValidTemperatureData(sensor);
}

bool tlx493d_hasValidMagneticFieldData(TLx493D_ts *sensor) {
   return sensor->functions->hasValidMagneticFieldData(sensor);
}

bool tlx493d_isFunctional(TLx493D_ts *sensor) {
   return sensor->functions->isFunctional(sensor);
}


bool tlx493d_isWakeUpActive(TLx493D_ts *sensor) {
   return sensor->functions->isWakeUpActive(sensor);
}

bool tlx493d_enableWakeUpMode(TLx493D_ts *sensor) {
   return sensor->functions->enableWakeUpMode(sensor);
}

bool tlx493d_disableWakeUpMode(TLx493D_ts *sensor) {
   return sensor->functions->disableWakeUpMode(sensor);
}

bool tlx493d_setUpperWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold) {
   return sensor->functions->setUpperWakeUpThresholdX(sensor, threshold);
}

bool tlx493d_setUpperWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold) {
   return sensor->functions->setUpperWakeUpThresholdY(sensor, threshold);
}

bool tlx493d_setUpperWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold) {
   return sensor->functions->setUpperWakeUpThresholdZ(sensor, threshold);
}

bool tlx493d_setLowerWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold) {
   return sensor->functions->setLowerWakeUpThresholdX(sensor, threshold);
}

bool tlx493d_setLowerWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold) {
   return sensor->functions->setLowerWakeUpThresholdY(sensor, threshold);
}

bool tlx493d_setLowerWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold) {
   return sensor->functions->setLowerWakeUpThresholdZ(sensor, threshold);
}

bool tlx493d_setWakeUpThresholds(TLx493D_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
   return sensor->functions->setWakeUpThresholds(sensor, xl_th, xh_th, yl_th, yh_th, zl_th, zh_th);
}

// thesholds im mT, to be converted to proper format
bool tlx493d_setWakeupThesholds(TLx493D_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh) {
   return sensor->functions->setWakeUpThresholds(sensor, xLow, xHigh, yLow, yHigh, zLow, zHigh);
}


// bool tlx493d_softReset(TLx493D_ts *sensor) {
//    return sensor->functions->reset(sensor);
// }


const char *tlx493d_getTypeAsString(TLx493D_ts *sensor) {
   switch(sensor->sensorType) {
      case TLx493D_A1B6_e : return "TLx493D_A1B6";
                           break;

      case TLx493D_A2B6_e : return "TLx493D_A2B6";
                            break;

      case TLx493D_P2B6_e : return "TLx493D_P2B6";
                            break;

      case TLx493D_W2B6_e : return "TLx493D_W2B6";
                            break;

      case TLx493D_W2BW_e : return "TLx493D_W2BW";
                           break;

      case TLx493D_P3B6_e : return "TLx493D_P3B6";
                            break;

      case TLx493D_P3I8_e : return "TLx493D_P3I8";
                            break;

      default : return "ERROR : Unknown sensorType !";
               break;
   }
}
