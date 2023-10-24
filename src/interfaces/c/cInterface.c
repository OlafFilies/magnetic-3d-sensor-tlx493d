// std includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_common_defines.h"
#include "sensors_common.h"

// sensor specific includes
// #include "TLx493D_A1B6.h"
#include "TLE493D_A2B6.h"
// #include "TLE493D_P2B6.h"
// #include "TLE493D_W2B6.h"
// #include "TLV493D_A2BW.h"
// // #include "TLV493D_W2BW.h"
// #include "TLE493D_P3B6.h"
// #include "TLE493D_P3I8.h"


// functions common to all sensors
bool init(Sensor_ts *sensor, SupportedSensorTypes_te sensorType) {
   switch(sensorType) {
      // case TLx493D_A1B6_e : return TLx493D_A1B6_init(sensor);
      //                       break;                              

     case TLE493D_A2B6_e : return TLE493D_A2B6_init(sensor);
                           break;

     // case TLV493D_A2BW_e : return TLV493D_A2BW_init(sensor);
     //                       break;

     // case TLE493D_P2B6_e : return TLE493D_P2B6_init(sensor);
     //                       break;

     // case TLE493D_W2B6_e : return TLE493D_W2B6_init(sensor);
     //                       break;

     // // case TLV493D_W2BW_e : return TLV493D_W2BW_init(sensor);
     // //                       break;

     // case TLE493D_P3B6_e : return TLE493D_P3B6_init(sensor);
     //                       break;

     // case TLE493D_P3I8_e : return TLE493D_P3I8_init(sensor);
     //                       break;

      default : return false;
   }
}


bool deinit(Sensor_ts *sensor) {
   return sensor->functions->deinit(sensor);
}


/***
 * Generations 2 and 3, not 1.
*/
bool readRegisters(Sensor_ts *sensor) {
   return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}


// bool readRegisters(Sensor_ts *sensor) {
//    return sensor->functions->readRegisters(sensor);
// }


bool setDefaultConfig(Sensor_ts *sensor) {
   return sensor->functions->setDefaultConfig(sensor);
}


bool setPowerMode(Sensor_ts *sensor, uint8_t mode) {
   return sensor->functions->setPowerMode(sensor, mode);
}


bool setIICAddress(Sensor_ts *sensor, uint8_t addr) {
   return sensor->functions->setIICAddress(sensor, addr);
}


/***
 * 
*/
bool getTemperature(Sensor_ts *sensor, double *temp) {
// bool getSensorTemperature(Sensor_ts *sensor, double *temp) {
   if( sensor->functions->readRegisters(sensor) ) {
      sensor->functions->calculateTemperature(sensor, temp);
     return true;
   }

   return false;
}


/***
 * 
*/
bool getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z ) {
// bool getSensorMagneticField(Sensor_ts *sensor, double *x, double *y, double *z ) {
   if( sensor->functions->readRegisters(sensor) ) {
      sensor->functions->calculateMagneticField(sensor, x, y, z);
      return true;
   }

   return false;
}


/***
 * 
*/
bool getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
// bool getSensorMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
   if( sensor->functions->readRegisters(sensor) ) {
      sensor->functions->calculateMagneticFieldAndTemperature(sensor, x, y, z, temp);
      return true;
   }

   return false;
}

// bool getTemperature(Sensor_ts *sensor, double *temp) {
//    return sensor->functions->getTemperature(sensor, temp);
// }

// bool getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z) {
//    return sensor->functions->getMagneticField(sensor, x, y, z);
//  }

// bool getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
//    return sensor->functions->getMagneticFieldAndTemperature(sensor, x, y, z, temp);
//  }


bool hasValidData(Sensor_ts *sensor) {
   return sensor->functions->hasValidData(sensor);
}

// bool hasValidDataTemperatureData(Sensor_ts *sensor);
// bool hasValidDataMagneticFieldData(Sensor_ts *sensor);



bool isFunctional(Sensor_ts *sensor) {
   return sensor->functions->isFunctional(sensor);
}


bool enableTemperatureMeasurement(Sensor_ts *sensor) {
   return sensor->functions->enableTemperature(sensor);
}


bool disableTemperatureMeasurement(Sensor_ts *sensor) {
   return sensor->functions->disableTemperature(sensor);
}


bool enableAngularMeasurement(Sensor_ts *sensor) {
   return sensor->functions->enableAngularMeasurement(sensor);
}


bool disableAngularMeasurement(Sensor_ts *sensor) {
   return sensor->functions->disableAngularMeasurement(sensor);
}

bool setUpdateRate(Sensor_ts *sensor, uint8_t bit) {
   return sensor->functions->setUpdateRate(sensor, bit);
}


// bool setSensorRange(Sensor_ts *sensor, enum <possible combinations> range); // full, short, extreme short : whatever is supported

// bool setInterruptAndCollisionAvoidance(Sensor_ts *sensor, enum <possible combinations> eVal);
bool enableInterrupt(Sensor_ts *sensor) {
   return sensor->functions->enableInterrupt(sensor);
}


bool disableInterrupt(Sensor_ts *sensor) {
   return sensor->functions->disableInterrupt(sensor);
}


// bool enableWakeup(Sensor_ts *sensor);
// bool disableWakeup(Sensor_ts *sensor);
// // thesholds im mT, to be converted to proper format
// bool setWakeupThesholds(Sensor_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);

// bool softReset(Sensor_ts *sensor) {
//    return sensor->functions->reset(sensor);
// }


// set register bits
// bool setTrigger(Sensor_ts *sensor, uint8_t trigger);
// trigger bits shall be ORed to register address always by shifting left by 5 -> default is 0b000
bool setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits) {
   return sensor->functions->setTriggerBits(sensor, triggerBits);
}




// utility function
const char *getTypeAsString(SupportedSensorTypes_te sensorType) {
   switch(sensorType) {
      // case TLx493D_A1B6_e : return "TLx493D_A1B6";
      //                      break;

      case TLE493D_A2B6_e : return "TLE493D_A2B6";
                            break;

      // case TLV493D_A2BW_e : return "TLV493D_A2BW";
      //                       break;

      // case TLE493D_P2B6_e : return "TLE493D_P2B6";
      //                       break;

      // case TLE493D_W2B6_e : return "TLE493D_W2B6";
      //                       break;

      // // case TLV493D_W2BW_e : return "TLV493D_W2BW";
      // //                      break;

      // case TLE493D_P3B6_e : return "TLE493D_P3B6";
      //                       break;

      // case TLE493D_P3I8_e : return "TLE493D_P3I8";
      //                       break;

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


/**
 * More generic version wrt size and offsets of MSB and LSB. Register values are in two's complement form.
 * Assumptions :
 *    - registers are 8 bits wide
*/
void concatBytes(Sensor_ts *sensor, uint8_t msbBitfield, uint8_t lsbBitfield, int16_t *result) {
    Register_ts *msb = &sensor->regDef[msbBitfield];
    Register_ts *lsb = &sensor->regDef[lsbBitfield];

    *result   = ((sensor->regMap[msb->address] & msb->mask) << (8 + 8 - msb->numBits - msb->offset)); // Set minus flag if highest bit is set
    *result >>= (16 - msb->numBits - lsb->numBits); // shift back and make space for LSB
    *result  |= ((sensor->regMap[lsb->address] & lsb->mask) >> lsb->offset); // OR with LSB
}


// /**
//  * More generic version wrt size and offsets of MSB and LSB. Register values are in two's complement form.
//  * Assumptions :
//  *    - registers are 8 bits wide
// */
// void concatBytes(Sensor_ts *sensor, Register_ts *msb, Register_ts *lsb, int16_t *result) {
//     *result   = ((sensor->regMap[msb->address] & msb->mask) << (8 + 8 - msb->numBits - msb->offset)); // Set minus flag if highest bit is set
//     *result >>= (16 - msb->numBits - lsb->numBits); // shift back and make space for LSB
//     *result  |= ((sensor->regMap[lsb->address] & lsb->mask) >> lsb->offset); // OR with LSB
// }
