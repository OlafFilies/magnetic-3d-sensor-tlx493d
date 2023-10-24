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


/***
*/
bool initSensor(Sensor_ts *sensor, uint8_t regMapSize, Register_ts *regDef, CommonFunctions_ts *commonFuncs, SupportedSensorTypes_te sensorType, SupportedComLibraryInterfaceTypes_te comIFType) {
   sensor->regMap            = (uint8_t*) malloc(sizeof(uint8_t) * regMapSize);
   sensor->regDef            = regDef;
   sensor->functions         = commonFuncs;
   sensor->regMapSize        = regMapSize;
   sensor->sensorType        = sensorType;
   sensor->comIFType         = comIFType;
   sensor->comLibIF          = NULL;
   sensor->comLibObj.i2c_obj = NULL;

   memset(sensor->regMap, 0, sensor->regMapSize);
    
   // setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);
    
   return true;
}


bool deinit(Sensor_ts *sensor) {
   return sensor->functions->deinit(sensor);
}


/***
 * 
*/
bool deinitSensor(Sensor_ts *sensor) {
   free(sensor->regMap);
   free(sensor->comLibObj.i2c_obj); // TODO: provide central function to deallocate SPI/IIC objects !

   sensor->regMap            = NULL;
   sensor->comLibObj.i2c_obj = NULL; // TODO: provide central function to set to null SPI/IIC objects !
   return true;
}


/***
 * Generations 2 and 3, not 1.
*/
bool readRegisters(Sensor_ts *sensor) {
   return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
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


// void calculateTemperature(Sensor_ts *sensor, double *temp) {
//    return sensor->functions->calculateTemperature(sensor, temp);
// }

// bool getTemperature(Sensor_ts *sensor, double *temp) {
//    return sensor->functions->getTemperature(sensor, temp);
// }

// // void calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z) {
// //    sensor->functions->calculateMagneticField(sensor, x, y, z);
// // }

// bool getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z) {
//    return sensor->functions->getMagneticField(sensor, x, y, z);
//  }


// bool getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
//    return sensor->functions->getMagneticFieldAndTemperature(sensor, x, y, z, temp);
//  }


// bool reset(Sensor_ts *sensor) {
//    return sensor->functions->reset(sensor);
// }


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


bool setDefaultConfig(Sensor_ts *sensor) {
   return sensor->functions->setDefaultConfig(sensor);
}


// bool readRegisters(Sensor_ts *sensor) {
//    return sensor->functions->readRegisters(sensor);
// }


bool enableTemperatureMeasurement(Sensor_ts *sensor) {
   return sensor->functions->enableTemperatureMeasurement(sensor);
}


bool disableTemperatureMeasurement(Sensor_ts *sensor) {
   return sensor->functions->disableTemperatureMeasurement(sensor);
}


bool enableInterrupt(Sensor_ts *sensor) {
   return sensor->functions->enableInterrupt(sensor);
}


bool disableInterrupt(Sensor_ts *sensor) {
   return sensor->functions->disableInterrupt(sensor);
}


bool setPowerMode(Sensor_ts *sensor, uint8_t mode) {
   return sensor->functions->setPowerMode(sensor, mode);
}


bool setIICAddress(Sensor_ts *sensor, uint8_t addr) {
   return sensor->functions->setIICAddress(sensor, addr);
}


bool enableAngularMeasurement(Sensor_ts *sensor) {
   return sensor->functions->enableAngularMeasurement(sensor);
}


bool disableAngularMeasurement(Sensor_ts *sensor) {
   return sensor->functions->disableAngularMeasurement(sensor);
}

bool setTriggerBits(Sensor_ts *sensor, uint8_t bits) {
   return sensor->functions->setTriggerBits(sensor, bits);
}

bool setUpdateRate(Sensor_ts *sensor, uint8_t bit) {
   return sensor->functions->setUpdateRate(sensor, bit);
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
