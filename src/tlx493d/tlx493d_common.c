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

// #include "Logger.h"


/***
*/
bool tlx493d_init(Sensor_ts *sensor, uint8_t regMapSize, Register_ts *regDef, CommonFunctions_ts *commonFuncs, SupportedSensorTypes_te sensorType, SupportedComLibraryInterfaceTypes_te comIFType) {
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


/***
 * 
*/
bool tlx493d_deinit(Sensor_ts *sensor) {
   free(sensor->regMap);
   free(sensor->comLibObj.i2c_obj); // TODO: provide central function to deallocate SPI/IIC objects !

   sensor->regMap            = NULL;
   sensor->comLibObj.i2c_obj = NULL; // TODO: provide central function to set to null SPI/IIC objects !
   return true;
}


void tlx493d_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == READ_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }
}


void tlx493d_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        sensor->regMap[bf->address] = (sensor->regMap[bf->address] & ~bf->mask) | ((newBitFieldValue << bf->offset) & bf->mask);
    }
}


bool tlx493d_writeRegister(Sensor_ts* sensor, uint8_t bitField) {
    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        uint8_t transBuffer[2] = { bf->address, sensor->regMap[bf->address] };

        return transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
    }

    return false;
}

// /***
//  * Generations 2 and 3, not 1.
// */
bool tlx493d_readRegisters(Sensor_ts *sensor) {
   return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}


// bool tlx493d_setDefaultConfig(Sensor_ts *sensor) {
//    return sensor->functions->setDefaultConfig(sensor);
// }


// // bool tlx493d_setPowerMode(Sensor_ts *sensor, enum <possible combinations> mode);  // value of mode is sensor / generation specific !
// bool tlx493d_setPowerMode(Sensor_ts *sensor, uint8_t mode) {
//    return sensor->functions->setPowerMode(sensor, mode);
// }


// // bool tlx493d_setIICAddress(Sensor_ts *sensor, enum <possible combinations> addr); // Gen. 1 and 2
// bool tlx493d_setIICAddress(Sensor_ts *sensor, uint8_t addr) {
//    return sensor->functions->setIICAddress(sensor, addr);
// }


/***
 * 
*/
bool tlx493d_getTemperature(Sensor_ts *sensor, double *temp) {
   if( sensor->functions->readRegisters(sensor) ) {
      sensor->functions->calculateTemperature(sensor, temp);
     return true;
   }

   return false;
}


/***
 * 
*/
bool tlx493d_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z ) {
   if( sensor->functions->readRegisters(sensor) ) {
      sensor->functions->calculateMagneticField(sensor, x, y, z);
      return true;
   }

   return false;
}


/***
 * 
*/
bool tlx493d_getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
   if( sensor->functions->readRegisters(sensor) ) {
      sensor->functions->calculateMagneticFieldAndTemperature(sensor, x, y, z, temp);
      return true;
   }

   return false;
}


// // bool tlx493d_selectMeasuredValues(Sensor_ts *sensor, enum <possible combinations> mVals); // Bx/By/Bz, Bx/By, Bx/By/Temp, ...
// // or do it separately ? Like :
// bool tlx493d_enableTemperatureMeasurement(Sensor_ts *sensor) {
//    return sensor->functions->enableTemperatureMeasurement(sensor);
// }
// bool tlx493d_disableTemperatureMeasurement(Sensor_ts *sensor) {
//    return sensor->functions->disableTemperatureMeasurement(sensor);
// }
// bool tlx493d_enableAngularMeasurement(Sensor_ts *sensor) {
//    return sensor->functions->enableAngularMeasurement(sensor);
// }
// bool tlx493d_disableAngularMeasurement(Sensor_ts *sensor) {
//    return sensor->functions->disableAngularMeasurement(sensor);
// }


// // value of update rate is sensor / generation specific !
// // bool setUpdateRate(Sensor_ts *sensor, enum <possible combinations> rate);
// bool tlx493d_setUpdateRate(Sensor_ts *sensor, uint8_t bit) {
//    return sensor->functions->setUpdateRate(sensor, bit);
// }


// bool tlx493d_setRange(Sensor_ts *sensor, enum <possible combinations> range); // full, short, extreme short : whatever is supported


// // bool tlx493d_setInterruptAndCollisionAvoidance(Sensor_ts *sensor, enum <possible combinations> eVal);
// bool tlx493d_enableInterrupt(Sensor_ts *sensor) {
//    return sensor->functions->enableInterrupt(sensor);
// }
// bool tlx493d_disableInterrupt(Sensor_ts *sensor) {
//    return sensor->functions->disableInterrupt(sensor);
// }
// bool tlx493d_enableCollisionAvoidance(Sensor_ts *sensor);
// bool tlx493d_disableCollisionAvoidance(Sensor_ts *sensor);


// set register bits
// bool tlx493d_setTrigger(Sensor_ts *sensor, uint8_t trigger);
// trigger bits shall be ORed to register address always by shifting left by 5 -> default is 0b000
// bool tlx493d_setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits);
// bool tlx493d_setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits) {
//    return sensor->functions->setTriggerBits(sensor, triggerBits);
// }


// bool tlx493d_isWakeUpActive(Sensor_ts *sensor) {
//    return sensor->functions->isWakeUpActive(sensor);
// }

// bool tlx493d_enableWakeUpMode(Sensor_ts *sensor) {
//    return sensor->functions->enableWakeUpMode(sensor);
// }

// bool tlx493d_disableWakeUpMode(Sensor_ts *sensor) {
//    return sensor->functions->disableWakeUpMode(sensor);
// }

// bool tlx493d_setUpperWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold) {
//    return sensor->functions->setUpperWakeUpThresholdX(sensor, threshold);
// }

// bool tlx493d_setUpperWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold) {
//    return sensor->functions->setUpperWakeUpThresholdY(sensor, threshold);
// }

// bool tlx493d_setUpperWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold) {
//    return sensor->functions->setUpperWakeUpThresholdZ(sensor, threshold);
// }

// bool tlx493d_setLowerWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold) {
//    return sensor->functions->setLowerWakeUpThresholdX(sensor, threshold);
// }

// bool tlx493d_setLowerWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold) {
//    return sensor->functions->setLowerWakeUpThresholdY(sensor, threshold);
// }

// bool tlx493d_setLowerWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold) {
//    return sensor->functions->setLowerWakeUpThresholdZ(sensor, threshold);
// }

// bool tlx493d_setWakeUpThresholds(Sensor_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
//    return sensor->functions->setWakeUpThresholds(sensor, xl_th, xh_th, yl_th, yh_th, zl_th, zh_th);
// }

// // thesholds im mT, to be converted to proper format
// bool tlx493d_setWakeupThesholds(Sensor_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh) {
//    return false;
// }

// bool tlx493d_softReset(Sensor_ts *sensor) {
//    return sensor->functions->softReset(sensor);
// }


uint8_t tlx493d_calculateParity(uint8_t data) {
	data ^= data >> 4;
	data ^= data >> 2;
	data ^= data >> 1;
	return data & 1U;
}


uint8_t tlx493d_getOddParity(uint8_t parity) {
    return (parity ^ 1U) & 1U;
}


uint8_t tlx493d_getEvenParity(uint8_t parity) {
    return parity & 1U;
}


/**
 * More generic version wrt size and offsets of MSB and LSB. Register values are in two's complement form.
 * Assumptions :
 *    - registers are 8 bits wide
*/
void tlx493d_concatBytes(Sensor_ts *sensor, uint8_t msbBitfield, uint8_t lsbBitfield, int16_t *result) {
    Register_ts *msb = &sensor->regDef[msbBitfield];
    Register_ts *lsb = &sensor->regDef[lsbBitfield];

    *result   = ((sensor->regMap[msb->address] & msb->mask) << (8 + 8 - msb->numBits - msb->offset)); // Set minus flag if highest bit is set
    *result >>= (16 - msb->numBits - lsb->numBits); // shift back and make space for LSB
    *result  |= ((sensor->regMap[lsb->address] & lsb->mask) >> lsb->offset); // OR with LSB
}
