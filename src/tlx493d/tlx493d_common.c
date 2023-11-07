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
#include "Logger.h"

// sensor specific includes
#include "TLx493D_A1B6.h"
#include "TLx493D_A2B6.h"
#include "TLx493D_P2B6.h"
#include "TLx493D_W2B6.h"
#include "TLx493D_W2BW.h"
#include "TLx493D_P3B6.h"
#include "TLx493D_P3I8.h"


/***
*/
bool tlx493d_common_init(TLx493D_ts *sensor, uint8_t regMapSize, TLx493D_Register_ts *regDef, TLx493D_CommonFunctions_ts *commonFuncs,
                         TLx493D_SupportedSensorType_te sensorType, TLx493D_SupportedComLibraryInterfaceType_te comIFType) {
   sensor->regMap            = (uint8_t*) malloc(sizeof(uint8_t) * regMapSize);
   sensor->regDef            = regDef;
   sensor->functions         = commonFuncs;
   sensor->regMapSize        = regMapSize;
   sensor->sensorType        = sensorType;
   sensor->comIFType         = comIFType;
   sensor->comLibIF          = NULL;
   sensor->comLibObj.i2c_obj = NULL;

   memset(sensor->regMap, 0, sensor->regMapSize);
    
   // TODO: set address in TLx493D_initCommunication !
   // TLx493D_setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);
    
   return true;
}


/***
 * 
*/
bool tlx493d_common_deinit(TLx493D_ts *sensor) {
   free(sensor->regMap);
   free(sensor->comLibObj.i2c_obj); // TODO: provide central function to deallocate SPI/IIC objects !

   sensor->regMap            = NULL;
   sensor->comLibObj.i2c_obj = NULL; // TODO: provide central function to set to null SPI/IIC objects !
   return true;
}


// /***
//  * Generations 2 and 3, not 1.
// */
bool tlx493d_common_readRegisters(TLx493D_ts *sensor) {
   return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}


/***
 * 
*/
bool tlx493d_common_getTemperature(TLx493D_ts *sensor, double *temp) {
   if( sensor->functions->readRegisters(sensor) ) {
      sensor->functions->calculateTemperature(sensor, temp);
     return true;
   }

   return false;
}


/***
 * 
*/
bool tlx493d_common_getMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z ) {
   if( sensor->functions->readRegisters(sensor) ) {
      sensor->functions->calculateMagneticField(sensor, x, y, z);
      return true;
   }

   return false;
}


/***
 * 
*/
bool tlx493d_common_getMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp) {
   if( sensor->functions->readRegisters(sensor) ) {
      sensor->functions->calculateMagneticFieldAndTemperature(sensor, x, y, z, temp);
      return true;
   }

   return false;
}


void tlx493d_common_getBitfield(TLx493D_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    TLx493D_Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == TLx493D_READ_MODE_e) || (bf->accessMode == TLx493D_READ_WRITE_MODE_e)) {
        *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }
}


void tlx493d_common_setBitfield(TLx493D_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    TLx493D_Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == TLx493D_WRITE_MODE_e) || (bf->accessMode == TLx493D_READ_WRITE_MODE_e)) {
        sensor->regMap[bf->address] = (sensor->regMap[bf->address] & ~bf->mask) | ((newBitFieldValue << bf->offset) & bf->mask);
    }
}


bool tlx493d_common_writeRegister(TLx493D_ts* sensor, uint8_t bitField) {
    TLx493D_Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == TLx493D_WRITE_MODE_e) || (bf->accessMode == TLx493D_READ_WRITE_MODE_e)) {
        uint8_t transBuffer[2] = { bf->address, sensor->regMap[bf->address] };

        return transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
    }

    return false;
}


uint8_t tlx493d_common_calculateParity(uint8_t data) {
	data ^= data >> 4;
	data ^= data >> 2;
	data ^= data >> 1;
	return data & 1U;
}


uint8_t tlx493d_common_getOddParity(uint8_t parity) {
    return (parity ^ 1U) & 1U;
}


uint8_t tlx493d_common_getEvenParity(uint8_t parity) {
    return parity & 1U;
}


/**
 * More generic version wrt size and offsets of MSB and LSB. Register values are in two's complement form.
 * Assumptions :
 *    - registers are 8 bits wide
*/
void tlx493d_common_concatBytes(TLx493D_ts *sensor, uint8_t msbBitfield, uint8_t lsbBitfield, int16_t *result) {
    TLx493D_Register_ts *msb = &sensor->regDef[msbBitfield];
    TLx493D_Register_ts *lsb = &sensor->regDef[lsbBitfield];
    // print("\nmsb = %#x   %#x\n", sensor->regMap[msb->address] & msb->mask, (sensor->regMap[msb->address] & msb->mask) >> msb->offset);
    // print("\nlsb = %#x   %#x\n", sensor->regMap[lsb->address] & lsb->mask, (sensor->regMap[lsb->address] & lsb->mask) >> lsb->offset);

    // print("\nmsb->numBits = %#x    mask : %#x\n", msb->numBits, msb->mask);
    // print("\nlsb->numBits = %#x    mask : %#x\n", lsb->numBits, lsb->mask);

    *result   = ((sensor->regMap[msb->address] & msb->mask) << (16 - msb->numBits - msb->offset)); // Set minus flag if highest bit is set
    // print("\nshift = %#x\n", 16 - msb->numBits - msb->offset);
    // print("\nresult 0 = %#x\n", *result);
    // print("\nresult >> 1 = %#x\n", *result >> 1);
    // print("\nresult >> 2 = %#x\n", *result >> 2);
    // print("\nresult >> 3 = %#x\n", *result >> 3);
    // print("\nresult >> 4 = %#x\n", *result >> 4);
    // print("\nresult >> 5 = %#x\n", *result >> 5);
    // print("\nresult >> 6 = %#x\n", *result >> 6);
    *result >>= (16 - msb->numBits - lsb->numBits); // shift back and make space for LSB
    // print("\nshift = %#x\n", 16 - msb->numBits - lsb->numBits);
    // print("\nresult 1 = %#x\n", *result);
    *result  |= ((sensor->regMap[lsb->address] & lsb->mask) >> lsb->offset); // OR with LSB
    // print("\nresult 2 = %#x\n", *result);
}


const char *tlx493d_common_getTypeAsString(TLx493D_ts *sensor) {
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


void warnFeatureNotAvailableForSensorType(TLx493D_ts *sensor, const char *featureName) {
    warn("Feature '%s' not available for sensor type '%s' !\n", featureName, tlx493d_common_getTypeAsString(sensor));
}