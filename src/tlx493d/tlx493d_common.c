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
bool tlx493d_common_init(TLx493D_t *sensor, uint8_t regMapSize, TLx493D_Register_t *regDef, TLx493D_CommonFunctions_t *commonFuncs,
                         TLx493D_SupportedSensorType_t sensorType, TLx493D_SupportedComLibraryInterfaceType_t comIFType) {
    sensor->regMap            = (uint8_t*) malloc(sizeof(uint8_t) * regMapSize);
    sensor->regDef            = regDef;
    sensor->functions         = commonFuncs;
    sensor->regMapSize        = regMapSize;
    sensor->sensorType        = sensorType;
    sensor->comIFType         = comIFType;
    sensor->comLibIF          = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    memset(sensor->regMap, 0, sensor->regMapSize);

    sensor->functions->setResetValues(sensor);

    // TODO: set address in TLx493D_initCommunication !
    // tlx493d_setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);
    
    return true;
}


/***
 * 
*/
bool tlx493d_common_deinit(TLx493D_t *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj); // TODO: provide central function to deallocate SPI/IIC objects !

    sensor->regMap            = NULL;
    sensor->comLibObj.i2c_obj = NULL; // TODO: provide central function to set to null SPI/IIC objects !
    return true;
}


// /***
//  * Generations 2 and 3, not 1.
// */
bool tlx493d_common_readRegisters(TLx493D_t *sensor) {
    return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}


void tlx493d_common_calculateRawTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, uint16_t *temperature) {
    tlx493d_common_concatBytes(sensor, tempMSBBF, tempLSBBF, temperature);
}


bool tlx493d_common_getRawTemperature(TLx493D_t *sensor, uint16_t *temperature) {
    if( sensor->functions->readRegisters(sensor) ) {
        sensor->functions->calculateRawTemperature(sensor, temperature);
        return true;
    }
}


void tlx493d_common_calculateRawMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                              uint8_t bzMSBBF, uint8_t bzLSBBF, uint16_t *x, uint16_t *y, uint16_t *z) {
    tlx493d_common_concatBytes(sensor, bxMSBBF, bxLSBBF, x);
    tlx493d_common_concatBytes(sensor, byMSBBF, byLSBBF, y);
    tlx493d_common_concatBytes(sensor, bzMSBBF, bzLSBBF, z);
}


bool tlx493d_common_getRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z) {
    if( sensor->functions->readRegisters(sensor) ) {
        sensor->functions->calculateRawMagneticField(sensor, x, y, z);
        return true;
    }

    return false;
}


bool tlx493d_common_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature) {
    if( sensor->functions->readRegisters(sensor) ) {
        sensor->functions->calculateRawMagneticFieldAndTemperature(sensor, x, y, z, temperature);
        return true;
    }

    return false;
}


/***
 * 
*/
bool tlx493d_common_getTemperature(TLx493D_t *sensor, double *temperature) {
    if( sensor->functions->readRegisters(sensor) ) {
        sensor->functions->calculateTemperature(sensor, temperature);
        return true;
    }

    return false;
}


/***
 * 
*/
bool tlx493d_common_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z ) {
    if( sensor->functions->readRegisters(sensor) ) {
        sensor->functions->calculateMagneticField(sensor, x, y, z);
        return true;
    }

    return false;
}


/***
 * 
*/
bool tlx493d_common_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temperature) {
    if( sensor->functions->readRegisters(sensor) ) {
        sensor->functions->calculateMagneticFieldAndTemperature(sensor, x, y, z, temperature);
        return true;
    }

    return false;
}


uint8_t tlx493d_common_returnBitfield(TLx493D_t *sensor, uint8_t bitField) {
    TLx493D_Register_t *bf = &sensor->regDef[bitField];

    if((bf->accessMode == TLx493D_READ_MODE_e) || (bf->accessMode == TLx493D_READ_WRITE_MODE_e)) {
        return (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }
    
    errorBitfieldNotReadableForSensorType(sensor, bitField);
    return 0;
}


void tlx493d_common_getBitfield(TLx493D_t *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    *bitFieldValue = tlx493d_common_returnBitfield(sensor, bitField);

    // TLx493D_Register_t *bf = &sensor->regDef[bitField];

    // if((bf->accessMode == TLx493D_READ_MODE_e) || (bf->accessMode == TLx493D_READ_WRITE_MODE_e)) {
    //     *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    // }

    // errorBitfieldNotReadableForSensorType(sensor, bitField);
}


void tlx493d_common_setBitfield(TLx493D_t *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    TLx493D_Register_t *bf = &sensor->regDef[bitField];

    if((bf->accessMode == TLx493D_WRITE_MODE_e) || (bf->accessMode == TLx493D_READ_WRITE_MODE_e)) {
        sensor->regMap[bf->address] = (sensor->regMap[bf->address] & ~bf->mask) | ((newBitFieldValue << bf->offset) & bf->mask);
    }
    else {
        errorBitfieldNotWritableForSensorType(sensor, bitField);
    }
}


bool tlx493d_common_writeRegister(TLx493D_t* sensor, uint8_t bitField) {
    TLx493D_Register_t *bf = &sensor->regDef[bitField];

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
void tlx493d_common_concatBytes(TLx493D_t *sensor, uint8_t msbBitfield, uint8_t lsbBitfield, int16_t *result) {
    TLx493D_Register_t *msb = &sensor->regDef[msbBitfield];
    TLx493D_Register_t *lsb = &sensor->regDef[lsbBitfield];
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


const char *tlx493d_common_getTypeAsString(TLx493D_t *sensor) {
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


void tlx493d_common_getSensitivityScaleFactor(TLx493D_t *sensor, TLx493D_AvailableSensitivityType_t sens, uint8_t x2BF, uint8_t x4BF, double *sf) {
    switch(sens) {
        case TLx493D_HAS_X1_e : *sf = 1.0;
                                return;
    
        case TLx493D_HAS_X2_e : {
                                    TLx493D_Register_t *x2 = &sensor->regDef[x2BF];
                                    *sf = (sensor->regMap[x2->address] & x2->mask) == 0 ? 1.0 : 2.0;
                                    return;
        }
    
        case TLx493D_HAS_X4_e : {
                                    TLx493D_Register_t *x2 = &sensor->regDef[x2BF];
                                    TLx493D_Register_t *x4 = &sensor->regDef[x4BF];
                                    *sf = (sensor->regMap[x2->address] & x2->mask) == 0 ? 1.0
                                                                                        : (sensor->regMap[x4->address] & x4->mask) == 0 ? 2.0 : 4.0;
                                    return;
        }
    
        default : errorSelectionNotSupportedForSensorType(sensor, sens, "TLx493D_AvailableSensitivityType_t");
    }
}


void tlx493d_common_setIICAddress(TLx493D_t *sensor, uint8_t addr) {
    sensor->comLibIFParams.i2c_params.address = addr;
}


void tlx493d_common_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens, double mT, int16_t *rawMF) {
}


void warnFeatureNotAvailableForSensorType(TLx493D_t *sensor, const char *featureName) {
    print("");
    warn("Feature '%s' not available for sensor type '%s' !", featureName, tlx493d_common_getTypeAsString(sensor));
}


void errorBitfieldNotReadableForSensorType(TLx493D_t *sensor, uint8_t bf) {
    print("");
    error("Bitfield '%d' not readable for sensor type '%s' !", bf, tlx493d_common_getTypeAsString(sensor));
}


void errorBitfieldNotWritableForSensorType(TLx493D_t *sensor, uint8_t bf) {
    print("");
    error("Bitfield '%d' not writable for sensor type '%s' !", bf, tlx493d_common_getTypeAsString(sensor));
}


void errorFunctionNotSupportedForSensorType(TLx493D_t *sensor, const char *func) {
    print("");
    error("Function '%s' not supported for sensor type '%s' !", func, tlx493d_common_getTypeAsString(sensor));
}


void errorSelectionNotSupportedForSensorType(TLx493D_t *sensor, uint8_t sel, const char *selType) {
    print("");
    error("Selection '%d' for type '%s' not supported for sensor type '%s' !", sel, selType, tlx493d_common_getTypeAsString(sensor));
}

