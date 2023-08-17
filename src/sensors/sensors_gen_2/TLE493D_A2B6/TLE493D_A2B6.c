// std includes
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_config_common.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_config_common.h"
#include "sensors_gen_2_common.h"

// sensor specicifc includes
#include "TLE493D_A2B6_config.h"
#include "TLE493D_A2B6.h"


// framework functions
// TODO: replace by function pointers in comLibIF structure
extern void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr);


/*
  Listing of all register names for this sensor.
  Used to index "TLE493D_A2B6_regDef" defined below, so index values must match !
*/
typedef enum {
               BX_MSB = 0,
               BY_MSB,
               BZ_MSB,
               TEMP_MSB,
               BX_LSB,
               BY_LSB,
               TEMP_LSB,
               ID,
               BZ_LSB,
               P,
               FF,
               CF,
               T,
               PD3,
               PD0,
               FRM,
               DT,
               AM,
               TRIG,
               X2,
               TL_MAG,
               CP,
               FP,
               IICADDR,
               PR,
               CA,
               INT,
               MODE,
               PRD,
               TYPE,
               HWV } TLE493D_A2B6_registerNames_te;


Register_ts TLE493D_A2B6_regDef[] = {
    { BX_MSB,   READ_MODE_e,       0x00, 0xFF, 0, 8 },
    { BY_MSB,   READ_MODE_e,       0x01, 0xFF, 0, 8 },
    { BZ_MSB,   READ_MODE_e,       0x02, 0xFF, 0, 8 }, 
    { TEMP_MSB, READ_MODE_e,       0x03, 0xFF, 0, 8 },
    { BX_LSB,   READ_MODE_e,       0x04, 0xF0, 4, 4 },
    { BY_LSB,   READ_MODE_e,       0x04, 0x0F, 0, 4 },
    { TEMP_LSB, READ_MODE_e,       0x05, 0xC0, 6, 2 },
    { ID,       READ_MODE_e,       0x05, 0x30, 4, 2 },
    { BZ_LSB,   READ_MODE_e,       0x05, 0x0F, 0, 4 },
    { P,        READ_MODE_e,       0x06, 0x80, 7, 1 },
    { FF,       READ_MODE_e,       0x06, 0x40, 6, 1 },
    { CF,       READ_MODE_e,       0x06, 0x20, 5, 1 },
    { T,        READ_MODE_e,       0x06, 0x10, 4, 1 },
    { PD3,      READ_MODE_e,       0x06, 0x08, 3, 1 },
    { PD0,      READ_MODE_e,       0x06, 0x04, 2, 1 },
    { FRM,      READ_MODE_e,       0x06, 0x03, 0, 2 },
    { DT,       READ_WRITE_MODE_e, 0x10, 0x80, 7, 1 },
    { AM,       READ_WRITE_MODE_e, 0x10, 0x40, 6, 1 },
    { TRIG,     READ_WRITE_MODE_e, 0x10, 0x30, 4, 2 },
    { X2,       READ_WRITE_MODE_e, 0x10, 0x08, 3, 1 },
    { TL_MAG,   READ_WRITE_MODE_e, 0x10, 0x06, 1, 2 },
    { CP,       READ_WRITE_MODE_e, 0x10, 0x01, 0, 1 },
    { FP,       READ_WRITE_MODE_e, 0x11, 0x80, 7, 1 },
    { IICADDR,  READ_WRITE_MODE_e, 0x11, 0x60, 5, 2 },
    { PR,       READ_WRITE_MODE_e, 0x11, 0x10, 4, 1 },
    { CA,       READ_WRITE_MODE_e, 0x11, 0x08, 3, 1 },
    { INT,      READ_WRITE_MODE_e, 0x11, 0x04, 2, 1 },
    { MODE,     READ_WRITE_MODE_e, 0x11, 0x03, 0, 2 },
    // Does not match register overview in manual, but fits and default value
    // textual description of register PRD ! Confirmed by Severin.
    { PRD,      READ_WRITE_MODE_e, 0x13, 0xE0, 5, 3 },
    // { PRD,      READ_WRITE_MODE_e, 0x13, 0x80, 7, 1 },
    { TYPE,     READ_MODE_e,       0x16, 0x30, 4, 2 },
    { HWV,      READ_MODE_e,       0x16, 0x0F, 0, 4 }
};


typedef enum { 
               TEMP2_BITFIELD  = 0x05,
               DIAG_BITFIELD   = 0x06,
               CONFIG_BITFIELD = 0x10,
               MOD1_BITFIELD   = 0x11,
               MOD2_BITFIELD   = 0x13,
               VER_BITFIELD    = 0x16 } TLE493D_A2B6_special_bitfields_te;


typedef enum {
               LOW_POWER_MODE         = 0x00,
               MASTER_CONTROLLED_MODE = 0x01,
               RESERVED_MODE          = 0x10,
               FAST_MODE              = 0x11 } TLE493D_A2B6_modes_te;


CommonFunctions_ts TLE493D_A2B6_commonFunctions = {
                                .init                  = TLE493D_A2B6_init,
                                .deinit                = TLE493D_A2B6_deinit,

                                .getTemperature        = TLE493D_A2B6_getTemperature,
                                .updateGetTemperature  = TLE493D_A2B6_updateGetTemperature,

                                .getFieldValues        = TLE493D_A2B6_getFieldValues,
                                .updateGetFieldValues  = TLE493D_A2B6_updateGetFieldValues,

                                .reset                 = TLE493D_A2B6_reset,
                                .getDiagnosis          = TLE493D_A2B6_getDiagnosis,
                                .calculateParity       = TLE493D_A2B6_calculateParity,

                                .setDefaultConfig      = TLE493D_A2B6_setDefaultConfig,
                                .updateRegisterMap     = TLE493D_A2B6_updateRegisterMap,
                              };


// TODO: add parameter IICAddress or ad function to set address.
bool TLE493D_A2B6_init(Sensor_ts *sensor) {
    // regMap must be sensor specific, not sensor type specific, therefore malloc.
    sensor->regMap            = (uint8_t*) malloc(sizeof(uint8_t) * TLE493D_A2B6_REGISTER_MAP_SIZE);
    sensor->regDef            = TLE493D_A2B6_regDef;
    sensor->functions         = &TLE493D_A2B6_commonFunctions;
    sensor->regMapSize        = TLE493D_A2B6_REGISTER_MAP_SIZE;
    sensor->sensorType        = TLE493D_A2B6_e;
    sensor->comIFType         = I2C_e;
    sensor->comLibIF          = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    setI2CParameters(&sensor->comLibIFParams, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return true;
}


bool TLE493D_A2B6_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);

    sensor->regMap            = NULL;
    sensor->comLibObj.i2c_obj = NULL;
    return true;
}


bool TLE493D_A2B6_updateGetTemperature(Sensor_ts *sensor, float *temp) {
    bool b = TLE493D_A2B6_updateRegisterMap(sensor);
    return b && TLE493D_A2B6_getTemperature(sensor, temp);
}


/**
 * More generic version wrt size and offsets of MSB and LSB. Register values are in two's complement form.
 * Assumptions :
 *    - MSB is 8 bits wide
*/
void TLE493D_A2B6_concatBytes(Sensor_ts *sensor, Register_ts *msb, Register_ts *lsb, int16_t *result) {
    *result   = ((sensor->regMap[msb->address] & msb->mask) << TLE493D_A2B6_REGISTER_SIZE_IN_BITS); // Set minus flag if highest bit is set
    *result >>= (TLE493D_A2B6_REGISTER_SIZE_IN_BITS - lsb->numBits); // shift back and make space for LSB
    *result  |= ((sensor->regMap[lsb->address] & lsb->mask) >> lsb->offset); // or with LSB
}


bool TLE493D_A2B6_getTemperature(Sensor_ts *sensor, float *temp) {
    int16_t value = 0;

    TLE493D_A2B6_concatBytes(sensor, &sensor->regDef[TEMP_MSB], &sensor->regDef[TEMP_LSB], &value);

    value <<= 2; // least significant 2 bits are implicit, therefore shift by 2 !
    *temp = (((float) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF;

    return true;
}


bool TLE493D_A2B6_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    bool b = TLE493D_A2B6_updateRegisterMap(sensor);
    return b && TLE493D_A2B6_getFieldValues(sensor, x, y, z);
}


bool TLE493D_A2B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    TLE493D_A2B6_concatBytes(sensor, &sensor->regDef[BX_MSB], &sensor->regDef[BX_LSB], &valueX);
    TLE493D_A2B6_concatBytes(sensor, &sensor->regDef[BY_MSB], &sensor->regDef[BY_LSB], &valueY);
    TLE493D_A2B6_concatBytes(sensor, &sensor->regDef[BZ_MSB], &sensor->regDef[BZ_LSB], &valueZ);

    *x = ((float) valueX) * GEN_2_MAG_FIELD_MULT;
    *y = ((float) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((float) valueZ) * GEN_2_MAG_FIELD_MULT;

    return true;
}


// TODO: not yet working !
bool TLE493D_A2B6_reset(Sensor_ts *sensor) {
    // frameworkReset(sensor);

    return true;
}


// TODO: define enum to differentiate the various diagnosis messages possible across sensors
bool TLE493D_A2B6_getDiagnosis(Sensor_ts *sensor) {
    return true;
}


uint8_t TLE493D_A2B6_calculateMeasurementParity(Sensor_ts *sensor) {
    return 1;
}


bool isValidMeasurement(Sensor_ts *sensor) {
   return TLE493D_A2B6_calculateMeasurementParity(sensor) & 0x1;
}


bool isValidMeasurementParityBit(Sensor_ts *sensor) {
    return sensor->regMap[TLE493D_A2B6_regDef[T].address] && TLE493D_A2B6_regDef[T].mask == 0;
}


bool isSensorFunctional(Sensor_ts *sensor) {
    return sensor->regMap[TLE493D_A2B6_regDef[FF].address] && TLE493D_A2B6_regDef[FF].mask == 1;
}


void TLE493D_A2B6_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == READ_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }
}

void TLE493D_A2B6_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        sensor->regMap[bf->address] = (sensor->regMap[bf->address] & ~bf->mask) | ((newBitFieldValue << bf->offset) & bf->mask);
    }
}


bool TLE493D_A2B6_writeRegister(Sensor_ts* sensor, uint8_t bitField) {
    Register_ts *bf = &sensor->regDef[bitField];
    bool err = false;

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        uint8_t transBuffer[2];
        uint8_t bufLen = 2;

        transBuffer[0] = bf->address;
        transBuffer[1] = sensor->regMap[bf->address];

        err = sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, NULL, 0);
    }

    return err;
}


uint8_t getParity(uint8_t data) {
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


bool TLE493D_A2B6_calculateParity(Sensor_ts *sensor) {
    return true;
}


// Bus (data) parity bit P
uint8_t TLE493D_A2B6_calculateBusParityBit(Sensor_ts *sensor) {
	// compute bus parity of data values in registers 0 to 5
	uint8_t parity = sensor->regMap[0];

	for (uint8_t i = 1; i < 6; ++i) {
		parity ^= sensor->regMap[i];
	}

	return getOddParity(getParity(parity)) << TLE493D_A2B6_regDef[P].offset;
}


// Fuse/mode parity bit FP
uint8_t TLE493D_A2B6_calculateFuseParityBit(Sensor_ts *sensor) {
	// compute parity of MOD1 register
	uint8_t parity = getParity(sensor->regMap[MOD1_BITFIELD] & ~TLE493D_A2B6_regDef[FP].mask);

	// add parity of MOD2:PRD register bits
	parity ^= getParity(sensor->regMap[MOD2_BITFIELD] & TLE493D_A2B6_regDef[PRD].mask);

	return getOddParity(parity) << TLE493D_A2B6_regDef[FP].offset;
}


// Configuration parity bit CP
uint8_t TLE493D_A2B6_calculateConfigurationParityBit(Sensor_ts *sensor) {
	// compute parity of Config register
	uint8_t parity = getParity(sensor->regMap[CONFIG_BITFIELD] & ~TLE493D_A2B6_regDef[CP].mask);

	return getEvenParity(getParity(parity)) << TLE493D_A2B6_regDef[CP].offset;
}


void TLE493D_A2B6_set1ByteMode(Sensor_ts *sensor) {
    // TODO: the next 2 lines must go into init !
    sensor->regMap[MOD1_BITFIELD]  = 0;
    sensor->regMap[MOD2_BITFIELD]  = 0;

    // this is already setDefaultConfig ! Should set only PR bit !
    sensor->regMap[MOD1_BITFIELD]  = TLE493D_A2B6_regDef[PR].mask
                                   | TLE493D_A2B6_regDef[INT].mask;
                                //    | (MASTER_CONTROLLED_MODE << TLE493D_A2B6_regDef[MODE].offset);

    sensor->regMap[MOD1_BITFIELD] |= TLE493D_A2B6_calculateFuseParityBit(sensor);
}


/**
 * - set 1-byte mode
 * - disable interrupts
 * - set parity flag
 * 
*/
void TLE493D_A2B6_get1ByteModeBuffer(Sensor_ts *sensor, uint8_t *buf, uint8_t *bufLen) {
    // old :
    // buf[0] = 0x11;
    // buf[1] = 0x94;

    TLE493D_A2B6_set1ByteMode(sensor);

    buf[0]   = MOD1_BITFIELD;
    buf[1]   = sensor->regMap[MOD1_BITFIELD];
    // // buf[0]   = TLE493D_A2B6_regDef[PR].address;
    // // buf[1]  = TLE493D_A2B6_regDef[PR].mask | TLE493D_A2B6_regDef[INT].mask | TLE493D_A2B6_regDef[FP].mask;
    // buf[1]   = TLE493D_A2B6_regDef[PR].mask | TLE493D_A2B6_regDef[INT].mask | (MASTER_CONTROLLED_MODE << TLE493D_A2B6_regDef[MODE].offset);
    // buf[1]  |= TLE493D_A2B6_calculateFuseParityBit(sensor);
    *bufLen  = 2;
}


bool TLE493D_A2B6_enable1ByteMode(Sensor_ts *sensor) {
    uint8_t transBuffer[2];
    uint8_t bufLen = 0;

    TLE493D_A2B6_get1ByteModeBuffer(sensor, transBuffer, &bufLen);
   
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, sensor->regMapSize);
}


/**
 * - enable temperature measurements
 * - hardcoded version also 
 *  - preserve all bits except parity and lower TL_MAG bit
*/
void TLE493D_A2B6_getTemperatureMeasurementsBuffer(Sensor_ts *sensor, uint8_t *buf, uint8_t *bufLen) {
    // old :
    // buf[0] = 0x10;
    // buf[1] = regMap[0x10] & 0x7C;

    buf[0]  = TLE493D_A2B6_regDef[DT].address;
    buf[1]  = 0x00 | TLE493D_A2B6_calculateConfigurationParityBit(sensor);
    // buf[1]  = regMap[TLE493D_A2B6_regDef[DT].address] & ~(TLE493D_A2B6_regDef[DT].mask);
    *bufLen = 2;
}


bool TLE493D_A2B6_enableTemperatureMeasurements(Sensor_ts *sensor) {
    uint8_t transBuffer[2];
    uint8_t bufLen = 0;

    TLE493D_A2B6_getTemperatureMeasurementsBuffer(sensor, transBuffer, &bufLen);
    // return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, NULL, 0);
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, sensor->regMapSize);
}


// TODO: Must be set in conjunction with MOD1 and MOD2 in order to set FP bit correctly
// -> read regmap first to get value for register @ 0x12
// -> then set PRD bit and recalc. FP bit in MOD2
// -> then write regMap from MOD1 to MOD2
bool TLE493D_A2B6_setSlowUpdates(Sensor_ts *sensor) {
    sensor->regMap[TLE493D_A2B6_regDef[PRD].address]  = 0x80;

    uint8_t buf[2];
    buf[0]  = TLE493D_A2B6_regDef[PRD].address;
    buf[1]  = 0x80;

    return sensor->comLibIF->transfer.i2c_transfer(sensor, buf, 2, sensor->regMap, sensor->regMapSize);
}


bool TLE493D_A2B6_setDefaultConfig(Sensor_ts *sensor) {
    // TLE493D_A2B6_setSlowUpdates(sensor);
    bool b = TLE493D_A2B6_enable1ByteMode(sensor);
    b |= TLE493D_A2B6_enableTemperatureMeasurements(sensor);

    return b;
}


// TODO: rename to readRegisterMap ? And writeRegisterMap when changing sensor values ? For consistency.
bool TLE493D_A2B6_updateRegisterMap(Sensor_ts *sensor) {
    // Currently only 1 interface is supported per sensor, either I2C or SPI for some 3rd generation sensors.
    // In case multiple interfaces are supported, switch according to IF type and call appropriate function.
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}
