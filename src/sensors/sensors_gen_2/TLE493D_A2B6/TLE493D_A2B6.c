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
    // { PRD,      READ_WRITE_MODE_e, 0x13, 0xE0, 5, 3 },
    { PRD,      READ_WRITE_MODE_e, 0x13, 0x80, 7, 1 },
    { TYPE,     READ_MODE_e,       0x16, 0x30, 4, 2 },
    { HWV,      READ_MODE_e,       0x16, 0x0F, 0, 4 }
};


typedef enum {
               LOW_POWER_MODE         = 0x00,
               MASTER_CONTROLLED_MODE = 0x01,
               RESERVED_MODE          = 0x10,
               FAST_MODE              = 0x11 } TLE493D_A2B6_modes_te;


typedef enum { 
               TEMP2_REG  = 0x05,
               DIAG_REG   = 0x06,
               CONFIG_REG = 0x10,
               MOD1_REG   = 0x11,
               MOD2_REG   = 0x13,
               VER_REG    = 0x16 } SpecialRegisters_te;


CommonFunctions_ts TLE493D_A2B6_commonFunctions = {
                                .init                  = TLE493D_A2B6_init,
                                .deinit                = TLE493D_A2B6_deinit,

                                .calculateTemperature  = TLE493D_A2B6_calculateTemperature,
                                .getTemperature        = TLE493D_A2B6_getTemperature,

                                .calculateFieldValues  = TLE493D_A2B6_calculateFieldValues,
                                .getFieldValues        = TLE493D_A2B6_getFieldValues,

                                .getSensorValues       = TLE493D_A2B6_getSensorValues,

                                // .reset                 = TLE493D_A2B6_reset,
                                .hasValidData          = TLE493D_A2B6_hasValidData,
                                .isFunctional          = TLE493D_A2B6_isFunctional,

                                .setDefaultConfig      = TLE493D_A2B6_setDefaultConfig,
                                .updateRegisterMap     = TLE493D_A2B6_updateRegisterMap,
                              };


// TODO: add parameter IICAddress or ad function to set address.
bool TLE493D_A2B6_init(Sensor_ts *sensor) {
    // regMap must be sensor specific, not sensor type specific, therefore malloc.
    sensor->regMap            = (uint8_t*) malloc(sizeof(uint8_t) * TLE493D_A2B6_REGISTER_MAP_SIZE);
    sensor->regDef            = TLE493D_A2B6_regDef;
    sensor->commonBitfields   = (CommonBitfields_ts) { .ID = ID, .P = P, .FF = FF, .CF = CF, .T = T, .PD3 = PD3, .PD0 = PD0, .FRM = FRM, .PRD = PRD, .TYPE = TYPE, .HWV = HWV,
                                                       .BX_MSB = BX_MSB, .BY_MSB = BY_MSB, .BZ_MSB = BZ_MSB, .TEMP_MSB = TEMP_MSB,
                                                       .BX_LSB = BX_LSB, .BY_LSB = BY_LSB, .TEMP_LSB = TEMP_LSB, .BZ_LSB = BZ_LSB,
                                                       .TEMP2 = TEMP2_REG, .DIAG = DIAG_REG, .CONFIG = CONFIG_REG, .MOD1 = MOD1_REG, .MOD2 = MOD2_REG, .VER = VER_REG };

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


void TLE493D_A2B6_calculateTemperature(Sensor_ts *sensor, float *temp) {
    int16_t value = 0;

    // concatBytes(sensor, &sensor->regDef[TEMP_MSB], &sensor->regDef[TEMP_LSB], &value);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.TEMP_MSB], &sensor->regDef[sensor->commonBitfields.TEMP_LSB], &value);

    value <<= 2; // least significant 2 bits are implicit, therefore shift by 2 !
    *temp = (((float) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF;
}


bool TLE493D_A2B6_getTemperature(Sensor_ts *sensor, float *temp) {
    if( TLE493D_A2B6_updateRegisterMap(sensor) ) {
        TLE493D_A2B6_calculateTemperature(sensor, temp);
        return true;
    }

    return false;
}


void TLE493D_A2B6_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    // concatBytes(sensor, &sensor->regDef[BX_MSB], &sensor->regDef[BX_LSB], &valueX);
    // concatBytes(sensor, &sensor->regDef[BY_MSB], &sensor->regDef[BY_LSB], &valueY);
    // concatBytes(sensor, &sensor->regDef[BZ_MSB], &sensor->regDef[BZ_LSB], &valueZ);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BX_MSB], &sensor->regDef[sensor->commonBitfields.BX_LSB], &valueX);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BY_MSB], &sensor->regDef[sensor->commonBitfields.BY_LSB], &valueY);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BZ_MSB], &sensor->regDef[sensor->commonBitfields.BZ_LSB], &valueZ);

    *x = ((float) valueX) * GEN_2_MAG_FIELD_MULT;
    *y = ((float) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((float) valueZ) * GEN_2_MAG_FIELD_MULT;
}


bool TLE493D_A2B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    if( TLE493D_A2B6_updateRegisterMap(sensor) ) {
        TLE493D_A2B6_calculateFieldValues(sensor, x, y, z);
        return true;
    }

    return false;
}


void TLE493D_A2B6_calculateSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp) {
    TLE493D_A2B6_calculateFieldValues(sensor, x, y, z);
    TLE493D_A2B6_calculateTemperature(sensor, temp);
}


bool TLE493D_A2B6_getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp) {
    if( TLE493D_A2B6_updateRegisterMap(sensor) ) {
        TLE493D_A2B6_calculateSensorValues(sensor, x, y, z, temp);
        return true;
    }

    return false;
}


// // TODO: not yet working !
// bool TLE493D_A2B6_reset(Sensor_ts *sensor) {
//     // frameworkReset(sensor);

//     return true;
// }


// Calculate bus (data) parity bit P
uint8_t TLE493D_A2B6_calculateBusParityBit(Sensor_ts *sensor) {
	// compute bus parity of data values in registers 0 to 5
	uint8_t parity = sensor->regMap[0];

	for (uint8_t i = 1; i < 6; ++i) {
		parity ^= sensor->regMap[i];
	}

	return getOddParity(calculateParity(parity)) << sensor->regDef[sensor->commonBitfields.P].offset;
}


// // TODO: define enum to differentiate the various diagnosis messages possible across sensors
// bool TLE493D_A2B6_getDiagnosis(Sensor_ts *sensor) {
//     return true;
// }


// TODO: implement !
bool TLE493D_A2B6_hasValidData(Sensor_ts *sensor) {
    return true;
}


// TODO: implement !
bool TLE493D_A2B6_isFunctional(Sensor_ts *sensor) {
    return true;
}


bool TLE493D_A2B6_hasValidBusParity(Sensor_ts *sensor) {
    uint8_t p = sensor->commonBitfields.P;
    return TLE493D_A2B6_calculateBusParityBit(sensor) == (sensor->regMap[sensor->regDef[p].address] && sensor->regDef[p].mask);
}


bool TLE493D_A2B6_hasValidFuseParity(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.FF];
    return sensor->regMap[bf->address] && bf->mask != 0;
}


bool TLE493D_A2B6_hasValidConfigurationParity(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.CF];
    return sensor->regMap[bf->address] && bf->mask != 0;
}


bool TLE493D_A2B6_hasValidTBit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.T];
    return sensor->regMap[bf->address] && bf->mask == 0;
}


bool TLE493D_A2B6_hasValidPD3Bit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.PD3];
    return sensor->regMap[bf->address] && bf->mask != 0;
}


bool TLE493D_A2B6_hasValidPD0Bit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.PD0];
    return sensor->regMap[bf->address] && bf->mask != 0;
}


uint8_t TLE493D_A2B6_getID(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.ID];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}


uint8_t TLE493D_A2B6_getFrameCounter(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.FRM];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}


uint8_t TLE493D_A2B6_getType(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.TYPE];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}


uint8_t TLE493D_A2B6_getHWV(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.HWV];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}


// Fuse/mode parity bit FP
uint8_t TLE493D_A2B6_calculateFuseParityBit(Sensor_ts *sensor) {
	// compute parity of MOD1 register
	uint8_t parity = calculateParity(sensor->regMap[sensor->commonBitfields.MOD1] & ~sensor->regDef[FP].mask);

	// add parity of MOD2:PRD register bits
	parity ^= calculateParity(sensor->regMap[sensor->commonBitfields.MOD2] & sensor->regDef[PRD].mask);

	return getOddParity(parity) << sensor->regDef[FP].offset;
}


// Configuration parity bit CP
uint8_t TLE493D_A2B6_calculateConfigurationParityBit(Sensor_ts *sensor) {
	// compute parity of Config register
	uint8_t parity = calculateParity(sensor->regMap[sensor->commonBitfields.CONFIG] & ~sensor->regDef[CP].mask);

	return getEvenParity(calculateParity(parity)) << sensor->regDef[CP].offset;
}


// TODO: test !
bool TLE493D_A2B6_writeRegister(Sensor_ts* sensor, uint8_t bitField) {
    Register_ts *bf  = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        uint8_t buf[2] = { bf->address, sensor->regMap[bf->address] };

        return sensor->comLibIF->transfer.i2c_transfer(sensor, buf, sizeof(buf), NULL, 0);
    }

    return false;
}


/**
 * - set 1-byte mode
 * - disable interrupts
 * - set parity flag
 * 
*/
bool TLE493D_A2B6_enable1ByteMode(Sensor_ts *sensor) {
    uint8_t mod1 = sensor->commonBitfields.MOD1;

    // TODO: the next 2 lines must go into init !
    sensor->regMap[mod1]  = 0;
    sensor->regMap[sensor->commonBitfields.MOD2]  = 0; // because of FP calculation !

    // setBitfield(sensor, MOD1_BITFIELD, 0);
    // setBitfield(sensor, MOD2_BITFIELD, 0); // because of FP calculation !

    // this is already setDefaultConfig ! Should set only PR bit !
    sensor->regMap[mod1]  = sensor->regDef[PR].mask
                                                  | sensor->regDef[INT].mask;
//                                                  | (MASTER_CONTROLLED_MODE << sensor->regDef[MODE].offset);

    sensor->regMap[mod1] |= TLE493D_A2B6_calculateFuseParityBit(sensor);

    uint8_t buf[2] = { mod1, sensor->regMap[mod1] };

    return sensor->comLibIF->transfer.i2c_transfer(sensor, buf, sizeof(buf), sensor->regMap, sensor->regMapSize);
}


// TODO: provide ?? implement !
bool TLE493D_A2B6_disable1ByteMode(Sensor_ts *sensor) {
    return false;
}


/**
 * - enable temperature measurements
 * - hardcoded version also 
 *  - preserve all bits except parity and lower TL_MAG bit
*/
bool TLE493D_A2B6_enableTemperatureMeasurements(Sensor_ts *sensor) {
    uint8_t buf[2] = { sensor->regDef[DT].address, 0x00 | TLE493D_A2B6_calculateConfigurationParityBit(sensor) };
    // buf[1]  = regMap[sensor->regDef[DT].address] & ~(sensor->regDef[DT].mask);

    return sensor->comLibIF->transfer.i2c_transfer(sensor, buf, sizeof(buf), sensor->regMap, sensor->regMapSize);
}


// TODO: implement !
bool TLE493D_A2B6_disableTemperatureMeasurements(Sensor_ts *sensor) {
    return false;
}


// TODO: Must be set in conjunction with MOD1 and MOD2 in order to set FP bit correctly
// -> read regmap first to get value for register @ 0x12
// -> then set PRD bit and recalc. FP bit in MOD2
// -> then write regMap from MOD1 to MOD2
bool TLE493D_A2B6_setSlowUpdates(Sensor_ts *sensor) {
    uint8_t mod1 = sensor->commonBitfields.MOD1;

    sensor->regMap[sensor->regDef[PRD].address]  |= sensor->regDef[PRD].mask;
    sensor->regMap[mod1]                          = (sensor->regMap[mod1] & ~sensor->regDef[FP].mask) | TLE493D_A2B6_calculateFuseParityBit(sensor);

    // uint8_t buf[2] = { sensor->regDef[PRD].address, 0x80 };
    uint8_t buf[4] = { sensor->regDef[mod1].address,
                       sensor->regMap[mod1],
                       sensor->regMap[mod1 + 1],
                       sensor->regMap[sensor->commonBitfields.MOD2] };

    return sensor->comLibIF->transfer.i2c_transfer(sensor, buf, sizeof(buf), sensor->regMap, sensor->regMapSize);
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
