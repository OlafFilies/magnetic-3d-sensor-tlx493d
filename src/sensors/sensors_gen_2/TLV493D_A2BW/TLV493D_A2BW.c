/**
 * @file        TLV493D_A2BW.c
 * @author      Infineon Technologies AG
 * @brief       Contains the implementation of the complete sensor functionality
 * @copyright   Copyright (c) 2023
 *
 * SPDX-License-Identifier: MIT
 */

/** Sensor specific includes */
#include "TLV493D_A2BW.h"

extern ComLibraryFunctions_ts comLibIF_i2c;
extern void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr);

typedef enum {
    BX_MSB = 0,
    BY_MSB,
    BZ_MSB,
    TEMP_MSB,
    BX_LSB, BY_LSB,
    TEMP_LSB, ID, BZ_LSB,
    P, FF, CF, T, PD3, PD0, FRM,
    DT, AM, TRIG, X2, TL_MAG, CP,
    FP, IICADR, PR, CA, INT, MODE,
    PRD,
    X4,
    TYPE, HWV
} TLV493D_A2BW_registerNames_te;

Register_ts TLV493D_A2BW_regDef[] = {
    {BX_MSB,    READ_MODE_e,        0x00,   0xFF,   0,  8},
    {BY_MSB,    READ_MODE_e,        0x01,   0xFF,   0,  8},
    {BZ_MSB,    READ_MODE_e,        0x02,   0xFF,   0,  8},
    {TEMP_MSB,  READ_MODE_e,        0x03,   0xFF,   0,  8},
    {BX_LSB,    READ_MODE_e,        0x04,   0xF0,   4,  4},
    {BY_LSB,    READ_MODE_e,        0x04,   0x0F,   0,  4},
    {TEMP_LSB,  READ_MODE_e,        0x05,   0xC0,   6,  2},
    {ID,        READ_MODE_e,        0x05,   0x30,   4,  2},
    {BZ_LSB,    READ_MODE_e,        0x05,   0x0F,   0,  4},
    {P,         READ_MODE_e,        0x06,   0x80,   7,  1},
    {FF,        READ_MODE_e,        0x06,   0x40,   6,  1},
    {CF,        READ_MODE_e,        0x06,   0x20,   5,  1},
    {T,         READ_MODE_e,        0x06,   0x10,   4,  1},
    {PD3,       READ_MODE_e,        0x06,   0x08,   3,  1},
    {PD0,       READ_MODE_e,        0x06,   0x04,   2,  1},
    {FRM,       READ_MODE_e,        0x06,   0x03,   0,  2},
    {DT,        READ_WRITE_MODE_e,  0x10,   0x80,   7,  1},
    {AM,        READ_WRITE_MODE_e,  0x10,   0x40,   6,  1},
    {TRIG,      READ_WRITE_MODE_e,  0x10,   0x30,   4,  2},
    {X2,        READ_WRITE_MODE_e,  0x10,   0x08,   3,  1},
    {TL_MAG,    READ_WRITE_MODE_e,  0x10,   0x06,   1,  2},
    {CP,        READ_WRITE_MODE_e,  0x10,   0x01,   0,  1},
    {FP,        READ_WRITE_MODE_e,  0x11,   0x80,   7,  1},
    {IICADR,    READ_WRITE_MODE_e,  0x11,   0x60,   5,  2},
    {PR,        READ_WRITE_MODE_e,  0x11,   0x10,   4,  1},
    {CA,        READ_WRITE_MODE_e,  0x11,   0x08,   3,  1},
    {INT,       READ_WRITE_MODE_e,  0x11,   0x04,   2,  1},
    {MODE,      READ_WRITE_MODE_e,  0x11,   0x03,   0,  2},
    {PRD,       READ_WRITE_MODE_e,  0x13,   0x80,   7,  1},
    {X4,        WRITE_MODE_e,       0x14,   0x01,   0,  1},
    {TYPE,      READ_MODE_e,        0x16,   0x30,   4,  2},
    {HWV,       READ_MODE_e,        0x16,   0x0F,   0,  4}
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

CommonFunctions_ts TLV493D_A2BW_commonFunctions = {
    .init                               = TLV493D_A2BW_init,
    .deinit                             = TLV493D_A2BW_deinit,

    .calculateTemperature               = TLV493D_A2BW_calculateTemperature,
    .getTemperature                     = TLV493D_A2BW_getTemperature,
    
    .calculateFieldValues               = TLV493D_A2BW_calculateFieldValues,
    .getFieldValues                     = TLV493D_A2BW_getFieldValues,
    
    .getSensorValues                    = TLV493D_A2BW_getSensorValues,

    .setDefaultConfig                   = TLV493D_A2BW_setDefaultConfig,
    .updateRegisterMap                  = TLV493D_A2BW_updateRegisterMap,

    .enableTemperature                  = TLV493D_A2BW_enableTemperature,
    .disableTemperature                 = TLV493D_A2BW_disableTemperature,

    .enableInterrupt                    = TLV493D_A2BW_enableInterrupt,
    .disableInterrupt                   = TLV493D_A2BW_disableInterrupt
};

bool TLV493D_A2BW_init(Sensor_ts *sensor) {
    sensor->regMap                  = (uint8_t*)malloc(sizeof(uint8_t) * GEN_2_REG_MAP_SIZE);
    sensor->regDef                  = TLV493D_A2BW_regDef;
    sensor->commonBitfields         = (CommonBitfields_ts) {.ID = ID, .P = P, .FF = FF, .CF = CF, .T = T, .PD3 = PD3, .PD0 = PD0, .FRM = FRM, .PRD = PRD, .TYPE = TYPE, .HWV = HWV,
                                                            .BX_MSB = BX_MSB, .BY_MSB = BY_MSB, .BZ_MSB = BZ_MSB, .TEMP_MSB = TEMP_MSB,
                                                            .BX_LSB = BX_LSB, .BY_LSB = BY_LSB, .TEMP_LSB = TEMP_LSB, .BZ_LSB = BZ_LSB,
                                                            .TEMP2 = TEMP2_REG, .DIAG = DIAG_REG, .CONFIG = CONFIG_REG, .MOD1 = MOD1_REG, .MOD2 = MOD2_REG, .VER = VER_REG};
    sensor->functions               = &TLV493D_A2BW_commonFunctions;
    sensor->regMapSize              = GEN_2_REG_MAP_SIZE;
    sensor->sensorType              = TLV493D_A2BW_e;
    sensor->comIFType               = I2C_e;
    sensor->comLibIF                = NULL;
    sensor->comLibObj.i2c_obj       = NULL;

    setI2CParameters(&sensor->comLibIFParams, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return true;
}

bool TLV493D_A2BW_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);
   
    sensor->regMap = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    return true;
}

void TLV493D_A2BW_calculateTemperature(Sensor_ts *sensor, float *temp) {
    int16_t value = 0;

    // value = ((uint16_t)sensor->regMap[sensor->regDef[TEMP_MSB].address]) << 8;
    // value |= (uint16_t)(sensor->regMap[sensor->regDef[TEMP_LSB].address] & sensor->regDef[TEMP_LSB].mask);
    // value >>= 4;
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.TEMP_MSB], &sensor->regDef[sensor->commonBitfields.TEMP_LSB], &value);

    value <<= 2;
    *temp = (float)((((float)value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF);
}

bool TLV493D_A2BW_getTemperature(Sensor_ts *sensor, float *temp) {
    if (TLV493D_A2BW_updateRegisterMap(sensor)) {
        TLV493D_A2BW_calculateTemperature(sensor, temp);
        return true;
    }

    return false;
}

void TLV493D_A2BW_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BX_MSB], &sensor->regDef[sensor->commonBitfields.BX_LSB], &valueX);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BY_MSB], &sensor->regDef[sensor->commonBitfields.BY_LSB], &valueY);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BZ_MSB], &sensor->regDef[sensor->commonBitfields.BZ_LSB], &valueZ);
    // valueX = sensor->regMap[sensor->regDef[BX_MSB].address] << 8;
    // valueX |= (sensor->regMap[sensor->regDef[BX_LSB].address] & sensor->regDef[BX_LSB].mask);
    // valueX >>= 4;

    // valueY = sensor->regMap[sensor->regDef[BY_MSB].address] << 8;
    // valueY |= (sensor->regMap[sensor->regDef[BY_LSB].address] & sensor->regDef[BY_LSB].mask) << sensor->regDef[BY_LSB].offset;
    // valueY >>= 4;

    // valueZ = sensor->regMap[sensor->regDef[BZ_MSB].address] << 8;
    // valueZ |= (sensor->regMap[sensor->regDef[BZ_LSB].address] & sensor->regDef[BZ_LSB].mask) << sensor->regDef[BZ_LSB].offset;
    // valueZ >>= 4;

    *x = ((float) valueX) * GEN_2_MAG_FIELD_MULT;
    *y = ((float) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((float) valueZ) * GEN_2_MAG_FIELD_MULT;
}

bool TLV493D_A2BW_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    if (TLV493D_A2BW_updateRegisterMap(sensor)) {
        TLV493D_A2BW_calculateFieldValues(sensor, x, y, z);
        return true;
    }
    
    return false;
}

bool TLV493D_A2BW_getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp) {
    return true;
}

bool TLV493D_A2BW_reset(Sensor_ts *sensor) {
        return true;
}

bool TLV493D_A2BW_getDiagnosis(Sensor_ts *sensor) {
        return true;
}

// Fuse/mode parity bit FP
uint8_t TLV493D_A2BW_calculateFuseParityBit(Sensor_ts *sensor) {
	// compute parity of MOD1 register
	uint8_t parity = calculateParity(sensor->regMap[sensor->commonBitfields.MOD1] & ~sensor->regDef[FP].mask);

	// add parity of MOD2:PRD register bits
	parity ^= calculateParity(sensor->regMap[sensor->commonBitfields.MOD2] & sensor->regDef[PRD].mask);

	return getOddParity(parity) << sensor->regDef[FP].offset;
}

// Configuration parity bit CP
uint8_t TLV493D_A2BW_calculateConfigurationParityBit(Sensor_ts *sensor) {
	// compute parity of Config register
	uint8_t parity = calculateParity(sensor->regMap[sensor->commonBitfields.CONFIG] & ~sensor->regDef[CP].mask);

	return getEvenParity(calculateParity(parity)) << sensor->regDef[CP].offset;
}

// void TLV493D_A2BW_calculateParity(Sensor_ts *sensor) {
//     uint8_t regParity = 0;

//     setBitfield(sensor, CP, 1);
//     setBitfield(sensor, FP, 1);

//     regParity ^= sensor->regMap[sensor->regDef[CP].address];

//     regParity ^= (regParity >> 1);
//     regParity ^= (regParity >> 2);
//     regParity ^= (regParity >> 4);

//     setBitfield(sensor, CP, (regParity ^ 0x01));

//     regParity = 0;

//     regParity ^= sensor->regMap[sensor->regDef[FP].address];
//     regParity ^= (sensor->regMap[sensor->regDef[PRD].address] & sensor->regDef[PRD].mask);

//     regParity ^= (regParity >> 1);
//     regParity ^= (regParity >> 2);
//     regParity ^= (regParity >> 4);

//     setBitfield(sensor, FP, (regParity & 0x01));
// }

bool TLV493D_A2BW_updateRegisterMap(Sensor_ts *sensor) {
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}

static bool TLV493D_A2BW_enable1ByteMode(Sensor_ts *sensor) {
    bool b = false;

    sensor->regMap[sensor->regDef[FP].address] = 0;

    setBitfield(sensor, FP, 0);
    setBitfield(sensor, PR, 1);
    setBitfield(sensor, INT, 1);
    setBitfield(sensor, CA, 1);

    b = writeRegister(sensor, FP);

    return b;
}

static bool TLV493D_A2BW_enableTemperatureMeasurements(Sensor_ts *sensor) {
    bool b = false;

    sensor->regMap[sensor->regDef[CP].address] = 0;

    setBitfield(sensor, DT, 0);
    setBitfield(sensor, CP, 0);

    b = writeRegister(sensor, DT);

    return b;
}

bool TLV493D_A2BW_setDefaultConfig(Sensor_ts *sensor) {
    bool b = TLV493D_A2BW_enable1ByteMode(sensor);
    return b && TLV493D_A2BW_enableTemperature(sensor);
}

bool TLV493D_A2BW_enableTemperature(Sensor_ts *sensor) {
    bool b = updateRegisterMap(sensor);

    setBitfield(sensor, DT, 0);
    sensor->regMap[sensor->commonBitfields.CONFIG] = (sensor->regMap[sensor->commonBitfields.CONFIG] & ~sensor->regDef[CP].mask) | TLV493D_A2BW_calculateConfigurationParityBit(sensor);

    return b && writeRegister(sensor, DT);
}

bool TLV493D_A2BW_disableTemperature(Sensor_ts *sensor) {
    bool b = updateRegisterMap(sensor);

    setBitfield(sensor, DT, 1);
    sensor->regMap[sensor->commonBitfields.CONFIG] = (sensor->regMap[sensor->commonBitfields.CONFIG] & ~sensor->regDef[CP].mask) | TLV493D_A2BW_calculateConfigurationParityBit(sensor);
    
    return b && writeRegister(sensor, DT);
}

bool TLV493D_A2BW_enableInterrupt(Sensor_ts *sensor) {
    bool b = updateRegisterMap(sensor);

    setBitfield(sensor, INT, 0);
    setBitfield(sensor, CA, 1);
    sensor->regMap[sensor->commonBitfields.MOD1] = (sensor->regMap[sensor->commonBitfields.MOD1] & ~sensor->regDef[FP].mask) | TLV493D_A2BW_calculateFuseParityBit(sensor);
    
    return b && writeRegister(sensor, INT); 
}

bool TLV493D_A2BW_disableInterrupt(Sensor_ts *sensor) {
    bool b = updateRegisterMap(sensor);

    setBitfield(sensor, INT, 1);
    setBitfield(sensor, CA, 1);
    sensor->regMap[sensor->commonBitfields.MOD1] = (sensor->regMap[sensor->commonBitfields.MOD1] & ~sensor->regDef[FP].mask) | TLV493D_A2BW_calculateFuseParityBit(sensor);

    return b && writeRegister(sensor, INT);
}