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
    {TL_MAG,    READ_WRITE_MODE_e,  0x10,   0x06,   1,  2},
    {CP,        READ_WRITE_MODE_e,  0x10,   0x01,   0,  1},
    {X2,        READ_WRITE_MODE_e,  0x10,   0x08,   3,  1},
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

CommonFunctions_ts TLV493D_A2BW_commonFunctions = {
    .init                               = TLV493D_A2BW_init,
    .deinit                             = TLV493D_A2BW_deinit,

    .getTemperature                     = TLV493D_A2BW_getTemperature,
    .updateGetTemperature               = TLV493D_A2BW_updateGetTemperature,

    .getFieldValues                     = TLV493D_A2BW_getFieldValues,
    .updateGetFieldValues               = TLV493D_A2BW_updateGetFieldValues,

    .reset                              = TLV493D_A2BW_reset,
    .getDiagnosis                       = TLV493D_A2BW_getDiagnosis,
    .calculateParity                    = TLV493D_A2BW_calculateParity,

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

bool TLV493D_A2BW_getTemperature(Sensor_ts *sensor, float *temp) {
    int16_t value = 0;

    value = ((uint16_t)sensor->regMap[sensor->regDef[TEMP_MSB].address]) << 8;
    value |= (uint16_t)(sensor->regMap[sensor->regDef[TEMP_LSB].address] & sensor->regDef[TEMP_LSB].mask);
    value >>= 4;

    *temp = (float)((((float)value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF);

    return true;
}

bool TLV493D_A2BW_updateGetTemperature(Sensor_ts *sensor, float *temp) {
    bool b = TLV493D_A2BW_updateRegisterMap(sensor);
    return b && TLV493D_A2BW_getTemperature(sensor, temp);
}

bool TLV493D_A2BW_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    valueX = sensor->regMap[sensor->regDef[BX_MSB].address] << 8;
    valueX |= (sensor->regMap[sensor->regDef[BX_LSB].address] & sensor->regDef[BX_LSB].mask);
    valueX >>= 4;

    valueY = sensor->regMap[sensor->regDef[BY_MSB].address] << 8;
    valueY |= (sensor->regMap[sensor->regDef[BY_LSB].address] & sensor->regDef[BY_LSB].mask) << sensor->regDef[BY_LSB].offset;
    valueY >>= 4;

    valueZ = sensor->regMap[sensor->regDef[BZ_MSB].address] << 8;
    valueZ |= (sensor->regMap[sensor->regDef[BZ_LSB].address] & sensor->regDef[BZ_LSB].mask) << sensor->regDef[BZ_LSB].offset;
    valueZ >>= 4;

    *x = ((float) valueX) * GEN_2_MAG_FIELD_MULT;
    *y = ((float) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((float) valueZ) * GEN_2_MAG_FIELD_MULT;

    return true;
}

bool TLV493D_A2BW_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    bool b = TLV493D_A2BW_updateRegisterMap(sensor);
    return b && TLV493D_A2BW_getFieldValues(sensor, x, y, z);
}

bool TLV493D_A2BW_reset(Sensor_ts *sensor) {
        return true;
}

bool TLV493D_A2BW_getDiagnosis(Sensor_ts *sensor) {
        return true;
}

void TLV493D_A2BW_calculateParity(Sensor_ts *sensor) {
    uint8_t regParity = 0;

    regParity ^= sensor->regMap[sensor->regDef[CP].address];

    regParity ^= (regParity >> 1);
    regParity ^= (regParity >> 2);
    regParity ^= (regParity >> 4);

    setBitfield(sensor, CP, (regParity & 0x01));

    regParity = 0;

    regParity ^= sensor->regMap[sensor->regDef[FP].address];
    regParity ^= (sensor->regMap[sensor->regDef[PRD].address] & sensor->regDef[PRD].mask);

    regParity ^= (regParity >> 1);
    regParity ^= (regParity >> 2);
    regParity ^= (regParity >> 4);

    setBitfield(sensor, FP, (regParity & 0x01));
}

bool TLV493D_A2BW_updateRegisterMap(Sensor_ts *sensor) {
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}

static bool TLV493D_A2BW_enable1ByteMode(Sensor_ts *sensor) {
    bool b = false;

    sensor->regMap[sensor->regDef[FP].address] = 0;

    setBitfield(sensor, FP, 1);
    setBitfield(sensor, PR, 1);
    setBitfield(sensor, INT, 1);

    b = writeRegister(sensor, FP);

    return b;
}

static bool TLV493D_A2BW_enableTemperatureMeasurements(Sensor_ts *sensor) {
    bool b = updateRegisterMap(sensor);

    setBitfield(sensor, DT, 0);
    b = writeRegister(sensor, DT);

    return b;
}

bool TLV493D_A2BW_setDefaultConfig(Sensor_ts *sensor) {
    bool b = TLV493D_A2BW_enable1ByteMode(sensor);
    return b && TLV493D_A2BW_enableTemperatureMeasurements(sensor);
}

bool TLV493D_A2BW_enableTemperature(Sensor_ts *sensor) {
    bool b = updateRegisterMap(sensor);

    setBitfield(sensor, DT, 0);
    calculateParity(sensor);

    return b && writeRegister(sensor, DT);
}

bool TLV493D_A2BW_disableTemperature(Sensor_ts *sensor) {
    bool b = updateRegisterMap(sensor);

    setBitfield(sensor, DT, 1);
    calculateParity(sensor);
    
    return b && writeRegister(sensor, DT);
}

bool TLV493D_A2BW_enableInterrupt(Sensor_ts *sensor) {
    bool b = updateRegisterMap(sensor);

    setBitfield(sensor, INT, 0);
    setBitfield(sensor, TRIG, 0);
    calculateParity(sensor);

    b = writeRegister(sensor, TRIG);
    return b && writeRegister(sensor, INT); 
}

bool TLV493D_A2BW_disableInterrupt(Sensor_ts *sensor) {
    return true;
}