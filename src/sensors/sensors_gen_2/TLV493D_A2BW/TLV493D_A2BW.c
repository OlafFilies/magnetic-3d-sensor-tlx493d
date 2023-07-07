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
    {BX_MSB,    READ_MODE_e,    0x00,   0xFF,   0},
    {BY_MSB,    READ_MODE_e,    0x01,   0xFF,   0},
    {BZ_MSB,    READ_MODE_e,    0x02,   0xFF,   0},
    {TEMP_MSB,  READ_MODE_e,    0x03,   0xFF,   0},
    {BX_LSB,    READ_MODE_e,    0x04,   0xF0,   4},
    {BY_LSB,    READ_MODE_e,    0x04,   0x0F,   0},
    {TEMP_LSB,  READ_MODE_e,    0x05,   0xC0,   6},
    {ID,        READ_MODE_e,    0x05,   0x30,   4},
    {BZ_LSB,    READ_MODE_e,    0x05,   0x0F,   0},
    {P,         READ_MODE_e,    0x06,   0x80,   7},
    {FF,        READ_MODE_e,    0x06,   0x40,   6},
    {CF,        READ_MODE_e,    0x06,   0x20,   5},
    {T,         READ_MODE_e,    0x06,   0x10,   4},
    {PD3,       READ_MODE_e,    0x06,   0x08,   3},
    {PD0,       READ_MODE_e,    0x06,   0x04,   2},
    {FRM,       READ_MODE_e,    0x06,   0x03,   0},
    {DT,        WRITE_MODE_e,   0x10,   0x80,   7},
    {AM,        WRITE_MODE_e,   0x10,   0x40,   6},
    {TRIG,      WRITE_MODE_e,   0x10,   0x30,   4},
    {X2,        WRITE_MODE_e,   0x10,   0x08,   3},
    {TL_MAG,    WRITE_MODE_e,   0x10,   0x06,   1},
    {CP,        WRITE_MODE_e,   0x10,   0x01,   0},
    {FP,        WRITE_MODE_e,   0x11,   0x80,   7},
    {IICADR,    WRITE_MODE_e,   0x11,   0x60,   5},
    {PR,        WRITE_MODE_e,   0x11,   0x10,   4},
    {CA,        WRITE_MODE_e,   0x11,   0x08,   3},
    {INT,       WRITE_MODE_e,   0x11,   0x04,   2},
    {MODE,      WRITE_MODE_e,   0x11,   0x03,   0},
    {PRD,       WRITE_MODE_e,   0x13,   0x80,   7},
    {X4,        WRITE_MODE_e,   0x14,   0x01,   0},
    {TYPE,      WRITE_MODE_e,   0x16,   0x30,   4},
    {HWV,       WRITE_MODE_e,   0x16,   0x0F,   0}
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
};

bool TLV493D_A2BW_init(Sensor_ts *sensor, SupportedComLibraryInterfaceTypes_te comLibIF) {
    if( comLibIF != I2C_e ) {
        assert(0);
        return false;
    }

    sensor->regMap                  = (uint8_t*)malloc(sizeof(uint8_t) * GEN_2_REG_MAP_SIZE);
    sensor->regDef                  = TLV493D_A2BW_regDef;
    sensor->functions               = &TLV493D_A2BW_commonFunctions;
    sensor->regMapSize              = GEN_2_REG_MAP_SIZE;
    sensor->sensorType              = TLV493D_A2BW_e;
    sensor->comIFType               = comLibIF;
    sensor->comLibIF                = &comLibIF_i2c;
    sensor->comLibObj.i2c_obj       = NULL;

    setI2CParameters(&sensor->comLibIFParams, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return true;
}

bool TLV493D_A2BW_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->regDef);
    free(sensor->comLibIF);
    free(sensor->functions);

    sensor->regMap = NULL;
    sensor->regDef = NULL;
    sensor->comLibIF = NULL;
    sensor->functions = NULL;
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
        return true;
}

bool TLV493D_A2BW_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
        return true;
}

bool TLV493D_A2BW_reset(Sensor_ts *sensor) {
        return true;
}

bool TLV493D_A2BW_getDiagnosis(Sensor_ts *sensor) {
        return true;
}

bool TLV493D_A2BW_calculateParity(Sensor_ts *sensor) {
        return true;
}

bool TLV493D_A2BW_updateRegisterMap(Sensor_ts *sensor) {
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}

static void TLV493D_A2BW_get1ByteModeBuffer(Sensor_ts *sensor, uint8_t *buf, uint8_t *bufLen) {
    buf[0] = sensor->regDef[MODE].address;
    buf[1] = sensor->regDef[FP].mask | sensor->regDef[PR].mask | sensor->regDef[INT].mask;
    *bufLen = 2;
}

static bool TLV493D_A2BW_enable1ByteMode(Sensor_ts *sensor) {
    uint8_t transBuffer[sensor->regMapSize];
    uint8_t bufLen = 0;

    TLV493D_A2BW_get1ByteModeBuffer(sensor, transBuffer, &bufLen);
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, sensor->regMapSize);
}

static void TLV493D_A2BW_getTemperatureMeasurementsBuffer(Sensor_ts *sensor, uint8_t *buf, uint8_t *bufLen) {
    buf[0] = sensor->regDef[DT].address;
    buf[1] = sensor->regMap[sensor->regDef[DT].address] & ~(sensor->regDef[DT].mask);
    *bufLen = 2;
}

static bool TLV493D_A2BW_enableTemperatureMeasurements(Sensor_ts *sensor) {
    uint8_t transBuffer[sensor->regMapSize];
    uint8_t bufLen = 0;

    TLV493D_A2BW_getTemperatureMeasurementsBuffer(sensor, transBuffer, &bufLen);
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, sensor->regMapSize);
}

bool TLV493D_A2BW_setDefaultConfig(Sensor_ts *sensor) {
    bool b = TLV493D_A2BW_enable1ByteMode(sensor);
    return b && TLV493D_A2BW_enableTemperatureMeasurements(sensor);
}