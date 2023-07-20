/**
 * @file        TLV493D_W2B6.c
 * @author      Infineon Technologies AG
 * @brief       Contains the implementation of the complete sensor functionality
 * @copyright   Copyright (c) 2023
 *
 * SPDX-License-Identifier: MIT
 */

/** Sensor specific includes */
#include "TLE493D_W2B6.h"

extern ComLibraryFunctions_ts comLibIF_i2c;
extern void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr);


/*
  Listing of all register names for this sensor.
  Used to index "TLE493D_W2B6_regDef" defined below, so index values must match !
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
               XL,
               XH,
               YL,
               YH,
               ZL,
               ZH,
               WA,
               WU,
               XH_LSB,
               XL_LSB,
               TST,
               YH_LSB,
               YL_LSB,
               PH,
               ZH_LSB,
               ZL_LSB,
               DT,
               AM,
               TRIG,
               X2,
               TL_mag,
               CP,
               FP,
               IICaddr,
               PR,
               CA,
               INT,
               MODE,
               PRD,
               Type,
               HWV } TLE493D_W2B6_registerNames_te;


Register_ts TLE493D_W2B6_regDef[] = {
    {BX_MSB,    READ_MODE_e, 0x00, 0xFF, 0, 8},
    {BY_MSB,    READ_MODE_e, 0x01, 0xFF, 0, 8},
    {BZ_MSB,    READ_MODE_e, 0x02, 0xFF, 0, 8},
    {TEMP_MSB,  READ_MODE_e, 0x03, 0xFF, 0, 8},
    {BX_LSB,    READ_MODE_e, 0x04, 0xF0, 4, 4},
    {BY_LSB,    READ_MODE_e, 0x04, 0x0F, 0, 4},
    {TEMP_LSB,  READ_MODE_e, 0x05, 0xC0, 6, 2},
    {ID,        READ_MODE_e, 0x05, 0x30, 4, 2},
    {BZ_LSB,    READ_MODE_e, 0x05, 0x0F, 0, 4},
    {P,         READ_MODE_e, 0x06, 0x80, 7, 1},
    {FF,        READ_MODE_e, 0x06, 0x40, 6, 1},
    {CF,        READ_MODE_e, 0x06, 0x20, 5, 1},
    {T,         READ_MODE_e, 0x06, 0x10, 4, 1},
    {PD3,       READ_MODE_e, 0x06, 0x08, 2, 1},
    {PD0,       READ_MODE_e, 0x06, 0x04, 2, 1},
    {FRM,       READ_MODE_e, 0x06, 0x03, 0, 2},
    {XL,       WRITE_MODE_e, 0x07, 0xFF, 0, 8},
    {XH,       WRITE_MODE_e, 0x08, 0xFF, 0, 8},
    {YL,       WRITE_MODE_e, 0x09, 0xFF, 0, 8},
    {YH,       WRITE_MODE_e, 0x0A, 0xFF, 0, 8},
    {ZL,       WRITE_MODE_e, 0x0B, 0xFF, 0, 8},
    {ZH,       WRITE_MODE_e, 0x0C, 0xFF, 0, 8},
    {WA,       READ_MODE_e,  0x0D, 0x80, 7, 1},
    {WU,       WRITE_MODE_e, 0x0D, 0x40, 6, 1},
    {XH_LSB,   WRITE_MODE_e, 0x0D, 0x38, 3, 3},
    {XL_LSB,   WRITE_MODE_e, 0x0D, 0x07, 0, 3},
    {TST,      WRITE_MODE_e, 0x0E, 0xC0, 6, 2},
    {YH_LSB,   WRITE_MODE_e, 0x0E, 0x38, 3, 3},  
    {YL_LSB,   WRITE_MODE_e, 0x0E, 0x07, 0, 3},
    {PH,       WRITE_MODE_e, 0x0F, 0xC0, 6, 2},
    {ZH_LSB,   WRITE_MODE_e, 0x0F, 0x38, 3, 3},
    {ZL_LSB,   WRITE_MODE_e, 0x0F, 0x07, 0, 3},
    {DT,        WRITE_MODE_e, 0x10, 0x80, 7, 1},
    {AM,        WRITE_MODE_e, 0x10, 0x40, 6, 1},
    {TRIG,      WRITE_MODE_e, 0x10, 0x30, 4, 2},
    {X2,        WRITE_MODE_e, 0x10, 0x08, 3, 1},
    {TL_mag,    WRITE_MODE_e, 0x10, 0x06, 1, 2},
    {CP,        WRITE_MODE_e, 0x10, 0x01, 0, 1},
    {FP,        WRITE_MODE_e, 0x11, 0x80, 7, 1},
    {IICaddr,   WRITE_MODE_e, 0x11, 0x60, 5, 2},
    {PR,        WRITE_MODE_e, 0x11, 0x10, 4, 1},
    {CA,        WRITE_MODE_e, 0x11, 0x08, 3, 1},
    {INT,       WRITE_MODE_e, 0x11, 0x04, 2, 1},
    {MODE,      WRITE_MODE_e, 0x11, 0x03, 0, 2},
    {PRD,       WRITE_MODE_e, 0x13, 0x80, 7, 3},
    {Type,      WRITE_MODE_e, 0x16, 0x30, 4, 2},
    {HWV,       WRITE_MODE_e, 0x16, 0x0F, 0, 4}
};


CommonFunctions_ts TLE493D_W2B6_commonFunctions = {
                                .init                  = TLE493D_W2B6_init,
                                .deinit                = TLE493D_W2B6_deinit,

                                .setDefaultConfig      = TLE493D_W2B6_setDefaultConfig,
};

bool TLE493D_W2B6_init(Sensor_ts *sensor, SupportedComLibraryInterfaceTypes_te comLibIF) {
    // This sensor only supports I2C.
    if( comLibIF != I2C_e ) {
        assert(0);
        return false;
    }

    sensor->regMap            = (uint8_t*)malloc(sizeof(uint8_t) * GEN_2_REG_MAP_SIZE);
    sensor->regDef            = TLE493D_W2B6_regDef;
    sensor->functions         = &TLE493D_W2B6_commonFunctions;
    sensor->regMapSize        = GEN_2_REG_MAP_SIZE;
    sensor->sensorType        = TLE493D_W2B6_e;
    sensor->comIFType         = comLibIF;
    sensor->comLibIF          = &comLibIF_i2c;
    sensor->comLibObj.i2c_obj = NULL;

    setI2CParameters(&sensor->comLibIFParams, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return true;
}

bool TLE493D_W2B6_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);

    sensor->regMap = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    return true;
}

static void TLE493D_W2B6_get1ByteModeBuffer(Sensor_ts *sensor, uint8_t *buf, uint8_t *bufLen) {
    buf[0] = sensor->regDef[MODE].address;
    buf[1] = sensor->regDef[FP].mask | sensor->regDef[PR].mask | sensor->regDef[INT].mask;
    *bufLen = 2;
}

static bool TLE493D_W2B6_enable1ByteMode(Sensor_ts *sensor) {
    uint8_t transBuffer[sensor->regMapSize];
    uint8_t bufLen = 0;

    TLE493D_W2B6_get1ByteModeBuffer(sensor, transBuffer, &bufLen);
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, sensor->regMapSize);
}

bool TLE493D_W2B6_setDefaultConfig(Sensor_ts *sensor) {
    bool b = TLE493D_W2B6_enable1ByteMode(sensor);
    return b;
}