/**
 * @file        TLV493D_A2BW.c
 * @author      Infineon Technologies AG
 * @brief       Contains the implementation of the complete sensor functionality
 * @copyright   Copyright (c) 2023
 *
 * SPDX-License-Identifier: MIT
 */

<<<<<<< HEAD
=======
/** Std includes */
#include <stdio.h>
#include <stdlib.h>

>>>>>>> 1746abda37c1f13d68754602bb3d39c192b594b6
/** Sensor specific includes */
#include "TLV493D_A2BW.h"

extern ComLibraryFunctions_ts comLibIF_i2c;
extern void setI2CParameters(ComLibraryParameters_ts *params);

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

Register_ts TLV493D_A2BW_regDef[HWV+1] = {
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
    {INT,       WRITE_MODE_e,   0x11,   0x06,   2},
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
<<<<<<< HEAD
    assert(comLibIF == I2C_e);

=======
>>>>>>> 1746abda37c1f13d68754602bb3d39c192b594b6
    sensor->regMap                  = (uint8_t*)(sizeof(uint8_t) * GEN_2_REG_MAP_SIZE);
    sensor->regDef                  = TLV493D_A2BW_regDef;
    sensor->functions               = &TLV493D_A2BW_commonFunctions;
    sensor->regMapSize              = GEN_2_REG_MAP_SIZE;
    sensor->sensorType              = TLV493D_A2BW_e;
    sensor->commIFType              = comLibIF;
    sensor->comLibIF                = &comLibIF_i2c;
    setI2CParameters(&sensor->comLibIFParams);

    return true;
<<<<<<< HEAD
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

    return true;
=======
>>>>>>> 1746abda37c1f13d68754602bb3d39c192b594b6
}