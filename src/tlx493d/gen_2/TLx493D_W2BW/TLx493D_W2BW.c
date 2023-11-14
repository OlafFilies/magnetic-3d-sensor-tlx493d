/**
 * @file        TLx493D_W2BW.c
 * @author      Infineon Technologies AG
 * @brief       Contains the implementation of the complete sensor functionality
 * @copyright   Copyright (c) 2023
 *
 * SPDX-License-Identifier: MIT
 */

// std includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

// common to same generation of sensors
#include "tlx493d_gen_2_common_defines.h"
#include "tlx493d_gen_2_common.h"

// sensor specific includes
#include "TLx493D_W2BW_defines.h"
#include "TLx493D_W2BW.h"


typedef enum {
    BX_MSB = 0,
    BY_MSB,
    BZ_MSB,
    TEMP_MSB,
    BX_LSB, BY_LSB,
    TEMP_LSB, ID, BZ_LSB,
    P, FF, CF, T, PD3, PD0, FRM,
    XL_MSB, 
    XH_MSB,
    YL_MSB, 
    YH_MSB, 
    ZL_MSB, 
    ZH_MSB,
    WA, WU, XH_LSB, XL_LSB,
    YH_LSB, YL_LSB,
    ZH_LSB, ZL_LSB, 
    DT, AM, TRIG, X2, TL_MAG, CP,
    FP, IICADR, PR, CA, INT, MODE,
    PRD,
    X4,
    TYPE, HWV
} TLx493D_W2BW_registerNames_te;

TLx493D_Register_t TLx493D_W2BW_regDef[] = {
    {BX_MSB,    TLx493D_READ_MODE_e,        0x00,   0xFF,   0,  8},
    {BY_MSB,    TLx493D_READ_MODE_e,        0x01,   0xFF,   0,  8},
    {BZ_MSB,    TLx493D_READ_MODE_e,        0x02,   0xFF,   0,  8},
    {TEMP_MSB,  TLx493D_READ_MODE_e,        0x03,   0xFF,   0,  8},
    {BX_LSB,    TLx493D_READ_MODE_e,        0x04,   0xF0,   4,  4},
    {BY_LSB,    TLx493D_READ_MODE_e,        0x04,   0x0F,   0,  4},
    {TEMP_LSB,  TLx493D_READ_MODE_e,        0x05,   0xC0,   6,  2},
    {ID,        TLx493D_READ_MODE_e,        0x05,   0x30,   4,  2},
    {BZ_LSB,    TLx493D_READ_MODE_e,        0x05,   0x0F,   0,  4},
    {P,         TLx493D_READ_MODE_e,        0x06,   0x80,   7,  1},
    {FF,        TLx493D_READ_MODE_e,        0x06,   0x40,   6,  1},
    {CF,        TLx493D_READ_MODE_e,        0x06,   0x20,   5,  1},
    {T,         TLx493D_READ_MODE_e,        0x06,   0x10,   4,  1},
    {PD3,       TLx493D_READ_MODE_e,        0x06,   0x08,   3,  1},
    {PD0,       TLx493D_READ_MODE_e,        0x06,   0x04,   2,  1},
    {FRM,       TLx493D_READ_MODE_e,        0x06,   0x03,   0,  2},
    {XL_MSB,    TLx493D_READ_WRITE_MODE_e,  0x07,   0xFF,   0,  8},
    {XH_MSB,    TLx493D_READ_WRITE_MODE_e,  0x08,   0xFF,   0,  8},
    {YL_MSB,    TLx493D_READ_WRITE_MODE_e,  0x09,   0xFF,   0,  8},
    {YH_MSB,    TLx493D_READ_WRITE_MODE_e,  0x0A,   0xFF,   0,  8},
    {ZL_MSB,    TLx493D_READ_WRITE_MODE_e,  0x0B,   0xFF,   0,  8},
    {ZH_MSB,    TLx493D_READ_WRITE_MODE_e,  0x0C,   0xFF,   0,  8},
    {WA,        TLx493D_READ_MODE_e,        0x0D,   0x80,   7,  1},
    {WU,        TLx493D_READ_WRITE_MODE_e,  0x0D,   0x40,   6,  1},
    {XH_LSB,    TLx493D_READ_WRITE_MODE_e,  0x0D,   0x38,   3,  3},
    {XL_LSB,    TLx493D_READ_WRITE_MODE_e,  0x0D,   0x07,   0,  3},
    {YH_LSB,    TLx493D_READ_WRITE_MODE_e,  0x0E,   0x38,   3,  3},
    {YL_LSB,    TLx493D_READ_WRITE_MODE_e,  0x0E,   0x07,   0,  3},
    {ZH_LSB,    TLx493D_READ_WRITE_MODE_e,  0x0F,   0x38,   3,  3},
    {ZL_LSB,    TLx493D_READ_WRITE_MODE_e,  0x0F,   0x07,   0,  3},
    {DT,        TLx493D_READ_WRITE_MODE_e,  0x10,   0x80,   7,  1},
    {AM,        TLx493D_READ_WRITE_MODE_e,  0x10,   0x40,   6,  1},
    {TRIG,      TLx493D_READ_WRITE_MODE_e,  0x10,   0x30,   4,  2},
    {X2,        TLx493D_READ_WRITE_MODE_e,  0x10,   0x08,   3,  1},
    {TL_MAG,    TLx493D_READ_WRITE_MODE_e,  0x10,   0x06,   1,  2},
    {CP,        TLx493D_READ_WRITE_MODE_e,  0x10,   0x01,   0,  1},
    {FP,        TLx493D_READ_WRITE_MODE_e,  0x11,   0x80,   7,  1},
    {IICADR,    TLx493D_READ_WRITE_MODE_e,  0x11,   0x60,   5,  2},
    {PR,        TLx493D_READ_WRITE_MODE_e,  0x11,   0x10,   4,  1},
    {CA,        TLx493D_READ_WRITE_MODE_e,  0x11,   0x08,   3,  1},
    {INT,       TLx493D_READ_WRITE_MODE_e,  0x11,   0x04,   2,  1},
    {MODE,      TLx493D_READ_WRITE_MODE_e,  0x11,   0x03,   0,  2},
    {PRD,       TLx493D_READ_WRITE_MODE_e,  0x13,   0x80,   7,  1},
    {X4,        TLx493D_WRITE_MODE_e,       0x14,   0x01,   0,  1},
    {TYPE,      TLx493D_READ_MODE_e,        0x16,   0x30,   4,  2},
    {HWV,       TLx493D_READ_MODE_e,        0x16,   0x0F,   0,  4}
};


typedef enum { 
               TEMP2_REG  = 0x05,
               DIAG_REG   = 0x06,
               CONFIG_REG = 0x10,
               MOD1_REG   = 0x11,
               MOD2_REG   = 0x13,
               VER_REG    = 0x16 } SpecialRegisters_te;

TLx493D_CommonFunctions_t TLx493D_W2BW_commonFunctions = {
    .init                           = TLx493D_W2BW_init,
    .deinit                         = TLx493D_W2BW_deinit,

    .readRegisters                  = tlx493d_common_readRegisters,

    .calculateTemperature           = TLx493D_W2BW_calculateTemperature,
    .getTemperature                 = TLx493D_W2BW_getTemperature,
    
    .calculateMagneticField         = TLx493D_W2BW_calculateMagneticField,
    .getMagneticField               = TLx493D_W2BW_getMagneticField,
    
    .calculateMagneticFieldAndTemperature = TLx493D_W2BW_calculateMagneticFieldAndTemperature,
    .getMagneticFieldAndTemperature = TLx493D_W2BW_getMagneticFieldAndTemperature,

    // .enableTemperatureMeasurement  = TLx493D_W2BW_enableTemperatureMeasurement,
    // .disableTemperatureMeasurement = TLx493D_W2BW_disableTemperatureMeasurement,
    // .enableAngularMeasurement      = tlx493d_gen_2_enableAngularMeasurement,
    // .disableAngularMeasurement     = tlx493d_gen_2_disableAngularMeasurement,

    .setTrigger                     = TLx493D_W2BW_setTrigger,


    .setDefaultConfig               = TLx493D_W2BW_setDefaultConfig,
    .setIICAddress                  = TLx493D_W2BW_setIICAddress,

    .enableInterrupt                = TLx493D_W2BW_enableInterrupt,
    .disableInterrupt               = TLx493D_W2BW_disableInterrupt,

    .setPowerMode                   = TLx493D_W2BW_setPowerMode,
    .setUpdateRate                  = TLx493D_W2BW_setUpdateRate,


    .hasWakeUp                      = TLx493D_W2BW_hasWakeUp,
    .isWakeUpEnabled                = TLx493D_W2BW_isWakeUpEnabled,
    .enableWakeUpMode               = TLx493D_W2BW_enableWakeUpMode,
    .disableWakeUpMode              = TLx493D_W2BW_disableWakeUpMode,

    .setLowerWakeUpThresholdX       = TLx493D_W2BW_setLowerWakeUpThresholdX,
    .setLowerWakeUpThresholdY       = TLx493D_W2BW_setLowerWakeUpThresholdY,
    .setLowerWakeUpThresholdZ       = TLx493D_W2BW_setLowerWakeUpThresholdZ,

    .setUpperWakeUpThresholdX       = TLx493D_W2BW_setUpperWakeUpThresholdX,
    .setUpperWakeUpThresholdY       = TLx493D_W2BW_setUpperWakeUpThresholdY,
    .setUpperWakeUpThresholdZ       = TLx493D_W2BW_setUpperWakeUpThresholdZ,

    .setWakeUpThresholdsAsInteger   = TLx493D_W2BW_setWakeUpThresholdsAsInteger,
    .setWakeUpThresholds            = TLx493D_W2BW_setWakeUpThresholds,

    .calculateConfigurationParity   = TLx493D_W2BW_calculateConfigurationParityBit,
    
    .setResetValues                 = TLx493D_W2BW_setResetValues,
};

bool TLx493D_W2BW_init(TLx493D_t *sensor) {
    // sensor->regMap                  = (uint8_t*)malloc(sizeof(uint8_t) * GEN_2_REG_MAP_SIZE);
    // sensor->regDef                  = TLx493D_W2BW_regDef;
    // // sensor->commonBitfields         = (CommonBitfields_ts) { .CP = CP, .FP = FP, .ID = ID, .P = P, .FF = FF, .CF = CF, .T = T, .PD3 = PD3, .PD0 = PD0, .FRM = FRM, .PRD = PRD, .TYPE = TYPE, .HWV = HWV,
    // //                                                          .DT = DT, .AM = AM, .TRIG = TRIG, .X2 = X2, .TL_MAG = TL_MAG, .IICADR = IICADR, .PR = PR, .CA = CA, .INT = INT, .MODE = MODE,
    // //                                                          .BX_MSB = BX_MSB, .BY_MSB = BY_MSB, .BZ_MSB = BZ_MSB, .TEMP_MSB = TEMP_MSB,
    // //                                                          .BX_LSB = BX_LSB, .BY_LSB = BY_LSB, .TEMP_LSB = TEMP_LSB, .BZ_LSB = BZ_LSB, .TEMP2 = TEMP2_REG,
    // //                                                         };
    // // sensor->commonRegisters         = (CommonRegisters_ts) { .DIAG = DIAG_REG, .CONFIG = CONFIG_REG, .MOD1 = MOD1_REG, .MOD2 = MOD2_REG, .VER = VER_REG };

    // sensor->functions               = &TLx493D_W2BW_commonFunctions;
    // sensor->regMapSize              = GEN_2_REG_MAP_SIZE;
    // sensor->sensorType              = TLx493D_W2BW_e;
    // sensor->comIFType               = TLx493D_I2C_e;
    // sensor->comLibIF                = NULL;
    // sensor->comLibObj.i2c_obj       = NULL;

    // memset(sensor->regMap, 0, sensor->regMapSize);

    tlx493d_setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return tlx493d_common_init(sensor, GEN_2_REG_MAP_SIZE, TLx493D_W2BW_regDef, &TLx493D_W2BW_commonFunctions, TLx493D_W2BW_e, TLx493D_I2C_e);

    // return true;
}

bool TLx493D_W2BW_deinit(TLx493D_t *sensor) {
    return tlx493d_common_deinit(sensor);
    // free(sensor->regMap);
    // free(sensor->comLibObj.i2c_obj);
   
    // sensor->regMap = NULL;
    // sensor->comLibObj.i2c_obj = NULL;

    // return true;
}


bool TLx493D_W2BW_readRegisters(TLx493D_t *sensor) {
    return tlx493d_common_readRegisters(sensor);
}


void TLx493D_W2BW_calculateTemperature(TLx493D_t *sensor, double *temp) {
    tlx493d_gen_2_calculateTemperature(sensor, temp, TEMP_MSB, TEMP_LSB);
    // int16_t value = 0;

    // tlx493d_common_concatBytes(sensor, TEMP_MSB, TEMP_LSB, &value);

    // value <<= 2;
    // *temp = (double)((((double) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF);
}

bool TLx493D_W2BW_getTemperature(TLx493D_t *sensor, double *temp) {
    return tlx493d_common_getTemperature(sensor, temp);
    // if (tlx493d_common_readRegisters(sensor)) {
    //     TLx493D_W2BW_calculateTemperature(sensor, temp);
    //     return true;
    // }

    // return false;
}

void TLx493D_W2BW_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    tlx493d_gen_2_calculateMagneticField(sensor, x, y, z, BX_MSB, BX_LSB, BY_MSB, BY_LSB, BZ_MSB, BZ_LSB);
    // int16_t valueX = 0, valueY = 0, valueZ = 0;

    // tlx493d_common_concatBytes(sensor, BX_MSB, BX_LSB, &valueX);
    // tlx493d_common_concatBytes(sensor, BY_MSB, BY_LSB, &valueY);
    // tlx493d_common_concatBytes(sensor, BZ_MSB, BZ_LSB, &valueZ);
    
    // *x = ((double) valueX) * GEN_2_MAG_FIELD_MULT;
    // *y = ((double) valueY) * GEN_2_MAG_FIELD_MULT;
    // *z = ((double) valueZ) * GEN_2_MAG_FIELD_MULT;
}

bool TLx493D_W2BW_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    return tlx493d_common_getMagneticField(sensor, x, y, z);
    // if (tlx493d_common_readRegisters(sensor)) {
    //     TLx493D_W2BW_calculateMagneticField(sensor, x, y, z);
    //     return true;
    // }
    
    // return false;
}


void TLx493D_W2BW_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_W2BW_calculateMagneticField(sensor, x, y, z);
    TLx493D_W2BW_calculateTemperature(sensor, temp);
}


bool TLx493D_W2BW_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    return tlx493d_common_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


// bool TLx493D_W2BW_enableTemperatureMeasurement(TLx493D_t *sensor) {
//     bool b = tlx493d_common_readRegisters(sensor);

//     tlx493d_common_setBitfield(sensor, DT, 0);
//     tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));
//     // sensor->regMap[sensor->commonRegisters.CONFIG] = (sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask) | TLx493D_W2BW_calculateConfigurationParityBit(sensor);

//     return b && tlx493d_common_writeRegister(sensor, DT);
// }

// bool TLx493D_W2BW_disableTemperatureMeasurement(TLx493D_t *sensor) {
//     bool b = tlx493d_common_readRegisters(sensor);

//     tlx493d_common_setBitfield(sensor, DT, 1);
//     tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));
//     // sensor->regMap[sensor->commonRegisters.CONFIG] = (sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask) | TLx493D_W2BW_calculateConfigurationParityBit(sensor);
    
//     return b && tlx493d_common_writeRegister(sensor, DT);
// }


// static bool TLx493D_W2BW_enableTemperatureMeasurements(TLx493D_t *sensor) {
//     bool b = false;

//     sensor->regMap[sensor->regDef[CP].address] = 0;

//     tlx493d_common_setBitfield(sensor, DT, 0);
//     tlx493d_common_setBitfield(sensor, CP, 0);

//     b = tlx493d_common_writeRegister(sensor, DT);

//     return b;
// }


bool TLx493D_W2BW_setTrigger(TLx493D_t *sensor, TLx493D_TriggerType_t trigger) {
    // return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, trigger);
    return false;
}



bool TLx493D_W2BW_setDefaultConfig(TLx493D_t *sensor) {
    // bool b = TLx493D_W2BW_enable1ByteMode(sensor);
    // return b && TLx493D_W2BW_enableTemperature(sensor);

    sensor->regMap[CONFIG_REG] = 0x01;
    sensor->regMap[MOD1_REG]   = 0x80;
    sensor->regMap[MOD2_REG]   = 0x00;

    tlx493d_common_writeRegister(sensor, CP);
    tlx493d_common_writeRegister(sensor, FP);

    tlx493d_common_setBitfield(sensor, CA, 0);
    tlx493d_common_setBitfield(sensor, INT, 1);

    if( TLx493D_W2BW_enable1ByteMode(sensor) ) {
        return true;
    }

    return false;
}


bool TLx493D_W2BW_setIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t address) {
    return tlx493d_gen_2_setIICAddress(sensor, IICADR, FP, address);
}


bool TLx493D_W2BW_enableInterrupt(TLx493D_t *sensor) {
    bool b = tlx493d_common_readRegisters(sensor);

    tlx493d_common_setBitfield(sensor, INT, 0);
    tlx493d_common_setBitfield(sensor, CA, 1);
    tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));
    // sensor->regMap[sensor->commonRegisters.MOD1] = (sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[FP].mask) | TLx493D_W2BW_calculateFuseParityBit(sensor);
    
    return b && tlx493d_common_writeRegister(sensor, INT); 
}

bool TLx493D_W2BW_disableInterrupt(TLx493D_t *sensor) {
    bool b = tlx493d_common_readRegisters(sensor);

    tlx493d_common_setBitfield(sensor, INT, 1);
    tlx493d_common_setBitfield(sensor, CA, 1);
    tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));
    // sensor->regMap[sensor->commonRegisters.MOD1] = (sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[FP].mask) | TLx493D_W2BW_calculateFuseParityBit(sensor);

    return b && tlx493d_common_writeRegister(sensor, INT);
}


bool TLx493D_W2BW_setPowerMode(TLx493D_t *sensor, uint8_t mode) {
    return tlx493d_gen_2_setPowerMode(sensor, MODE, FP, mode);
}


bool TLx493D_W2BW_setUpdateRate(TLx493D_t *sensor, uint8_t bit) {
    bool b = tlx493d_common_readRegisters(sensor);

    tlx493d_common_setBitfield(sensor, PRD, bit);
    tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

    uint8_t tx_buffer[4] = { MOD1_REG,
                             sensor->regMap[MOD1_REG],
                             sensor->regMap[MOD1_REG + 1],
                             sensor->regMap[MOD1_REG + 2] 
                            };

    return b && transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}


bool TLx493D_W2BW_hasValidData(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidData(sensor);
}


bool TLx493D_W2BW_isFunctional(TLx493D_t *sensor) {
    return tlx493d_gen_2_isFunctional(sensor);
}


bool TLx493D_W2BW_hasWakeUp(TLx493D_t *sensor) {
    // return tlx493d_gen_2_hasValidData(sensor);
    return true;
}


bool TLx493D_W2BW_isWakeUpEnabled(TLx493D_t *sensor) {
    uint8_t bitFieldValue = 0;
    tlx493d_common_getBitfield(sensor, WA, &bitFieldValue);

    return bitFieldValue;
}

bool TLx493D_W2BW_enableWakeUpMode(TLx493D_t *sensor) {
    uint8_t bitFieldValue = 0;
    tlx493d_common_getBitfield(sensor, T, &bitFieldValue);

    if (!bitFieldValue) {
        tlx493d_common_setBitfield(sensor, WU, 1);
        tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

        uint8_t tx_buffer[5] = { sensor->regDef[WU].address,
                                  sensor->regMap[sensor->regDef[WU].address],
                                  sensor->regMap[sensor->regDef[WU].address + 1],
                                  sensor->regMap[sensor->regDef[WU].address + 2],
                                  sensor->regMap[sensor->regDef[WU].address + 3]
                                };
        
        return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
    }
    else {
        return false;
    }
}

bool TLx493D_W2BW_disableWakeUpMode(TLx493D_t *sensor) {
    tlx493d_common_setBitfield(sensor, WU, 0);
    tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[5] = { sensor->regDef[WU].address,
                                sensor->regMap[sensor->regDef[WU].address],
                                sensor->regMap[sensor->regDef[WU].address + 1],
                                sensor->regMap[sensor->regDef[WU].address + 2],
                                sensor->regMap[sensor->regDef[WU].address + 3]
                            };
    
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setLowerWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_common_setBitfield(sensor, XL_MSB, msb);
    tlx493d_common_setBitfield(sensor, XL_LSB, lsb);

    tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}


bool TLx493D_W2BW_setLowerWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_common_setBitfield(sensor, YL_MSB, msb);
    tlx493d_common_setBitfield(sensor, YL_LSB, lsb);

    tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setLowerWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_common_setBitfield(sensor, ZL_MSB, msb);
    tlx493d_common_setBitfield(sensor, ZL_LSB, lsb);

    tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

// Upper Limit is 2047 and lower -2048
bool TLx493D_W2BW_setUpperWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_common_setBitfield(sensor, XH_MSB, msb);
    tlx493d_common_setBitfield(sensor, XH_LSB, lsb);

    tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setUpperWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_common_setBitfield(sensor, YH_MSB, msb);
    tlx493d_common_setBitfield(sensor, YH_LSB, lsb);

    tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setUpperWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_common_setBitfield(sensor, ZH_MSB, msb);
    tlx493d_common_setBitfield(sensor, ZH_LSB, lsb);

    tlx493d_common_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
    bool returnValue = TLx493D_W2BW_setLowerWakeUpThresholdX(sensor, xl_th);
    returnValue &= TLx493D_W2BW_setUpperWakeUpThresholdX(sensor, xh_th);

    returnValue &= TLx493D_W2BW_setLowerWakeUpThresholdY(sensor, yl_th);
    returnValue &= TLx493D_W2BW_setUpperWakeUpThresholdY(sensor, yh_th);
    
    returnValue &= TLx493D_W2BW_setLowerWakeUpThresholdZ(sensor, zl_th);
    return  returnValue & TLx493D_W2BW_setUpperWakeUpThresholdZ(sensor, zh_th);
}

// thesholds im mT, to be converted to proper format
bool TLx493D_W2BW_setWakeUpThresholds(TLx493D_t *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh) {
    return false;
}



bool TLx493D_W2BW_softReset(TLx493D_t *sensor) {
        return true;
}

//static
bool TLx493D_W2BW_enable1ByteMode(TLx493D_t *sensor) {
    // bool b = false;

    // sensor->regMap[sensor->regDef[FP].address] = 0;

    // tlx493d_common_setBitfield(sensor, FP, 0);
    // tlx493d_common_setBitfield(sensor, PR, 1);
    // tlx493d_common_setBitfield(sensor, INT, 1);
    // tlx493d_common_setBitfield(sensor, CA, 1);

    // b = tlx493d_common_writeRegister(sensor, FP);

    // return b;

    tlx493d_common_setBitfield(sensor, PR, 1);
    tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));
    return tlx493d_common_writeRegister(sensor, FP);
}


// // Fuse/mode parity bit FP
// uint8_t TLx493D_W2BW_calculateFuseParityBit(TLx493D_t *sensor) {
// 	// compute parity of MOD1 register
// 	uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[FP].mask);

// 	// add parity of MOD2:PRD register bits
// 	parity ^= calculateParity(sensor->regMap[sensor->commonRegisters.MOD2] & sensor->regDef[PRD].mask);

// 	return getOddParity(parity) << sensor->regDef[FP].offset;
// }

// Configuration parity bit CP
uint8_t TLx493D_W2BW_calculateConfigurationParityBit(TLx493D_t *sensor) {
	// uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
	// return getEvenParity(parity);

	uint8_t parity = 0;
    for(uint8_t x = sensor->regDef[XL_MSB].address; x <= sensor->regDef[ZH_MSB].address; x++) {
        parity ^= sensor->regMap[x];
    }
    parity ^= sensor->regMap[sensor->regDef[WU].address] & ~sensor->regDef[WA].mask;
    parity ^= sensor->regMap[sensor->regDef[YH_LSB].address] & (sensor->regDef[YH_LSB].mask & sensor->regDef[YL_LSB].mask);
    parity ^= sensor->regMap[sensor->regDef[ZH_LSB].address] & (sensor->regDef[ZH_LSB].mask & sensor->regDef[ZL_LSB].mask);
    parity ^= sensor->regMap[CONFIG_REG] & ~sensor->regDef[CP].mask;
    
    parity = tlx493d_common_calculateParity(parity);

	return tlx493d_common_getOddParity(parity);
}


void TLx493D_W2BW_setResetValues(TLx493D_t *sensor) {
    sensor->regMap[0x07] = 0x80;
    sensor->regMap[0x08] = 0x7F;
    sensor->regMap[0x09] = 0x80;
    sensor->regMap[0x0A] = 0x7F;
    sensor->regMap[0x0B] = 0x80;
    sensor->regMap[0x0C] = 0x7F;
    sensor->regMap[0x0D] = 0x38;
    sensor->regMap[0x0E] = 0x38;
    sensor->regMap[0x0F] = 0x38;  
    sensor->regMap[0x10] = 0x01; // CONFIG
    sensor->regMap[0x11] = 0x80; // MOD1 : A0 : 0x80, A1 : 0x20, A2 : 0x40, A3 : 0xE0
    sensor->regMap[0x13] = 0x00; // MOD2
    sensor->regMap[0x14] = 0x00; // CONFIG2
}
