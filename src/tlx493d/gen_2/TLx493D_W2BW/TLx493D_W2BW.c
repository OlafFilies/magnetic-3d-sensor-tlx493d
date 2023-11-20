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
#include "TLx493D_W2BW_enums.h"
#include "TLx493D_W2BW.h"


TLx493D_Register_t TLx493D_W2BW_regDef[] = {
    { W2BW_BX_MSBS_e,    TLx493D_READ_MODE_e,        0x00,   0xFF,   0,  8 },
    { W2BW_BY_MSBS_e,    TLx493D_READ_MODE_e,        0x01,   0xFF,   0,  8 },
    { W2BW_BZ_MSBS_e,    TLx493D_READ_MODE_e,        0x02,   0xFF,   0,  8 },
    { W2BW_TEMP_MSBS_e,  TLx493D_READ_MODE_e,        0x03,   0xFF,   0,  8 },
    { W2BW_BX_LSBS_e,    TLx493D_READ_MODE_e,        0x04,   0xF0,   4,  4 },
    { W2BW_BY_LSBS_e,    TLx493D_READ_MODE_e,        0x04,   0x0F,   0,  4 },
    { W2BW_TEMP_LSBS_e,  TLx493D_READ_MODE_e,        0x05,   0xC0,   6,  2 },
    { W2BW_ID_e,         TLx493D_READ_MODE_e,        0x05,   0x30,   4,  2 },
    { W2BW_BZ_LSBS_e,    TLx493D_READ_MODE_e,        0x05,   0x0F,   0,  4 },
    { W2BW_P_e,          TLx493D_READ_MODE_e,        0x06,   0x80,   7,  1 },
    { W2BW_FF_e,         TLx493D_READ_MODE_e,        0x06,   0x40,   6,  1 },
    { W2BW_CF_e,         TLx493D_READ_MODE_e,        0x06,   0x20,   5,  1 },
    { W2BW_T_e,          TLx493D_READ_MODE_e,        0x06,   0x10,   4,  1 },
    { W2BW_PD3_e,        TLx493D_READ_MODE_e,        0x06,   0x08,   3,  1 },
    { W2BW_PD0_e,        TLx493D_READ_MODE_e,        0x06,   0x04,   2,  1 },
    { W2BW_FRM_e,        TLx493D_READ_MODE_e,        0x06,   0x03,   0,  2 },
    { W2BW_XL_MSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x07,   0xFF,   0,  8 },
    { W2BW_XH_MSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x08,   0xFF,   0,  8 },
    { W2BW_YL_MSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x09,   0xFF,   0,  8 },
    { W2BW_YH_MSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x0A,   0xFF,   0,  8 },
    { W2BW_ZL_MSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x0B,   0xFF,   0,  8 },
    { W2BW_ZH_MSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x0C,   0xFF,   0,  8 },
    { W2BW_WA_e,         TLx493D_READ_MODE_e,        0x0D,   0x80,   7,  1 },
    { W2BW_WU_e,         TLx493D_READ_WRITE_MODE_e,  0x0D,   0x40,   6,  1 },
    { W2BW_XH_LSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x0D,   0x38,   3,  3 },
    { W2BW_XL_LSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x0D,   0x07,   0,  3 },
    { W2BW_YH_LSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x0E,   0x38,   3,  3 },
    { W2BW_YL_LSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x0E,   0x07,   0,  3 },
    { W2BW_ZH_LSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x0F,   0x38,   3,  3 },
    { W2BW_ZL_LSBS_e,    TLx493D_READ_WRITE_MODE_e,  0x0F,   0x07,   0,  3 },
    { W2BW_DT_e,         TLx493D_READ_WRITE_MODE_e,  0x10,   0x80,   7,  1 },
    { W2BW_AM_e,         TLx493D_READ_WRITE_MODE_e,  0x10,   0x40,   6,  1 },
    { W2BW_TRIG_e,       TLx493D_READ_WRITE_MODE_e,  0x10,   0x30,   4,  2 },
    { W2BW_X2_e,         TLx493D_READ_WRITE_MODE_e,  0x10,   0x08,   3,  1 },
    { W2BW_TL_MAG_e,     TLx493D_READ_WRITE_MODE_e,  0x10,   0x06,   1,  2 },
    { W2BW_CP_e,         TLx493D_READ_WRITE_MODE_e,  0x10,   0x01,   0,  1 },
    { W2BW_FP_e,         TLx493D_READ_WRITE_MODE_e,  0x11,   0x80,   7,  1 },
    { W2BW_IICADR_e,     TLx493D_READ_WRITE_MODE_e,  0x11,   0x60,   5,  2 },
    { W2BW_PR_e,         TLx493D_READ_WRITE_MODE_e,  0x11,   0x10,   4,  1 },
    { W2BW_CA_e,         TLx493D_READ_WRITE_MODE_e,  0x11,   0x08,   3,  1 },
    { W2BW_INT_e,        TLx493D_READ_WRITE_MODE_e,  0x11,   0x04,   2,  1 },
    { W2BW_MODE_e,       TLx493D_READ_WRITE_MODE_e,  0x11,   0x03,   0,  2 },
    { W2BW_PRD_e,        TLx493D_READ_WRITE_MODE_e,  0x13,   0x80,   7,  1 },
    { W2BW_X4_e,         TLx493D_WRITE_MODE_e,       0x14,   0x01,   0,  1 },
    { W2BW_TYPE_e,       TLx493D_READ_MODE_e,        0x16,   0x30,   4,  2 },
    { W2BW_HWV_e,        TLx493D_READ_MODE_e,        0x16,   0x0F,   0,  4 },
};


TLx493D_CommonFunctions_t TLx493D_W2BW_commonFunctions = {
    .init                           = TLx493D_W2BW_init,
    .deinit                         = TLx493D_W2BW_deinit,

    .readRegisters                  = tlx493d_common_readRegisters,

    .calculateRawTemperature        = TLx493D_W2BW_calculateRawTemperature,
    .getRawTemperature              = TLx493D_W2BW_getRawTemperature,

    .calculateRawMagneticField      = TLx493D_W2BW_calculateRawMagneticField,
    .getRawMagneticField            = TLx493D_W2BW_getRawMagneticField,

    .calculateRawMagneticFieldAndTemperature = TLx493D_W2BW_calculateRawMagneticFieldAndTemperature,
    .getRawMagneticFieldAndTemperature       = TLx493D_W2BW_getRawMagneticFieldAndTemperature,

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

    // .setLowerWakeUpThresholdX       = TLx493D_W2BW_setLowerWakeUpThresholdX,
    // .setLowerWakeUpThresholdY       = TLx493D_W2BW_setLowerWakeUpThresholdY,
    // .setLowerWakeUpThresholdZ       = TLx493D_W2BW_setLowerWakeUpThresholdZ,

    // .setUpperWakeUpThresholdX       = TLx493D_W2BW_setUpperWakeUpThresholdX,
    // .setUpperWakeUpThresholdY       = TLx493D_W2BW_setUpperWakeUpThresholdY,
    // .setUpperWakeUpThresholdZ       = TLx493D_W2BW_setUpperWakeUpThresholdZ,

    .setWakeUpThresholdsAsInteger   = TLx493D_W2BW_setWakeUpThresholdsAsInteger,
    .setWakeUpThresholds            = TLx493D_W2BW_setWakeUpThresholds,

    .calculateConfigurationParity   = TLx493D_W2BW_calculateConfigurationParityBit,
    
    .setResetValues                 = TLx493D_W2BW_setResetValues,

    .selectIICAddress               = TLx493D_W2BW_selectIICAddress,

    .calculateRawMagneticFieldAtTemperature = TLx493D_W2BW_calculateRawMagneticFieldAtTemperature,

    .getSensitivityScaleFactor      = TLx493D_W2BW_getSensitivityScaleFactor,
};

bool TLx493D_W2BW_init(TLx493D_t *sensor) {
    return tlx493d_common_init(sensor, GEN_2_REG_MAP_SIZE, TLx493D_W2BW_regDef, &TLx493D_W2BW_commonFunctions, TLx493D_W2BW_e, TLx493D_I2C_e);
}

bool TLx493D_W2BW_deinit(TLx493D_t *sensor) {
    return tlx493d_common_deinit(sensor);
}


bool TLx493D_W2BW_readRegisters(TLx493D_t *sensor) {
    return tlx493d_common_readRegisters(sensor);
}


void TLx493D_W2BW_calculateRawTemperature(TLx493D_t *sensor, uint16_t *temperature) {
    tlx493d_gen_2_calculateRawTemperature(sensor, W2BW_TEMP_MSBS_e, W2BW_TEMP_LSBS_e, temperature);
}


bool TLx493D_W2BW_getRawTemperature(TLx493D_t *sensor, uint16_t *temperature) {
    return tlx493d_common_getRawTemperature(sensor, temperature);
}


void TLx493D_W2BW_calculateRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z) {
    tlx493d_gen_2_calculateRawMagneticField(sensor, W2BW_BX_MSBS_e, W2BW_BX_LSBS_e, W2BW_BY_MSBS_e, W2BW_BY_LSBS_e, W2BW_BZ_MSBS_e, W2BW_BZ_LSBS_e, x, y, z);
}


bool TLx493D_W2BW_getRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z) {
    return tlx493d_common_getRawMagneticField(sensor, x, y, z);
}


void TLx493D_W2BW_calculateRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature) {
    TLx493D_W2BW_calculateRawMagneticField(sensor, x, y, z);
    TLx493D_W2BW_calculateRawTemperature(sensor, temperature);
}


bool TLx493D_W2BW_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature) {
    return tlx493d_common_getRawMagneticFieldAndTemperature(sensor, x, y, z, temperature);
}


void TLx493D_W2BW_calculateTemperature(TLx493D_t *sensor, double *temp) {
    tlx493d_gen_2_calculateTemperature(sensor, W2BW_TEMP_MSBS_e, W2BW_TEMP_LSBS_e, temp);
    // int16_t value = 0;

    // tlx493d_common_concatBytes(sensor, W2BW_TEMP_MSBS_e, W2BW_TEMP_LSBS_e, &value);

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
    tlx493d_gen_2_calculateMagneticField(sensor, W2BW_BX_MSBS_e, W2BW_BX_LSBS_e, W2BW_BY_MSBS_e, W2BW_BY_LSBS_e, W2BW_BZ_MSBS_e, W2BW_BZ_LSBS_e, x, y, z);
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

    sensor->regMap[W2BW_CONFIG_REG_e] = 0x01;
    sensor->regMap[W2BW_MOD1_REG_e]   = 0x80;
    sensor->regMap[W2BW_MOD2_REG_e]   = 0x00;

    tlx493d_common_writeRegister(sensor, W2BW_CP_e);
    tlx493d_common_writeRegister(sensor, W2BW_FP_e);

    tlx493d_common_setBitfield(sensor, W2BW_CA_e, 0);
    tlx493d_common_setBitfield(sensor, W2BW_INT_e, 1);

    if( TLx493D_W2BW_enable1ByteMode(sensor) ) {
        return true;
    }

    return false;
}


bool TLx493D_W2BW_setIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t address) {
    return tlx493d_gen_2_setIICAddress(sensor, W2BW_IICADR_e, W2BW_FP_e, address);
}


bool TLx493D_W2BW_enableInterrupt(TLx493D_t *sensor) {
    bool b = tlx493d_common_readRegisters(sensor);

    tlx493d_common_setBitfield(sensor, W2BW_INT_e, 0);
    tlx493d_common_setBitfield(sensor, W2BW_CA_e, 1);
    tlx493d_common_setBitfield(sensor, W2BW_FP_e, tlx493d_gen_2_calculateFuseParity(sensor, W2BW_FP_e, W2BW_PRD_e));
    // sensor->regMap[sensor->commonRegisters.MOD1] = (sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[FP].mask) | TLx493D_W2BW_calculateFuseParityBit(sensor);
    
    return b && tlx493d_common_writeRegister(sensor, W2BW_INT_e); 
}

bool TLx493D_W2BW_disableInterrupt(TLx493D_t *sensor) {
    bool b = tlx493d_common_readRegisters(sensor);

    tlx493d_common_setBitfield(sensor, W2BW_INT_e, 1);
    tlx493d_common_setBitfield(sensor, W2BW_CA_e, 1);
    tlx493d_common_setBitfield(sensor, W2BW_FP_e, tlx493d_gen_2_calculateFuseParity(sensor, W2BW_FP_e, W2BW_PRD_e));
    // sensor->regMap[sensor->commonRegisters.MOD1] = (sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[FP].mask) | TLx493D_W2BW_calculateFuseParityBit(sensor);

    return b && tlx493d_common_writeRegister(sensor, W2BW_INT_e);
}


bool TLx493D_W2BW_setPowerMode(TLx493D_t *sensor, uint8_t mode) {
    return tlx493d_gen_2_setPowerMode(sensor, W2BW_MODE_e, W2BW_FP_e, mode);
}


bool TLx493D_W2BW_setUpdateRate(TLx493D_t *sensor, uint8_t bit) {
    bool b = tlx493d_common_readRegisters(sensor);

    tlx493d_common_setBitfield(sensor, W2BW_PRD_e, bit);
    tlx493d_common_setBitfield(sensor, W2BW_FP_e, tlx493d_gen_2_calculateFuseParity(sensor, W2BW_FP_e, W2BW_PRD_e));

    uint8_t tx_buffer[4] = { W2BW_MOD1_REG_e,
                             sensor->regMap[W2BW_MOD1_REG_e],
                             sensor->regMap[W2BW_MOD1_REG_e + 1],
                             sensor->regMap[W2BW_MOD1_REG_e + 2] 
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
    tlx493d_common_getBitfield(sensor, W2BW_WA_e, &bitFieldValue);

    return bitFieldValue;
}

bool TLx493D_W2BW_enableWakeUpMode(TLx493D_t *sensor) {
    uint8_t bitFieldValue = 0;
    tlx493d_common_getBitfield(sensor, W2BW_T_e, &bitFieldValue);

    if (!bitFieldValue) {
        tlx493d_common_setBitfield(sensor, W2BW_WU_e, 1);
        tlx493d_common_setBitfield(sensor, W2BW_CP_e, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

        uint8_t tx_buffer[5] = { sensor->regDef[W2BW_WU_e].address,
                                  sensor->regMap[sensor->regDef[W2BW_WU_e].address],
                                  sensor->regMap[sensor->regDef[W2BW_WU_e].address + 1],
                                  sensor->regMap[sensor->regDef[W2BW_WU_e].address + 2],
                                  sensor->regMap[sensor->regDef[W2BW_WU_e].address + 3]
                                };
        
        return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
    }
    else {
        return false;
    }
}

bool TLx493D_W2BW_disableWakeUpMode(TLx493D_t *sensor) {
    tlx493d_common_setBitfield(sensor, W2BW_WU_e, 0);
    tlx493d_common_setBitfield(sensor, W2BW_CP_e, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[5] = { sensor->regDef[W2BW_WU_e].address,
                                sensor->regMap[sensor->regDef[W2BW_WU_e].address],
                                sensor->regMap[sensor->regDef[W2BW_WU_e].address + 1],
                                sensor->regMap[sensor->regDef[W2BW_WU_e].address + 2],
                                sensor->regMap[sensor->regDef[W2BW_WU_e].address + 3]
                            };
    
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

// bool TLx493D_W2BW_setLowerWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold) {
//     uint8_t msb = (threshold & 0xFF0) >> 4;
//     uint8_t lsb = (threshold & 0x00E) >> 1;

//     tlx493d_common_setBitfield(sensor, W2BW_XL_MSBS_e, msb);
//     tlx493d_common_setBitfield(sensor, W2BW_XL_LSBS_e, lsb);

//     tlx493d_common_setBitfield(sensor, W2BW_CP_e, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

//     uint8_t tx_buffer[11] = {   sensor->regDef[W2BW_XL_MSBS_e].address,
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 1],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 2],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 3],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 4],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 5],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 6],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 7],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 8],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 9]
//                             };
//     return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
// }


// bool TLx493D_W2BW_setLowerWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold) {
//     uint8_t msb = (threshold & 0xFF0) >> 4;
//     uint8_t lsb = (threshold & 0x00E) >> 1;

//     tlx493d_common_setBitfield(sensor, W2BW_YL_MSBS_e, msb);
//     tlx493d_common_setBitfield(sensor, W2BW_YL_LSBS_e, lsb);

//     tlx493d_common_setBitfield(sensor, W2BW_CP_e, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

//     uint8_t tx_buffer[11] = {   sensor->regDef[W2BW_XL_MSBS_e].address,
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 1],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 2],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 3],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 4],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 5],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 6],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 7],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 8],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 9]
//                             };
//     return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
// }

// bool TLx493D_W2BW_setLowerWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold) {
//     uint8_t msb = (threshold & 0xFF0) >> 4;
//     uint8_t lsb = (threshold & 0x00E) >> 1;

//     tlx493d_common_setBitfield(sensor, W2BW_ZL_MSBS_e, msb);
//     tlx493d_common_setBitfield(sensor, W2BW_ZL_LSBS_e, lsb);

//     tlx493d_common_setBitfield(sensor, W2BW_CP_e, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

//     uint8_t tx_buffer[11] = {   sensor->regDef[W2BW_XL_MSBS_e].address,
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 1],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 2],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 3],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 4],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 5],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 6],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 7],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 8],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 9]
//                             };
//     return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
// }

// // Upper Limit is 2047 and lower -2048
// bool TLx493D_W2BW_setUpperWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold) {
//     uint8_t msb = (threshold & 0xFF0) >> 4;
//     uint8_t lsb = (threshold & 0x00E) >> 1;

//     tlx493d_common_setBitfield(sensor, W2BW_XH_MSBS_e, msb);
//     tlx493d_common_setBitfield(sensor, W2BW_XH_LSBS_e, lsb);

//     tlx493d_common_setBitfield(sensor, W2BW_CP_e, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

//     uint8_t tx_buffer[11] = {   sensor->regDef[W2BW_XL_MSBS_e].address,
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 1],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 2],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 3],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 4],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 5],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 6],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 7],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 8],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 9]
//                             };
//     return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
// }

// bool TLx493D_W2BW_setUpperWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold) {
//     uint8_t msb = (threshold & 0xFF0) >> 4;
//     uint8_t lsb = (threshold & 0x00E) >> 1;

//     tlx493d_common_setBitfield(sensor, W2BW_YH_MSBS_e, msb);
//     tlx493d_common_setBitfield(sensor, W2BW_YH_LSBS_e, lsb);

//     tlx493d_common_setBitfield(sensor, W2BW_CP_e, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

//     uint8_t tx_buffer[11] = {   sensor->regDef[W2BW_XL_MSBS_e].address,
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 1],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 2],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 3],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 4],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 5],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 6],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 7],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 8],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 9]
//                             };
//     return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
// }

// bool TLx493D_W2BW_setUpperWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold) {
//     uint8_t msb = (threshold & 0xFF0) >> 4;
//     uint8_t lsb = (threshold & 0x00E) >> 1;

//     tlx493d_common_setBitfield(sensor, W2BW_ZH_MSBS_e, msb);
//     tlx493d_common_setBitfield(sensor, W2BW_ZH_LSBS_e, lsb);

//     tlx493d_common_setBitfield(sensor, W2BW_CP_e, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

//     uint8_t tx_buffer[11] = {   sensor->regDef[W2BW_XL_MSBS_e].address,
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 1],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 2],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 3],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 4],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 5],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 6],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 7],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 8],
//                                 sensor->regMap[sensor->regDef[W2BW_XL_MSBS_e].address + 9]
//                             };
//     return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
// }

bool TLx493D_W2BW_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
    // bool returnValue = TLx493D_W2BW_setLowerWakeUpThresholdX(sensor, xl_th);
    // returnValue &= TLx493D_W2BW_setUpperWakeUpThresholdX(sensor, xh_th);

    // returnValue &= TLx493D_W2BW_setLowerWakeUpThresholdY(sensor, yl_th);
    // returnValue &= TLx493D_W2BW_setUpperWakeUpThresholdY(sensor, yh_th);
    
    // returnValue &= TLx493D_W2BW_setLowerWakeUpThresholdZ(sensor, zl_th);
    // return  returnValue & TLx493D_W2BW_setUpperWakeUpThresholdZ(sensor, zh_th);
    return false;
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

    tlx493d_common_setBitfield(sensor, W2BW_PR_e, 1);
    tlx493d_common_setBitfield(sensor, W2BW_FP_e, tlx493d_gen_2_calculateFuseParity(sensor, W2BW_FP_e, W2BW_PRD_e));
    return tlx493d_common_writeRegister(sensor, W2BW_FP_e);
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

    for(uint8_t x = sensor->regDef[W2BW_XL_MSBS_e].address; x <= sensor->regDef[W2BW_ZH_MSBS_e].address; x++) {
        parity ^= sensor->regMap[x];
    }

    parity ^= sensor->regMap[sensor->regDef[W2BW_WU_e].address] & ~sensor->regDef[W2BW_WA_e].mask;
    parity ^= sensor->regMap[sensor->regDef[W2BW_YH_LSBS_e].address] & (sensor->regDef[W2BW_YH_LSBS_e].mask & sensor->regDef[W2BW_YL_LSBS_e].mask);
    parity ^= sensor->regMap[sensor->regDef[W2BW_ZH_LSBS_e].address] & (sensor->regDef[W2BW_ZH_LSBS_e].mask & sensor->regDef[W2BW_ZL_LSBS_e].mask);
    parity ^= sensor->regMap[W2BW_CONFIG_REG_e] & ~sensor->regDef[W2BW_CP_e].mask;
    
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


uint8_t TLx493D_W2BW_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr) {
    return tlx493d_gen_2_selectIICAddress(sensor, addr);
}


void TLx493D_W2BW_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens, double mT, int16_t *rawMF) {

}


void TLx493D_W2BW_getSensitivityScaleFactor(TLx493D_t *sensor, double *sf) {
    tlx493d_common_getSensitivityScaleFactor(sensor, TLx493D_HAS_X4_e, W2BW_X2_e, W2BW_X4_e, sf);
}
