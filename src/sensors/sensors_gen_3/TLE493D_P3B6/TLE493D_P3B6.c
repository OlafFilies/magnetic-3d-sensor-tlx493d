/**
 * @file        TLV493D_P3B6.c
 * @author      Infineon Technologies AG
 * @brief       Contains the implementation of the complete sensor functionality
 * @copyright   Copyright (c) 2023
 *
 * SPDX-License-Identifier: MIT
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_common_defines.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_common_defines.h"
#include "sensors_gen_2_common.h"

#include "sensors_gen_3_common_defines.h"
#include "sensors_gen_3_common.h"

// sensor specicifc includes
#include "TLE493D_P3B6_defines.h"
#include "TLE493D_P3B6.h"


extern void setI2CParameters(Sensor_ts *sensor, uint8_t addr);

/*
  Listing of all register names for this sensor.
  Used to index "TLE493D_P3B6_regDef" defined below, so index values must match !
*/
typedef enum {
               BX_MSBS = 0,
               IIC_ADR_CP,
               BX_LSBS,
               BY_MSBS,
               BY_LSBS,
               BZ_MSBS,
               BZ_LSBS,
               TEMP_MSBS,
               TEMP_LSBS,
               CRC,
               MEAS_FLG,
               TEST_FLG,
               FRAME_COUNTER,
               RST_FLG,
               WU_PAR_FLG,
               CRC_WR_FLG,
               FUSE_PAR_FLG,
               MODE_SEL,
               INT_DIS,
               COLLISION_DIS,
               WU_EN,
               TRIGGER_SEL,
               PROT_SEL,
               CRC_WR_EN,
               CHANNEL_SEL,
               F_UPDATE_SEL,
               XTR_SHORT_EN,
               SHORT_EN,
               WU_XH_MSBS,
               WU_XL_MSBS,
               WU_YH_MSBS,
               WU_YL_MSBS,
               WU_ZH_MSBS,
               WU_ZL_MSBS,
               WU_XH_LSBS,
               WU_XL_LSBS,
               WU_YH_LSBS,
               WU_YL_LSBS,
               WU_PAR,
               WU_EN_CP,
               WU_ZH_LSBS,
               WU_ZL_LSBS,
               RST_FLG_CLR,
               SOFT_RST,
               CHIP_ID_0,
               CHIP_ID_1,
               CHIP_ID_2,
               CHIP_ID_3,
               CHIP_ID_4,
               ID_PAR,
               CHIP_ID_5 } TLE493D_P3B6_registerNames_te;


Register_ts TLE493D_P3B6_regDef[] = {
    { BX_MSBS,       READ_MODE_e,       0x00, 0xFF, 0, 8 },
    { IIC_ADR_CP,    READ_MODE_e,       0x01, 0xC0, 6, 2 },
    { BX_LSBS,       READ_MODE_e,       0x01, 0x3F, 0, 6 },
    { BY_MSBS,       READ_MODE_e,       0x02, 0xFF, 0, 8 },
    { BY_LSBS,       READ_MODE_e,       0x03, 0x3F, 0, 6 },
    { BZ_MSBS,       READ_MODE_e,       0x04, 0xFF, 0, 8 },
    { BZ_LSBS,       READ_MODE_e,       0x05, 0x3F, 0, 6 },
    { TEMP_MSBS,     READ_MODE_e,       0x06, 0xFF, 0, 8 },
    { TEMP_LSBS,     READ_MODE_e,       0x07, 0x3F, 0, 6 },
    { CRC,           READ_MODE_e,       0x08, 0xFF, 0, 8 },
    { MEAS_FLG,      READ_MODE_e,       0x09, 0x80, 7, 1 },
    { TEST_FLG,      READ_MODE_e,       0x09, 0x40, 6, 1 },
    { FRAME_COUNTER, READ_MODE_e,       0x09, 0x30, 4, 2 },
    { RST_FLG,       READ_MODE_e,       0x09, 0x08, 3, 1 },
    { WU_PAR_FLG,    READ_MODE_e,       0x09, 0x04, 2, 1 },
    { CRC_WR_FLG,    READ_MODE_e,       0x09, 0x02, 1, 1 },
    { FUSE_PAR_FLG,  READ_MODE_e,       0x09, 0x01, 0, 1 },
    { MODE_SEL,      READ_WRITE_MODE_e, 0x0A, 0x80, 7, 1 },
    { INT_DIS,       READ_WRITE_MODE_e, 0x0A, 0x40, 6, 1 },
    { COLLISION_DIS, READ_WRITE_MODE_e, 0x0A, 0x20, 5, 1 },
    { WU_EN,         READ_WRITE_MODE_e, 0x0A, 0x10, 4, 1 },
    { TRIGGER_SEL,   READ_WRITE_MODE_e, 0x0A, 0x0C, 2, 2 },
    { PROT_SEL,      READ_WRITE_MODE_e, 0x0A, 0x02, 1, 1 },
    { CRC_WR_EN,     READ_WRITE_MODE_e, 0x0A, 0x01, 0, 1 },
    { CHANNEL_SEL,   READ_WRITE_MODE_e, 0x0B, 0xF0, 4, 4 },
    { F_UPDATE_SEL,  READ_WRITE_MODE_e, 0x0B, 0x0C, 2, 2 },
    { XTR_SHORT_EN,  READ_WRITE_MODE_e, 0x0B, 0x02, 1, 1 },
    { SHORT_EN,      READ_WRITE_MODE_e, 0x0B, 0x01, 0, 1 },
    { WU_XH_MSBS,    READ_WRITE_MODE_e, 0x0C, 0xFF, 0, 8 },
    { WU_XL_MSBS,    READ_WRITE_MODE_e, 0x0D, 0xFF, 0, 8 },
    { WU_YH_MSBS,    READ_WRITE_MODE_e, 0x0E, 0xFF, 0, 8 },
    { WU_YL_MSBS,    READ_WRITE_MODE_e, 0x0F, 0xFF, 0, 8 },
    { WU_ZH_MSBS,    READ_WRITE_MODE_e, 0x10, 0xFF, 0, 8 },
    { WU_ZL_MSBS,    READ_WRITE_MODE_e, 0x11, 0xFF, 0, 8 },
    { WU_XH_LSBS,    READ_WRITE_MODE_e, 0x12, 0xC0, 6, 2 },
    { WU_XL_LSBS,    READ_WRITE_MODE_e, 0x12, 0x30, 4, 2 },
    { WU_YH_LSBS,    READ_WRITE_MODE_e, 0x12, 0x0C, 2, 2 },
    { WU_YL_LSBS,    READ_WRITE_MODE_e, 0x12, 0x03, 0, 2 },
    { WU_PAR,        READ_WRITE_MODE_e, 0x13, 0x20, 5, 1 },
    { WU_EN_CP,      WRITE_MODE_e,      0x13, 0x10, 4, 1 },
    { WU_ZH_LSBS,    READ_WRITE_MODE_e, 0x13, 0x0C, 2, 2 },
    { WU_ZL_LSBS,    READ_WRITE_MODE_e, 0x13, 0x03, 0, 2 },
    { RST_FLG_CLR,   WRITE_MODE_e,      0x14, 0x02, 1, 1 },
    { SOFT_RST,      WRITE_MODE_e,      0x14, 0x01, 0, 1 },
    { CHIP_ID_0,     READ_MODE_e,       0x15, 0xFF, 0, 8 },
    { CHIP_ID_1,     READ_MODE_e,       0x16, 0xFF, 0, 8 },
    { CHIP_ID_2,     READ_MODE_e,       0x17, 0xFF, 0, 8 },
    { CHIP_ID_3,     READ_MODE_e,       0x18, 0xFF, 0, 8 },
    { CHIP_ID_4,     READ_MODE_e,       0x19, 0xFF, 0, 8 },
    { ID_PAR,        READ_MODE_e,       0x1A, 0x40, 6, 1},
    { CHIP_ID_5,     READ_MODE_e,       0x1A, 0x3F, 0, 6 }
};


/***
 * 
*/
typedef enum {
               LOW_POWER_MODE         = 0x00,
               MASTER_CONTROLLED_MODE = 0x01,
               RESERVED_MODE          = 0x10,
               FAST_MODE              = 0x11 } TLE493D_P3B6_modes_te;


/***
 * 
*/
// typedef enum { 
//                TEMP2_REG  = 0x05,
//                DIAG_REG   = 0x06,
//                CONFIG_REG = 0x10,
//                MOD1_REG   = 0x11,
//                MOD2_REG   = 0x13,
//                VER_REG    = 0x16 } SpecialRegisters_te;


CommonFunctions_ts TLE493D_P3B6_commonFunctions = {
                                .init                 = TLE493D_P3B6_init,
                                .deinit               = TLE493D_P3B6_deinit,

                                .getTemperature       = TLE493D_P3B6_getTemperature,

                                .calculateFieldValues = TLE493D_P3B6_calculateFieldValues,
                                .getFieldValues       = TLE493D_P3B6_getFieldValues,

                                .setDefaultConfig     = TLE493D_P3B6_setDefaultConfig,
                                .readRegisters        = gen_2_readRegisters,
};


/***
 * TODO: add parameter IICAddress or ad function to set address.
*/
bool TLE493D_P3B6_init(Sensor_ts *sensor) {
    // regMap must be sensor specific, not sensor type specific, therefore malloc.
    sensor->regMap            = (uint8_t*) malloc(sizeof(uint8_t) * GEN_3_REG_MAP_SIZE);
    sensor->regDef            = TLE493D_P3B6_regDef;
    sensor->functions         = &TLE493D_P3B6_commonFunctions;
    sensor->regMapSize        = GEN_3_REG_MAP_SIZE;
    sensor->sensorType        = TLE493D_P3B6_e;
    sensor->comIFType         = I2C_e;
    sensor->comLibIF          = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    memset(sensor->regMap, 0, sensor->regMapSize);
    
    // TODO: make address settable by user ! In intI2CComLibIF ?
    // setI2CParameters(sensor, GEN_3_STD_IIC_ADDR_WRITE_A0);
    setI2CParameters(sensor, GEN_3_STD_IIC_ADDR_WRITE_A1);

    return true;
}


/***
 * 
*/
bool TLE493D_P3B6_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);

    sensor->regMap            = NULL;
    sensor->comLibObj.i2c_obj = NULL;
    return true;
}


/***
 * 
*/
void TLE493D_P3B6_calculateRawTemperature(Sensor_ts *sensor, float *temp) {
    int16_t value = 0;

    concatBytes(sensor, &sensor->regDef[TEMP_MSBS], &sensor->regDef[TEMP_LSBS], &value);
    *temp = (float) value;
}


/***
 * 
*/
void TLE493D_P3B6_calculateTemperature(Sensor_ts *sensor, float *temp) {
    int16_t value = 0;

    concatBytes(sensor, &sensor->regDef[TEMP_MSBS], &sensor->regDef[TEMP_LSBS], &value);
    *temp = (((float) value - GEN_3_TEMP_OFFSET) / GEN_3_TEMP_MULT) + GEN_3_TEMP_REF;
}


/***
 * 
*/
bool TLE493D_P3B6_getTemperature(Sensor_ts *sensor, float *temp) {
    if( gen_2_readRegisters(sensor) ) {
        TLE493D_P3B6_calculateTemperature(sensor, temp);
        return true;
    }

    return false;
}


/***
 * 
*/
void TLE493D_P3B6_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    concatBytes(sensor, &sensor->regDef[BX_MSBS], &sensor->regDef[BX_LSBS], &valueX);
    concatBytes(sensor, &sensor->regDef[BY_MSBS], &sensor->regDef[BY_LSBS], &valueY);
    concatBytes(sensor, &sensor->regDef[BZ_MSBS], &sensor->regDef[BZ_LSBS], &valueZ);

double r    = 1.0;
// double temp = 0.0;
float temp = 0.0;
double sensitivity = GEN_3_FULL_RANGE_FIELD_SENSITIVITY;
    
    TLE493D_P3B6_calculateRawTemperature(sensor, &temp);

    *x = (r * (GEN_3_O0x + temp * (GEN_3_O1x + temp * (GEN_3_O2x + GEN_3_O3x * temp)))
          + ((double) valueX) * (GEN_3_L0x + temp * (GEN_3_L1x + temp * (GEN_3_L2x + GEN_3_L3x * temp))))
       / sensitivity;

    *y = (r * (GEN_3_O0y + temp * (GEN_3_O1y + temp * (GEN_3_O2y + GEN_3_O3y * temp)))
          + ((double) valueY) * (GEN_3_L0y + temp * (GEN_3_L1y + temp * (GEN_3_L2y + GEN_3_L3y * temp))))
       / sensitivity;

    *z = (r * (GEN_3_O0z + temp * (GEN_3_O1z + temp * (GEN_3_O2z + GEN_3_O3z * temp)))
          + ((double) valueZ) * (GEN_3_L0z + temp * (GEN_3_L1z + temp * (GEN_3_L2z + GEN_3_L3z * temp))))
       / sensitivity;
}


/***
 * 
*/
bool TLE493D_P3B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    if( gen_2_readRegisters(sensor) ) {
        TLE493D_P3B6_calculateFieldValues(sensor, x, y, z);
        return true;
    }

    return false;
}


/***
 * 
*/
bool TLE493D_P3B6_set1ByteReadMode(Sensor_ts *sensor, uint8_t pr) {
    gen_2_setBitfield(sensor, PROT_SEL, pr);

    return gen_2_writeRegister(sensor, PROT_SEL);
}


bool TLE493D_P3B6_enable1ByteMode(Sensor_ts *sensor) {
     return TLE493D_P3B6_set1ByteReadMode(sensor, 1);
}


bool TLE493D_P3B6_disable1ByteMode(Sensor_ts *sensor) {
     return TLE493D_P3B6_set1ByteReadMode(sensor, 0);
}


/***
 * TODO: set all options that must be set, eg MODE ?, reset all bits to defaults ?
*/
bool TLE493D_P3B6_setDefaultConfig(Sensor_ts *sensor) {
    // MOD1 register
     gen_2_setBitfield(sensor, MODE_SEL, 0);
     gen_2_setBitfield(sensor, INT_DIS, 1);
     gen_2_setBitfield(sensor, COLLISION_DIS, 0);
     gen_2_setBitfield(sensor, WU_EN, 0);
     gen_2_setBitfield(sensor, CRC_WR_EN, 0);

    if( ! TLE493D_P3B6_enable1ByteMode(sensor) ) {
        // Read registers in order to retrieve values in reserved register at 0x12 and in MOD2 in order to make sure we are not 
        // accidentally changing a preset values to 0.
        // if( gen_2_readRegisters(sensor) )
        //     return TLE493D_P3B6_setLowUpdateRate(sensor);
        // }

        return false;
    }

    // gen_2_setBitfield(sensor, CHANNEL_SEL, 0);

    // #include "Logger.h"
    // printRegMap(sensor->regMap, sensor->regMapSize);

    return true;
}
