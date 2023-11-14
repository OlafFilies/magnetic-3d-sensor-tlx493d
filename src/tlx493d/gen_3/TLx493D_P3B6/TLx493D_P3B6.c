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
#include "tlx493d_gen_3_common_defines.h"
#include "tlx493d_gen_3_common.h"

// sensor specific includes
#include "TLx493D_P3B6_defines.h"
#include "TLx493D_P3B6.h"


/*
  Listing of all register names for this sensor.
  Used to index "TLx493D_P3B6_regDef" defined below, so index values must match !
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
               CHIP_ID_5 } TLx493D_P3B6_registerNames_te;


TLx493D_Register_t TLx493D_P3B6_regDef[] = {
    { BX_MSBS,       TLx493D_READ_MODE_e,       0x00, 0xFF, 0, 8 },
    { IIC_ADR_CP,    TLx493D_READ_MODE_e,       0x01, 0xC0, 6, 2 },
    { BX_LSBS,       TLx493D_READ_MODE_e,       0x01, 0x3F, 0, 6 },
    { BY_MSBS,       TLx493D_READ_MODE_e,       0x02, 0xFF, 0, 8 },
    { BY_LSBS,       TLx493D_READ_MODE_e,       0x03, 0x3F, 0, 6 },
    { BZ_MSBS,       TLx493D_READ_MODE_e,       0x04, 0xFF, 0, 8 },
    { BZ_LSBS,       TLx493D_READ_MODE_e,       0x05, 0x3F, 0, 6 },
    { TEMP_MSBS,     TLx493D_READ_MODE_e,       0x06, 0xFF, 0, 8 },
    { TEMP_LSBS,     TLx493D_READ_MODE_e,       0x07, 0x3F, 0, 6 },
    { CRC,           TLx493D_READ_MODE_e,       0x08, 0xFF, 0, 8 },
    { MEAS_FLG,      TLx493D_READ_MODE_e,       0x09, 0x80, 7, 1 },
    { TEST_FLG,      TLx493D_READ_MODE_e,       0x09, 0x40, 6, 1 },
    { FRAME_COUNTER, TLx493D_READ_MODE_e,       0x09, 0x30, 4, 2 },
    { RST_FLG,       TLx493D_READ_MODE_e,       0x09, 0x08, 3, 1 },
    { WU_PAR_FLG,    TLx493D_READ_MODE_e,       0x09, 0x04, 2, 1 },
    { CRC_WR_FLG,    TLx493D_READ_MODE_e,       0x09, 0x02, 1, 1 },
    { FUSE_PAR_FLG,  TLx493D_READ_MODE_e,       0x09, 0x01, 0, 1 },
    { MODE_SEL,      TLx493D_READ_WRITE_MODE_e, 0x0A, 0x80, 7, 1 },
    { INT_DIS,       TLx493D_READ_WRITE_MODE_e, 0x0A, 0x40, 6, 1 },
    { COLLISION_DIS, TLx493D_READ_WRITE_MODE_e, 0x0A, 0x20, 5, 1 },
    { WU_EN,         TLx493D_READ_WRITE_MODE_e, 0x0A, 0x10, 4, 1 },
    { TRIGGER_SEL,   TLx493D_READ_WRITE_MODE_e, 0x0A, 0x0C, 2, 2 },
    { PROT_SEL,      TLx493D_READ_WRITE_MODE_e, 0x0A, 0x02, 1, 1 },
    { CRC_WR_EN,     TLx493D_READ_WRITE_MODE_e, 0x0A, 0x01, 0, 1 },
    { CHANNEL_SEL,   TLx493D_READ_WRITE_MODE_e, 0x0B, 0xF0, 4, 4 },
    { F_UPDATE_SEL,  TLx493D_READ_WRITE_MODE_e, 0x0B, 0x0C, 2, 2 },
    { XTR_SHORT_EN,  TLx493D_READ_WRITE_MODE_e, 0x0B, 0x02, 1, 1 },
    { SHORT_EN,      TLx493D_READ_WRITE_MODE_e, 0x0B, 0x01, 0, 1 },
    { WU_XH_MSBS,    TLx493D_READ_WRITE_MODE_e, 0x0C, 0xFF, 0, 8 },
    { WU_XL_MSBS,    TLx493D_READ_WRITE_MODE_e, 0x0D, 0xFF, 0, 8 },
    { WU_YH_MSBS,    TLx493D_READ_WRITE_MODE_e, 0x0E, 0xFF, 0, 8 },
    { WU_YL_MSBS,    TLx493D_READ_WRITE_MODE_e, 0x0F, 0xFF, 0, 8 },
    { WU_ZH_MSBS,    TLx493D_READ_WRITE_MODE_e, 0x10, 0xFF, 0, 8 },
    { WU_ZL_MSBS,    TLx493D_READ_WRITE_MODE_e, 0x11, 0xFF, 0, 8 },
    { WU_XH_LSBS,    TLx493D_READ_WRITE_MODE_e, 0x12, 0xC0, 6, 2 },
    { WU_XL_LSBS,    TLx493D_READ_WRITE_MODE_e, 0x12, 0x30, 4, 2 },
    { WU_YH_LSBS,    TLx493D_READ_WRITE_MODE_e, 0x12, 0x0C, 2, 2 },
    { WU_YL_LSBS,    TLx493D_READ_WRITE_MODE_e, 0x12, 0x03, 0, 2 },
    { WU_PAR,        TLx493D_READ_WRITE_MODE_e, 0x13, 0x20, 5, 1 },
    { WU_EN_CP,      TLx493D_WRITE_MODE_e,      0x13, 0x10, 4, 1 },
    { WU_ZH_LSBS,    TLx493D_READ_WRITE_MODE_e, 0x13, 0x0C, 2, 2 },
    { WU_ZL_LSBS,    TLx493D_READ_WRITE_MODE_e, 0x13, 0x03, 0, 2 },
    { RST_FLG_CLR,   TLx493D_WRITE_MODE_e,      0x14, 0x02, 1, 1 },
    { SOFT_RST,      TLx493D_WRITE_MODE_e,      0x14, 0x01, 0, 1 },
    { CHIP_ID_0,     TLx493D_READ_MODE_e,       0x15, 0xFF, 0, 8 },
    { CHIP_ID_1,     TLx493D_READ_MODE_e,       0x16, 0xFF, 0, 8 },
    { CHIP_ID_2,     TLx493D_READ_MODE_e,       0x17, 0xFF, 0, 8 },
    { CHIP_ID_3,     TLx493D_READ_MODE_e,       0x18, 0xFF, 0, 8 },
    { CHIP_ID_4,     TLx493D_READ_MODE_e,       0x19, 0xFF, 0, 8 },
    { ID_PAR,        TLx493D_READ_MODE_e,       0x1A, 0x40, 6, 1},
    { CHIP_ID_5,     TLx493D_READ_MODE_e,       0x1A, 0x3F, 0, 6 }
};


// typedef enum { 
//                TEMP2_REG  = 0x05,
//                DIAG_REG   = 0x06,
//                CONFIG_REG = 0x10,
//                MOD1_REG   = 0x11,
//                MOD2_REG   = 0x13,
//                VER_REG    = 0x16 } SpecialRegisters_te;


TLx493D_CommonFunctions_t TLx493D_P3B6_commonFunctions = {
    .init                           = TLx493D_P3B6_init,
    .deinit                         = TLx493D_P3B6_deinit,

    .readRegisters                  = TLx493D_P3B6_readRegisters, //tlx493d_common_readRegisters,

    .calculateTemperature           = TLx493D_P3B6_calculateTemperature,
    .getTemperature                 = TLx493D_P3B6_getTemperature,

    .calculateMagneticField         = TLx493D_P3B6_calculateMagneticField,
    .getMagneticField               = TLx493D_P3B6_getMagneticField,

    .calculateMagneticFieldAndTemperature = TLx493D_P3B6_calculateMagneticFieldAndTemperature,
    .getMagneticFieldAndTemperature = TLx493D_P3B6_getMagneticFieldAndTemperature,


    .setDefaultConfig               = TLx493D_P3B6_setDefaultConfig,
    
    .setResetValues                 = TLx493D_P3B6_setResetValues,
};


// TODO: add parameter IICAddress or ad function to set address.
bool TLx493D_P3B6_init(TLx493D_t *sensor) {
    // TODO: use in TLx493D_initCommunication
    // tlx493d_setI2CParameters(sensor, GEN_3_STD_IIC_ADDR_WRITE_A0);
    tlx493d_setI2CParameters(sensor, GEN_3_STD_IIC_ADDR_WRITE_A1);

    return tlx493d_common_init(sensor, GEN_3_REG_MAP_SIZE, TLx493D_P3B6_regDef, &TLx493D_P3B6_commonFunctions, TLx493D_P3B6_e, TLx493D_I2C_e);
}


bool TLx493D_P3B6_deinit(TLx493D_t *sensor) {
    return tlx493d_common_deinit(sensor);
}


bool TLx493D_P3B6_readRegisters(TLx493D_t *sensor) {
    return tlx493d_common_readRegisters(sensor);
}


void TLx493D_P3B6_calculateRawTemperature(TLx493D_t *sensor, double *temp) {
    tlx493d_gen_3_calculateRawTemperature(sensor, temp, TEMP_MSBS, TEMP_LSBS);
}


void TLx493D_P3B6_calculateTemperature(TLx493D_t *sensor, double *temp) {
    tlx493d_gen_3_calculateTemperature(sensor, temp, TEMP_MSBS, TEMP_LSBS);
}


bool TLx493D_P3B6_getTemperature(TLx493D_t *sensor, double *temp) {
    return tlx493d_common_getTemperature(sensor, temp);
}


void TLx493D_P3B6_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    tlx493d_gen_3_calculateMagneticField(sensor, x, y, z, BX_MSBS, BX_LSBS, BY_MSBS, BY_LSBS, BZ_MSBS, BZ_LSBS, TEMP_MSBS, TEMP_LSBS);
}


bool TLx493D_P3B6_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
     return tlx493d_common_getMagneticField(sensor, x, y, z);
}


void TLx493D_P3B6_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_P3B6_calculateMagneticField(sensor, x, y, z);
    TLx493D_P3B6_calculateTemperature(sensor, temp);
}


bool TLx493D_P3B6_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    return tlx493d_common_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


// TODO: set all options that must be set, eg MODE ?, reset all bits to defaults ?
bool TLx493D_P3B6_setDefaultConfig(TLx493D_t *sensor) {
     tlx493d_common_setBitfield(sensor, MODE_SEL, 0);
     tlx493d_common_setBitfield(sensor, INT_DIS, 1);
     tlx493d_common_setBitfield(sensor, COLLISION_DIS, 0);
     tlx493d_common_setBitfield(sensor, WU_EN, 0);
     tlx493d_common_setBitfield(sensor, CRC_WR_EN, 0);

    if( ! TLx493D_P3B6_enable1ByteMode(sensor) ) {
        // Read registers in order to retrieve values in reserved register at 0x12 and in MOD2 in order to make sure we are not 
        // accidentally changing a preset values to 0.
        // if( tlx493d_gen_2_readRegisters(sensor) )
        //     return TLx493D_P3B6_setLowUpdateRate(sensor);
        // }

        return false;
    }

    // tlx493d_common_setBitfield(sensor, CHANNEL_SEL, 0);

    return true;
}


bool TLx493D_P3B6_set1ByteReadMode(TLx493D_t *sensor, uint8_t pr) {
    tlx493d_common_setBitfield(sensor, PROT_SEL, pr);

    return tlx493d_common_writeRegister(sensor, PROT_SEL);
}


bool TLx493D_P3B6_enable1ByteMode(TLx493D_t *sensor) {
     return TLx493D_P3B6_set1ByteReadMode(sensor, 1);
}


bool TLx493D_P3B6_disable1ByteMode(TLx493D_t *sensor) {
     return TLx493D_P3B6_set1ByteReadMode(sensor, 0);
}


void TLx493D_P3B6_setResetValues(TLx493D_t *sensor) {
    sensor->regMap[0x0A] = 0x62; // MOD1
    sensor->regMap[0x0B] = 0x00; // MOD2

    // for wake-up parity calculation
    sensor->regMap[0x0C] = 0x7F;
    sensor->regMap[0x0D] = 0x80;
    sensor->regMap[0x0E] = 0x7F;
    sensor->regMap[0x0F] = 0x80;  
    sensor->regMap[0x10] = 0x7F;
    sensor->regMap[0x11] = 0x80;
    sensor->regMap[0x12] = 0xCC;
    sensor->regMap[0x13] = 0x2C;
}
