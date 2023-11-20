#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"
#include "Logger.h"

// common to same generation of sensors
#include "tlx493d_gen_3_common_defines.h"
#include "tlx493d_gen_3_common.h"

// sensor specific includes
#include "TLx493D_P3I8_defines.h"
#include "TLx493D_P3I8_enums.h"
#include "TLx493D_P3I8.h"


TLx493D_Register_t TLx493D_P3I8_regDef[] = {
    { P3I8_BX_MSBS_e,       TLx493D_READ_MODE_e,       0x00, 0xFF, 0, 8 },
    { P3I8_BX_LSBS_e,       TLx493D_READ_MODE_e,       0x01, 0x3F, 0, 6 },
    { P3I8_BY_MSBS_e,       TLx493D_READ_MODE_e,       0x02, 0xFF, 0, 8 },
    { P3I8_BY_LSBS_e,       TLx493D_READ_MODE_e,       0x03, 0x3F, 0, 6 },
    { P3I8_BZ_MSBS_e,       TLx493D_READ_MODE_e,       0x04, 0xFF, 0, 8 },
    { P3I8_BZ_LSBS_e,       TLx493D_READ_MODE_e,       0x05, 0x3F, 0, 6 },
    { P3I8_TEMP_MSBS_e,     TLx493D_READ_MODE_e,       0x06, 0xFF, 0, 8 },
    { P3I8_TEMP_LSBS_e,     TLx493D_READ_MODE_e,       0x07, 0x3F, 0, 6 },
    { P3I8_CRC_e,           TLx493D_READ_MODE_e,       0x08, 0xFF, 0, 8 },
    { P3I8_MEAS_FLG_e,      TLx493D_READ_MODE_e,       0x09, 0x80, 7, 1 },
    { P3I8_TEST_FLG_e,      TLx493D_READ_MODE_e,       0x09, 0x40, 6, 1 },
    { P3I8_FRAME_COUNTER_e, TLx493D_READ_MODE_e,       0x09, 0x30, 4, 2 },
    { P3I8_RST_FLG_e,       TLx493D_READ_MODE_e,       0x09, 0x08, 3, 1 },
    { P3I8_WU_PAR_FLG_e,    TLx493D_READ_MODE_e,       0x09, 0x04, 2, 1 },
    { P3I8_CRC_WR_FLG_e,    TLx493D_READ_MODE_e,       0x09, 0x02, 1, 1 },
    { P3I8_FUSE_PAR_FLG_e,  TLx493D_READ_MODE_e,       0x09, 0x01, 0, 1 },
    { P3I8_MODE_SEL_e,      TLx493D_READ_WRITE_MODE_e, 0x0A, 0x80, 7, 1 },
    { P3I8_INT_DIS_e,       TLx493D_READ_WRITE_MODE_e, 0x0A, 0x40, 6, 1 },
    { P3I8_WU_EN_e,         TLx493D_READ_WRITE_MODE_e, 0x0A, 0x10, 4, 1 },
    { P3I8_TRIGGER_SEL_e,   TLx493D_READ_WRITE_MODE_e, 0x0A, 0x0C, 2, 2 },
    { P3I8_CRC_WR_EN_e,     TLx493D_READ_WRITE_MODE_e, 0x0A, 0x01, 0, 1 },
    { P3I8_CHANNEL_SEL_e,   TLx493D_READ_WRITE_MODE_e, 0x0B, 0xF0, 4, 4 },
    { P3I8_F_UPDATE_SEL_e,  TLx493D_READ_WRITE_MODE_e, 0x0B, 0x0C, 2, 2 },
    { P3I8_XTR_SHORT_EN_e,  TLx493D_READ_WRITE_MODE_e, 0x0B, 0x02, 1, 1 },
    { P3I8_SHORT_EN_e,      TLx493D_READ_WRITE_MODE_e, 0x0B, 0x01, 0, 1 },
    { P3I8_WU_XH_MSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0C, 0xFF, 0, 8 },
    { P3I8_WU_XL_MSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0D, 0xFF, 0, 8 },
    { P3I8_WU_YH_MSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0E, 0xFF, 0, 8 },
    { P3I8_WU_YL_MSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0F, 0xFF, 0, 8 },
    { P3I8_WU_ZH_MSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x10, 0xFF, 0, 8 },
    { P3I8_WU_ZL_MSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x11, 0xFF, 0, 8 },
    { P3I8_WU_XH_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x12, 0xC0, 6, 2 },
    { P3I8_WU_XL_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x12, 0x30, 4, 2 },
    { P3I8_WU_YH_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x12, 0x0C, 2, 2 },
    { P3I8_WU_YL_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x12, 0x03, 0, 2 },
    { P3I8_WU_PAR_e,        TLx493D_READ_WRITE_MODE_e, 0x13, 0x20, 5, 1 },
    { P3I8_WU_EN_CP_e,      TLx493D_WRITE_MODE_e,      0x13, 0x10, 4, 1 },
    { P3I8_WU_ZH_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x13, 0x0C, 2, 2 },
    { P3I8_WU_ZL_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x13, 0x03, 0, 2 },
    { P3I8_RST_FLG_CLR_e,   TLx493D_WRITE_MODE_e,      0x14, 0x02, 1, 1 },
    { P3I8_SOFT_RST_e,      TLx493D_WRITE_MODE_e,      0x14, 0x01, 0, 1 },
    { P3I8_CHIP_ID_0_e,     TLx493D_READ_MODE_e,       0x15, 0xFF, 0, 8 },
    { P3I8_CHIP_ID_1_e,     TLx493D_READ_MODE_e,       0x16, 0xFF, 0, 8 },
    { P3I8_CHIP_ID_2_e,     TLx493D_READ_MODE_e,       0x17, 0xFF, 0, 8 },
    { P3I8_CHIP_ID_3_e,     TLx493D_READ_MODE_e,       0x18, 0xFF, 0, 8 },
    { P3I8_CHIP_ID_4_e,     TLx493D_READ_MODE_e,       0x19, 0xFF, 0, 8 },
    { P3I8_ID_PAR_e,        TLx493D_READ_MODE_e,       0x1A, 0x40, 6, 1 },
    { P3I8_CHIP_ID_5_e,     TLx493D_READ_MODE_e,       0x1A, 0x3F, 0, 6 },
};


TLx493D_CommonFunctions_t TLx493D_P3I8_commonFunctions = {
    .init                           = TLx493D_P3I8_init,
    .deinit                         = TLx493D_P3I8_deinit,

    .readRegisters                  = TLx493D_P3I8_readRegisters,

    .calculateRawTemperature        = TLx493D_P3I8_calculateRawTemperature,
    .getRawTemperature              = TLx493D_P3I8_getRawTemperature,

    .calculateRawMagneticField      = TLx493D_P3I8_calculateRawMagneticField,
    .getRawMagneticField            = TLx493D_P3I8_getRawMagneticField,

    .calculateRawMagneticFieldAndTemperature = TLx493D_P3I8_calculateRawMagneticFieldAndTemperature,
    .getRawMagneticFieldAndTemperature       = TLx493D_P3I8_getRawMagneticFieldAndTemperature,

    .calculateTemperature           = TLx493D_P3I8_calculateTemperature,
    .getTemperature                 = TLx493D_P3I8_getTemperature,

    .calculateMagneticField         = TLx493D_P3I8_calculateMagneticField,
    .getMagneticField               = TLx493D_P3I8_getMagneticField,

    .calculateMagneticFieldAndTemperature = TLx493D_P3I8_calculateMagneticFieldAndTemperature,
    .getMagneticFieldAndTemperature = TLx493D_P3I8_getMagneticFieldAndTemperature,


    .setDefaultConfig               = TLx493D_P3I8_setDefaultConfig,
    
    .setResetValues                 = TLx493D_P3I8_setResetValues,

    .selectIICAddress               = TLx493D_P3I8_selectIICAddress,

    .calculateRawMagneticFieldAtTemperature = TLx493D_P3I8_calculateRawMagneticFieldAtTemperature,

    .getSensitivityScaleFactor      = TLx493D_P3I8_getSensitivityScaleFactor,
};


bool TLx493D_P3I8_init(TLx493D_t *sensor) {
    return tlx493d_common_init(sensor, GEN_3_REG_MAP_SIZE, TLx493D_P3I8_regDef, &TLx493D_P3I8_commonFunctions, TLx493D_P3I8_e, TLx493D_SPI_e);
}


bool TLx493D_P3I8_deinit(TLx493D_t *sensor) {
    return tlx493d_common_deinit(sensor);
}


bool TLx493D_P3I8_readRegisters(TLx493D_t *sensor) {
    return tlx493d_gen_3_readRegistersSPI(sensor);
}


void TLx493D_P3I8_calculateRawTemperature(TLx493D_t *sensor, uint16_t *temperature) {
    tlx493d_gen_3_calculateRawTemperature(sensor, P3I8_TEMP_MSBS_e, P3I8_TEMP_LSBS_e, temperature);
}


bool TLx493D_P3I8_getRawTemperature(TLx493D_t *sensor, uint16_t *temperature) {
    return tlx493d_common_getRawTemperature(sensor, temperature);
}


void TLx493D_P3I8_calculateRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z) {
    tlx493d_gen_3_calculateRawMagneticField(sensor, P3I8_BX_MSBS_e, P3I8_BX_LSBS_e, P3I8_BY_MSBS_e, P3I8_BY_LSBS_e, P3I8_BZ_MSBS_e, P3I8_BZ_LSBS_e, x, y, z);
}


bool TLx493D_P3I8_getRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z) {
    return tlx493d_common_getRawMagneticField(sensor, x, y, z);
}


void TLx493D_P3I8_calculateRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature) {
    TLx493D_P3I8_calculateRawMagneticField(sensor, x, y, z);
    TLx493D_P3I8_calculateRawTemperature(sensor, temperature);
}


bool TLx493D_P3I8_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature) {
    return tlx493d_common_getRawMagneticFieldAndTemperature(sensor, x, y, z, temperature);
}


void TLx493D_P3I8_calculateTemperature(TLx493D_t *sensor, double *temp) {
    tlx493d_gen_3_calculateTemperature(sensor, P3I8_TEMP_MSBS_e, P3I8_TEMP_LSBS_e, temp);
}


bool TLx493D_P3I8_getTemperature(TLx493D_t *sensor, double *temp) {
    return tlx493d_common_getTemperature(sensor, temp);
}


void TLx493D_P3I8_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    tlx493d_gen_3_calculateMagneticField(sensor, P3I8_BX_MSBS_e, P3I8_BX_LSBS_e, P3I8_BY_MSBS_e, P3I8_BY_LSBS_e, P3I8_BZ_MSBS_e, P3I8_BZ_LSBS_e, P3I8_TEMP_MSBS_e, P3I8_TEMP_LSBS_e, x, y, z);
}


bool TLx493D_P3I8_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    return tlx493d_common_getMagneticField(sensor, x, y, z);
}


void TLx493D_P3I8_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_P3I8_calculateMagneticField(sensor, x, y, z);
    TLx493D_P3I8_calculateTemperature(sensor, temp);
}


bool TLx493D_P3I8_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    return tlx493d_common_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


// TODO: set all options that must be set, eg MODE ?, reset all bits to defaults ?
bool TLx493D_P3I8_setDefaultConfig(TLx493D_t *sensor) {
    sensor->regMap[0x0A] = 0x02; // Bit 1 is set to constant 1 !

    tlx493d_common_setBitfield(sensor, P3I8_MODE_SEL_e, 0);
    tlx493d_common_setBitfield(sensor, P3I8_INT_DIS_e, 1);
    tlx493d_common_setBitfield(sensor, P3I8_WU_EN_e, 0);
    tlx493d_common_setBitfield(sensor, P3I8_CRC_WR_EN_e, 0);

    tlx493d_common_writeRegister(sensor, P3I8_MODE_SEL_e);

    // return tlx493d_gen_3_readRegistersSPI(sensor);
 
    // tlx493d_common_setBitfield(sensor, CHANNEL_SEL, 0);

    return true;
}


void TLx493D_P3I8_setResetValues(TLx493D_t *sensor) {
    sensor->regMap[0x0A] = 0x02; // MOD1
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


uint8_t TLx493D_P3I8_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr) {
    errorFunctionNotSupportedForSensorType(sensor, "selectIICAddress");
    return 0;
}


void TLx493D_P3I8_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens, double mT, int16_t *rawMF) {
}


void TLx493D_P3I8_getSensitivityScaleFactor(TLx493D_t *sensor, double *sf) {
    tlx493d_common_getSensitivityScaleFactor(sensor, TLx493D_HAS_X2_e, P3I8_SHORT_EN_e, P3I8_XTR_SHORT_EN_e, sf);
}
