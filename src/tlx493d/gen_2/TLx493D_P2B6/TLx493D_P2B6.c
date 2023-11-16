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
#include "TLx493D_P2B6_defines.h"
#include "TLx493D_P2B6.h"


/*
  Listing of all register names for this sensor.
  Used to index "TLx493D_P2B6_regDef" defined below, so index values must match !
*/
typedef enum {
               P2B6_BX_MSBS_e = 0,
               P2B6_BY_MSBS_e,
               P2B6_BZ_MSBS_e,
               P2B6_TEMP_MSBS_e,
               P2B6_BX_LSBS_e,
               P2B6_BY_LSBS_e,
               P2B6_TEMP_LSBS_e,
               P2B6_ID_e,
               P2B6_BZ_LSBS_e,
               P2B6_P_e,
               P2B6_FF_e,
               P2B6_CF_e,
               P2B6_T_e,
               P2B6_PD3_e,
               P2B6_PD0_e,
               P2B6_FRM_e,
               P2B6_XL_e,
               P2B6_XH_e,
               P2B6_YL_e,
               P2B6_YH_e,
               P2B6_ZL_e,
               P2B6_ZH_e,
               P2B6_WA_e,
               P2B6_WU_e,
               P2B6_XH_LSBS_e,
               P2B6_XL_LSBS_e,
               P2B6_TST_e,
               P2B6_YH_LSBS_e,
               P2B6_YL_LSBS_e,
               P2B6_PH_e,
               P2B6_ZH_LSBS_e,
               P2B6_ZL_LSBS_e,
               P2B6_DT_e,
               P2B6_AM_e,
               P2B6_TRIG_e,
               P2B6_X2_e,
               P2B6_TL_MAG_e,
               P2B6_CP_e,
               P2B6_FP_e,
               P2B6_IICADR_e,
               P2B6_PR_e,
               P2B6_CA_e,
               P2B6_INT_e,
               P2B6_MODE_e,
               P2B6_PRD_e,
               P2B6_TYPE_e,
               P2B6_HWV_e,
} TLx493D_P2B6_registerNames_te;


TLx493D_Register_t TLx493D_P2B6_regDef[] = {
    { P2B6_BX_MSBS_e,    TLx493D_READ_MODE_e,       0x00, 0xFF, 0, 8},
    { P2B6_BY_MSBS_e,    TLx493D_READ_MODE_e,       0x01, 0xFF, 0, 8},
    { P2B6_BZ_MSBS_e,    TLx493D_READ_MODE_e,       0x02, 0xFF, 0, 8},
    { P2B6_TEMP_MSBS_e,  TLx493D_READ_MODE_e,       0x03, 0xFF, 0, 8},
    { P2B6_BX_LSBS_e,    TLx493D_READ_MODE_e,       0x04, 0xF0, 4, 4},
    { P2B6_BY_LSBS_e,    TLx493D_READ_MODE_e,       0x04, 0x0F, 0, 4},
    { P2B6_TEMP_LSBS_e,  TLx493D_READ_MODE_e,       0x05, 0xC0, 6, 2},
    { P2B6_ID_e,        TLx493D_READ_MODE_e,       0x05, 0x30, 4, 2},
    { P2B6_BZ_LSBS_e,    TLx493D_READ_MODE_e,       0x05, 0x0F, 0, 4},
    { P2B6_P_e,         TLx493D_READ_MODE_e,       0x06, 0x80, 7, 1},
    { P2B6_FF_e,        TLx493D_READ_MODE_e,       0x06, 0x40, 6, 1},
    { P2B6_CF_e,        TLx493D_READ_MODE_e,       0x06, 0x20, 5, 1},
    { P2B6_T_e,         TLx493D_READ_MODE_e,       0x06, 0x10, 4, 1},
    { P2B6_PD3_e,       TLx493D_READ_MODE_e,       0x06, 0x08, 3, 1},
    { P2B6_PD0_e,       TLx493D_READ_MODE_e,       0x06, 0x04, 2, 1},
    { P2B6_FRM_e,       TLx493D_READ_MODE_e,       0x06, 0x03, 0, 2},
    { P2B6_XL_e,        TLx493D_READ_WRITE_MODE_e, 0x07, 0xFF, 0, 8},
    { P2B6_XH_e,        TLx493D_READ_WRITE_MODE_e, 0x08, 0xFF, 0, 8},
    { P2B6_YL_e,        TLx493D_READ_WRITE_MODE_e, 0x09, 0xFF, 0, 8},
    { P2B6_YH_e,        TLx493D_READ_WRITE_MODE_e, 0x0A, 0xFF, 0, 8},
    { P2B6_ZL_e,        TLx493D_READ_WRITE_MODE_e, 0x0B, 0xFF, 0, 8},
    { P2B6_ZH_e,        TLx493D_READ_WRITE_MODE_e, 0x0C, 0xFF, 0, 8},
    { P2B6_WA_e,        TLx493D_READ_MODE_e,       0x0D, 0x80, 7, 1},
    { P2B6_WU_e,        TLx493D_READ_WRITE_MODE_e, 0x0D, 0x40, 6, 1},
    { P2B6_XH_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0D, 0x38, 3, 3},
    { P2B6_XL_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0D, 0x07, 0, 3},
    { P2B6_TST_e,       TLx493D_READ_WRITE_MODE_e, 0x0E, 0xC0, 6, 2},
    { P2B6_YH_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0E, 0x38, 3, 3},  
    { P2B6_YL_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0E, 0x07, 0, 3},
    { P2B6_PH_e,        TLx493D_READ_WRITE_MODE_e, 0x0F, 0xC0, 6, 2},
    { P2B6_ZH_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0F, 0x38, 3, 3},
    { P2B6_ZL_LSBS_e,    TLx493D_READ_WRITE_MODE_e, 0x0F, 0x07, 0, 3},
    { P2B6_DT_e,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x80, 7, 1},
    { P2B6_AM_e,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x40, 6, 1},
    { P2B6_TRIG_e,      TLx493D_READ_WRITE_MODE_e, 0x10, 0x30, 4, 2},
    { P2B6_X2_e,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x08, 3, 1},
    { P2B6_TL_MAG_e,    TLx493D_READ_WRITE_MODE_e, 0x10, 0x06, 1, 2},
    { P2B6_CP_e,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x01, 0, 1},
    { P2B6_FP_e,        TLx493D_READ_WRITE_MODE_e, 0x11, 0x80, 7, 1},
    { P2B6_IICADR_e,    TLx493D_READ_WRITE_MODE_e, 0x11, 0x60, 5, 2},
    { P2B6_PR_e,        TLx493D_READ_WRITE_MODE_e, 0x11, 0x10, 4, 1},
    { P2B6_CA_e,        TLx493D_READ_WRITE_MODE_e, 0x11, 0x08, 3, 1},
    { P2B6_INT_e,       TLx493D_READ_WRITE_MODE_e, 0x11, 0x04, 2, 1},
    { P2B6_MODE_e,      TLx493D_READ_WRITE_MODE_e, 0x11, 0x03, 0, 2},
    { P2B6_PRD_e,       TLx493D_READ_WRITE_MODE_e, 0x13, 0xE0, 5, 3},
    { P2B6_TYPE_e,      TLx493D_READ_MODE_e,       0x16, 0x30, 4, 2},
    { P2B6_HWV_e,       TLx493D_READ_MODE_e,       0x16, 0x0F, 0, 4}
};


typedef enum { 
               P2B6_TEMP2_REG_e  = 0x05,
               P2B6_DIAG_REG_e   = 0x06,
               P2B6_CONFIG_REG_e = 0x10,
               P2B6_MOD1_REG_e   = 0x11,
               P2B6_MOD2_REG_e   = 0x13,
               P2B6_VER_REG_e    = 0x16 } TLx493D_P2B6_SpecialRegisters_te;


TLx493D_CommonFunctions_t TLx493D_P2B6_commonFunctions = {
    .init                           = TLx493D_P2B6_init,
    .deinit                         = TLx493D_P2B6_deinit,

    .readRegisters                  = TLx493D_P2B6_readRegisters, //tlx493d_common_readRegisters,

    .calculateRawTemperature        = TLx493D_P2B6_calculateRawTemperature,
    .getRawTemperature              = TLx493D_P2B6_getRawTemperature,

    .calculateRawMagneticField      = TLx493D_P2B6_calculateRawMagneticField,
    .getRawMagneticField            = TLx493D_P2B6_getRawMagneticField,

    .calculateRawMagneticFieldAndTemperature = TLx493D_P2B6_calculateRawMagneticFieldAndTemperature,
    .getRawMagneticFieldAndTemperature       = TLx493D_P2B6_getRawMagneticFieldAndTemperature,

    .calculateTemperature           = TLx493D_P2B6_calculateTemperature,
    .getTemperature                 = TLx493D_P2B6_getTemperature,

    .calculateMagneticField         = TLx493D_P2B6_calculateMagneticField,
    .getMagneticField               = TLx493D_P2B6_getMagneticField,

    .calculateMagneticFieldAndTemperature = TLx493D_P2B6_calculateMagneticFieldAndTemperature,
    .getMagneticFieldAndTemperature = TLx493D_P2B6_getMagneticFieldAndTemperature,

    .setDefaultConfig               = TLx493D_P2B6_setDefaultConfig,

    .setResetValues                 = TLx493D_P2B6_setResetValues,

    .calculateRawMagneticFieldAtTemperature = TLx493D_P2B6_calculateRawMagneticFieldAtTemperature,

    .getSensitivityScaleFactor      = TLx493D_P2B6_getSensitivityScaleFactor,
};


// TODO: add parameter IICAddress or ad function to set address.
bool TLx493D_P2B6_init(TLx493D_t *sensor) {
    // TODO: use in TLx493D_initCommunication
    tlx493d_setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return tlx493d_common_init(sensor, GEN_2_REG_MAP_SIZE, TLx493D_P2B6_regDef, &TLx493D_P2B6_commonFunctions, TLx493D_P2B6_e, TLx493D_I2C_e);
}


bool TLx493D_P2B6_deinit(TLx493D_t *sensor) {
    return tlx493d_common_deinit(sensor);
}


bool TLx493D_P2B6_readRegisters(TLx493D_t *sensor) {
    return tlx493d_common_readRegisters(sensor);
}


void TLx493D_P2B6_calculateRawTemperature(TLx493D_t *sensor, uint16_t *temperature) {
    tlx493d_gen_2_calculateRawTemperature(sensor, P2B6_TEMP_MSBS_e, P2B6_TEMP_LSBS_e, temperature);
}


bool TLx493D_P2B6_getRawTemperature(TLx493D_t *sensor, uint16_t *temperature) {
    return tlx493d_common_getRawTemperature(sensor, temperature);
}


void TLx493D_P2B6_calculateRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z) {
    tlx493d_gen_2_calculateRawMagneticField(sensor, P2B6_BX_MSBS_e, P2B6_BX_LSBS_e, P2B6_BY_MSBS_e, P2B6_BY_LSBS_e, P2B6_BZ_MSBS_e, P2B6_BZ_LSBS_e, x, y, z);
}


bool TLx493D_P2B6_getRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z) {
    return tlx493d_common_getRawMagneticField(sensor, x, y, z);
}


void TLx493D_P2B6_calculateRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature) {
    TLx493D_P2B6_calculateRawMagneticField(sensor, x, y, z);
    TLx493D_P2B6_calculateRawTemperature(sensor, temperature);
}


bool TLx493D_P2B6_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature) {
    return tlx493d_common_getRawMagneticFieldAndTemperature(sensor, x, y, z, temperature);
}


void TLx493D_P2B6_calculateTemperature(TLx493D_t *sensor, double *temp) {
    tlx493d_gen_2_calculateTemperature(sensor, P2B6_TEMP_MSBS_e, P2B6_TEMP_LSBS_e, temp);
}


bool TLx493D_P2B6_getTemperature(TLx493D_t *sensor, double *temp) {
    return tlx493d_common_getTemperature(sensor, temp);
}


void TLx493D_P2B6_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    tlx493d_gen_2_calculateMagneticField(sensor, P2B6_BX_MSBS_e, P2B6_BX_LSBS_e, P2B6_BY_MSBS_e, P2B6_BY_LSBS_e, P2B6_BZ_MSBS_e, P2B6_BZ_LSBS_e, x, y, z);
}


bool TLx493D_P2B6_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    return tlx493d_common_getMagneticField(sensor, x, y, z);
}


void TLx493D_P2B6_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_P2B6_calculateMagneticField(sensor, x, y, z);
    TLx493D_P2B6_calculateTemperature(sensor, temp);
}


bool TLx493D_P2B6_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    return tlx493d_common_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}

bool TLx493D_P2B6_setDisableTemperatureMeasurements(TLx493D_t *sensor, uint8_t dt) {
    // uint8_t config = sensor->commonRegisters.CONFIG;

    // CONFIG register
    tlx493d_common_setBitfield(sensor, P2B6_DT_e, dt);
    tlx493d_common_setBitfield(sensor, P2B6_CP_e, TLx493D_P2B6_calculateConfigurationParity(sensor));

    tlx493d_common_writeRegister(sensor, P2B6_DT_e);
}


bool TLx493D_P2B6_enableTemperatureMeasurements(TLx493D_t *sensor) {
    return TLx493D_P2B6_setDisableTemperatureMeasurements(sensor, 0);
}


bool TLx493D_P2B6_disableTemperatureMeasurements(TLx493D_t *sensor) {
    return TLx493D_P2B6_setDisableTemperatureMeasurements(sensor, 1);
}



// TODO: set all options that must be set, eg MODE ?, reset all bits to defaults !
bool TLx493D_P2B6_setDefaultConfig(TLx493D_t *sensor) {
    sensor->regMap[P2B6_CONFIG_REG_e] = 0x00;
    sensor->regMap[P2B6_MOD1_REG_e]   = 0x00;
    sensor->regMap[P2B6_MOD2_REG_e]   = 0x00;;

    // MOD1 register
    tlx493d_common_setBitfield(sensor, P2B6_CA_e, 0);
    tlx493d_common_setBitfield(sensor, P2B6_INT_e, 1);

    if( TLx493D_P2B6_enable1ByteMode(sensor) ) {
        if( TLx493D_P2B6_enableTemperatureMeasurements(sensor) ) {
            // Read registers in order to retrieve values in reserved register at 0x12 and in MOD2 in order to make sure we are not 
            // accidentally changing a preset values to 0.
            // if( tlx493d_gen_2_readRegisters(sensor) )
            //     return TLx493D_P2B6_setLowUpdateRate(sensor);

            return true;
        }
    }

    return false;
}


bool TLx493D_P2B6_set1ByteReadMode(TLx493D_t *sensor, uint8_t pr) {
    tlx493d_common_setBitfield(sensor, P2B6_PR_e, pr);
    tlx493d_common_setBitfield(sensor, P2B6_FP_e, tlx493d_gen_2_calculateFuseParity(sensor, P2B6_FP_e, P2B6_PRD_e));

    tlx493d_common_writeRegister(sensor, P2B6_PR_e);
}


bool TLx493D_P2B6_enable1ByteMode(TLx493D_t *sensor) {
     return TLx493D_P2B6_set1ByteReadMode(sensor, 1);
}


bool TLx493D_P2B6_disable1ByteMode(TLx493D_t *sensor) {
     return TLx493D_P2B6_set1ByteReadMode(sensor, 0);
}


// Must be regs 0x07 - 0x10
uint8_t TLx493D_P2B6_calculateConfigurationParity(TLx493D_t *sensor) {
	uint8_t parity = tlx493d_common_calculateParity(sensor->regMap[P2B6_CONFIG_REG_e] & ~sensor->regDef[P2B6_CP_e].mask);
	return tlx493d_common_getEvenParity(parity);
}


void TLx493D_P2B6_setResetValues(TLx493D_t *sensor) {
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
}


void TLx493D_P2B6_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t *rawTemp, TLx493D_SensitivityType_t sens, double mT, int16_t *rawMF) {

}


void TLx493D_P2B6_getSensitivityScaleFactor(TLx493D_t *sensor, double *sf) {
    tlx493d_common_getSensitivityScaleFactor(sensor, TLx493D_HAS_X2_e, P2B6_X2_e, 0, sf);
}
