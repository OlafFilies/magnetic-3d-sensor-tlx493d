// std includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// project c includes
#include "Logger.h"

// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

// common to same generation of sensors
#include "tlx493d_gen_2_common_defines.h"
#include "tlx493d_gen_2_common.h"

// sensor specific includes
#include "TLx493D_A2B6_defines.h"
#include "TLx493D_A2B6_enums.h"
#include "TLx493D_A2B6.h"


TLx493D_Register_t TLx493D_A2B6_regDef[] = {
    { A2B6_BX_MSBS_e,   TLx493D_READ_MODE_e,       0x00, 0xFF, 0, 8 },
    { A2B6_BY_MSBS_e,   TLx493D_READ_MODE_e,       0x01, 0xFF, 0, 8 },
    { A2B6_BZ_MSBS_e,   TLx493D_READ_MODE_e,       0x02, 0xFF, 0, 8 }, 
    { A2B6_TEMP_MSBS_e, TLx493D_READ_MODE_e,       0x03, 0xFF, 0, 8 },
    { A2B6_BX_LSBS_e,   TLx493D_READ_MODE_e,       0x04, 0xF0, 4, 4 },
    { A2B6_BY_LSBS_e,   TLx493D_READ_MODE_e,       0x04, 0x0F, 0, 4 },
    { A2B6_TEMP_LSBS_e, TLx493D_READ_MODE_e,       0x05, 0xC0, 6, 2 },
    { A2B6_ID_e,        TLx493D_READ_MODE_e,       0x05, 0x30, 4, 2 },
    { A2B6_BZ_LSBS_e,   TLx493D_READ_MODE_e,       0x05, 0x0F, 0, 4 },
    { A2B6_P_e,         TLx493D_READ_MODE_e,       0x06, 0x80, 7, 1 },
    { A2B6_FF_e,        TLx493D_READ_MODE_e,       0x06, 0x40, 6, 1 },
    { A2B6_CF_e,        TLx493D_READ_MODE_e,       0x06, 0x20, 5, 1 },
    { A2B6_T_e,         TLx493D_READ_MODE_e,       0x06, 0x10, 4, 1 },
    { A2B6_PD3_e,       TLx493D_READ_MODE_e,       0x06, 0x08, 3, 1 },
    { A2B6_PD0_e,       TLx493D_READ_MODE_e,       0x06, 0x04, 2, 1 },
    { A2B6_FRM_e,       TLx493D_READ_MODE_e,       0x06, 0x03, 0, 2 },
    { A2B6_DT_e,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x80, 7, 1 },
    { A2B6_AM_e,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x40, 6, 1 },
    { A2B6_TRIG_e,      TLx493D_READ_WRITE_MODE_e, 0x10, 0x30, 4, 2 },
    { A2B6_X2_e,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x08, 3, 1 },
    { A2B6_TL_MAG_e,    TLx493D_READ_WRITE_MODE_e, 0x10, 0x06, 1, 2 },
    { A2B6_CP_e,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x01, 0, 1 },
    { A2B6_FP_e,        TLx493D_READ_WRITE_MODE_e, 0x11, 0x80, 7, 1 },
    { A2B6_IICADR_e,    TLx493D_READ_WRITE_MODE_e, 0x11, 0x60, 5, 2 },
    { A2B6_PR_e,        TLx493D_READ_WRITE_MODE_e, 0x11, 0x10, 4, 1 },
    { A2B6_CA_e,        TLx493D_READ_WRITE_MODE_e, 0x11, 0x08, 3, 1 },
    { A2B6_INT_e,       TLx493D_READ_WRITE_MODE_e, 0x11, 0x04, 2, 1 },
    { A2B6_MODE_e,      TLx493D_READ_WRITE_MODE_e, 0x11, 0x03, 0, 2 },
    // Does not match register overview in manual, but fits and default value
    // textual description of register PRD ! Confirmed by Severin.
    // { A2B6_PRD_e,       TLx493D_READ_WRITE_MODE_e, 0x13, 0xE0, 5, 3 },
    { A2B6_PRD_e,       TLx493D_READ_WRITE_MODE_e, 0x13, 0x80, 7, 1 },
    { A2B6_TYPE_e,      TLx493D_READ_MODE_e,       0x16, 0x30, 4, 2 },
    { A2B6_HWV_e,       TLx493D_READ_MODE_e,       0x16, 0x0F, 0, 4 }
};


TLx493D_CommonFunctions_t TLx493D_A2B6_commonFunctions = {
    .init                           = TLx493D_A2B6_init,
    .deinit                         = TLx493D_A2B6_deinit,

    .readRegisters                  = TLx493D_A2B6_readRegisters,

    .calculateTemperature           = TLx493D_A2B6_calculateTemperature,
    .getTemperature                 = TLx493D_A2B6_getTemperature,

    .calculateMagneticField         = TLx493D_A2B6_calculateMagneticField,
    .getMagneticField               = TLx493D_A2B6_getMagneticField,

    .calculateMagneticFieldAndTemperature = TLx493D_A2B6_calculateMagneticFieldAndTemperature,
    .getMagneticFieldAndTemperature = TLx493D_A2B6_getMagneticFieldAndTemperature,

    // functions related to the "Config" register
    .setMeasurement                 = TLx493D_A2B6_setMeasurement,
    .setTrigger                     = TLx493D_A2B6_setTrigger,
    .setSensitivity                 = TLx493D_A2B6_setSensitivity,
    
    // functions related to the "Mod1" and "Mod2" registers
    .setDefaultConfig               = TLx493D_A2B6_setDefaultConfig,
    .setIICAddress                  = TLx493D_A2B6_setIICAddress,
    .enable1ByteReadMode            = TLx493D_A2B6_enable1ByteReadMode,
    // .disable1ByteReadMode           = TLx493D_A2B6_disable1ByteReadMode,
    .enableInterrupt                = TLx493D_A2B6_enableInterrupt,
    .disableInterrupt               = TLx493D_A2B6_disableInterrupt,
    .enableCollisionAvoidance       = TLx493D_A2B6_enableCollisionAvoidance,
    .disableCollisionAvoidance      = TLx493D_A2B6_disableCollisionAvoidance,
    .setPowerMode                   = TLx493D_A2B6_setPowerMode,
    .setUpdateRate                  = TLx493D_A2B6_setUpdateRate,

    // functions related to the "Diag" register
    .hasValidData                   = TLx493D_A2B6_hasValidData,
    .isFunctional                   = TLx493D_A2B6_isFunctional,

    // functions available only to a subset of sensors with wake-up functionality
    // functions related to the "WU" register
    .hasWakeUp                      = TLx493D_A2B6_hasWakeUp,
    .isWakeUpEnabled                = TLx493D_A2B6_isWakeUpEnabled,
    .enableWakeUpMode               = TLx493D_A2B6_enableWakeUpMode,
    .disableWakeUpMode              = TLx493D_A2B6_disableWakeUpMode,

    .setLowerWakeUpThresholdX       = TLx493D_A2B6_setLowerWakeUpThresholdX,
    .setLowerWakeUpThresholdY       = TLx493D_A2B6_setLowerWakeUpThresholdY,
    .setLowerWakeUpThresholdZ       = TLx493D_A2B6_setLowerWakeUpThresholdZ,

    .setUpperWakeUpThresholdX       = TLx493D_A2B6_setUpperWakeUpThresholdX,
    .setUpperWakeUpThresholdY       = TLx493D_A2B6_setUpperWakeUpThresholdY,
    .setUpperWakeUpThresholdZ       = TLx493D_A2B6_setUpperWakeUpThresholdZ,

    .setWakeUpThresholdsAsInteger   = TLx493D_A2B6_setWakeUpThresholdsAsInteger,
    .setWakeUpThresholds            = TLx493D_A2B6_setWakeUpThresholds,

    .softReset                      = TLx493D_A2B6_softReset,

    // functions used internally and not accessible through the common interface
    .calculateFuseParity            = TLx493D_A2B6_calculateFuseParity,
    .calculateBusParity             = TLx493D_A2B6_calculateBusParity,
    .calculateConfigurationParity   = TLx493D_A2B6_calculateConfigurationParity,

    .hasValidFuseParity             = TLx493D_A2B6_hasValidFuseParity,
    .hasValidBusParity              = TLx493D_A2B6_hasValidBusParity,
    .hasValidConfigurationParity    = TLx493D_A2B6_hasValidConfigurationParity,
    .hasValidTBit                   = TLx493D_A2B6_hasValidTBit,

    .setResetValues                 = TLx493D_A2B6_setResetValues,
};


bool TLx493D_A2B6_init(TLx493D_t *sensor) {
    // TODO: use in TLx493D_initCommunication
    tlx493d_setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return tlx493d_common_init(sensor, GEN_2_REG_MAP_SIZE, TLx493D_A2B6_regDef, &TLx493D_A2B6_commonFunctions, TLx493D_A2B6_e, TLx493D_I2C_e);
}


bool TLx493D_A2B6_deinit(TLx493D_t *sensor) {
    return tlx493d_common_deinit(sensor);
}


bool TLx493D_A2B6_readRegisters(TLx493D_t *sensor) {
    return tlx493d_common_readRegisters(sensor);
}


void TLx493D_A2B6_calculateTemperature(TLx493D_t *sensor, double *temp) {
    tlx493d_gen_2_calculateTemperature(sensor, temp, A2B6_TEMP_MSBS_e, A2B6_TEMP_LSBS_e);
}


bool TLx493D_A2B6_getTemperature(TLx493D_t *sensor, double *temp) {
    return tlx493d_common_getTemperature(sensor, temp);
}


void TLx493D_A2B6_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    tlx493d_gen_2_calculateMagneticField(sensor, x, y, z, A2B6_BX_MSBS_e, A2B6_BX_LSBS_e, A2B6_BY_MSBS_e, A2B6_BY_LSBS_e, A2B6_BZ_MSBS_e, A2B6_BZ_LSBS_e);
}


bool TLx493D_A2B6_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    return tlx493d_common_getMagneticField(sensor, x, y, z);
}


void TLx493D_A2B6_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_A2B6_calculateMagneticField(sensor, x, y, z);
    TLx493D_A2B6_calculateTemperature(sensor, temp);
}


bool TLx493D_A2B6_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    return tlx493d_common_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


bool TLx493D_A2B6_setMeasurement(TLx493D_t *sensor, TLx493D_MeasurementType_t val) {
    return tlx493d_gen_2_setMeasurement(sensor, A2B6_DT_e, A2B6_AM_e, A2B6_CP_e, val);

    // uint8_t dt = 0;
    // uint8_t am = 0;

    // switch(val) {
    //     case TLx493D_BxByBzTemp_e : dt = 0;
    //                                 am = 0;
    //                                 break;

    //     case TLx493D_BxByBz_e : dt = 1;
    //                             am = 0;
    //                             break;
        
    //     case TLx493D_BxBy_e : dt = 1;
    //                           am = 1;
    //                           break;
        
    //     default : errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_MeasurementType_t");
    //               return false;
    // }

    // return tlx493d_gen_2_setTwoConfigBitfields(sensor, A2B6_DT_e, A2B6_AM_e, A2B6_CP_e, dt, am);
}


//  // This option depends on PR and MODE.
bool TLx493D_A2B6_setTrigger(TLx493D_t *sensor, TLx493D_TriggerType_t val) {
    return tlx493d_gen_2_setTrigger(sensor, A2B6_TRIG_e, A2B6_CP_e, val);

    // uint8_t trig = 0;

    // switch(val) {
    //     case TLx493D_NO_ADC_ON_READ_e : trig = 0;
    //                                     break;

    //     case TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e : trig = 1;
    //                                                   break;
        
    //     case TLx493D_ADC_ON_READ_AFTER_REG_05_e : trig = 2;
    //                                               break;
        
    //     default : errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_TriggerType_t");
    //               return false;
    // }

    // return tlx493d_gen_2_setOneConfigBitfield(sensor, A2B6_TRIG_e, A2B6_CP_e, trig);
}


bool TLx493D_A2B6_setSensitivity(TLx493D_t *sensor, TLx493D_SensitivityType_t val) {
    return tlx493d_gen_2_setSensitivity(sensor, A2B6_X2_e, A2B6_CP_e, val);

    // uint8_t sens = 0;

    // switch(val) {
    //     case TLx493D_FULL_RANGE_e : sens = 0;
    //                                 break;

    //     case TLx493D_SHORT_RANGE_e : sens = 1;
    //                                  break;
        
    //     default : errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_SensitivityType_t");
    //               return false;
    // }

    // return tlx493d_gen_2_setOneConfigBitfield(sensor, A2B6_X2_e, A2B6_CP_e, sens);
}


// bool TLx493D_A2B6_setTC0MagneticTemperatureCompensation(TLx493D_t *sensor) {
//     return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, A2B6_TL_MAG_e, A2B6_CP_e, 0b00);
// }


// bool TLx493D_A2B6_setTC1MagneticTemperatureCompensation(TLx493D_t *sensor) {
//     return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, A2B6_TL_MAG_e, A2B6_CP_e, 0b01);
// }


// bool TLx493D_A2B6_setTC2MagneticTemperatureCompensation(TLx493D_t *sensor) {
//     return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, A2B6_TL_MAG_e, A2B6_CP_e, 0b10);
// }


// bool TLx493D_A2B6_setTC3MagneticTemperatureCompensation(TLx493D_t *sensor) {
//     return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, A2B6_TL_MAG_e, A2B6_CP_e, 0b11);
// }


bool TLx493D_A2B6_setDefaultConfig(TLx493D_t *sensor) {
    return tlx493d_gen_2_setDefaultConfig(sensor, A2B6_CONFIG_REG_e, A2B6_MOD1_REG_e, A2B6_MOD2_REG_e, A2B6_CP_e, A2B6_CA_e, A2B6_INT_e);
}


bool TLx493D_A2B6_setIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t address) {
    return tlx493d_gen_2_setIICAddress(sensor, A2B6_IICADR_e, A2B6_FP_e, address);
}


bool TLx493D_A2B6_enable1ByteReadMode(TLx493D_t *sensor) {
    return tlx493d_gen_2_set1ByteReadMode(sensor, A2B6_PR_e, A2B6_FP_e, A2B6_PRD_e, 1);
}


// bool TLx493D_A2B6_disable1ByteReadMode(TLx493D_t *sensor) {
//     return tlx493d_gen_2_set1ByteReadMode(sensor, A2B6_PR_e, A2B6_FP_e, A2B6_PRD_e, 0);
// }


bool TLx493D_A2B6_enableCollisionAvoidance(TLx493D_t *sensor) {
    return tlx493d_gen_2_setCollisionAvoidance(sensor, A2B6_CA_e, A2B6_FP_e, A2B6_PRD_e, 0);
}


bool TLx493D_A2B6_disableCollisionAvoidance(TLx493D_t *sensor) {
    return tlx493d_gen_2_setCollisionAvoidance(sensor, A2B6_CA_e, A2B6_FP_e, A2B6_PRD_e, 1);
}


bool TLx493D_A2B6_enableInterrupt(TLx493D_t *sensor) {
    return tlx493d_gen_2_setInterrupt(sensor, A2B6_INT_e, A2B6_FP_e, A2B6_PRD_e, 0);
}


bool TLx493D_A2B6_disableInterrupt(TLx493D_t *sensor) {
    return tlx493d_gen_2_setInterrupt(sensor, A2B6_INT_e, A2B6_FP_e, A2B6_PRD_e, 1);
}


bool TLx493D_A2B6_setPowerMode(TLx493D_t *sensor, TLx493D_PowerModeType_t mode) {
    return tlx493d_gen_2_setPowerMode(sensor, A2B6_MODE_e, A2B6_FP_e, mode);
}


bool TLx493D_A2B6_setUpdateRate(TLx493D_t *sensor, TLx493D_UpdateRateType_t val) {
    uint8_t mod1 = A2B6_MOD1_REG_e; // sensor->regDef[fpBF].address;
    uint8_t rate = 0;

    switch(val) {
        case UPDATE_RATE_FAST_e : rate = 0;
                                  break;

        case UPDATE_RATE_SLOW_e : rate = 1;
                                  break;

        default : errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_UpdateRateType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, A2B6_PRD_e, rate);
    tlx493d_common_setBitfield(sensor, A2B6_FP_e, tlx493d_gen_2_calculateFuseParity(sensor, A2B6_FP_e, A2B6_PRD_e));

    uint8_t buf[4] = { mod1,
                       sensor->regMap[mod1],     // MOD1 register
                       sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
                       sensor->regMap[mod1 + 2]  // MOD2 register
                     };

    return transfer(sensor, buf, sizeof(buf), NULL, 0);
}


bool TLx493D_A2B6_hasValidData(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidData(sensor);
}


bool TLx493D_A2B6_isFunctional(TLx493D_t *sensor) {
    return tlx493d_gen_2_isFunctional(sensor);
}


bool TLx493D_A2B6_hasWakeUp(TLx493D_t *sensor) {
    // return tlx493d_gen_2_hasWakeUp(sensor, TYPE);
    return false;
}


bool TLx493D_A2B6_isWakeUpEnabled(TLx493D_t *sensor) {
    warnFeatureNotAvailableForSensorType(sensor, "isWakeUpEnabled");
    return false;
}

bool TLx493D_A2B6_enableWakeUpMode(TLx493D_t *sensor) {
    warnFeatureNotAvailableForSensorType(sensor, "enableWakeUpMode");
    return false;
}

bool TLx493D_A2B6_disableWakeUpMode(TLx493D_t *sensor) {
    warnFeatureNotAvailableForSensorType(sensor, "disableWakeUpMode");
    return false;
}

bool TLx493D_A2B6_setLowerWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold) {
    warnFeatureNotAvailableForSensorType(sensor, "setLowerWakeUpThresholdX");
    return false;
}

bool TLx493D_A2B6_setLowerWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold) {
    warnFeatureNotAvailableForSensorType(sensor, "setLowerWakeUpThresholdY");
    return false;
}

bool TLx493D_A2B6_setLowerWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold) {
    warnFeatureNotAvailableForSensorType(sensor, "setLowerWakeUpThresholdZ");
    return false;
}

bool TLx493D_A2B6_setUpperWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold) {
    warnFeatureNotAvailableForSensorType(sensor, "setUpperWakeUpThresholdX");
    return false;
}

bool TLx493D_A2B6_setUpperWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold) {
    warnFeatureNotAvailableForSensorType(sensor, "setUpperWakeUpThresholdY");
    return false;
}

bool TLx493D_A2B6_setUpperWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold) {
    warnFeatureNotAvailableForSensorType(sensor, "setUpperWakeUpThresholdZ");
    return false;
}


bool TLx493D_A2B6_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
    warnFeatureNotAvailableForSensorType(sensor, "setWakeUpThresholdsAsInteger");
    return false;
}

// thesholds im mT, to be converted to proper format
bool TLx493D_A2B6_setWakeUpThresholds(TLx493D_t *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh) {
    warnFeatureNotAvailableForSensorType(sensor, "setWakeUpThresholds");
    return false;
}

bool TLx493D_A2B6_softReset(TLx493D_t *sensor) {
    warnFeatureNotAvailableForSensorType(sensor, "softReset");
    return false;
}



uint8_t TLx493D_A2B6_calculateFuseParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_calculateFuseParity(sensor, A2B6_FP_e, A2B6_PRD_e);
}


uint8_t TLx493D_A2B6_calculateBusParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_calculateBusParity(sensor, 5);
}


uint8_t TLx493D_A2B6_calculateConfigurationParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_calculateConfigurationParity(sensor, A2B6_CP_e);
}


bool TLx493D_A2B6_hasValidFuseParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidFuseParity(sensor, A2B6_FF_e);
}


bool TLx493D_A2B6_hasValidBusParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidBusParity(sensor, A2B6_P_e);
}


bool TLx493D_A2B6_hasValidConfigurationParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidConfigurationParity(sensor, A2B6_CF_e);
}


bool TLx493D_A2B6_hasValidIICadr(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidIICadr(sensor, A2B6_ID_e, A2B6_IICADR_e);
}


bool TLx493D_A2B6_hasValidTBit(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidTBit(sensor, A2B6_T_e);
}


// bool TLx493D_A2B6_hasValidTemperatureData(TLx493D_t *sensor) {
//     return tlx493d_gen_2_hasValidTemperatureData(sensor);
// }


// bool TLx493D_A2B6_hasValidMagneticFieldData(TLx493D_t *sensor) {
//     return tlx493d_gen_2_hasValidMagneticFieldData(sensor);
// }


// bool TLx493D_A2B6_hasValidPD0Bit(TLx493D_t *sensor) {
//     return tlx493d_gen_2_hasValidPD0Bit(sensor, PD0);
// }


// bool TLx493D_A2B6_hasValidPD3Bit(TLx493D_t *sensor) {
//     return tlx493d_gen_2_hasValidPD3Bit(sensor, PD3);
// }


void TLx493D_A2B6_setResetValues(TLx493D_t *sensor) {
    sensor->regMap[0x10] = 0x00; // CONFIG
    sensor->regMap[0x11] = 0x00; // MOD1
    sensor->regMap[0x13] = 0x00; // MOD2
}
