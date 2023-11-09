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
#include "TLx493D_A2B6_defines.h"
#include "TLx493D_A2B6.h"


/***
 * Listing of all register names for this sensor.
 * Used to index "TLx493D_A2B6_regDef" defined below, so index values must match !
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
               DT,
               AM,
               TRIG,
               X2,
               TL_MAG,
               CP,
               FP,
               IICADR,
               PR,
               CA,
               INT,
               MODE,
               PRD,
               TYPE,
               HWV } TLx493D_A2B6_registerNames_te;


TLx493D_Register_ts TLx493D_A2B6_regDef[] = {
    { BX_MSB,   TLx493D_READ_MODE_e,       0x00, 0xFF, 0, 8 },
    { BY_MSB,   TLx493D_READ_MODE_e,       0x01, 0xFF, 0, 8 },
    { BZ_MSB,   TLx493D_READ_MODE_e,       0x02, 0xFF, 0, 8 }, 
    { TEMP_MSB, TLx493D_READ_MODE_e,       0x03, 0xFF, 0, 8 },
    { BX_LSB,   TLx493D_READ_MODE_e,       0x04, 0xF0, 4, 4 },
    { BY_LSB,   TLx493D_READ_MODE_e,       0x04, 0x0F, 0, 4 },
    { TEMP_LSB, TLx493D_READ_MODE_e,       0x05, 0xC0, 6, 2 },
    { ID,       TLx493D_READ_MODE_e,       0x05, 0x30, 4, 2 },
    { BZ_LSB,   TLx493D_READ_MODE_e,       0x05, 0x0F, 0, 4 },
    { P,        TLx493D_READ_MODE_e,       0x06, 0x80, 7, 1 },
    { FF,       TLx493D_READ_MODE_e,       0x06, 0x40, 6, 1 },
    { CF,       TLx493D_READ_MODE_e,       0x06, 0x20, 5, 1 },
    { T,        TLx493D_READ_MODE_e,       0x06, 0x10, 4, 1 },
    { PD3,      TLx493D_READ_MODE_e,       0x06, 0x08, 3, 1 },
    { PD0,      TLx493D_READ_MODE_e,       0x06, 0x04, 2, 1 },
    { FRM,      TLx493D_READ_MODE_e,       0x06, 0x03, 0, 2 },
    { DT,       TLx493D_READ_WRITE_MODE_e, 0x10, 0x80, 7, 1 },
    { AM,       TLx493D_READ_WRITE_MODE_e, 0x10, 0x40, 6, 1 },
    { TRIG,     TLx493D_READ_WRITE_MODE_e, 0x10, 0x30, 4, 2 },
    { X2,       TLx493D_READ_WRITE_MODE_e, 0x10, 0x08, 3, 1 },
    { TL_MAG,   TLx493D_READ_WRITE_MODE_e, 0x10, 0x06, 1, 2 },
    { CP,       TLx493D_READ_WRITE_MODE_e, 0x10, 0x01, 0, 1 },
    { FP,       TLx493D_READ_WRITE_MODE_e, 0x11, 0x80, 7, 1 },
    { IICADR,   TLx493D_READ_WRITE_MODE_e, 0x11, 0x60, 5, 2 },
    { PR,       TLx493D_READ_WRITE_MODE_e, 0x11, 0x10, 4, 1 },
    { CA,       TLx493D_READ_WRITE_MODE_e, 0x11, 0x08, 3, 1 },
    { INT,      TLx493D_READ_WRITE_MODE_e, 0x11, 0x04, 2, 1 },
    { MODE,     TLx493D_READ_WRITE_MODE_e, 0x11, 0x03, 0, 2 },
    // Does not match register overview in manual, but fits and default value
    // textual description of register PRD ! Confirmed by Severin.
    // { PRD,      TLx493D_READ_WRITE_MODE_e, 0x13, 0xE0, 5, 3 },
    { PRD,      TLx493D_READ_WRITE_MODE_e, 0x13, 0x80, 7, 1 },
    { TYPE,     TLx493D_READ_MODE_e,       0x16, 0x30, 4, 2 },
    { HWV,      TLx493D_READ_MODE_e,       0x16, 0x0F, 0, 4 }
};


typedef enum { 
               TEMP2_REG  = 0x05,
               DIAG_REG   = 0x06,
               CONFIG_REG = 0x10,
               MOD1_REG   = 0x11,
               MOD2_REG   = 0x13,
               VER_REG    = 0x16 } SpecialRegisters_te;


TLx493D_CommonFunctions_ts TLx493D_A2B6_commonFunctions = {
    .init                           = TLx493D_A2B6_init,
    .deinit                         = TLx493D_A2B6_deinit,

    .readRegisters                  = TLx493D_A2B6_readRegisters, // tlx493d_common_readRegisters,

    .calculateTemperature           = TLx493D_A2B6_calculateTemperature,
    .getTemperature                 = TLx493D_A2B6_getTemperature,

    .calculateMagneticField         = TLx493D_A2B6_calculateMagneticField,
    .getMagneticField               = TLx493D_A2B6_getMagneticField,

    .calculateMagneticFieldAndTemperature = TLx493D_A2B6_calculateMagneticFieldAndTemperature,
    .getMagneticFieldAndTemperature = TLx493D_A2B6_getMagneticFieldAndTemperature,

    // functions related to the "Config" register
    .setMeasurement                 = TLx493D_A2B6_setMeasurement,
    // .enableTemperatureMeasurement  = TLx493D_A2B6_enableTemperatureMeasurement,
    // .disableTemperatureMeasurement = TLx493D_A2B6_disableTemperatureMeasurement,
    // .enableAngularMeasurement  = TLx493D_A2B6_enableAngularMeasurement,
    // .disableAngularMeasurement = TLx493D_A2B6_disableAngularMeasurement,


    .setTrigger                     = TLx493D_A2B6_setTrigger,
    .setSensitivity                 = TLx493D_A2B6_setSensitivity,

    
    // functions related to the "Mod1" and "Mod2" registers
    .setDefaultConfig               = TLx493D_A2B6_setDefaultConfig,
    .setIICAddress                  = TLx493D_A2B6_setIICAddress,

    .enableInterrupt                = TLx493D_A2B6_enableInterrupt,
    .disableInterrupt               = TLx493D_A2B6_disableInterrupt,
    .enableCollisionAvoidance       = TLx493D_A2B6_enableCollisionAvoidance,
    .disableCollisionAvoidance      = TLx493D_A2B6_disableCollisionAvoidance,

    .setPowerMode                   = TLx493D_A2B6_setPowerMode,
    .setUpdateRate                  = TLx493D_A2B6_setUpdateRate,


    // functions related to the "Diag" register
    .hasValidData                   = TLx493D_A2B6_hasValidData, // tlx493d_gen_2_hasValidData,
    // .hasValidTemperatureData   = TLx493D_A2B6_hasValidTemperatureData; // tlx493d_gen_2_hasValidTemperatureData,
    // .hasValidMagneticFieldData = TLx493D_A2B6_hasValidMagneticFieldData; // tlx493d_gen_2_hasValidMagneticFieldData,
    .isFunctional                   = TLx493D_A2B6_isFunctional, // tlx493d_gen_2_isFunctional,


    // functions available only to a subset of sensors with wake-up functionality
    // functions related to the "WU" register
    .isWakeUpActive                 = TLx493D_A2B6_isWakeUpActive,
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
    // TODO: required ?
    .enable1ByteReadMode            = TLx493D_A2B6_enable1ByteReadMode,
    .disable1ByteReadMode           = TLx493D_A2B6_disable1ByteReadMode,

    // TODO: required ?
    .calculateFuseParity            = TLx493D_A2B6_calculateFuseParity,
    .calculateBusParity             = TLx493D_A2B6_calculateBusParity,
    .calculateConfigurationParity   = TLx493D_A2B6_calculateConfigurationParity,

    // TODO: required ?
    .hasValidFuseParity             = TLx493D_A2B6_hasValidFuseParity,
    .hasValidBusParity              = TLx493D_A2B6_hasValidBusParity,
    .hasValidConfigurationParity    = TLx493D_A2B6_hasValidConfigurationParity,

    // TODO: required ?
    .hasValidTBit                   = TLx493D_A2B6_hasValidTBit,
    // .hasValidPD0Bit                 = TLx493D_A2B6_hasValidPD0Bit,
    // .hasValidPD3Bit                 = TLx493D_A2B6_hasValidPD3Bit,

    .setResetValues                 = TLx493D_A2B6_setResetValues,
};


bool TLx493D_A2B6_init(TLx493D_ts *sensor) {
    // TODO: use in TLx493D_initCommunication
    TLx493D_setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return tlx493d_common_init(sensor, GEN_2_REG_MAP_SIZE, TLx493D_A2B6_regDef, &TLx493D_A2B6_commonFunctions, TLx493D_A2B6_e, TLx493D_I2C_e);
}


bool TLx493D_A2B6_deinit(TLx493D_ts *sensor) {
    return tlx493d_common_deinit(sensor);
}


bool TLx493D_A2B6_readRegisters(TLx493D_ts *sensor) {
    return tlx493d_common_readRegisters(sensor);
}


void TLx493D_A2B6_calculateTemperature(TLx493D_ts *sensor, double *temp) {
    tlx493d_gen_2_calculateTemperature(sensor, temp, TEMP_MSB, TEMP_LSB);
}


bool TLx493D_A2B6_getTemperature(TLx493D_ts *sensor, double *temp) {
    return tlx493d_common_getTemperature(sensor, temp);
}


void TLx493D_A2B6_calculateMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z) {
    tlx493d_gen_2_calculateMagneticField(sensor, x, y, z, BX_MSB, BX_LSB, BY_MSB, BY_LSB, BZ_MSB, BZ_LSB);
}


bool TLx493D_A2B6_getMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z) {
    return tlx493d_common_getMagneticField(sensor, x, y, z);
}


void TLx493D_A2B6_calculateMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_A2B6_calculateMagneticField(sensor, x, y, z);
    TLx493D_A2B6_calculateTemperature(sensor, temp);
}


bool TLx493D_A2B6_getMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp) {
    return tlx493d_common_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


bool TLx493D_A2B6_setMeasurement(TLx493D_ts *sensor, TLx493D_MeasurementType_te val) {
    uint8_t dt = 0;
    uint8_t am = 0;

    switch(val) {
        case TLx493D_BxByBzTemp_e : dt = 0;
                                    am = 0;
                                    break;

        case TLx493D_BxByBz_e : dt = 1;
                                am = 0;
                                break;
        
        case TLx493D_BxBy_e : dt = 1;
                              am = 1;
                              break;
        
        default : warnSelectionNotSupportedForSensorType(sensor, val, "TLx493D_MeasurementType_te");
                  return false;
    }

    return tlx493d_gen_2_setTwoConfigBitfields(sensor, DT, AM, CP, dt, am);
}


//  // This option depends on PR and MODE.
bool TLx493D_A2B6_setTrigger(TLx493D_ts *sensor, TLx493D_TriggerType_te val) {
    uint8_t trig = 0;

    switch(val) {
        case TLx493D_NO_ADC_ON_READ_e : trig = 0;
                                        break;

        case TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e : trig = 1;
                                                      break;
        
        case TLx493D_ADC_ON_READ_AFTER_REG_05_e : trig = 2;
                                                  break;
        
        default : warnSelectionNotSupportedForSensorType(sensor, val, "TLx493D_TriggerType_te");
                  return false;
    }

    return tlx493d_gen_2_setOneConfigBitfield(sensor, TRIG, CP, trig);
}


bool TLx493D_A2B6_setSensitivity(TLx493D_ts *sensor, TLx493D_SensitivityType_te val) {
    uint8_t sens = 0;

    switch(val) {
        case TLx493D_FULL_RANGE_e : sens = 0;
                                    break;

        case TLx493D_SHORT_RANGE_e : sens = 1;
                                     break;
        
        default : warnSelectionNotSupportedForSensorType(sensor, val, "TLx493D_SensitivityType_te");
                  return false;
    }

    return tlx493d_gen_2_setOneConfigBitfield(sensor, X2, CP, sens);
}


bool TLx493D_A2B6_setDefaultConfig(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setDefaultConfig(sensor, CONFIG_REG, MOD1_REG, MOD2_REG, CP, CA, INT);
}


bool TLx493D_A2B6_setIICAddress(TLx493D_ts *sensor, TLx493D_IICAddressType_te address) {
    return tlx493d_gen_2_setIICAddress(sensor, IICADR, FP, address);
}


bool TLx493D_A2B6_enableInterrupt(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setInterrupt(sensor, INT, FP, PRD, 0);
}


bool TLx493D_A2B6_disableInterrupt(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setInterrupt(sensor, INT, FP, PRD, 1);
}


bool TLx493D_A2B6_enableCollisionAvoidance(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setCollisionAvoidance(sensor, CA, FP, PRD, 0);
}


bool TLx493D_A2B6_disableCollisionAvoidance(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setCollisionAvoidance(sensor, CA, FP, PRD, 1);
}


bool TLx493D_A2B6_setPowerMode(TLx493D_ts *sensor, TLx493D_PowerModeType_te mode) {
    return tlx493d_gen_2_setPowerMode(sensor, MODE, FP, mode);
}


bool TLx493D_A2B6_setUpdateRate(TLx493D_ts *sensor, TLx493D_UpdateRateType_te val) {
    uint8_t mod1 = MOD1_REG; // sensor->regDef[fpBF].address;
    uint8_t rate = 0;

    switch(val) {
        case UPDATE_RATE_FAST_e : rate = 0;
                                  break;

        case UPDATE_RATE_SLOW_e : rate = 1;
                                  break;

        default : warnSelectionNotSupportedForSensorType(sensor, val, "TLx493D_UpdateRateType_te");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, PRD, rate);
    tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

    uint8_t buf[4] = { mod1,
                       sensor->regMap[mod1],     // MOD1 register
                       sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
                       sensor->regMap[mod1 + 2]  // MOD2 register
                     };

    return transfer(sensor, buf, sizeof(buf), NULL, 0);
}


bool TLx493D_A2B6_setHighUpdateRate(TLx493D_ts *sensor) {
    return TLx493D_A2B6_setUpdateRate(sensor, 0);
}


bool TLx493D_A2B6_setLowUpdateRate(TLx493D_ts *sensor) {
    return TLx493D_A2B6_setUpdateRate(sensor, 1);
}


bool TLx493D_A2B6_hasValidData(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidData(sensor);
}


bool TLx493D_A2B6_isFunctional(TLx493D_ts *sensor) {
    return tlx493d_gen_2_isFunctional(sensor);
}


bool TLx493D_A2B6_isWakeUpActive(TLx493D_ts *sensor) {
    warnFeatureNotAvailableForSensorType(sensor, "isWakeUpActive");
    return false;
}

bool TLx493D_A2B6_enableWakeUpMode(TLx493D_ts *sensor) {
    return false;
}

bool TLx493D_A2B6_disableWakeUpMode(TLx493D_ts *sensor) {
    return false;
}

bool TLx493D_A2B6_setLowerWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold) {
    return false;
}

bool TLx493D_A2B6_setLowerWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold) {
    return false;
}

bool TLx493D_A2B6_setLowerWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold) {
    return false;
}

bool TLx493D_A2B6_setUpperWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold) {
    return false;
}

bool TLx493D_A2B6_setUpperWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold) {
    return false;
}

bool TLx493D_A2B6_setUpperWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold) {
    return false;
}


bool TLx493D_A2B6_setWakeUpThresholdsAsInteger(TLx493D_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
    return false;
}

// thesholds im mT, to be converted to proper format
bool TLx493D_A2B6_setWakeUpThresholds(TLx493D_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh) {
    return false;
}

bool TLx493D_A2B6_softReset(TLx493D_ts *sensor) {
    return false;
}


// bool TLx493D_A2B6_enableTemperatureMeasurement(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setDisableTemperatureMeasurement(sensor, DT, CP, 0);
// }


// bool TLx493D_A2B6_disableTemperatureMeasurement(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setDisableTemperatureMeasurement(sensor, DT, CP, 1);
// }


// bool TLx493D_A2B6_enableAngularMeasurement(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setAngularMeasurement(sensor, AM, DT, CP, 1, 1);
// }


// bool TLx493D_A2B6_disableAngularMeasurement(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setAngularMeasurement(sensor, AM, DT, CP, 0, 0);
// }


// bool TLx493D_A2B6_setNoTriggerOnReadTriggerOption(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, 0b00);
// }


// bool TLx493D_A2B6_setTriggerOnReadBeforeFirstMSBTriggerOption(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, 0b01);
// }


// bool TLx493D_A2B6_setTriggerOnReadAfterRegister05TriggerOption(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, 0b10);
// }


// bool TLx493D_A2B6_enableShortRangeSensitivity(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setShortRangeSensitivity(sensor, X2, CP, 1);
// }


// bool TLx493D_A2B6_disableShortRangeSensitivity(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setShortRangeSensitivity(sensor, X2, CP, 0);
// }


// bool TLx493D_A2B6_setTC0MagneticTemperatureCompensation(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b00);
// }


// bool TLx493D_A2B6_setTC1MagneticTemperatureCompensation(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b01);
// }


// bool TLx493D_A2B6_setTC2MagneticTemperatureCompensation(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b10);
// }


// bool TLx493D_A2B6_setTC3MagneticTemperatureCompensation(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b11);
// }


bool TLx493D_A2B6_enable1ByteReadMode(TLx493D_ts *sensor) {
    return tlx493d_gen_2_set1ByteReadMode(sensor, PR, FP, PRD, 1);
}


bool TLx493D_A2B6_disable1ByteReadMode(TLx493D_ts *sensor) {
    return tlx493d_gen_2_set1ByteReadMode(sensor, PR, FP, PRD, 0);
}



uint8_t TLx493D_A2B6_calculateFuseParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD);
}


uint8_t TLx493D_A2B6_calculateBusParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_calculateBusParity(sensor, 5);
}


uint8_t TLx493D_A2B6_calculateConfigurationParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_calculateConfigurationParity(sensor, CP);
}


// bool TLx493D_A2B6_hasValidTemperatureData(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_hasValidTemperatureData(sensor);
// }


// bool TLx493D_A2B6_hasValidMagneticFieldData(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_hasValidMagneticFieldData(sensor);
// }


bool TLx493D_A2B6_hasValidTBit(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidTBit(sensor, T);
}


// bool TLx493D_A2B6_hasValidPD0Bit(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_hasValidPD0Bit(sensor, PD0);
// }


// bool TLx493D_A2B6_hasValidPD3Bit(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_hasValidPD3Bit(sensor, PD3);
// }


bool TLx493D_A2B6_hasValidIICadr(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidIICadr(sensor, ID, IICADR);
}


// bool TLx493D_A2B6_hasWakeup(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_hasWakeup(sensor, TYPE);
// }


bool TLx493D_A2B6_hasValidFuseParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidFuseParity(sensor, FF);
}


bool TLx493D_A2B6_hasValidBusParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidBusParity(sensor, P);
}


bool TLx493D_A2B6_hasValidConfigurationParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidConfigurationParity(sensor, CF);
}


void TLx493D_A2B6_setResetValues(TLx493D_ts *sensor) {
    sensor->regMap[0x10] = 0x00; // CONFIG
    sensor->regMap[0x11] = 0x00; // MOD1
    sensor->regMap[0x13] = 0x00; // MOD2
}
