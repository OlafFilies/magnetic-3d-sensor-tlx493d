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

// common to same generation of sensors
#include "tlx493d_gen_2_common_defines.h"
#include "tlx493d_gen_2_common.h"

// sensor specific includes
#include "TLx493D_W2B6_defines.h"
#include "TLx493D_W2B6.h"


/*
  Listing of all register names for this sensor.
  Used to index "TLx493D_W2B6_regDef" defined below, so index values must match !
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
               HWV } TLx493D_W2B6_registerNames_te;


TLx493D_Register_ts TLx493D_W2B6_regDef[] = {
    {BX_MSB,    TLx493D_READ_MODE_e,       0x00, 0xFF, 0, 8},
    {BY_MSB,    TLx493D_READ_MODE_e,       0x01, 0xFF, 0, 8},
    {BZ_MSB,    TLx493D_READ_MODE_e,       0x02, 0xFF, 0, 8},
    {TEMP_MSB,  TLx493D_READ_MODE_e,       0x03, 0xFF, 0, 8},
    {BX_LSB,    TLx493D_READ_MODE_e,       0x04, 0xF0, 4, 4},
    {BY_LSB,    TLx493D_READ_MODE_e,       0x04, 0x0F, 0, 4},
    {TEMP_LSB,  TLx493D_READ_MODE_e,       0x05, 0xC0, 6, 2},
    {ID,        TLx493D_READ_MODE_e,       0x05, 0x30, 4, 2},
    {BZ_LSB,    TLx493D_READ_MODE_e,       0x05, 0x0F, 0, 4},
    {P,         TLx493D_READ_MODE_e,       0x06, 0x80, 7, 1},
    {FF,        TLx493D_READ_MODE_e,       0x06, 0x40, 6, 1},
    {CF,        TLx493D_READ_MODE_e,       0x06, 0x20, 5, 1},
    {T,         TLx493D_READ_MODE_e,       0x06, 0x10, 4, 1},
    {PD3,       TLx493D_READ_MODE_e,       0x06, 0x08, 3, 1},
    {PD0,       TLx493D_READ_MODE_e,       0x06, 0x04, 2, 1},
    {FRM,       TLx493D_READ_MODE_e,       0x06, 0x03, 0, 2},
    {XL,        TLx493D_READ_WRITE_MODE_e, 0x07, 0xFF, 0, 8},
    {XH,        TLx493D_READ_WRITE_MODE_e, 0x08, 0xFF, 0, 8},
    {YL,        TLx493D_READ_WRITE_MODE_e, 0x09, 0xFF, 0, 8},
    {YH,        TLx493D_READ_WRITE_MODE_e, 0x0A, 0xFF, 0, 8},
    {ZL,        TLx493D_READ_WRITE_MODE_e, 0x0B, 0xFF, 0, 8},
    {ZH,        TLx493D_READ_WRITE_MODE_e, 0x0C, 0xFF, 0, 8},
    {WA,        TLx493D_READ_MODE_e,       0x0D, 0x80, 7, 1},
    {WU,        TLx493D_READ_WRITE_MODE_e, 0x0D, 0x40, 6, 1},
    {XH_LSB,    TLx493D_READ_WRITE_MODE_e, 0x0D, 0x38, 3, 3},
    {XL_LSB,    TLx493D_READ_WRITE_MODE_e, 0x0D, 0x07, 0, 3},
    {TST,       TLx493D_READ_WRITE_MODE_e, 0x0E, 0xC0, 6, 2},
    {YH_LSB,    TLx493D_READ_WRITE_MODE_e, 0x0E, 0x38, 3, 3},  
    {YL_LSB,    TLx493D_READ_WRITE_MODE_e, 0x0E, 0x07, 0, 3},
    {PH,        TLx493D_READ_WRITE_MODE_e, 0x0F, 0xC0, 6, 2},
    {ZH_LSB,    TLx493D_READ_WRITE_MODE_e, 0x0F, 0x38, 3, 3},
    {ZL_LSB,    TLx493D_READ_WRITE_MODE_e, 0x0F, 0x07, 0, 3},
    {DT,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x80, 7, 1},
    {AM,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x40, 6, 1},
    {TRIG,      TLx493D_READ_WRITE_MODE_e, 0x10, 0x30, 4, 2},
    {X2,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x08, 3, 1},
    {TL_MAG,    TLx493D_READ_WRITE_MODE_e, 0x10, 0x06, 1, 2},
    {CP,        TLx493D_READ_WRITE_MODE_e, 0x10, 0x01, 0, 1},
    {FP,        TLx493D_READ_WRITE_MODE_e, 0x11, 0x80, 7, 1},
    {IICADR,    TLx493D_READ_WRITE_MODE_e, 0x11, 0x60, 5, 2},
    {PR,        TLx493D_READ_WRITE_MODE_e, 0x11, 0x10, 4, 1},
    {CA,        TLx493D_READ_WRITE_MODE_e, 0x11, 0x08, 3, 1},
    {INT,       TLx493D_READ_WRITE_MODE_e, 0x11, 0x04, 2, 1},
    {MODE,      TLx493D_READ_WRITE_MODE_e, 0x11, 0x03, 0, 2},
    {PRD,       TLx493D_READ_WRITE_MODE_e, 0x13, 0xE0, 5, 3},
    {TYPE,      TLx493D_READ_MODE_e,       0x16, 0x30, 4, 2},
    {HWV,       TLx493D_READ_MODE_e,       0x16, 0x0F, 0, 4}
};


typedef enum { 
               TEMP2_REG  = 0x05,
               DIAG_REG   = 0x06,
               CONFIG_REG = 0x10,
               MOD1_REG   = 0x11,
               MOD2_REG   = 0x13,
               VER_REG    = 0x16 } SpecialRegisters_te;


TLx493D_CommonFunctions_ts TLx493D_W2B6_commonFunctions = {
    .init                           = TLx493D_W2B6_init,
    .deinit                         = TLx493D_W2B6_deinit,


    .readRegisters                  = tlx493d_common_readRegisters,

    .calculateTemperature           = TLx493D_W2B6_calculateTemperature,
    .getTemperature                 = TLx493D_W2B6_getTemperature,

    .calculateMagneticField         = TLx493D_W2B6_calculateMagneticField,
    .getMagneticField               = TLx493D_W2B6_getMagneticField,

    .calculateMagneticFieldAndTemperature = TLx493D_W2B6_calculateMagneticFieldAndTemperature,
    .getMagneticFieldAndTemperature = TLx493D_W2B6_getMagneticFieldAndTemperature,

    // .enableAngularMeasurement      = TLx493D_W2B6_enableAngularMeasurement,
    // .disableAngularMeasurement     = TLx493D_W2B6_disableAngularMeasurement,

    // .enableTemperatureMeasurement  = TLx493D_W2B6_enableTemperatureMeasurement,
    // .disableTemperatureMeasurement = TLx493D_W2B6_disableTemperatureMeasurement,

    .setDefaultConfig               = TLx493D_W2B6_setDefaultConfig,
    .setIICAddress                  = TLx493D_W2B6_setIICAddress,
    .setPowerMode                   = TLx493D_W2B6_setPowerMode,

    .hasValidData                   = tlx493d_gen_2_hasValidData,
    .isFunctional                   = tlx493d_gen_2_isFunctional,

    .enable1ByteReadMode            = TLx493D_W2B6_enable1ByteReadMode,
    .disable1ByteReadMode           = TLx493D_W2B6_disable1ByteReadMode,

    .calculateFuseParity            = TLx493D_W2B6_calculateFuseParity,
    .calculateBusParity             = TLx493D_W2B6_calculateBusParity,
    .calculateConfigurationParity   = TLx493D_W2B6_calculateConfigurationParity,


    .hasValidFuseParity             = TLx493D_W2B6_hasValidFuseParity,
    .hasValidBusParity              = TLx493D_W2B6_hasValidBusParity,
    .hasValidConfigurationParity    = TLx493D_W2B6_hasValidConfigurationParity,

    // .hasValidTemperatureData       = tlx493d_gen_2_hasValidTemperatureData,
    // .hasValidMagneticFieldData     = tlx493d_gen_2_hasValidMagneticFieldData,

    .hasValidTBit                   = TLx493D_W2B6_hasValidTBit,
    .hasValidPD0Bit                 = TLx493D_W2B6_hasValidPD0Bit,
    .hasValidPD3Bit                 = TLx493D_W2B6_hasValidPD3Bit,
    
    .setResetValues                 = TLx493D_W2B6_setResetValues,
};


// TODO: add parameter IICAddress or ad function to set address.
bool TLx493D_W2B6_init(TLx493D_ts *sensor) {
    TLx493D_setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);
    
    return tlx493d_common_init(sensor, GEN_2_REG_MAP_SIZE, TLx493D_W2B6_regDef, &TLx493D_W2B6_commonFunctions, TLx493D_W2B6_e, TLx493D_I2C_e);
}


bool TLx493D_W2B6_deinit(TLx493D_ts *sensor) {
    return tlx493d_common_deinit(sensor);
}


void TLx493D_W2B6_calculateTemperature(TLx493D_ts *sensor, double *temp) {
    tlx493d_gen_2_calculateTemperature(sensor, temp, TEMP_MSB, TEMP_LSB);
}


bool TLx493D_W2B6_getTemperature(TLx493D_ts *sensor, double *temp) {
    return tlx493d_common_getTemperature(sensor, temp);
}


void TLx493D_W2B6_calculateMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z) {
    tlx493d_gen_2_calculateMagneticField(sensor, x, y, z, BX_MSB, BX_LSB, BY_MSB, BY_LSB, BZ_MSB, BZ_LSB);
}


bool TLx493D_W2B6_getMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z) {
    return tlx493d_common_getMagneticField(sensor, x, y, z);
}


void TLx493D_W2B6_calculateMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_W2B6_calculateMagneticField(sensor, x, y, z);
    TLx493D_W2B6_calculateTemperature(sensor, temp);
}


bool TLx493D_W2B6_getMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp) {
    return tlx493d_common_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


//  // This option depends on PR and MODE.
// bool TLx493D_W2B6_setTrigger(TLx493D_ts *sensor, uint8_t trig) {
//     tlx493d_common_setBitfield(sensor, TRIG, trig);
//     tlx493d_common_setBitfield(sensor, CP, TLx493D_W2B6_calculateConfigurationParity(sensor));

//     return tlx493d_common_writeRegister(sensor, TRIG);
// }



bool TLx493D_W2B6_setDefaultConfig(TLx493D_ts *sensor) {
    sensor->regMap[CONFIG_REG] = 0x00; // sensor->functions->getConfigResetValue();
    sensor->regMap[MOD1_REG]   = 0x00;
    sensor->regMap[MOD2_REG]   = 0x00;

// TODO: Differentiate first call after power-up/reset and subsequent calls ! For subsequent calls the regMap values must be considered as they may be different from reset values !
// printRegisters(sensor->regMap, sensor->regMapSize);
//     sensor->functions->readRegisters(sensor);
// printRegisters(sensor->regMap, sensor->regMapSize);

//     // TODO: parity calculation only meaningful if regMap has same values as device !
// 	uint8_t parity = sensor->regMap[0x07];

//     for(uint8_t i = 0x08; i <= 0x0C; ++i) {
//         parity ^= sensor->regMap[i];
//     }

//     parity ^= (sensor->regMap[0x0D] & ~sensor->regDef[WA].mask);
//     parity ^= (sensor->regMap[0x0E] & ~sensor->regDef[TST].mask);
//     parity ^= (sensor->regMap[0x0F] & ~sensor->regDef[PH].mask);
//     parity ^= (sensor->regMap[0x10] & ~sensor->regDef[CP].mask);

//     info("parity : 0x%x\n", parity);
//     info("calculateParity : 0x%x\n", calculateParity(parity));
//     info("getEvenParity : 0x%x\n", getEvenParity(calculateParity(parity)));
//     info("getOddParity : 0x%x\n", getOddParity(calculateParity(parity)));
// 	// parity = getOddParity(calculateParity(parity));
// 	parity = getEvenParity(calculateParity(parity));

    uint8_t parity = 0x00;  // TODO: the reset values are odd, so CP must be set to 0 for odd parity protection !
    // uint8_t parity = 0x01;

    tlx493d_common_setBitfield(sensor, CP, parity);
    tlx493d_common_writeRegister(sensor, CP);

    tlx493d_common_setBitfield(sensor, CA, 0);
    tlx493d_common_setBitfield(sensor, INT, 1);

    if( sensor->functions->enable1ByteReadMode(sensor) ) {
        // if( sensor->functions->disableTemperatureMeasurement(sensor) ) {
            // Read registers in order to retrieve values in reserved register at 0x12 and in MOD2 in order to make sure we are not 
            // accidentally changing a preset values to 0.
            return sensor->functions->readRegisters(sensor);
        // }
    }

    return false;

    // return tlx493d_gen_2_setDefaultConfig(sensor, CONFIG_REG, MOD1_REG, MOD2_REG, CA, INT);
    // return tlx493d_gen_2_setDefaultConfig(sensor, CONFIG_REG, TLx493D_W2B6_CONFIG_RESET_VALUE, MOD1_REG, MOD2_REG, CA, INT);
}


// // TODO: set all options that must be set, eg MODE ?, reset all bits to defaults ?
// bool TLx493D_W2B6_setDefaultConfig(TLx493D_ts *sensor) {
//     sensor->regMap[CONFIG_REG] = 0x00;
//     sensor->regMap[MOD1_REG]   = 0x00;
//     sensor->regMap[MOD2_REG]   = 0x00;;

//     // MOD1 register
//     tlx493d_common_setBitfield(sensor, CA, 0);
//     tlx493d_common_setBitfield(sensor, INT, 1);

//     if( TLx493D_W2B6_enable1ByteReadMode(sensor) ) {
//         // return readRegisters(sensor);

//         if( TLx493D_W2B6_enableTemperatureMeasurement(sensor) ) {
//         //     // Read registers in order to retrieve values in reserved register at 0x12 and in MOD2 in order to make sure we are not 
//         //     // accidentally changing a preset values to 0.
//             if( readRegisters(sensor) ) {
//         //      return TLx493D_W2B6_setLowUpdateRate(sensor);

//                 return true;
//             }
//         }
//     }

//     return false;
// }


// bool TLx493D_W2B6_setDefaultConfig(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setDefaultConfig(sensor, CONFIG_REG, MOD1_REG, MOD2_REG, CA, INT);
// }


bool TLx493D_W2B6_setIICAddress(TLx493D_ts *sensor, TLx493D_IICAddressType_te address) {
    return tlx493d_gen_2_setIICAddress(sensor, IICADR, FP, address);
}


// bool TLx493D_W2B6_setInterrupt(TLx493D_ts *sensor, uint8_t irq) {
//     tlx493d_common_setBitfield(sensor, INT, irq);
//     tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     return tlx493d_common_writeRegister(sensor, INT);
// }


// bool TLx493D_W2B6_enableInterrupt(TLx493D_ts *sensor) {
//     TLx493D_W2B6_setInterrupt(sensor, 0);
// }


// bool TLx493D_W2B6_disableInterrupt(TLx493D_ts *sensor) {
//     TLx493D_W2B6_setInterrupt(sensor, 1);
// }


// bool  TLx493D_W2B6_setInterruptAndCollisionAvoidance(TLx493D_ts *sensor, bool intIsOn, bool caIsOn) {
//     tlx493d_common_setBitfield(sensor, INT, intIsOn ? 0 : 1);
//     tlx493d_common_setBitfield(sensor, CA, caIsOn ? 1 : 0);
 
//     tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));
    
//     return tlx493d_common_writeRegister(sensor, INT); 
// }


//  // CA only in Low Power and MCM mode, not in Fast Mode !
//  // MODE depends on PR and TRIG !
// bool TLx493D_W2B6_setCollisionAvoidance(TLx493D_ts *sensor, uint8_t ca) {
// // tlx493d_common_setBitfield(sensor, sensor->commonBitfields.MODE, 0b11);

//     tlx493d_common_setBitfield(sensor, CA, ca);
//     tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     return tlx493d_common_writeRegister(sensor, CA);
// }


// bool TLx493D_W2B6_enableCollisionAvoidance(TLx493D_ts *sensor) {
//     TLx493D_W2B6_setCollisionAvoidance(sensor, 0);
// }


// bool TLx493D_W2B6_disableCollisionAvoidance(TLx493D_ts *sensor) {
//     TLx493D_W2B6_setCollisionAvoidance(sensor, 1);
// }


bool TLx493D_W2B6_setPowerMode(TLx493D_ts *sensor, uint8_t mode) {
    return tlx493d_gen_2_setPowerMode(sensor, MODE, FP, mode);
}


// bool TLx493D_W2B6_setUpdateRate(TLx493D_ts *sensor, uint8_t ur) {
//     uint8_t mod1 = sensor->commonRegisters.MOD1;

//     // MOD2 register
//     tlx493d_common_setBitfield(sensor, PRD, ur);

//     // MOD1 register : FP includes PRD bit !
//     tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     uint8_t buf[4] = { mod1,
//                        sensor->regMap[mod1],     // MOD1 register
//                        sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
//                        sensor->regMap[mod1 + 2]  // MOD2 register
//                      };

//     return transfer(sensor, buf, sizeof(buf), NULL, 0);
// }


bool TLx493D_W2B6_setHighUpdateRate(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setUpdateRate(sensor, FP, PRD, 0);
}


bool TLx493D_W2B6_setLowUpdateRate(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setUpdateRate(sensor, FP, PRD, 1);
}


bool TLx493D_W2B6_hasValidData(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidData(sensor);
}


bool TLx493D_W2B6_isFunctional(TLx493D_ts *sensor) {
    return tlx493d_gen_2_isFunctional(sensor);
}


uint8_t TLx493D_W2B6_calculateFuseParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD);
	// uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
	// return getEvenParity(parity);
}


uint8_t TLx493D_W2B6_calculateBusParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_calculateBusParity(sensor, 5);
	// uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
	// return getEvenParity(parity);
}


uint8_t TLx493D_W2B6_calculateConfigurationParity(TLx493D_ts *sensor) {
    return tlx493d_common_getOddParity(tlx493d_gen_2_calculateConfigurationParity(sensor, CP));
	// uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
	// return getEvenParity(parity);
}


// uint8_t TLx493D_W2B6_calculateConfigurationParity(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_calculateConfigurationParity(sensor, CP);
// 	// uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
// 	// return getEvenParity(parity);
// }


// bool TLx493D_W2B6_setMagneticTemperatureCompensation(TLx493D_ts *sensor, uint8_t mtc) {
//     tlx493d_common_setBitfield(sensor, TL_MAG, mtc);
//     tlx493d_common_setBitfield(sensor, CP, TLx493D_W2B6_calculateConfigurationParity(sensor));

//     return tlx493d_common_writeRegister(sensor, TL_MAG);
// }


bool TLx493D_W2B6_enableTemperatureMeasurement(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setDisableTemperatureMeasurement(sensor, DT, CP, 0);
}


bool TLx493D_W2B6_disableTemperatureMeasurement(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setDisableTemperatureMeasurement(sensor, DT, CP, 1);
}



// bool TLx493D_W2B6_enableTemperatureMeasurement(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setDisableTemperatureMeasurement(sensor, DT, CP, 0);
// }


// bool TLx493D_W2B6_disableTemperatureMeasurement(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_setDisableTemperatureMeasurement(sensor, DT, CP, 1);
// }


// bool TLx493D_W2B6_setDisableTemperatureMeasurements(TLx493D_ts *sensor, uint8_t dt) {
// //     uint8_t config = sensor->commonRegisters.CONFIG;

//     // CONFIG register
//     tlx493d_common_setBitfield(sensor, DT, dt);
//     tlx493d_common_setBitfield(sensor, CP, TLx493D_W2B6_calculateConfigurationParity(sensor));

//     tlx493d_common_writeRegister(sensor, DT);
// }


// bool TLx493D_W2B6_enableTemperatureMeasurements(TLx493D_ts *sensor) {
//     return TLx493D_W2B6_setDisableTemperatureMeasurements(sensor, 0);
// }


// bool TLx493D_W2B6_disableTemperatureMeasurements(TLx493D_ts *sensor) {
//     return TLx493D_W2B6_setDisableTemperatureMeasurements(sensor, 1);
// }


//  // This option depends on value of DT page 7.
// bool TLx493D_W2B6_setAngularMesurements(TLx493D_ts *sensor, uint8_t am) {
//     tlx493d_common_setBitfield(sensor, AM, am);
//     tlx493d_common_setBitfield(sensor, CP, TLx493D_W2B6_calculateConfigurationParity(sensor));

//     return tlx493d_common_writeRegister(sensor, AM);
// }


bool TLx493D_W2B6_enableAngularMeasurement(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setAngularMeasurement(sensor, AM, DT, CP, 1, 1);
}


bool TLx493D_W2B6_disableAngularMeasurement(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setAngularMeasurement(sensor, AM, DT, CP, 0, 0);
}


bool TLx493D_W2B6_setNoTriggerOnReadTriggerOption(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, 0b00);
}


bool TLx493D_W2B6_setTriggerOnReadBeforeFirstMSBTriggerOption(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, 0b01);
}


bool TLx493D_W2B6_setTriggerOnReadAfterRegister05TriggerOption(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, 0b10);
}


// bool TLx493D_W2B6_setShortRangeSensitivity(TLx493D_ts *sensor, uint8_t srs) {
//     tlx493d_common_setBitfield(sensor, X2, srs);
//     tlx493d_common_setBitfield(sensor, CP, TLx493D_W2B6_calculateConfigurationParity(sensor));

//     return tlx493d_common_writeRegister(sensor, X2);
// }


bool TLx493D_W2B6_enableShortRangeSensitivity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setShortRangeSensitivity(sensor, X2, CP, 1);
}


bool TLx493D_W2B6_disableShortRangeSensitivity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setShortRangeSensitivity(sensor, X2, CP, 0);
}


bool TLx493D_W2B6_setTC0MagneticTemperatureCompensation(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b00);
}


bool TLx493D_W2B6_setTC1MagneticTemperatureCompensation(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b01);
}


bool TLx493D_W2B6_setTC2MagneticTemperatureCompensation(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b10);
}


bool TLx493D_W2B6_setTC3MagneticTemperatureCompensation(TLx493D_ts *sensor) {
    return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b11);
}



// bool TLx493D_W2B6_set1ByteReadMode(TLx493D_ts *sensor, uint8_t pr) {
//     // uint8_t mod1 = sensor->commonRegisters.MOD1;

//     tlx493d_common_setBitfield(sensor, PR, pr);
//     tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     tlx493d_common_writeRegister(sensor, PR);
// }


bool TLx493D_W2B6_enable1ByteReadMode(TLx493D_ts *sensor) {
    return tlx493d_gen_2_set1ByteReadMode(sensor, PR, FP, PRD, 1);

// return TLx493D_W2B6_set1ByteReadMode(sensor, 1);
}


bool TLx493D_W2B6_disable1ByteReadMode(TLx493D_ts *sensor) {
    return tlx493d_gen_2_set1ByteReadMode(sensor, PR, FP, PRD, 0);
}


// bool TLx493D_W2B6_enable1ByteReadMode(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_set1ByteReadMode(sensor, PR, FP, PRD, 1);

// // return TLx493D_W2B6_set1ByteReadMode(sensor, 1);
// }


// bool TLx493D_W2B6_disable1ByteReadMode(TLx493D_ts *sensor) {
//     return tlx493d_gen_2_set1ByteReadMode(sensor, PR, FP, PRD, 0);
// }


bool TLx493D_W2B6_hasValidFuseParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidFuseParity(sensor, FF);
}


bool TLx493D_W2B6_hasValidBusParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidBusParity(sensor, P);
}


bool TLx493D_W2B6_hasValidConfigurationParity(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidConfigurationParity(sensor, CF);
}


bool TLx493D_W2B6_hasValidIICadr(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidIICadr(sensor, ID, IICADR);
}


bool TLx493D_W2B6_hasWakeup(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasWakeup(sensor, TYPE);
}


bool TLx493D_W2B6_hasValidTemperatureData(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidTemperatureData(sensor);
}


bool TLx493D_W2B6_hasValidMagneticFieldData(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidMagneticFieldData(sensor);
}


bool TLx493D_W2B6_hasValidTBit(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidTBit(sensor, T);
}


bool TLx493D_W2B6_hasValidPD0Bit(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidPD0Bit(sensor, PD0);
}


bool TLx493D_W2B6_hasValidPD3Bit(TLx493D_ts *sensor) {
    return tlx493d_gen_2_hasValidPD3Bit(sensor, PD3);
}


void TLx493D_W2B6_setResetValues(TLx493D_ts *sensor) {
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
