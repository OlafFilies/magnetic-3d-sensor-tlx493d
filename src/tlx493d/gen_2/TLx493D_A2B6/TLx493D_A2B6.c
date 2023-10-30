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


// #include "Logger.h"


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


/***
 * 
*/
Register_ts TLx493D_A2B6_regDef[] = {
    { BX_MSB,   READ_MODE_e,       0x00, 0xFF, 0, 8 },
    { BY_MSB,   READ_MODE_e,       0x01, 0xFF, 0, 8 },
    { BZ_MSB,   READ_MODE_e,       0x02, 0xFF, 0, 8 }, 
    { TEMP_MSB, READ_MODE_e,       0x03, 0xFF, 0, 8 },
    { BX_LSB,   READ_MODE_e,       0x04, 0xF0, 4, 4 },
    { BY_LSB,   READ_MODE_e,       0x04, 0x0F, 0, 4 },
    { TEMP_LSB, READ_MODE_e,       0x05, 0xC0, 6, 2 },
    { ID,       READ_MODE_e,       0x05, 0x30, 4, 2 },
    { BZ_LSB,   READ_MODE_e,       0x05, 0x0F, 0, 4 },
    { P,        READ_MODE_e,       0x06, 0x80, 7, 1 },
    { FF,       READ_MODE_e,       0x06, 0x40, 6, 1 },
    { CF,       READ_MODE_e,       0x06, 0x20, 5, 1 },
    { T,        READ_MODE_e,       0x06, 0x10, 4, 1 },
    { PD3,      READ_MODE_e,       0x06, 0x08, 3, 1 },
    { PD0,      READ_MODE_e,       0x06, 0x04, 2, 1 },
    { FRM,      READ_MODE_e,       0x06, 0x03, 0, 2 },
    { DT,       READ_WRITE_MODE_e, 0x10, 0x80, 7, 1 },
    { AM,       READ_WRITE_MODE_e, 0x10, 0x40, 6, 1 },
    { TRIG,     READ_WRITE_MODE_e, 0x10, 0x30, 4, 2 },
    { X2,       READ_WRITE_MODE_e, 0x10, 0x08, 3, 1 },
    { TL_MAG,   READ_WRITE_MODE_e, 0x10, 0x06, 1, 2 },
    { CP,       READ_WRITE_MODE_e, 0x10, 0x01, 0, 1 },
    { FP,       READ_WRITE_MODE_e, 0x11, 0x80, 7, 1 },
    { IICADR,   READ_WRITE_MODE_e, 0x11, 0x60, 5, 2 },
    { PR,       READ_WRITE_MODE_e, 0x11, 0x10, 4, 1 },
    { CA,       READ_WRITE_MODE_e, 0x11, 0x08, 3, 1 },
    { INT,      READ_WRITE_MODE_e, 0x11, 0x04, 2, 1 },
    { MODE,     READ_WRITE_MODE_e, 0x11, 0x03, 0, 2 },
    // Does not match register overview in manual, but fits and default value
    // textual description of register PRD ! Confirmed by Severin.
    // { PRD,      READ_WRITE_MODE_e, 0x13, 0xE0, 5, 3 },
    { PRD,      READ_WRITE_MODE_e, 0x13, 0x80, 7, 1 },
    { TYPE,     READ_MODE_e,       0x16, 0x30, 4, 2 },
    { HWV,      READ_MODE_e,       0x16, 0x0F, 0, 4 }
};


/***
 * 
*/
typedef enum {
               LOW_POWER_MODE         = 0x00,
               MASTER_CONTROLLED_MODE = 0x01,
               RESERVED_MODE          = 0x10,
               FAST_MODE              = 0x11 } TLx493D_A2B6_modes_te;


/***
 * 
*/
typedef enum { 
               TEMP2_REG  = 0x05,
               DIAG_REG   = 0x06,
               CONFIG_REG = 0x10,
               MOD1_REG   = 0x11,
               MOD2_REG   = 0x13,
               VER_REG    = 0x16 } SpecialRegisters_te;


/***
 * 
*/
CommonFunctions_ts TLx493D_A2B6_commonFunctions = {
    .init                   = TLx493D_A2B6_init,
    .deinit                 = TLx493D_A2B6_deinit,

    .readRegisters          = tlx493d_readRegisters,
    .setDefaultConfig       = TLx493D_A2B6_setDefaultConfig,
    .setPowerMode           = TLx493D_A2B6_setPowerMode,
    .setIICAddress          = TLx493D_A2B6_setIICAddress,

    .calculateTemperature   = TLx493D_A2B6_calculateTemperature,
    // .getTemperature         = TLx493D_A2B6_getTemperature,

    .calculateMagneticField = TLx493D_A2B6_calculateMagneticField,
    // .getMagneticField       = TLx493D_A2B6_getMagneticField,

    .calculateMagneticFieldAndTemperature = TLx493D_A2B6_calculateMagneticFieldAndTemperature,
    // .getMagneticFieldAndTemperature       = TLx493D_A2B6_getMagneticFieldAndTemperature,

    .enableAngularMeasurement  = TLx493D_A2B6_enableAngularMeasurement,
    .disableAngularMeasurement = TLx493D_A2B6_disableAngularMeasurement,

    .enableTemperatureMeasurement  = TLx493D_A2B6_enableTemperatureMeasurement,
    .disableTemperatureMeasurement = TLx493D_A2B6_disableTemperatureMeasurement,

    .enable1ByteReadMode           = TLx493D_A2B6_enable1ByteReadMode,
    .disable1ByteReadMode          = TLx493D_A2B6_disable1ByteReadMode,

    .calculateFuseParity   = TLx493D_A2B6_calculateFuseParity,
    .calculateBusParity    = TLx493D_A2B6_calculateBusParity,
    .calculateConfigurationParity = TLx493D_A2B6_calculateConfigurationParity,

    .hasValidFuseParity    = TLx493D_A2B6_hasValidFuseParity,
    .hasValidBusParity     = TLx493D_A2B6_hasValidBusParity,
    .hasValidConfigurationParity = TLx493D_A2B6_hasValidConfigurationParity,

    .hasValidData              = tlx493d_gen_2_hasValidData,
    .hasValidTemperatureData   = tlx493d_gen_2_hasValidTemperatureData,
    .hasValidMagneticFieldData = tlx493d_gen_2_hasValidMagneticFieldData,

    .hasValidTBit   = TLx493D_A2B6_hasValidTBit,
    .hasValidPD0Bit = TLx493D_A2B6_hasValidPD0Bit,
    .hasValidPD3Bit = TLx493D_A2B6_hasValidPD3Bit,

    .isFunctional          = tlx493d_gen_2_isFunctional,
};


/***
 */
bool TLx493D_A2B6_init(Sensor_ts *sensor) {
    // // regMap must be sensor specific, not sensor type specific, therefore malloc.
    // sensor->regMap            = (uint8_t*) malloc(sizeof(uint8_t) * GEN_2_REG_MAP_SIZE);
    // sensor->regDef            = TLx493D_A2B6_regDef;
    // // sensor->commonBitfields   = (CommonBitfields_ts) { .CP = CP, .FP = FP, .ID = ID, .P = P, .FF = FF, .CF = CF, .T = T, .PD3 = PD3, .PD0 = PD0, .FRM = FRM, .PRD = PRD, .TYPE = TYPE, .HWV = HWV,
    // //                                                    .DT = DT, .AM = AM, .TRIG = TRIG, .X2 = X2, .TL_MAG = TL_MAG, .IICADR = IICADR, .PR = PR, .CA = CA, .INT = INT, .MODE = MODE,
    // //                                                    .BX_MSB = BX_MSB, .BY_MSB = BY_MSB, .BZ_MSB = BZ_MSB, .TEMP_MSB = TEMP_MSB,
    // //                                                    .BX_LSB = BX_LSB, .BY_LSB = BY_LSB, .TEMP_LSB = TEMP_LSB, .BZ_LSB = BZ_LSB, .TEMP2 = TEMP2_REG,
    // //                                                  };
    // // sensor->commonRegisters   = (CommonRegisters_ts) { .DIAG = DIAG_REG, .CONFIG = CONFIG_REG, .MOD1 = MOD1_REG, .MOD2 = MOD2_REG, .VER = VER_REG };
    // sensor->functions         = &TLx493D_A2B6_commonFunctions;
    // sensor->regMapSize        = GEN_2_REG_MAP_SIZE;
    // sensor->sensorType        = TLx493D_A2B6_e;
    // sensor->comIFType         = I2C_e;
    // sensor->comLibIF          = NULL;
    // sensor->comLibObj.i2c_obj = NULL;

    // memset(sensor->regMap, 0, sensor->regMapSize);
    
    // TODO: use in initComLibIF
    setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);
    
    return tlx493d_init(sensor, GEN_2_REG_MAP_SIZE, TLx493D_A2B6_regDef, &TLx493D_A2B6_commonFunctions, TLx493D_A2B6_e, I2C_e);
}


/***
 * 
*/
bool TLx493D_A2B6_deinit(Sensor_ts *sensor) {
    return tlx493d_deinit(sensor);


    // free(sensor->regMap);
    // free(sensor->comLibObj.i2c_obj);

    // sensor->regMap            = NULL;
    // sensor->comLibObj.i2c_obj = NULL;
    // return true;
}


bool TLx493D_A2B6_setPowerMode(Sensor_ts *sensor, uint8_t mode) {
    return tlx493d_gen_2_setPowerMode(sensor, MODE, FP, mode);
}


bool TLx493D_A2B6_setIICAddress(Sensor_ts *sensor, StandardIICAddresses_te address) {
    return tlx493d_gen_2_setIICAddress(sensor, IICADR, FP, address);
}


/***
 * 
*/
void TLx493D_A2B6_calculateTemperature(Sensor_ts *sensor, double *temp) {
    tlx493d_gen_2_calculateTemperature(sensor, temp, TEMP_MSB, TEMP_LSB);

    // int16_t value = 0;

    // concatBytes(sensor, TEMP_MSB, TEMP_LSB, &value);

    // value <<= 2; // least significant 2 bits are implicit, therefore shift by 2 !
    // *temp = (((double) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF;
}


/***
 * 
*/
bool TLx493D_A2B6_getTemperature(Sensor_ts *sensor, double *temp) {
    return tlx493d_getTemperature(sensor, temp);

    // if( tlx493d_gen_2_readRegisters(sensor) ) {
    //     TLx493D_A2B6_calculateTemperature(sensor, temp);
    //     return true;
    // }

    // return false;
}


/***
 * 
*/
void TLx493D_A2B6_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z) {
    tlx493d_gen_2_calculateMagneticField(sensor, x, y, z, BX_MSB, BX_LSB, BY_MSB, BY_LSB, BZ_MSB, BZ_LSB);

    // int16_t valueX = 0, valueY = 0, valueZ = 0;

    // concatBytes(sensor, BX_MSB, BX_LSB, &valueX);
    // concatBytes(sensor, BY_MSB, BY_LSB, &valueY);
    // concatBytes(sensor, BZ_MSB, BZ_LSB, &valueZ);

    // *x = ((double) valueX) * GEN_2_MAG_FIELD_MULT;
    // *y = ((double) valueY) * GEN_2_MAG_FIELD_MULT;
    // *z = ((double) valueZ) * GEN_2_MAG_FIELD_MULT;
}


/***
 * 
*/
bool TLx493D_A2B6_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z) {
    return tlx493d_getMagneticField(sensor, x, y, z);

    // if( tlx493d_gen_2_readRegisters(sensor) ) {
    //     TLx493D_A2B6_calculateMagneticField(sensor, x, y, z);
    //     return true;
    // }

    // return false;
}


/***
 * 
*/
void TLx493D_A2B6_calculateMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_A2B6_calculateMagneticField(sensor, x, y, z);
    TLx493D_A2B6_calculateTemperature(sensor, temp);
}


/***
 * 
*/
bool TLx493D_A2B6_getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
    return tlx493d_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


/***
 * 
*/
uint8_t TLx493D_A2B6_calculateFuseParity(Sensor_ts *sensor) {
    return tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD);
	// uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
	// return getEvenParity(parity);
}


/***
 * 
*/
uint8_t TLx493D_A2B6_calculateBusParity(Sensor_ts *sensor) {
    return tlx493d_gen_2_calculateBusParity(sensor, 5);
	// uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
	// return getEvenParity(parity);
}


/***
 * 
*/
uint8_t TLx493D_A2B6_calculateConfigurationParity(Sensor_ts *sensor) {
    return tlx493d_gen_2_calculateConfigurationParity(sensor, CP);
	// uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
	// return getEvenParity(parity);
}


bool TLx493D_A2B6_enableTemperatureMeasurement(Sensor_ts *sensor) {
    return tlx493d_gen_2_setDisableTemperatureMeasurement(sensor, DT, CP, 0);
}


bool TLx493D_A2B6_disableTemperatureMeasurement(Sensor_ts *sensor) {
    return tlx493d_gen_2_setDisableTemperatureMeasurement(sensor, DT, CP, 1);
}


// /***
//  * 
// */
// bool TLx493D_A2B6_setDisableTemperatureMeasurements(Sensor_ts *sensor, uint8_t dt) {
// //     uint8_t config = sensor->commonRegisters.CONFIG;

//     // CONFIG register
//     tlx493d_setBitfield(sensor, DT, dt);
//     tlx493d_setBitfield(sensor, CP, TLx493D_A2B6_calculateConfigurationParity(sensor));

//     tlx493d_writeRegister(sensor, DT);
// }


// bool TLx493D_A2B6_enableTemperatureMeasurements(Sensor_ts *sensor) {
//     return TLx493D_A2B6_setDisableTemperatureMeasurements(sensor, 0);
// }


// bool TLx493D_A2B6_disableTemperatureMeasurements(Sensor_ts *sensor) {
//     return TLx493D_A2B6_setDisableTemperatureMeasurements(sensor, 1);
// }


// /***
//  * This option depends on value of DT page 7.
// */
// bool TLx493D_A2B6_setAngularMesurements(Sensor_ts *sensor, uint8_t am) {
//     tlx493d_setBitfield(sensor, AM, am);
//     tlx493d_setBitfield(sensor, CP, TLx493D_A2B6_calculateConfigurationParity(sensor));

//     return tlx493d_writeRegister(sensor, AM);
// }


bool TLx493D_A2B6_enableAngularMeasurement(Sensor_ts *sensor) {
    return tlx493d_gen_2_setAngularMeasurement(sensor, AM, DT, CP, 1, 1);
}


bool TLx493D_A2B6_disableAngularMeasurement(Sensor_ts *sensor) {
    return tlx493d_gen_2_setAngularMeasurement(sensor, AM, DT, CP, 0, 0);
}


// /***
//  * This option depends on PR and MODE.
// */
// bool TLx493D_A2B6_setTriggerOptions(Sensor_ts *sensor, uint8_t trig) {
//     tlx493d_setBitfield(sensor, TRIG, trig);
//     tlx493d_setBitfield(sensor, CP, TLx493D_A2B6_calculateConfigurationParity(sensor));

//     return tlx493d_writeRegister(sensor, TRIG);
// }


bool TLx493D_A2B6_setNoTriggerOnReadTriggerOption(Sensor_ts *sensor) {
    return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, 0b00);
}


bool TLx493D_A2B6_setTriggerOnReadBeforeFirstMSBTriggerOption(Sensor_ts *sensor) {
    return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, 0b01);
}


bool TLx493D_A2B6_setTriggerOnReadAfterRegister05TriggerOption(Sensor_ts *sensor) {
    return tlx493d_gen_2_setTrigger(sensor, TRIG, CP, 0b10);
}


// /***
//  * 
// */
// bool TLx493D_A2B6_setShortRangeSensitivity(Sensor_ts *sensor, uint8_t srs) {
//     tlx493d_setBitfield(sensor, X2, srs);
//     tlx493d_setBitfield(sensor, CP, TLx493D_A2B6_calculateConfigurationParity(sensor));

//     return tlx493d_writeRegister(sensor, X2);
// }


bool TLx493D_A2B6_enableShortRangeSensitivity(Sensor_ts *sensor) {
    return tlx493d_gen_2_setShortRangeSensitivity(sensor, X2, CP, 1);
}


bool TLx493D_A2B6_disableShortRangeSensitivity(Sensor_ts *sensor) {
    return tlx493d_gen_2_setShortRangeSensitivity(sensor, X2, CP, 0);
}


// /***
//  * 
// */
// bool TLx493D_A2B6_setMagneticTemperatureCompensation(Sensor_ts *sensor, uint8_t mtc) {
//     tlx493d_setBitfield(sensor, TL_MAG, mtc);
//     tlx493d_setBitfield(sensor, CP, TLx493D_A2B6_calculateConfigurationParity(sensor));

//     return tlx493d_writeRegister(sensor, TL_MAG);
// }


bool TLx493D_A2B6_setTC0MagneticTemperatureCompensation(Sensor_ts *sensor) {
    return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b00);
}


bool TLx493D_A2B6_setTC1MagneticTemperatureCompensation(Sensor_ts *sensor) {
    return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b01);
}


bool TLx493D_A2B6_setTC2MagneticTemperatureCompensation(Sensor_ts *sensor) {
    return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b10);
}


bool TLx493D_A2B6_setTC3MagneticTemperatureCompensation(Sensor_ts *sensor) {
    return tlx493d_gen_2_setMagneticTemperatureCompensation(sensor, TL_MAG, CP, 0b11);
}


// /***
//  * 
// */
// bool TLx493D_A2B6_set1ByteReadMode(Sensor_ts *sensor, uint8_t pr) {
//     // uint8_t mod1 = sensor->commonRegisters.MOD1;

//     tlx493d_setBitfield(sensor, PR, pr);
//     tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     tlx493d_writeRegister(sensor, PR);
// }


bool TLx493D_A2B6_enable1ByteReadMode(Sensor_ts *sensor) {
    return tlx493d_gen_2_set1ByteReadMode(sensor, PR, FP, PRD, 1);

// return TLx493D_A2B6_set1ByteReadMode(sensor, 1);
}


bool TLx493D_A2B6_disable1ByteReadMode(Sensor_ts *sensor) {
    return tlx493d_gen_2_set1ByteReadMode(sensor, PR, FP, PRD, 0);
}


// /***
//  * 
// */
// bool  TLx493D_A2B6_setInterruptAndCollisionAvoidance(Sensor_ts *sensor, bool intIsOn, bool caIsOn) {
//     tlx493d_setBitfield(sensor, INT, intIsOn ? 0 : 1);
//     tlx493d_setBitfield(sensor, CA, caIsOn ? 1 : 0);
 
//     tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));
    
//     return tlx493d_writeRegister(sensor, INT); 
// }


// /***
//  * CA only in Low Power and MCM mode, not in Fast Mode !
//  * MODE depends on PR and TRIG !
// */
// bool TLx493D_A2B6_setCollisionAvoidance(Sensor_ts *sensor, uint8_t ca) {
// // tlx493d_setBitfield(sensor, sensor->commonBitfields.MODE, 0b11);

//     tlx493d_setBitfield(sensor, CA, ca);
//     tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     return tlx493d_writeRegister(sensor, CA);
// }


// bool TLx493D_A2B6_enableCollisionAvoidance(Sensor_ts *sensor) {
//     TLx493D_A2B6_setCollisionAvoidance(sensor, 0);
// }


// bool TLx493D_A2B6_disableCollisionAvoidance(Sensor_ts *sensor) {
//     TLx493D_A2B6_setCollisionAvoidance(sensor, 1);
// }


// /***
//  * 
// */
// bool TLx493D_A2B6_setInterrupt(Sensor_ts *sensor, uint8_t irq) {
//     tlx493d_setBitfield(sensor, INT, irq);
//     tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     return tlx493d_writeRegister(sensor, INT);
// }


// bool TLx493D_A2B6_enableInterrupt(Sensor_ts *sensor) {
//     TLx493D_A2B6_setInterrupt(sensor, 0);
// }


// bool TLx493D_A2B6_disableInterrupt(Sensor_ts *sensor) {
//     TLx493D_A2B6_setInterrupt(sensor, 1);
// }


// /***
//  * 
// */
// bool TLx493D_A2B6_setUpdateRate(Sensor_ts *sensor, uint8_t ur) {
//     uint8_t mod1 = sensor->commonRegisters.MOD1;

//     // MOD2 register
//     tlx493d_setBitfield(sensor, PRD, ur);

//     // MOD1 register : FP includes PRD bit !
//     tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     uint8_t buf[4] = { mod1,
//                        sensor->regMap[mod1],     // MOD1 register
//                        sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
//                        sensor->regMap[mod1 + 2]  // MOD2 register
//                      };

//     return transfer(sensor, buf, sizeof(buf), NULL, 0);
// }


bool TLx493D_A2B6_setHighUpdateRate(Sensor_ts *sensor) {
    return tlx493d_gen_2_setUpdateRate(sensor, FP, PRD, 0);
}


bool TLx493D_A2B6_setLowUpdateRate(Sensor_ts *sensor) {
    return tlx493d_gen_2_setUpdateRate(sensor, FP, PRD, 1);
}


bool TLx493D_A2B6_setDefaultConfig(Sensor_ts *sensor) {
    return tlx493d_gen_2_setDefaultConfig(sensor, CONFIG_REG, MOD1_REG, MOD2_REG, CP, CA, INT);
    // return tlx493d_gen_2_setDefaultConfig(sensor, CONFIG_REG, TLx493D_A2B6_CONFIG_RESET_VALUE, MOD1_REG, MOD2_REG, CA, INT);
}


/***
 * TODO: set all options that must be set, eg MODE ?, reset all bits to defaults ?
*/
// bool TLx493D_A2B6_setDefaultConfig(Sensor_ts *sensor) {
//     sensor->regMap[CONFIG_REG] = 0x00;
//     sensor->regMap[MOD1_REG]   = 0x00;
//     sensor->regMap[MOD2_REG]   = 0x00;;

//     // MOD1 register
//     tlx493d_setBitfield(sensor, CA, 0);
//     tlx493d_setBitfield(sensor, INT, 1);

//     if( TLx493D_A2B6_enable1ByteReadMode(sensor) ) {
//         // return readRegisters(sensor);

//         if( TLx493D_A2B6_enableTemperatureMeasurement(sensor) ) {
//         //     // Read registers in order to retrieve values in reserved register at 0x12 and in MOD2 in order to make sure we are not 
//         //     // accidentally changing a preset values to 0.
//             if( readRegisters(sensor) ) {
//         //      return TLx493D_A2B6_setLowUpdateRate(sensor);

//                 return true;
//             }
//         }
//     }

//     return false;
// }


bool TLx493D_A2B6_hasValidFuseParity(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidFuseParity(sensor, FF);
}


bool TLx493D_A2B6_hasValidBusParity(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidBusParity(sensor, P);
}


bool TLx493D_A2B6_hasValidConfigurationParity(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidConfigurationParity(sensor, CF);
}


bool TLx493D_A2B6_hasValidIICadr(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidIICadr(sensor, ID, IICADR);
}


bool TLx493D_A2B6_hasWakeup(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasWakeup(sensor, TYPE);
}


bool TLx493D_A2B6_hasValidData(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidData(sensor);
}


bool TLx493D_A2B6_hasValidTemperatureData(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidTemperatureData(sensor);
}


bool TLx493D_A2B6_hasValidMagneticFieldData(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidMagneticFieldData(sensor);
}


bool TLx493D_A2B6_hasValidTBit(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidTBit(sensor, T);
}


bool TLx493D_A2B6_hasValidPD0Bit(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidPD0Bit(sensor, PD0);
}


bool TLx493D_A2B6_hasValidPD3Bit(Sensor_ts *sensor) {
    return tlx493d_gen_2_hasValidPD3Bit(sensor, PD3);
}


bool TLx493D_A2B6_isFunctional(Sensor_ts *sensor) {
    return tlx493d_gen_2_isFunctional(sensor);
}
