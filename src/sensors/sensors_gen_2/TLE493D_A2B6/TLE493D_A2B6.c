// std includes
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

// sensor specicifc includes
#include "TLE493D_A2B6_defines.h"
#include "TLE493D_A2B6.h"


// framework functions
// TODO: replace by function pointers in comLibIF structure
extern void setI2CParameters(Sensor_ts *sensor, uint8_t addr);


/***
 * Listing of all register names for this sensor.
 * Used to index "TLE493D_A2B6_regDef" defined below, so index values must match !
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
               HWV } TLE493D_A2B6_registerNames_te;


/***
 * 
*/
Register_ts TLE493D_A2B6_regDef[] = {
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
               FAST_MODE              = 0x11 } TLE493D_A2B6_modes_te;


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
CommonFunctions_ts TLE493D_A2B6_commonFunctions = {
    .init                  = TLE493D_A2B6_init,
    .deinit                = TLE493D_A2B6_deinit,

    // TODO: remove everywhere   .calculateTemperature  = TLE493D_A2B6_calculateTemperature,
    .getTemperature        = TLE493D_A2B6_getTemperature,

    // TODO: remove everywhere   .calculateFieldValues  = TLE493D_A2B6_calculateFieldValues,
    .getFieldValues        = TLE493D_A2B6_getFieldValues,

    .getSensorValues       = TLE493D_A2B6_getSensorValues,

    .hasValidData          = gen_2_hasValidData,
    .isFunctional          = gen_2_isFunctional,

    .setDefaultConfig      = TLE493D_A2B6_setDefaultConfig,
    .readRegisters         = gen_2_readRegisters,

    .setPowerMode          = gen_2_setPowerMode,
    .setIICAddress         = gen_2_setIICAddress,

    .transfer              = TLE493D_A2B6_transferRegisterMap,
};


/***
 * TODO: add parameter IICAddress or ad function to set address.
*/
bool TLE493D_A2B6_init(Sensor_ts *sensor) {
    // regMap must be sensor specific, not sensor type specific, therefore malloc.
    sensor->regMap            = (uint8_t*) malloc(sizeof(uint8_t) * GEN_2_REG_MAP_SIZE);
    sensor->regDef            = TLE493D_A2B6_regDef;
    sensor->commonBitfields   = (CommonBitfields_ts) { .CP = CP, .FP = FP, .ID = ID, .P = P, .FF = FF, .CF = CF, .T = T, .PD3 = PD3, .PD0 = PD0, .FRM = FRM, .PRD = PRD, .TYPE = TYPE, .HWV = HWV,
                                                       .DT = DT, .AM = AM, .TRIG = TRIG, .X2 = X2, .TL_MAG = TL_MAG, .IICADR = IICADR, .PR = PR, .CA = CA, .INT = INT, .MODE = MODE,
                                                       .BX_MSB = BX_MSB, .BY_MSB = BY_MSB, .BZ_MSB = BZ_MSB, .TEMP_MSB = TEMP_MSB,
                                                       .BX_LSB = BX_LSB, .BY_LSB = BY_LSB, .TEMP_LSB = TEMP_LSB, .BZ_LSB = BZ_LSB, .TEMP2 = TEMP2_REG,
                                                     };
    sensor->commonRegisters   = (CommonRegisters_ts) { .DIAG = DIAG_REG, .CONFIG = CONFIG_REG, .MOD1 = MOD1_REG, .MOD2 = MOD2_REG, .VER = VER_REG };
    sensor->functions         = &TLE493D_A2B6_commonFunctions;
    sensor->regMapSize        = GEN_2_REG_MAP_SIZE;
    sensor->sensorType        = TLE493D_A2B6_e;
    sensor->comIFType         = I2C_e;
    sensor->comLibIF          = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    memset(sensor->regMap, 0, sensor->regMapSize);
    
    setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);
    
    return true;
}


/***
 * 
*/
bool TLE493D_A2B6_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);

    sensor->regMap            = NULL;
    sensor->comLibObj.i2c_obj = NULL;
    return true;
}


/***
 * 
*/
void TLE493D_A2B6_calculateTemperature(Sensor_ts *sensor, float *temp) {
    int16_t value = 0;

    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.TEMP_MSB], &sensor->regDef[sensor->commonBitfields.TEMP_LSB], &value);

    value <<= 2; // least significant 2 bits are implicit, therefore shift by 2 !
    *temp = (((float) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF;
}


/***
 * 
*/
bool TLE493D_A2B6_getTemperature(Sensor_ts *sensor, float *temp) {
    if( gen_2_readRegisters(sensor) ) {
        TLE493D_A2B6_calculateTemperature(sensor, temp);
        return true;
    }

    return false;
}


/***
 * 
*/
void TLE493D_A2B6_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BX_MSB], &sensor->regDef[sensor->commonBitfields.BX_LSB], &valueX);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BY_MSB], &sensor->regDef[sensor->commonBitfields.BY_LSB], &valueY);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BZ_MSB], &sensor->regDef[sensor->commonBitfields.BZ_LSB], &valueZ);

    *x = ((float) valueX) * GEN_2_MAG_FIELD_MULT;
    *y = ((float) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((float) valueZ) * GEN_2_MAG_FIELD_MULT;
}


/***
 * 
*/
bool TLE493D_A2B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    if( gen_2_readRegisters(sensor) ) {
        TLE493D_A2B6_calculateFieldValues(sensor, x, y, z);
        return true;
    }

    return false;
}


/***
 * 
*/
void TLE493D_A2B6_calculateSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp) {
    TLE493D_A2B6_calculateFieldValues(sensor, x, y, z);
    TLE493D_A2B6_calculateTemperature(sensor, temp);
}


/***
 * 
*/
bool TLE493D_A2B6_getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp) {
    if( gen_2_readRegisters(sensor) ) {
        TLE493D_A2B6_calculateSensorValues(sensor, x, y, z, temp);
        return true;
    }

    return false;
}


/***
 * 
*/
uint8_t TLE493D_A2B6_calculateConfigurationParity(Sensor_ts *sensor) {
	uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
	return getEvenParity(parity);
}


/***
 * 
*/
bool TLE493D_A2B6_setDisableTemperatureMeasurements(Sensor_ts *sensor, uint8_t dt) {
    uint8_t config = sensor->commonRegisters.CONFIG;

    // CONFIG register
    gen_2_setBitfield(sensor, DT, dt);
    gen_2_setBitfield(sensor, CP, TLE493D_A2B6_calculateConfigurationParity(sensor));

    gen_2_writeRegister(sensor, DT);
}


bool TLE493D_A2B6_enableTemperatureMeasurements(Sensor_ts *sensor) {
    return TLE493D_A2B6_setDisableTemperatureMeasurements(sensor, 0);
}


bool TLE493D_A2B6_disableTemperatureMeasurements(Sensor_ts *sensor) {
    return TLE493D_A2B6_setDisableTemperatureMeasurements(sensor, 1);
}


/***
 * This option depends on value of DT page 7.
*/
bool TLE493D_A2B6_setAngularMesurements(Sensor_ts *sensor, uint8_t am) {
    gen_2_setBitfield(sensor, AM, am);
    gen_2_setBitfield(sensor, CP, TLE493D_A2B6_calculateConfigurationParity(sensor));

    return gen_2_writeRegister(sensor, AM);
}


bool TLE493D_A2B6_enableAngularMesurements(Sensor_ts *sensor) {
    TLE493D_A2B6_setAngularMesurements(sensor, 1);
}


bool TLE493D_A2B6_disableAngularMesurements(Sensor_ts *sensor) {
    TLE493D_A2B6_setAngularMesurements(sensor, 0);
}


/***
 * This option depends on PR and MODE.
*/
bool TLE493D_A2B6_setTriggerOptions(Sensor_ts *sensor, uint8_t trig) {
    gen_2_setBitfield(sensor, TRIG, trig);
    gen_2_setBitfield(sensor, CP, TLE493D_A2B6_calculateConfigurationParity(sensor));

    return gen_2_writeRegister(sensor, TRIG);
}


bool TLE493D_A2B6_setNoTriggerOnReadTriggerOption(Sensor_ts *sensor) {
    TLE493D_A2B6_setTriggerOptions(sensor, 0b00);
}


bool TLE493D_A2B6_setTriggerOnReadBeforeFirstMSBTriggerOption(Sensor_ts *sensor) {
    TLE493D_A2B6_setTriggerOptions(sensor, 0b01);
}


bool TLE493D_A2B6_setTriggerOnReadAfterRegister05TriggerOption(Sensor_ts *sensor) {
    TLE493D_A2B6_setTriggerOptions(sensor, 0b10);
}


/***
 * 
*/
bool TLE493D_A2B6_setShortRangeSensitivity(Sensor_ts *sensor, uint8_t srs) {
    gen_2_setBitfield(sensor, X2, srs);
    gen_2_setBitfield(sensor, CP, TLE493D_A2B6_calculateConfigurationParity(sensor));

    return gen_2_writeRegister(sensor, X2);
}


bool TLE493D_A2B6_enableShortRangeSensitivity(Sensor_ts *sensor) {
    TLE493D_A2B6_setShortRangeSensitivity(sensor, 1);
}


bool TLE493D_A2B6_disableShortRangeSensitivity(Sensor_ts *sensor) {
    TLE493D_A2B6_setShortRangeSensitivity(sensor, 0);
}


/***
 * 
*/
bool TLE493D_A2B6_setMagneticTemperatureCompensation(Sensor_ts *sensor, uint8_t mtc) {
    gen_2_setBitfield(sensor, TL_MAG, mtc);
    gen_2_setBitfield(sensor, CP, TLE493D_A2B6_calculateConfigurationParity(sensor));

    return gen_2_writeRegister(sensor, TL_MAG);
}


bool TLE493D_A2B6_setTC0MagneticTemperatureCompensation(Sensor_ts *sensor) {
    TLE493D_A2B6_setMagneticTemperatureCompensation(sensor, 0b00);
}


bool TLE493D_A2B6_setTC1MagneticTemperatureCompensation(Sensor_ts *sensor) {
    TLE493D_A2B6_setMagneticTemperatureCompensation(sensor, 0b01);
}


bool TLE493D_A2B6_setTC2MagneticTemperatureCompensation(Sensor_ts *sensor) {
    TLE493D_A2B6_setMagneticTemperatureCompensation(sensor, 0b10);
}


bool TLE493D_A2B6_setTC3MagneticTemperatureCompensation(Sensor_ts *sensor) {
    TLE493D_A2B6_setMagneticTemperatureCompensation(sensor, 0b11);
}


/***
 * 
*/
bool TLE493D_A2B6_set1ByteReadMode(Sensor_ts *sensor, uint8_t pr) {
    uint8_t mod1 = sensor->commonRegisters.MOD1;

    gen_2_setBitfield(sensor, PR, pr);
    gen_2_setBitfield(sensor, FP, gen_2_calculateFuseParity(sensor));

    gen_2_writeRegister(sensor, PR);
}


bool TLE493D_A2B6_enable1ByteMode(Sensor_ts *sensor) {
     return TLE493D_A2B6_set1ByteReadMode(sensor, 1);
}


bool TLE493D_A2B6_disable1ByteMode(Sensor_ts *sensor) {
     return TLE493D_A2B6_set1ByteReadMode(sensor, 0);
}


/***
 * 
*/
bool  TLE493D_A2B6_setInterruptAndCollisionAvoidance(Sensor_ts *sensor, bool intIsOn, bool caIsOn) {
    gen_2_setBitfield(sensor, INT, intIsOn ? 0 : 1);
    gen_2_setBitfield(sensor, CA, caIsOn ? 1 : 0);
 
    gen_2_setBitfield(sensor, FP, gen_2_calculateFuseParity(sensor));
    
    return gen_2_writeRegister(sensor, INT); 
}


/***
 * CA only in Low Power and MCM mode, not in Fast Mode !
 * MODE depends on PR and TRIG !
*/
bool TLE493D_A2B6_setCollisionAvoidance(Sensor_ts *sensor, uint8_t ca) {
// gen_2_setBitfield(sensor, sensor->commonBitfields.MODE, 0b11);

    gen_2_setBitfield(sensor, CA, ca);
    gen_2_setBitfield(sensor, FP, gen_2_calculateFuseParity(sensor));

    return gen_2_writeRegister(sensor, CA);
}


bool TLE493D_A2B6_enableCollisionAvoidance(Sensor_ts *sensor) {
    TLE493D_A2B6_setCollisionAvoidance(sensor, 0);
}


bool TLE493D_A2B6_disableCollisionAvoidance(Sensor_ts *sensor) {
    TLE493D_A2B6_setCollisionAvoidance(sensor, 1);
}


/***
 * 
*/
bool TLE493D_A2B6_setInterrupt(Sensor_ts *sensor, uint8_t irq) {
    gen_2_setBitfield(sensor, INT, irq);
    gen_2_setBitfield(sensor, FP, gen_2_calculateFuseParity(sensor));

    return gen_2_writeRegister(sensor, INT);
}


bool TLE493D_A2B6_enableInterrupt(Sensor_ts *sensor) {
    TLE493D_A2B6_setInterrupt(sensor, 0);
}


bool TLE493D_A2B6_disableInterrupt(Sensor_ts *sensor) {
    TLE493D_A2B6_setInterrupt(sensor, 1);
}


/***
 * 
*/
bool TLE493D_A2B6_setUpdateRate(Sensor_ts *sensor, uint8_t ur) {
    uint8_t mod1 = sensor->commonRegisters.MOD1;

    // MOD2 register
    gen_2_setBitfield(sensor, PRD, ur);

    // MOD1 register : FP includes PRD bit !
    gen_2_setBitfield(sensor, FP, gen_2_calculateFuseParity(sensor));

    uint8_t buf[4] = { mod1,
                       sensor->regMap[mod1],     // MOD1 register
                       sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
                       sensor->regMap[mod1 + 2]  // MOD2 register
                     };

    return TLE493D_A2B6_transferRegisterMap(sensor, buf, sizeof(buf), NULL, 0);
}


bool TLE493D_A2B6_setHighUpdateRate(Sensor_ts *sensor) {
    TLE493D_A2B6_setUpdateRate(sensor, 0);
}


bool TLE493D_A2B6_setLowUpdateRate(Sensor_ts *sensor) {
    TLE493D_A2B6_setUpdateRate(sensor, 1);
}


/***
 * TODO: set all options that must be set, eg MODE ?, reset all bits to defaults ?
*/
bool TLE493D_A2B6_setDefaultConfig(Sensor_ts *sensor) {
    sensor->regMap[CONFIG_REG] = 0x00;
    sensor->regMap[MOD1_REG]   = 0x00;
    sensor->regMap[MOD2_REG]   = 0x00;;

    // MOD1 register
    gen_2_setBitfield(sensor, CA, 0);
    gen_2_setBitfield(sensor, INT, 1);

    if( TLE493D_A2B6_enable1ByteMode(sensor) ) {
        if( TLE493D_A2B6_enableTemperatureMeasurements(sensor) ) {
            // Read registers in order to retrieve values in reserved register at 0x12 and in MOD2 in order to make sure we are not 
            // accidentally changing a preset values to 0.
            if( gen_2_readRegisters(sensor) )
                return TLE493D_A2B6_setLowUpdateRate(sensor);

            // return true;
        }
    }

    return false;
}


/***
 * 
*/
bool TLE493D_A2B6_transferRegisterMap(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
    return sensor->comLibIF->transfer.i2c_transfer(sensor, tx_buffer, tx_len, rx_buffer, rx_len);
}




bool TLE493D_A2B6_hasValidIICadr(Sensor_ts *sensor) {
    return gen_2_hasValidIICadr(sensor, ID, IICADR);
}


bool TLE493D_A2B6_hasWakeup(Sensor_ts *sensor) {
    return gen_2_hasWakeup(sensor, TYPE);
}
