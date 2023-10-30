/**
 * @file        TLx493D_W2BW.c
 * @author      Infineon Technologies AG
 * @brief       Contains the implementation of the complete sensor functionality
 * @copyright   Copyright (c) 2023
 *
 * SPDX-License-Identifier: MIT
 */

/** Sensor specific includes */
#include "TLx493D_W2BW.h"


extern void setI2CParameters(Sensor_ts *sensor, uint8_t addr);


typedef enum {
    BX_MSB = 0,
    BY_MSB,
    BZ_MSB,
    TEMP_MSB,
    BX_LSB, BY_LSB,
    TEMP_LSB, ID, BZ_LSB,
    P, FF, CF, T, PD3, PD0, FRM,
    XL_MSB, 
    XH_MSB,
    YL_MSB, 
    YH_MSB, 
    ZL_MSB, 
    ZH_MSB,
    WA, WU, XH_LSB, XL_LSB,
    YH_LSB, YL_LSB,
    ZH_LSB, ZL_LSB, 
    DT, AM, TRIG, X2, TL_MAG, CP,
    FP, IICADR, PR, CA, INT, MODE,
    PRD,
    X4,
    TYPE, HWV
} TLx493D_W2BW_registerNames_te;

Register_ts TLx493D_W2BW_regDef[] = {
    {BX_MSB,    READ_MODE_e,        0x00,   0xFF,   0,  8},
    {BY_MSB,    READ_MODE_e,        0x01,   0xFF,   0,  8},
    {BZ_MSB,    READ_MODE_e,        0x02,   0xFF,   0,  8},
    {TEMP_MSB,  READ_MODE_e,        0x03,   0xFF,   0,  8},
    {BX_LSB,    READ_MODE_e,        0x04,   0xF0,   4,  4},
    {BY_LSB,    READ_MODE_e,        0x04,   0x0F,   0,  4},
    {TEMP_LSB,  READ_MODE_e,        0x05,   0xC0,   6,  2},
    {ID,        READ_MODE_e,        0x05,   0x30,   4,  2},
    {BZ_LSB,    READ_MODE_e,        0x05,   0x0F,   0,  4},
    {P,         READ_MODE_e,        0x06,   0x80,   7,  1},
    {FF,        READ_MODE_e,        0x06,   0x40,   6,  1},
    {CF,        READ_MODE_e,        0x06,   0x20,   5,  1},
    {T,         READ_MODE_e,        0x06,   0x10,   4,  1},
    {PD3,       READ_MODE_e,        0x06,   0x08,   3,  1},
    {PD0,       READ_MODE_e,        0x06,   0x04,   2,  1},
    {FRM,       READ_MODE_e,        0x06,   0x03,   0,  2},
    {XL_MSB,    READ_WRITE_MODE_e,  0x07,   0xFF,   0,  8},
    {XH_MSB,    READ_WRITE_MODE_e,  0x08,   0xFF,   0,  8},
    {YL_MSB,    READ_WRITE_MODE_e,  0x09,   0xFF,   0,  8},
    {YH_MSB,    READ_WRITE_MODE_e,  0x0A,   0xFF,   0,  8},
    {ZL_MSB,    READ_WRITE_MODE_e,  0x0B,   0xFF,   0,  8},
    {ZH_MSB,    READ_WRITE_MODE_e,  0x0C,   0xFF,   0,  8},
    {WA,        READ_MODE_e,        0x0D,   0x80,   7,  1},
    {WU,        READ_WRITE_MODE_e,  0x0D,   0x40,   6,  1},
    {XH_LSB,    READ_WRITE_MODE_e,  0x0D,   0x38,   3,  3},
    {XL_LSB,    READ_WRITE_MODE_e,  0x0D,   0x07,   0,  3},
    {YH_LSB,    READ_WRITE_MODE_e,  0x0E,   0x38,   3,  3},
    {YL_LSB,    READ_WRITE_MODE_e,  0x0E,   0x07,   0,  3},
    {ZH_LSB,    READ_WRITE_MODE_e,  0x0F,   0x38,   3,  3},
    {ZL_LSB,    READ_WRITE_MODE_e,  0x0F,   0x07,   0,  3},
    {DT,        READ_WRITE_MODE_e,  0x10,   0x80,   7,  1},
    {AM,        READ_WRITE_MODE_e,  0x10,   0x40,   6,  1},
    {TRIG,      READ_WRITE_MODE_e,  0x10,   0x30,   4,  2},
    {X2,        READ_WRITE_MODE_e,  0x10,   0x08,   3,  1},
    {TL_MAG,    READ_WRITE_MODE_e,  0x10,   0x06,   1,  2},
    {CP,        READ_WRITE_MODE_e,  0x10,   0x01,   0,  1},
    {FP,        READ_WRITE_MODE_e,  0x11,   0x80,   7,  1},
    {IICADR,    READ_WRITE_MODE_e,  0x11,   0x60,   5,  2},
    {PR,        READ_WRITE_MODE_e,  0x11,   0x10,   4,  1},
    {CA,        READ_WRITE_MODE_e,  0x11,   0x08,   3,  1},
    {INT,       READ_WRITE_MODE_e,  0x11,   0x04,   2,  1},
    {MODE,      READ_WRITE_MODE_e,  0x11,   0x03,   0,  2},
    {PRD,       READ_WRITE_MODE_e,  0x13,   0x80,   7,  1},
    {X4,        WRITE_MODE_e,       0x14,   0x01,   0,  1},
    {TYPE,      READ_MODE_e,        0x16,   0x30,   4,  2},
    {HWV,       READ_MODE_e,        0x16,   0x0F,   0,  4}
};

typedef enum {
               LOW_POWER_MODE         = 0x00,
               MASTER_CONTROLLED_MODE = 0x01,
               RESERVED_MODE          = 0x10,
               FAST_MODE              = 0x11 } TLE493D_A2B6_modes_te;


typedef enum { 
               TEMP2_REG  = 0x05,
               DIAG_REG   = 0x06,
               CONFIG_REG = 0x10,
               MOD1_REG   = 0x11,
               MOD2_REG   = 0x13,
               VER_REG    = 0x16 } SpecialRegisters_te;

CommonFunctions_ts TLx493D_W2BW_commonFunctions = {
    .init                               = TLx493D_W2BW_init,
    .deinit                             = TLx493D_W2BW_deinit,

    .calculateTemperature               = TLx493D_W2BW_calculateTemperature,
    // .getTemperature                     = TLx493D_W2BW_getTemperature,
    
    .calculateMagneticField             = TLx493D_W2BW_calculateMagneticField,
    // .getFieldValues                     = TLx493D_W2BW_getFieldValues,
    
    // .getSensorValues                    = TLx493D_W2BW_getSensorValues,

    .setDefaultConfig                   = TLx493D_W2BW_setDefaultConfig,
    .readRegisters                      = tlx493d_readRegisters,
    // .transfer                           = TLx493D_W2BW_transferRegisterMap,

    .enableTemperatureMeasurement       = TLx493D_W2BW_enableTemperatureMeasurement,
    .disableTemperatureMeasurement      = TLx493D_W2BW_disableTemperatureMeasurement,

    .enableInterrupt                    = TLx493D_W2BW_enableInterrupt,
    .disableInterrupt                   = TLx493D_W2BW_disableInterrupt,

    .setPowerMode                       = TLx493D_W2BW_setPowerMode,
    .setIICAddress                      = TLx493D_W2BW_setIICAddress,

    .calculateConfigurationParity       = TLx493D_W2BW_calculateConfigurationParityBit,

    // .enableAngularMeasurement           = tlx493d_gen_2_enableAngularMeasurement,
    // .disableAngularMeasurement          = tlx493d_gen_2_disableAngularMeasurement,

    .setTriggerBits                     = TLx493D_W2BW_setTriggerBits,
    .setUpdateRate                      = TLx493D_W2BW_setUpdateRate,

    .isWakeUpActive                     = TLx493D_W2BW_isWakeUpActive,
    .enableWakeUpMode                   = TLx493D_W2BW_enableWakeUpMode,
    .disableWakeUpMode                  = TLx493D_W2BW_disableWakeUpMode,

    .setLowerWakeUpThresholdX           = TLx493D_W2BW_setLowerWakeUpThresholdX,
    .setLowerWakeUpThresholdY           = TLx493D_W2BW_setLowerWakeUpThresholdY,
    .setLowerWakeUpThresholdZ           = TLx493D_W2BW_setLowerWakeUpThresholdZ,

    .setUpperWakeUpThresholdX           = TLx493D_W2BW_setUpperWakeUpThresholdX,
    .setUpperWakeUpThresholdY           = TLx493D_W2BW_setUpperWakeUpThresholdY,
    .setUpperWakeUpThresholdZ           = TLx493D_W2BW_setUpperWakeUpThresholdZ,

    .setWakeUpThresholds                = TLx493D_W2BW_setWakeUpThresholds
};

bool TLx493D_W2BW_init(Sensor_ts *sensor) {
    sensor->regMap                  = (uint8_t*)malloc(sizeof(uint8_t) * GEN_2_REG_MAP_SIZE);
    sensor->regDef                  = TLx493D_W2BW_regDef;
    // sensor->commonBitfields         = (CommonBitfields_ts) { .CP = CP, .FP = FP, .ID = ID, .P = P, .FF = FF, .CF = CF, .T = T, .PD3 = PD3, .PD0 = PD0, .FRM = FRM, .PRD = PRD, .TYPE = TYPE, .HWV = HWV,
    //                                                          .DT = DT, .AM = AM, .TRIG = TRIG, .X2 = X2, .TL_MAG = TL_MAG, .IICADR = IICADR, .PR = PR, .CA = CA, .INT = INT, .MODE = MODE,
    //                                                          .BX_MSB = BX_MSB, .BY_MSB = BY_MSB, .BZ_MSB = BZ_MSB, .TEMP_MSB = TEMP_MSB,
    //                                                          .BX_LSB = BX_LSB, .BY_LSB = BY_LSB, .TEMP_LSB = TEMP_LSB, .BZ_LSB = BZ_LSB, .TEMP2 = TEMP2_REG,
    //                                                         };
    // sensor->commonRegisters         = (CommonRegisters_ts) { .DIAG = DIAG_REG, .CONFIG = CONFIG_REG, .MOD1 = MOD1_REG, .MOD2 = MOD2_REG, .VER = VER_REG };

    sensor->functions               = &TLx493D_W2BW_commonFunctions;
    sensor->regMapSize              = GEN_2_REG_MAP_SIZE;
    sensor->sensorType              = TLx493D_W2BW_e;
    sensor->comIFType               = I2C_e;
    sensor->comLibIF                = NULL;
    sensor->comLibObj.i2c_obj       = NULL;

    memset(sensor->regMap, 0, sensor->regMapSize);

    setI2CParameters(sensor, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return true;
}

bool TLx493D_W2BW_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);
   
    sensor->regMap = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    return true;
}

void TLx493D_W2BW_calculateTemperature(Sensor_ts *sensor, double *temp) {
    int16_t value = 0;

    tlx493d_concatBytes(sensor, TEMP_MSB, TEMP_LSB, &value);

    value <<= 2;
    *temp = (double)((((double) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF);
}

bool TLx493D_W2BW_getTemperature(Sensor_ts *sensor, double *temp) {
    if (tlx493d_readRegisters(sensor)) {
        TLx493D_W2BW_calculateTemperature(sensor, temp);
        return true;
    }

    return false;
}

void TLx493D_W2BW_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    tlx493d_concatBytes(sensor, BX_MSB, BX_LSB, &valueX);
    tlx493d_concatBytes(sensor, BY_MSB, BY_LSB, &valueY);
    tlx493d_concatBytes(sensor, BZ_MSB, BZ_LSB, &valueZ);
    
    *x = ((double) valueX) * GEN_2_MAG_FIELD_MULT;
    *y = ((double) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((double) valueZ) * GEN_2_MAG_FIELD_MULT;
}

bool TLx493D_W2BW_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z) {
    if (tlx493d_readRegisters(sensor)) {
        TLx493D_W2BW_calculateMagneticField(sensor, x, y, z);
        return true;
    }
    
    return false;
}

// bool TLx493D_W2BW_getSensorValues(Sensor_ts *sensor, double *x, double *y, double *z, double *temp) {
//     return true;
// }

bool TLx493D_W2BW_reset(Sensor_ts *sensor) {
        return true;
}

bool TLx493D_W2BW_getDiagnosis(Sensor_ts *sensor) {
        return true;
}

// // Fuse/mode parity bit FP
// uint8_t TLx493D_W2BW_calculateFuseParityBit(Sensor_ts *sensor) {
// 	// compute parity of MOD1 register
// 	uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[FP].mask);

// 	// add parity of MOD2:PRD register bits
// 	parity ^= calculateParity(sensor->regMap[sensor->commonRegisters.MOD2] & sensor->regDef[PRD].mask);

// 	return getOddParity(parity) << sensor->regDef[FP].offset;
// }

// Configuration parity bit CP
uint8_t TLx493D_W2BW_calculateConfigurationParityBit(Sensor_ts *sensor) {
	// uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask);
	// return getEvenParity(parity);

	uint8_t parity = 0;
    for(uint8_t x = sensor->regDef[XL_MSB].address; x <= sensor->regDef[ZH_MSB].address; x++) {
        parity ^= sensor->regMap[x];
    }
    parity ^= sensor->regMap[sensor->regDef[WU].address] & ~sensor->regDef[WA].mask;
    parity ^= sensor->regMap[sensor->regDef[YH_LSB].address] & (sensor->regDef[YH_LSB].mask & sensor->regDef[YL_LSB].mask);
    parity ^= sensor->regMap[sensor->regDef[ZH_LSB].address] & (sensor->regDef[ZH_LSB].mask & sensor->regDef[ZL_LSB].mask);
    parity ^= sensor->regMap[CONFIG_REG] & ~sensor->regDef[CP].mask;
    
    parity = tlx493d_calculateParity(parity);

	return tlx493d_getOddParity(parity);
}

static bool TLx493D_W2BW_enable1ByteMode(Sensor_ts *sensor) {
    // bool b = false;

    // sensor->regMap[sensor->regDef[FP].address] = 0;

    // tlx493d_setBitfield(sensor, FP, 0);
    // tlx493d_setBitfield(sensor, PR, 1);
    // tlx493d_setBitfield(sensor, INT, 1);
    // tlx493d_setBitfield(sensor, CA, 1);

    // b = tlx493d_writeRegister(sensor, FP);

    // return b;

    tlx493d_setBitfield(sensor, PR, 1);
    tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));
    return tlx493d_writeRegister(sensor, FP);
}

static bool TLx493D_W2BW_enableTemperatureMeasurements(Sensor_ts *sensor) {
    bool b = false;

    sensor->regMap[sensor->regDef[CP].address] = 0;

    tlx493d_setBitfield(sensor, DT, 0);
    tlx493d_setBitfield(sensor, CP, 0);

    b = tlx493d_writeRegister(sensor, DT);

    return b;
}

bool TLx493D_W2BW_setDefaultConfig(Sensor_ts *sensor) {
    // bool b = TLx493D_W2BW_enable1ByteMode(sensor);
    // return b && TLx493D_W2BW_enableTemperature(sensor);

    sensor->regMap[CONFIG_REG] = 0x01;
    sensor->regMap[MOD1_REG]   = 0x80;
    sensor->regMap[MOD2_REG]   = 0x00;

    tlx493d_writeRegister(sensor, CP);
    tlx493d_writeRegister(sensor, FP);

    tlx493d_setBitfield(sensor, CA, 0);
    tlx493d_setBitfield(sensor, INT, 1);

    if( TLx493D_W2BW_enable1ByteMode(sensor) ) {
        return true;
    }

    return false;
}


bool TLx493D_W2BW_setPowerMode(Sensor_ts *sensor, uint8_t mode) {
    return tlx493d_gen_2_setPowerMode(sensor, MODE, FP, mode);
}


bool TLx493D_W2BW_setIICAddress(Sensor_ts *sensor, StandardIICAddresses_te address) {
    return tlx493d_gen_2_setIICAddress(sensor, IICADR, FP, address);
}




bool TLx493D_W2BW_setTriggerBits(Sensor_ts *sensor, uint8_t triggerBits) {
    return tlx493d_gen_2_setTriggerBits(sensor, TRIG, CP, triggerBits);
}


// bool TLx493D_W2BW_transferRegisterMap(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
//     return sensor->comLibIF->transfer.i2c_transfer(sensor, tx_buffer, tx_len, rx_buffer, rx_len);
// }

bool TLx493D_W2BW_enableTemperatureMeasurement(Sensor_ts *sensor) {
    bool b = tlx493d_readRegisters(sensor);

    tlx493d_setBitfield(sensor, DT, 0);
    tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));
    // sensor->regMap[sensor->commonRegisters.CONFIG] = (sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask) | TLx493D_W2BW_calculateConfigurationParityBit(sensor);

    return b && tlx493d_writeRegister(sensor, DT);
}

bool TLx493D_W2BW_disableTemperatureMeasurement(Sensor_ts *sensor) {
    bool b = tlx493d_readRegisters(sensor);

    tlx493d_setBitfield(sensor, DT, 1);
    tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));
    // sensor->regMap[sensor->commonRegisters.CONFIG] = (sensor->regMap[sensor->commonRegisters.CONFIG] & ~sensor->regDef[CP].mask) | TLx493D_W2BW_calculateConfigurationParityBit(sensor);
    
    return b && tlx493d_writeRegister(sensor, DT);
}

bool TLx493D_W2BW_enableInterrupt(Sensor_ts *sensor) {
    bool b = tlx493d_readRegisters(sensor);

    tlx493d_setBitfield(sensor, INT, 0);
    tlx493d_setBitfield(sensor, CA, 1);
    tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));
    // sensor->regMap[sensor->commonRegisters.MOD1] = (sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[FP].mask) | TLx493D_W2BW_calculateFuseParityBit(sensor);
    
    return b && tlx493d_writeRegister(sensor, INT); 
}

bool TLx493D_W2BW_disableInterrupt(Sensor_ts *sensor) {
    bool b = tlx493d_readRegisters(sensor);

    tlx493d_setBitfield(sensor, INT, 1);
    tlx493d_setBitfield(sensor, CA, 1);
    tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));
    // sensor->regMap[sensor->commonRegisters.MOD1] = (sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[FP].mask) | TLx493D_W2BW_calculateFuseParityBit(sensor);

    return b && tlx493d_writeRegister(sensor, INT);
}


bool TLx493D_W2BW_setUpdateRate(Sensor_ts *sensor, uint8_t bit) {
    bool b = tlx493d_readRegisters(sensor);

    tlx493d_setBitfield(sensor, PRD, bit);
    tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

    uint8_t tx_buffer[4] = { MOD1_REG,
                             sensor->regMap[MOD1_REG],
                             sensor->regMap[MOD1_REG + 1],
                             sensor->regMap[MOD1_REG + 2] 
                            };

    return b && transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}


bool TLx493D_W2BW_isWakeUpActive(Sensor_ts *sensor) {
    uint8_t bitFieldValue = 0;
    tlx493d_getBitfield(sensor, WA, &bitFieldValue);

    return bitFieldValue;
}

bool TLx493D_W2BW_enableWakeUpMode(Sensor_ts *sensor) {
    uint8_t bitFieldValue = 0;
    tlx493d_getBitfield(sensor, T, &bitFieldValue);

    if (!bitFieldValue) {
        tlx493d_setBitfield(sensor, WU, 1);
        tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

        uint8_t tx_buffer[5] = { sensor->regDef[WU].address,
                                  sensor->regMap[sensor->regDef[WU].address],
                                  sensor->regMap[sensor->regDef[WU].address + 1],
                                  sensor->regMap[sensor->regDef[WU].address + 2],
                                  sensor->regMap[sensor->regDef[WU].address + 3]
                                };
        
        return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
    }
    else {
        return false;
    }
}

bool TLx493D_W2BW_disableWakeUpMode(Sensor_ts *sensor) {
    tlx493d_setBitfield(sensor, WU, 0);
    tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[5] = { sensor->regDef[WU].address,
                                sensor->regMap[sensor->regDef[WU].address],
                                sensor->regMap[sensor->regDef[WU].address + 1],
                                sensor->regMap[sensor->regDef[WU].address + 2],
                                sensor->regMap[sensor->regDef[WU].address + 3]
                            };
    
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setLowerWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_setBitfield(sensor, XL_MSB, msb);
    tlx493d_setBitfield(sensor, XL_LSB, lsb);

    tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setLowerWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_setBitfield(sensor, YL_MSB, msb);
    tlx493d_setBitfield(sensor, YL_LSB, lsb);

    tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setLowerWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_setBitfield(sensor, ZL_MSB, msb);
    tlx493d_setBitfield(sensor, ZL_LSB, lsb);

    tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

// Upper Limit is 2047 and lower -2048
bool TLx493D_W2BW_setUpperWakeUpThresholdX(Sensor_ts *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_setBitfield(sensor, XH_MSB, msb);
    tlx493d_setBitfield(sensor, XH_LSB, lsb);

    tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setUpperWakeUpThresholdY(Sensor_ts *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_setBitfield(sensor, YH_MSB, msb);
    tlx493d_setBitfield(sensor, YH_LSB, lsb);

    tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setUpperWakeUpThresholdZ(Sensor_ts *sensor, int16_t threshold) {
    uint8_t msb = (threshold & 0xFF0) >> 4;
    uint8_t lsb = (threshold & 0x00E) >> 1;

    tlx493d_setBitfield(sensor, ZH_MSB, msb);
    tlx493d_setBitfield(sensor, ZH_LSB, lsb);

    tlx493d_setBitfield(sensor, CP, TLx493D_W2BW_calculateConfigurationParityBit(sensor));

    uint8_t tx_buffer[11] = {   sensor->regDef[XL_MSB].address,
                                sensor->regMap[sensor->regDef[XL_MSB].address],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 1],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 2],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 3],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 4],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 5],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 6],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 7],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 8],
                                sensor->regMap[sensor->regDef[XL_MSB].address + 9]
                            };
    return transfer(sensor, tx_buffer, sizeof(tx_buffer), NULL, 0);
}

bool TLx493D_W2BW_setWakeUpThresholds(Sensor_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
    bool returnValue = TLx493D_W2BW_setLowerWakeUpThresholdX(sensor, xl_th);
    returnValue &= TLx493D_W2BW_setUpperWakeUpThresholdX(sensor, xh_th);

    returnValue &= TLx493D_W2BW_setLowerWakeUpThresholdY(sensor, yl_th);
    returnValue &= TLx493D_W2BW_setUpperWakeUpThresholdY(sensor, yh_th);
    
    returnValue &= TLx493D_W2BW_setLowerWakeUpThresholdZ(sensor, zl_th);
    return  returnValue & TLx493D_W2BW_setUpperWakeUpThresholdZ(sensor, zh_th);
}
