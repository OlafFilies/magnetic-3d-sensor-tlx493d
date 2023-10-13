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
#include "sensors_gen_1_common_defines.h"
#include "sensors_gen_1_common.h"

// sensor specicifc includes
#include "TLx493D_A1B6_defines.h"
#include "TLx493D_A1B6.h"

//register enums
typedef enum {
    TLx493D_A1B6_Temp_ENABLE_default,
    TLx493D_A1B6_Temp_DISABLE
} TLx493D_A1B6_Reg_Temp_NEN;

typedef enum {
    TLx493D_A1B6_ODD_PARITY,
    TLx493D_A1B6_EVEN_PARITY
} TLx493D_A1B6_Reg_PARITY;

typedef enum {
    TLx493D_A1B6_CONFIG_00_default,
    TLx493D_A1B6_CONFIG_01,
    TLx493D_A1B6_CONFIG_10,
    TLx493D_A1B6_CONFIG_11
} TLx493D_A1B6_Reg_IICADDR;

typedef enum {
    TLx493D_A1B6_INT_ENABLE_default,
    TLx493D_A1B6_INT_DISABLE
} TLx493D_A1B6_Reg_INT;

typedef enum {
    TLx493D_A1B6_FAST_MODE_DISABLE_default,
    TLx493D_A1B6_FAST_MODE_ENABLE
} TLx493D_A1B6_Reg_FAST_MODE_NEN;

typedef enum {
    TLx493D_A1B6_LOW_POWER_MODE_DISABLE_default,
    TLx493D_A1B6_LOW_POWER_MODE_ENABLE
} TLx493D_A1B6_Reg_LOW_POWER_MODE_NEN;

typedef enum {
    TLx493D_A1B6_LOW_POWER_PERIOD_100MS_default,
    TLx493D_A1B6_LOW_POWER_PERIOD_12MS
} TLx493D_A1B6_Reg_LOW_POWER_PERIOD;

typedef enum {
    TLx493D_A1B6_PARITY_TEST_DISABLE,
    TLx493D_A1B6_PARITY_TEST_ENABLE_default
} TLx493D_A1B6_Reg_PARITY_TEST_NEN;

// framework functions
extern void setI2CParameters(Sensor_ts *sensor, uint8_t addr); 

/*
  Listing of all register names for this sensor.
  Used to index "TLx493D_A1B6_regDef" defined below, so index values must match !
*/
typedef enum {
               BX_MSB = 0,
               BY_MSB,
               BZ_MSB,
               TEMP_MSB,
               FRM,
               CH,
               BX_LSB,
               BY_LSB,
               TEST_Flag,
               FF,
               PD,
               BZ_LSB,
               TEMP_LSB,
               R_RES_1,
               R_RES_2,
               R_RES_3,
               W_RES_0,
               P,
               IICaddr,
               W_RES_1,
               INT,
               FAST,
               LOW_POWER,
               W_RES_2,
               Temp_NEN,
               LP,
               PT,
               W_RES_3 } TLx493D_A1B6_registerNames_te;

Register_ts TLx493D_A1B6_regDef[] = {
    // Read registers
    {BX_MSB,        READ_MODE_e,    0x00, 0xFF, 0, 8},
    {BY_MSB,        READ_MODE_e,    0x01, 0xFF, 0, 8},
    {BZ_MSB,        READ_MODE_e,    0x02, 0xFF, 0, 8},
    {TEMP_MSB,      READ_MODE_e,    0x03, 0xF0, 4, 4},
    {FRM,           READ_MODE_e,    0x03, 0x0C, 2, 2},
    {CH,            READ_MODE_e,    0x03, 0x03, 0, 2},
    {BX_LSB,        READ_MODE_e,    0x04, 0xF0, 4, 4},
    {BY_LSB,        READ_MODE_e,    0x04, 0x0F, 0, 4},
    {TEST_Flag,     READ_MODE_e,    0x05, 0x40, 6, 1},
    {FF,            READ_MODE_e,    0x05, 0x20, 5, 1},
    {PD,            READ_MODE_e,    0x05, 0x10, 4, 1},
    {BZ_LSB,        READ_MODE_e,    0x05, 0x0F, 0, 4},
    {TEMP_LSB,      READ_MODE_e,    0x06, 0xFF, 0, 8},
    {R_RES_1,       READ_MODE_e,    0x07, 0x18, 3, 2},
    {R_RES_2,       READ_MODE_e,    0x08, 0xFF, 0, 8},
    {R_RES_3,       READ_MODE_e,    0x09, 0x1F, 0, 5},
    // Write Registers
    {W_RES_0,       WRITE_MODE_e,   0x00, 0xFF, 0, 8},
    {P,             WRITE_MODE_e,   0x01, 0x80, 7, 1},
    {IICaddr,       WRITE_MODE_e,   0x01, 0x60, 5, 2},
    {W_RES_1,       WRITE_MODE_e,   0x01, 0x18, 3, 2},
    {INT,           WRITE_MODE_e,   0x01, 0x04, 2, 1},
    {FAST,          WRITE_MODE_e,   0x01, 0x02, 1, 1},
    {LOW_POWER,     WRITE_MODE_e,   0x01, 0x01, 0, 1},
    {W_RES_2,       WRITE_MODE_e,   0x02, 0xFF, 0, 8},
    {Temp_NEN,      WRITE_MODE_e,   0x03, 0x80, 7, 1},
    {LP,            WRITE_MODE_e,   0x03, 0x40, 6, 1},
    {PT,            WRITE_MODE_e,   0x03, 0x20, 5, 1},
    {W_RES_3,       WRITE_MODE_e,   0x03, 0x1F, 0, 5},
};

typedef enum { 
               MOD1_REG   = 0x01,
               MOD2_REG   = 0x03 } SpecialRegisters_te;           

CommonFunctions_ts TLx493D_A1B6_commonFunctions = {
                                .init                  = TLx493D_A1B6_init,
                                .deinit                = TLx493D_A1B6_deinit,

                                .calculateTemperature  = TLx493D_A1B6_calculateTemperature,
                                .getTemperature        = TLx493D_A1B6_getTemperature,

                                .calculateFieldValues  = TLx493D_A1B6_calculateFieldValues,
                                .getFieldValues        = TLx493D_A1B6_getFieldValues,

                                .getSensorValues       = TLE493D_A2B6_getSensorValues,

                                // .reset                 = TLx493D_A1B6_reset,

                                //.hasValidData          = gen_2_hasValidData,
                                //.isFunctional          = gen_2_isFunctional,

                                .setDefaultConfig      = TLx493D_A1B6_setDefaultConfig,
                                .readRegisters         = gen_1_readRegisters,

                                .enableTemperature     = TLx493D_A1B6_enableTemperatureMeasurements,
                                .disableTemperature    = TLx493D_A1B6_disableTemperatureMeasurements, //TODO: rebase common functions as per new common functions struct 

                                .setIICAddress         = TLx493D_A1B6_setIICAddress,
                              };


bool TLx493D_A1B6_init(Sensor_ts *sensor) {
    sensor->regMap            = (uint8_t *) malloc(sizeof(uint8_t) * GEN_1_REG_MAP_SIZE);
    sensor->regDef            = TLx493D_A1B6_regDef;
    sensor->commonBitfields   = (CommonBitfields_ts) {  .BX_MSB = BX_MSB, .BY_MSB = BY_MSB, 
                                                        .BZ_MSB = BZ_MSB, .TEMP_MSB = TEMP_MSB,
                                                        .BX_LSB = BX_LSB, .BY_LSB = BY_LSB, 
                                                        .T = TEST_Flag, .TEMP_LSB = TEMP_LSB, 
                                                        .BZ_LSB = BZ_LSB, .P = P, .FRM = FRM, 
                                                        .FF = FF, .CH = CH, .PD = PD, .IICADR = IICaddr, 
                                                        .INT = INT, .FAST = FAST, .LOW_POWER = LOW_POWER, 
                                                        .LP = LP, .PT = PT, .Temp_NEN = Temp_NEN, 
                                                        .PT = PT, .R_RES_1 = R_RES_1, 
                                                        .R_RES_2 = R_RES_2, .R_RES_3 = R_RES_3, 
                                                        .W_RES_0 = W_RES_0, .W_RES_1 = W_RES_1, 
                                                        .W_RES_2 = W_RES_2, .W_RES_3 = W_RES_3
                                                     };
    sensor->commonRegisters   = (CommonRegisters_ts) { .MOD1 = MOD1_REG, .MOD2 = MOD2_REG };                                                     
    sensor->functions         = &TLx493D_A1B6_commonFunctions;
    sensor->regMapSize        = GEN_1_REG_MAP_SIZE;
    sensor->sensorType        = TLx493D_A1B6_e;
    sensor->comIFType         = I2C_e;
    sensor->comLibIF          = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    setI2CParameters(sensor, GEN_1_STD_IIC_ADDR);

    return true;
}

bool TLx493D_A1B6_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);

    sensor->regMap            = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    return true;
}

void TLx493D_A1B6_setReservedRegisterValues(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, sensor->commonBitfields.W_RES_1, gen_1_returnBitfield(sensor, sensor->commonBitfields.R_RES_1));
    gen_1_setBitfield(sensor, sensor->commonBitfields.W_RES_2, gen_1_returnBitfield(sensor, sensor->commonBitfields.R_RES_2));
    gen_1_setBitfield(sensor, sensor->commonBitfields.W_RES_3, gen_1_returnBitfield(sensor, sensor->commonBitfields.R_RES_3));
}

void TLx493D_A1B6_setPowerDownMode(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, sensor->commonBitfields.FAST, TLx493D_A1B6_FAST_MODE_DISABLE_default);
    gen_1_setBitfield(sensor, sensor->commonBitfields.LOW_POWER, TLx493D_A1B6_LOW_POWER_MODE_DISABLE_default);
    gen_1_setBitfield(sensor, sensor->commonBitfields.LP, TLx493D_A1B6_LOW_POWER_PERIOD_100MS_default);
}

void TLx493D_A1B6_setMasterControlledMode(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, sensor->commonBitfields.FAST, TLx493D_A1B6_FAST_MODE_ENABLE);
    gen_1_setBitfield(sensor, sensor->commonBitfields.LOW_POWER, TLx493D_A1B6_LOW_POWER_MODE_ENABLE);
    gen_1_setBitfield(sensor, sensor->commonBitfields.LP, TLx493D_A1B6_LOW_POWER_PERIOD_12MS);
}

// note: make sure that the init function is called at reset to make sure the write default values are in sync.

bool TLx493D_A1B6_setDefaultConfig(Sensor_ts *sensor) {
    
    bool ret = true;
    // read READ register values
    ret = gen_1_readRegisters(sensor);
    
    // set WRITE register values to 0x00
    memset(sensor->regMap+TLx493D_A1B6_WRITE_REGISTERS_OFFSET,0,TLx493D_A1B6_WRITE_REGISTERS_MAX_COUNT);

    // todo: reset sensor here

    // set WRITE reserved register values to READ reserved register values
    TLx493D_A1B6_setReservedRegisterValues(sensor);

    //set to POWERDOWNMODE at boot
    TLx493D_A1B6_setPowerDownMode(sensor);

    // enable parity test and write to registers 
    ret &= TLx493D_A1B6_enableParityTest(sensor);

    // set to MASTERCONTROLLEDMODE to start measurement
    TLx493D_A1B6_setMasterControlledMode(sensor);

    // calculate parity
    TLx493D_A1B6_calculateParity(sensor);

    // write out register map to registers
    ret &= TLx493D_A1B6_transferWriteRegisters(sensor);

    // update register map
    ret &= gen_1_readRegisters(sensor);

    return ret;  
}

bool TLx493D_A1B6_disableTemperatureMeasurements(Sensor_ts *sensor) {
    
    gen_1_setBitfield(sensor, sensor->commonBitfields.Temp_NEN, TLx493D_A1B6_Temp_DISABLE);
    TLx493D_A1B6_calculateParity(sensor);
    bool ret = TLx493D_A1B6_transferWriteRegisters(sensor);
    
    return ret;                                                           
}

bool TLx493D_A1B6_enableTemperatureMeasurements(Sensor_ts *sensor) {

    gen_1_setBitfield(sensor, sensor->commonBitfields.Temp_NEN, TLx493D_A1B6_Temp_ENABLE_default);
    TLx493D_A1B6_calculateParity(sensor);
    bool ret = TLx493D_A1B6_transferWriteRegisters(sensor);

    return ret; 
}

bool TLx493D_A1B6_transferWriteRegisters(Sensor_ts *sensor) {
    
    bool retn = sensor->comLibIF->transfer.i2c_transfer(sensor, sensor->regMap+TLx493D_A1B6_WRITE_REGISTERS_OFFSET, TLx493D_A1B6_WRITE_REGISTERS_MAX_COUNT, NULL, 0);
    
    return retn;
}

// parity is calculated for all the WRITE register, including the parity bit
void TLx493D_A1B6_calculateParity(Sensor_ts *sensor) {
    uint8_t result = 0x00;

    //set parity as EVEN first
    gen_1_setBitfield(sensor, sensor->commonBitfields.P, TLx493D_A1B6_EVEN_PARITY); 

    // calculate bitwise XOR for all WRITE registers
    for (uint8_t addr = 0x00; addr<TLx493D_A1B6_WRITE_REGISTERS_MAX_COUNT; addr++) {
        result ^= sensor->regMap[addr+TLx493D_A1B6_WRITE_REGISTERS_OFFSET];
    }

    // then set calculated parity
    gen_1_setBitfield(sensor, sensor->commonBitfields.P, calculateParity(result));
}

void TLx493D_A1B6_calculateTemperature(Sensor_ts *sensor, float *temp) {
    int16_t value = 0;
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.TEMP_MSB], &sensor->regDef[sensor->commonBitfields.TEMP_LSB], &value);
    
    *temp = ((float)value - GEN_1_TEMP_OFFSET) * GEN_1_TEMP_MULT;
}

bool TLx493D_A1B6_getTemperature(Sensor_ts *sensor, float *temp) {
    if( gen_1_readRegisters(sensor) ) {
        TLx493D_A1B6_calculateTemperature(sensor, temp);
        return true;
    }
    return false;
}

bool TLx493D_A1B6_enableParityTest(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, sensor->commonBitfields.PT, TLx493D_A1B6_PARITY_TEST_ENABLE_default);
    TLx493D_A1B6_calculateParity(sensor);
    return TLx493D_A1B6_transferWriteRegisters(sensor);
}

bool TLx493D_A1B6_disableParityTest(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, sensor->commonBitfields.PT, TLx493D_A1B6_PARITY_TEST_DISABLE);
    TLx493D_A1B6_calculateParity(sensor);
    return TLx493D_A1B6_transferWriteRegisters(sensor);
}

void TLx493D_A1B6_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BX_MSB], &sensor->regDef[sensor->commonBitfields.BX_LSB], &valueX);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BY_MSB], &sensor->regDef[sensor->commonBitfields.BY_LSB], &valueY);
    concatBytes(sensor, &sensor->regDef[sensor->commonBitfields.BZ_MSB], &sensor->regDef[sensor->commonBitfields.BZ_LSB], &valueZ);

    *x = ((float) valueX) * GEN_1_MAG_FIELD_MULT;
    *y = ((float) valueY) * GEN_1_MAG_FIELD_MULT;
    *z = ((float) valueZ) * GEN_1_MAG_FIELD_MULT;
}

bool TLx493D_A1B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    if( gen_1_readRegisters(sensor) ) {
        TLx493D_A1B6_calculateFieldValues(sensor, x, y, z);
        return true;
    }
    return false;
}

void TLx493D_A1B6_calculateSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp) {
    TLx493D_A1B6_calculateFieldValues(sensor, x, y, z);
    TLx493D_A1B6_calculateTemperature(sensor, temp);
}

bool TLx493D_A1B6_getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp) {
    if( gen_1_readRegisters(sensor) ) {
        TLx493D_A1B6_calculateSensorValues(sensor, x, y, z, temp);
        return true;
    }
    return false;
}

bool TLx493D_A1B6_setIICAddress(Sensor_ts *sensor, TLx493D_StandardIICAddresses_te addr) {
    uint8_t bitfieldValue = 0;
    uint8_t deviceAddress = 0;

    switch (addr) {
        case GEN_1_STD_IIC_ADDR_00:
            bitfieldValue = 0b00;
            deviceAddress = 0xBC;
            break;

        case GEN_1_STD_IIC_ADDR_01:
            bitfieldValue = 0b01;
            deviceAddress = 0xB4;
            break;

        case GEN_1_STD_IIC_ADDR_10:
            bitfieldValue = 0b10;
            deviceAddress = 0x9C;
            break;

        case GEN_1_STD_IIC_ADDR_11:
            bitfieldValue = 0b11;
            deviceAddress = 0x94;
            break;
        
        default:
            return false;
    }

    gen_1_setBitfield(sensor, sensor->commonBitfields.IICADR, bitfieldValue);
    TLx493D_A1B6_calculateParity(sensor);
    bool ret = TLx493D_A1B6_transferWriteRegisters(sensor);

    setI2CParameters(sensor, deviceAddress);

    return ret;
}