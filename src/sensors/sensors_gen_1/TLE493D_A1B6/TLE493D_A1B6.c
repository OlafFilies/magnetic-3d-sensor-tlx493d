// std includes
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_config_common.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_1_config_common.h"
#include "sensors_gen_1_common.h"

// sensor specicifc includes
#include "TLE493D_A1B6_config.h"
#include "TLE493D_A1B6.h"

//register enums
typedef enum {
    TLE493D_A1B6_Temp_ENABLE_default,
    TLE493D_A1B6_Temp_DISABLE
} TLE493D_A1B6_Reg_Temp_NEN;

typedef enum {
    TLE493D_A1B6_ODD_PARITY,
    TLE493D_A1B6_EVEN_PARITY
} TLE493D_A1B6_Reg_PARITY;

typedef enum {
    TLE493D_A1B6_CONFIG_00_default,
    TLE493D_A1B6_CONFIG_01,
    TLE493D_A1B6_CONFIG_10,
    TLE493D_A1B6_CONFIG_11
} TLE493D_A1B6_Reg_IICADDR;

typedef enum {
    TLE493D_A1B6_INT_ENABLE_default,
    TLE493D_A1B6_INT_DISABLE
} TLE493D_A1B6_Reg_INT;

typedef enum {
    TLE493D_A1B6_FAST_MODE_DISABLE_default,
    TLE493D_A1B6_FAST_MODE_ENABLE
} TLE493D_A1B6_Reg_FAST_MODE_NEN;

typedef enum {
    TLE493D_A1B6_LOW_POWER_MODE_DISABLE_default,
    TLE493D_A1B6_LOW_POWER_MODE_ENABLE
} TLE493D_A1B6_Reg_LOW_POWER_MODE_NEN;

typedef enum {
    TLE493D_A1B6_LOW_POWER_PERIOD_100MS_default,
    TLE493D_A1B6_LOW_POWER_PERIOD_12MS
} TLE493D_A1B6_Reg_LOW_POWER_PERIOD;

typedef enum {
    TLE493D_A1B6_PARITY_TEST_DISABLE,
    TLE493D_A1B6_PARITY_TEST_ENABLE_default
} TLE493D_A1B6_Reg_PARITY_TEST_NEN;

// structures for holding communication params
extern struct ComLibraryFunctions_ts comLibIF_i2c;

extern void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr); 
extern void frameworkDelayMicroseconds(uint32_t us);

/*
  Listing of all register names for this sensor.
  Used to index "TLE493D_A1B6_regDef" defined below, so index values must match !
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
               LOW,
               W_RES_2,
               Temp_NEN, // remove: ask about duplicates
               LP,
               PT,
               W_RES_3 } TLE493D_A1B6_registerNames_te;

Register_ts TLE493D_A1B6_regDef[] = {
    // Read registers
    {BX_MSB,            READ_MODE_e,    0x00, 0xFF, 0, 8},
    {BY_MSB,            READ_MODE_e,    0x01, 0xFF, 0, 8},
    {BZ_MSB,            READ_MODE_e,    0x02, 0xFF, 0, 8},
    {TEMP_MSB,          READ_MODE_e,    0x03, 0xF0, 4, 4},
    {FRM,               READ_MODE_e,    0x03, 0x0C, 2, 2},
    {CH,                READ_MODE_e,    0x03, 0x03, 0, 2},
    {BX_LSB,            READ_MODE_e,    0x04, 0xF0, 4, 4},
    {BY_LSB,            READ_MODE_e,    0x04, 0x0F, 0, 4},
    {TEST_Flag,         READ_MODE_e,    0x05, 0x40, 6, 1},
    {FF,                READ_MODE_e,    0x05, 0x20, 5, 1},
    {PD,                READ_MODE_e,    0x05, 0x10, 4, 1},
    {BZ_LSB,            READ_MODE_e,    0x05, 0x0F, 0, 4},
    {TEMP_LSB,          READ_MODE_e,    0x06, 0xFF, 0, 8},
    {R_RES_1,           READ_MODE_e,    0x07, 0x18, 3, 2},
    {R_RES_2,           READ_MODE_e,    0x08, 0xFF, 0, 8},
    {R_RES_3,           READ_MODE_e,    0x09, 0x1F, 0, 5},
    // Write Registers
    {W_RES_0,           WRITE_MODE_e,   0x00, 0xFF, 0, 8},
    {P,                 WRITE_MODE_e,   0x01, 0x80, 7, 1},
    {IICaddr,           WRITE_MODE_e,   0x01, 0x60, 5, 2},
    {W_RES_1,           WRITE_MODE_e,   0x01, 0x18, 3, 2},
    {INT,               WRITE_MODE_e,   0x01, 0x04, 2, 1},
    {FAST,              WRITE_MODE_e,   0x01, 0x02, 1, 1},
    {LOW,               WRITE_MODE_e,   0x01, 0x01, 0, 1},
    {W_RES_2,           WRITE_MODE_e,   0x02, 0xFF, 0, 8},
    {Temp_NEN,          WRITE_MODE_e,   0x03, 0x80, 7, 1},
    {LP,                WRITE_MODE_e,   0x03, 0x40, 6, 1},
    {PT,                WRITE_MODE_e,   0x03, 0x20, 5, 1},
    {W_RES_3,           WRITE_MODE_e,   0x03, 0x1F, 0, 5},
};

CommonFunctions_ts TLE493D_A1B6_commonFunctions = {
                                .init                  = TLE493D_A1B6_init,
                                .deinit                = TLE493D_A1B6_deinit,

                                // .getTemperature        = TLE493D_A1B6_getTemperature,
                                // .updateGetTemperature  = TLE493D_A1B6_updateGetTemperature,

                                .getFieldValues        = TLE493D_A1B6_getFieldValues,
                                .updateGetFieldValues  = TLE493D_A1B6_updateGetFieldValues,

                                // .reset                 = TLE493D_A1B6_reset,
                                // .getDiagnosis          = TLE493D_A1B6_getDiagnosis,
                                // .calculateParity       = TLE493D_A1B6_calculateParity,

                                .setDefaultConfig      = TLE493D_A1B6_setDefaultConfig,
                                .updateRegisterMap     = TLE493D_A1B6_updateRegisterMap,

                                .enableTemperature     = TLE493D_A1B6_enableTemperatureMeasurements,
                                .disableTemperature    = TLE493D_A1B6_disableTemperatureMeasurements,
                              };


bool TLE493D_A1B6_init(Sensor_ts *sensor) {
    sensor->regMap            = (uint8_t *) malloc(sizeof(uint8_t) * TLE493D_A1B6_REGISTER_MAP_SIZE);
    sensor->regDef            = TLE493D_A1B6_regDef;
    sensor->functions         = &TLE493D_A1B6_commonFunctions;
    sensor->regMapSize        = TLE493D_A1B6_REGISTER_MAP_SIZE;
    sensor->sensorType        = TLE493D_A1B6_e;
    sensor->comIFType         = I2C_e;
    sensor->comLibIF          = NULL;
    sensor->comLibObj.i2c_obj = NULL;

    setI2CParameters(&sensor->comLibIFParams, GEN_1_STD_IIC_ADDR);

    return true;
}

bool TLE493D_A1B6_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);

    sensor->regMap            = NULL;
    sensor->comLibObj.i2c_obj = NULL; //remove: ask if this to be done for all members of Sensors struct

    return true;
}

void TLE493D_A1B6_setReservedRegisterValues(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, W_RES_1, gen_1_returnBitfield(sensor, R_RES_1));
    gen_1_setBitfield(sensor, W_RES_2, gen_1_returnBitfield(sensor, R_RES_2));
    gen_1_setBitfield(sensor, W_RES_3, gen_1_returnBitfield(sensor, R_RES_3));
}

void TLE493D_A1B6_setPowerDownMode(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, FAST, TLE493D_A1B6_FAST_MODE_DISABLE_default);
    gen_1_setBitfield(sensor, LOW, TLE493D_A1B6_LOW_POWER_MODE_DISABLE_default);
    gen_1_setBitfield(sensor, LP, TLE493D_A1B6_LOW_POWER_PERIOD_100MS_default);
}

void TLE493D_A1B6_setMasterControlledMode(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, FAST, TLE493D_A1B6_FAST_MODE_ENABLE);
    gen_1_setBitfield(sensor, LOW, TLE493D_A1B6_LOW_POWER_MODE_ENABLE);
    gen_1_setBitfield(sensor, LP, TLE493D_A1B6_LOW_POWER_PERIOD_12MS);
}

// note: make sure that the init function is called at reset to make sure the write default values are in sync.

bool TLE493D_A1B6_setDefaultConfig(Sensor_ts *sensor) {
    
    bool ret = true;
    // read READ register values
    ret = TLE493D_A1B6_updateRegisterMap(sensor);
    
    // set WRITE register values to 0x00
    for (uint8_t addr=0; addr<TLE493D_A1B6_WRITE_REGISTERS_MAX_COUNT; addr++)
        sensor->regMap[addr+TLE493D_A1B6_WRITE_REGISTERS_OFFSET] = 0x00;


    // todo: reset sensor here

    // set WRITE reserved register values to READ reserved register values
    TLE493D_A1B6_setReservedRegisterValues(sensor);

    //set to POWERDOWNMODE at boot
    TLE493D_A1B6_setPowerDownMode(sensor);

    // enable parity test and write to registers 
    ret &= TLE493D_A1B6_enableParityTest(sensor);

    // set to MASTERCONTROLLEDMODE to start measurement
    TLE493D_A1B6_setMasterControlledMode(sensor);

    // calculate parity
    TLE493D_A1B6_calculateParity(sensor);

    // write out register map to registers
    ret &= TLE493D_A1B6_transferWriteRegisters(sensor);

    // update register map
    ret &= TLE493D_A1B6_updateRegisterMap(sensor);

    return ret;  
}

bool TLE493D_A1B6_disableTemperatureMeasurements(Sensor_ts *sensor) {
    bool ret = true;
    
    gen_1_setBitfield(sensor, Temp_NEN, TLE493D_A1B6_Temp_DISABLE);
    TLE493D_A1B6_calculateParity(sensor);
    ret = TLE493D_A1B6_transferWriteRegisters(sensor);
    
    return ret;                                                           
}

bool TLE493D_A1B6_enableTemperatureMeasurements(Sensor_ts *sensor) {
    bool ret = true;

    gen_1_setBitfield(sensor, Temp_NEN, TLE493D_A1B6_Temp_ENABLE_default);
    TLE493D_A1B6_calculateParity(sensor);
    ret = TLE493D_A1B6_transferWriteRegisters(sensor);

    return ret; 
}

bool TLE493D_A1B6_transferWriteRegisters(Sensor_ts *sensor) {
    bool retn = true;
    
    retn &= sensor->comLibIF->transfer.i2c_transfer(sensor, sensor->regMap+TLE493D_A1B6_WRITE_REGISTERS_OFFSET, TLE493D_A1B6_WRITE_REGISTERS_MAX_COUNT, NULL, 0);
    
    return retn;
}

bool TLE493D_A1B6_updateRegisterMap(Sensor_ts *sensor) {
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, TLE493D_A1B6_READ_REGISTERS_MAX_COUNT);
}

// parity is calculated for all the WRITE register, including the parity bit
void TLE493D_A1B6_calculateParity(Sensor_ts *sensor) {
    uint8_t result = 0x00;

    //set parity as EVEN first
    gen_1_setBitfield(sensor, P, TLE493D_A1B6_EVEN_PARITY);

    // calculate bitwise XOR for all WRITE registers
    for (uint8_t addr = 0x00; addr<TLE493D_A1B6_WRITE_REGISTERS_MAX_COUNT; addr++) {
        result ^= sensor->regMap[addr+TLE493D_A1B6_WRITE_REGISTERS_OFFSET];
    }

    // store the XOR result of all bits at the LSB
    result ^= (result >> 1);
    result ^= (result >> 2);
    result ^= (result >> 4);

    // then set calculated parity
    gen_1_setBitfield(sensor, P, result & 0x01);
}

bool TLE493D_A1B6_getTemperature(Sensor_ts *sensor, float *temp) {

    int16_t value = gen_1_concat_values((sensor->regMap[sensor->regDef[TEMP_MSB].address] & sensor->regDef[TEMP_MSB].mask) >> sensor->regDef[TEMP_MSB].offset, sensor->regMap[sensor->regDef[TEMP_LSB].address], false);

    *temp = (float)(((float)value - GEN_1_TEMP_OFFSET) * GEN_1_TEMP_MULT) ;

    return true;
}

bool TLE493D_A1B6_updateGetTemperature(Sensor_ts *sensor, float *temp) {
    bool b = TLE493D_A1B6_updateRegisterMap(sensor);
    return b & TLE493D_A1B6_getTemperature(sensor, temp);
}

bool TLE493D_A1B6_enableParityTest(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, PT, TLE493D_A1B6_PARITY_TEST_ENABLE_default);
    TLE493D_A1B6_calculateParity(sensor);
    return TLE493D_A1B6_transferWriteRegisters(sensor);
}

bool TLE493D_A1B6_disableParityTest(Sensor_ts *sensor) {
    gen_1_setBitfield(sensor, PT, TLE493D_A1B6_PARITY_TEST_DISABLE);
    TLE493D_A1B6_calculateParity(sensor);
    return TLE493D_A1B6_transferWriteRegisters(sensor);
}

bool TLE493D_A1B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    valueX = gen_1_concat_values(sensor->regMap[sensor->regDef[BX_MSB].address], sensor->regMap[sensor->regDef[BX_LSB].address], true);

    valueY = gen_1_concat_values(sensor->regMap[sensor->regDef[BY_MSB].address], sensor->regMap[sensor->regDef[BY_LSB].address], true);

    valueZ = gen_1_concat_values(sensor->regMap[sensor->regDef[BZ_MSB].address], sensor->regMap[sensor->regDef[BZ_LSB].address], true);

    *x = ((float) valueX) * GEN_1_MAG_FIELD_MULT;
    *y = ((float) valueY) * GEN_1_MAG_FIELD_MULT;
    *z = ((float) valueZ) * GEN_1_MAG_FIELD_MULT;

    return true;
}

bool TLE493D_A1B6_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    bool b = TLE493D_A1B6_updateRegisterMap(sensor);
    return b && TLE493D_A1B6_getFieldValues(sensor, x, y, z);
}
