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
    TLE493D_A1B6_PARITY_TEST_ENABLE_default,
    TLE493D_A1B6_PARITY_TEST_DISABLE
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
               W_RES_1,
               P,
               IICaddr,
               W_RES_2,
               INT,
               FAST,
               LOW,
               W_RES_3,
               Temp_NEN, // remove: ask about duplicates
               LP,
               PT,
               W_RES_4 } TLE493D_A1B6_registerNames_te;

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
    {R_RES_1,           READ_MODE_e,    0x07, 0xFF, 0, 8},
    {R_RES_2,           READ_MODE_e,    0x08, 0xFF, 0, 8},
    {R_RES_3,           READ_MODE_e,    0x09, 0xFF, 0, 8},
    // Write Registers
    {W_RES_1,           WRITE_MODE_e,   0x00, 0xFF, 0, 8},
    {P,                 WRITE_MODE_e,   0x01, 0x80, 7, 1},
    {IICaddr,           WRITE_MODE_e,   0x01, 0x60, 5, 2},
    {W_RES_2,           WRITE_MODE_e,   0x01, 0x18, 3, 2},
    {INT,               WRITE_MODE_e,   0x01, 0x04, 2, 1},
    {FAST,              WRITE_MODE_e,   0x01, 0x02, 1, 1},
    {LOW,               WRITE_MODE_e,   0x01, 0x01, 0, 1},
    {W_RES_3,           WRITE_MODE_e,   0x02, 0xFF, 0, 8},
    {Temp_NEN,          WRITE_MODE_e,   0x03, 0x80, 7, 1},
    {LP,                WRITE_MODE_e,   0x03, 0x40, 6, 1},
    {PT,                WRITE_MODE_e,   0x03, 0x20, 5, 1},
    {W_RES_4,           WRITE_MODE_e,   0x03, 0x1F, 0, 5},
};

//remove: note: this array has to be put inside the sensor struct so that a unique array var is created for every instance of TLE_A1B6
uint8_t WriteRegisterValues[TLE493D_A1B6_WRITE_REGISTERS_MAX_COUNT]; //malloc ? YES but link it to a struct such as Sensor_ts or put it inside init 

CommonFunctions_ts TLE493D_A1B6_commonFunctions = {
                                .init                  = TLE493D_A1B6_init,
                                .deinit                = TLE493D_A1B6_deinit,

                                // .getTemperature        = TLE493D_A1B6_getTemperature,
                                // .updateGetTemperature  = TLE493D_A1B6_updateGetTemperature,

                                // .getFieldValues        = TLE493D_A1B6_getFieldValues,
                                // .updateGetFieldValues  = TLE493D_A1B6_updateGetFieldValues,

                                // .reset                 = TLE493D_A1B6_reset,
                                // .getDiagnosis          = TLE493D_A1B6_getDiagnosis,
                                .calculateParity       = TLE493D_A1B6_calculateParity,

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

bool TLE493D_A1B6_setWriteRegisterDefaultValues(Sensor_ts *sensor) {

    bool ret = true;

    // TODO: call update register map here to refresh READ register values
    ret &= TLE493D_A1B6_updateRegisterMap(sensor);

    // some WRITE register values are linked to READ register values. This is ensured using appropriate READ sensor->regMap[] values. 
    WriteRegisterValues[0]      =       0x00; //reserved and not linked to READ registers
    WriteRegisterValues[1]      =       (TLE493D_A1B6_ODD_PARITY << sensor->regDef[P].offset)                                                                   |
                                        (TLE493D_A1B6_CONFIG_00_default << sensor->regDef[IICaddr].offset)                                                      |
                                        (((sensor->regMap[sensor->regDef[R_RES_1].address]) & sensor->regDef[W_RES_2].mask) << sensor->regDef[W_RES_2].offset)  | 
                                        (TLE493D_A1B6_INT_DISABLE << sensor->regDef[INT].offset)                                                         | 
                                        (TLE493D_A1B6_FAST_MODE_DISABLE_default << sensor->regDef[FAST].offset)                                                 | 
                                        (TLE493D_A1B6_LOW_POWER_MODE_DISABLE_default << sensor->regDef[LOW].offset);
    WriteRegisterValues[2]      =       (((sensor->regMap[sensor->regDef[R_RES_2].address]) & sensor->regDef[W_RES_3].mask) << sensor->regDef[W_RES_3].offset);
    WriteRegisterValues[3]      =       (TLE493D_A1B6_Temp_ENABLE_default << sensor->regDef[Temp_NEN].offset)                                                   |
                                        (TLE493D_A1B6_LOW_POWER_PERIOD_100MS_default << sensor->regDef[LP].offset)                                              |
                                        (TLE493D_A1B6_PARITY_TEST_ENABLE_default << sensor->regDef[PT].offset)                                                  |
                                        (((sensor->regMap[sensor->regDef[R_RES_3].address]) & sensor->regDef[W_RES_4].mask) << sensor->regDef[W_RES_4].offset); 


    TLE493D_A1B6_calculateParity(sensor);

    ret &= TLE493D_A1B6_loadWriteRegisters(sensor);

    return ret;

}

// note: make sure that the init function is called at reset to make sure the write default values are in sync.

bool TLE493D_A1B6_setDefaultConfig(Sensor_ts *sensor) {
    return TLE493D_A1B6_setWriteRegisterDefaultValues(sensor);
}

bool TLE493D_A1B6_disableTemperatureMeasurements(Sensor_ts *sensor) {
    bool ret = true;
    uint8_t transBuffer[2];
    uint8_t bufLen = 2;

    WriteRegisterValues[sensor->regDef[Temp_NEN].address] = (WriteRegisterValues[sensor->regDef[Temp_NEN].address] & 
                                                            ~(sensor->regDef[Temp_NEN].mask)) | 
                                                            (TLE493D_A1B6_Temp_DISABLE << sensor->regDef[Temp_NEN].offset);

    transBuffer[0] = sensor->regDef[Temp_NEN].address;
    transBuffer[1] = WriteRegisterValues[sensor->regDef[Temp_NEN].address];

    TLE493D_A1B6_calculateParity(sensor);

    ret &= TLE493D_A1B6_loadWriteRegisters(sensor);

    return ret;                                                            
}

bool TLE493D_A1B6_enableTemperatureMeasurements(Sensor_ts *sensor) {
    bool ret = true;
    uint8_t transBuffer[2];
    uint8_t bufLen = 2;

    WriteRegisterValues[sensor->regDef[Temp_NEN].address] = (WriteRegisterValues[sensor->regDef[Temp_NEN].address] & 
                                                            ~(sensor->regDef[Temp_NEN].mask)) | 
                                                            (TLE493D_A1B6_Temp_ENABLE_default << sensor->regDef[Temp_NEN].offset);

    transBuffer[0] = sensor->regDef[Temp_NEN].address;
    transBuffer[1] = WriteRegisterValues[sensor->regDef[Temp_NEN].address];

    TLE493D_A1B6_calculateParity(sensor);
                                                                
    ret &= TLE493D_A1B6_loadWriteRegisters(sensor);

    return ret; 
}

// remove: note: all register writes must be atomic i.e. all 5 registers must be written to when anything is changed 
// why? since all of the WRITE regs affect the parity. so whenever parity is recalculated, everything must be written to be sure. 
bool TLE493D_A1B6_loadWriteRegisters(Sensor_ts *sensor) {
    bool retn = true;
    uint8_t transBuffer[2];
    uint8_t bufLen = 2;
    

    for (uint8_t addr = 0x00; addr<TLE493D_A1B6_WRITE_REGISTERS_MAX_COUNT; addr++) {
        transBuffer[0] = addr;
        transBuffer[1] = WriteRegisterValues[addr];

        retn &= sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, NULL, 0);
    }

    return retn;
}

bool TLE493D_A1B6_updateRegisterMap(Sensor_ts *sensor) {
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, TLE493D_A1B6_READ_REGISTERS_MAX_COUNT);
}

// parity is calculated for all the WRITE register, including the parity bit
void TLE493D_A1B6_calculateParity(Sensor_ts *sensor) {
    uint8_t result = 0x00;
    uint8_t parity = 0x00;

    // first set parity as even, reason: this P bit is also considered in parity calculation.
    WriteRegisterValues[sensor->regDef[P].address] = (WriteRegisterValues[sensor->regDef[P].address] & 
                                                            ~(sensor->regDef[P].mask)) | 
                                                            (TLE493D_A1B6_EVEN_PARITY << sensor->regDef[P].offset);

    // calculate bitwise XOR for all WRITE registers
    for (uint8_t addr = 0x00; addr<TLE493D_A1B6_WRITE_REGISTERS_MAX_COUNT; addr++) {
        result ^= WriteRegisterValues[addr];
    }

    // uncomment and check
    // while(result > 0) {
    //     parity ^= (result & 0x01);
    //     result >= 1;
    // }

    
    result ^= (result >> 1);
    result ^= (result >> 2);
    result ^= (result >> 4);

    // then set calculated parity
    WriteRegisterValues[sensor->regDef[P].address] = (WriteRegisterValues[sensor->regDef[P].address] & 
                                                            ~(sensor->regDef[P].mask)) | 
                                                            ((result & 0x01) << sensor->regDef[P].offset);
}