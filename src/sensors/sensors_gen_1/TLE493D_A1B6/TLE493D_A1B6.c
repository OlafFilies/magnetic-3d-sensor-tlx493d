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
               P,
               IICaddr,
               INT,
               FAST,
               LOW,
               Temp_NEN, // remove: ask about duplicates
               LP,
               PT } TLE493D_A1B6_registerNames_te;

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
    // Write Registers
    {P,                 WRITE_MODE_e,   0x01, 0x80, 7, 1},
    {IICaddr,           WRITE_MODE_e,   0x01, 0x60, 5, 2},
    {INT,               WRITE_MODE_e,   0x01, 0x04, 2, 1},
    {FAST,              WRITE_MODE_e,   0x01, 0x02, 1, 1},
    {LOW,               WRITE_MODE_e,   0x01, 0x01, 0, 1},
    {Temp_NEN,          WRITE_MODE_e,   0x03, 0x80, 7, 1},
    {LP,                WRITE_MODE_e,   0x03, 0x40, 6, 1},
    {PT,                WRITE_MODE_e,   0x03, 0x20, 5, 1},
};

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
                                // .calculateParity       = TLE493D_A1B6_calculateParity,

                                .setDefaultConfig      = TLE493D_A1B6_setDefaultConfig,
                                // .updateRegisterMap     = TLE493D_A1B6_updateRegisterMap,
                              };


bool TLE493D_A1B6_init(Sensor_ts *sensor, SupportedComLibraryInterfaceTypes_te comLibIF) {
    // This sensor only supports I2C.
    if( comLibIF != I2C_e ) {
        assert(0);
        return false;
    }

    sensor->regMap            = (uint8_t *) malloc(sizeof(uint8_t) * TLE493D_A1B6_REGISTER_MAP_SIZE);
    sensor->regDef            = TLE493D_A1B6_regDef;
    sensor->functions         = &TLE493D_A1B6_commonFunctions;
    sensor->regMapSize        = TLE493D_A1B6_REGISTER_MAP_SIZE;
    sensor->sensorType        = TLE493D_A1B6_e;
    sensor->comIFType         = comLibIF;
    sensor->comLibIF          = &comLibIF_i2c;
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

    WriteRegisterValues[0]      =       0x00; //reserved
    WriteRegisterValues[1]      =       (TLE493D_A1B6_ODD_PARITY << sensor->regDef[P].offset)                        | 
                                        (TLE493D_A1B6_INT_ENABLE_default << sensor->regDef[INT].offset)              | 
                                        (TLE493D_A1B6_FAST_MODE_DISABLE_default << sensor->regDef[FAST].offset)      | 
                                        (TLE493D_A1B6_LOW_POWER_MODE_DISABLE_default << sensor->regDef[LOW].offset);
    WriteRegisterValues[2]      =       0x00; //reserved
    WriteRegisterValues[3]      =       (TLE493D_A1B6_Temp_ENABLE_default << sensor->regDef[Temp_NEN].offset)        |
                                        (TLE493D_A1B6_LOW_POWER_PERIOD_100MS_default << sensor->regDef[LP].offset)   |
                                        (TLE493D_A1B6_PARITY_TEST_ENABLE_default << sensor->regDef[PT].offset);  //remove: todo: make the reserve fields track regmaps read values. 
}


// note: make sure that the init function is called at reset to make sure the write default values are in sync.

bool TLE493D_A1B6_setDefaultConfig(Sensor_ts *sensor) {
    return TLE493D_A1B6_setWriteRegisterDefaultValues(sensor);
}

bool TLE493D_A1B6_disableTemperatureMeasurements(Sensor_ts *sensor) {
    uint8_t transBuffer[2];
    uint8_t bufLen = 2;

    WriteRegisterValues[sensor->regDef[Temp_NEN].address] = (WriteRegisterValues[sensor->regDef[Temp_NEN].address] & 
                                                            ~(sensor->regDef[Temp_NEN].mask)) | 
                                                            (TLE493D_A1B6_Temp_DISABLE << sensor->regDef->offset);

    transBuffer[0] = sensor->regDef[Temp_NEN].address;
    transBuffer[1] = WriteRegisterValues[sensor->regDef[Temp_NEN].address];

    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, TLE493D_A1B6_READ_REGISTER_MAP_SIZE);   
}

bool TLE493D_A1B6_enableTemperatureMeasurements(Sensor_ts *sensor) {
    uint8_t transBuffer[2];
    uint8_t bufLen = 2;

    WriteRegisterValues[sensor->regDef[Temp_NEN].address] = (WriteRegisterValues[sensor->regDef[Temp_NEN].address] & 
                                                            ~(sensor->regDef[Temp_NEN].mask)) | 
                                                            (TLE493D_A1B6_Temp_ENABLE_default << sensor->regDef->offset);

    transBuffer[0] = sensor->regDef[Temp_NEN].address;
    transBuffer[1] = WriteRegisterValues[sensor->regDef[Temp_NEN].address];
                                                                
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, TLE493D_A1B6_READ_REGISTER_MAP_SIZE);
}

bool TLE493D_A1B6_updateRegisterMap(Sensor_ts *sensor) {
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, TLE493D_A1B6_READ_REGISTER_MAP_SIZE);
}