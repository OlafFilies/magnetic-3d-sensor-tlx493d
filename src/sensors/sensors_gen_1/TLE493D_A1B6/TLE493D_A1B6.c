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
               T_read,
               FF,
               PD,
               BZ_LSB,
               TEMP_LSB,
               P,
               IICaddr,
               INT,
               FAST,
               LOW,
               T_write, // remove: ask about duplicates
               LP,
               PT } TLE493D_A1B6_registerNames_te;


Register_ts TLE493D_A1B6_regDef[TLE493D_A1B6_REGISTER_MAP_SIZE] = {
    // Read registers
    { BX_MSB, READ_MODE_e, 0x00, 0xFF, 0 },
    { BY_MSB, READ_MODE_e, 0x01, 0xFF, 0 },
    { BZ_MSB, READ_MODE_e, 0x02, 0xFF, 0 },
    { TEMP_MSB, READ_MODE_e, 0x03, 0xF0, 4 },
    { FRM, READ_MODE_e, 0x03, 0x0C, 2},
    { CH, READ_MODE_e, 0x03, 0x03, 0 },
    { BX_LSB, READ_MODE_e, 0x04, 0xF0, 4 },
    { BY_LSB, READ_MODE_e, 0x04, 0x0F, 0 },
    { BX_MSB, READ_MODE_e, 0x00, 0xFF, 0 },
    { T_read, READ_MODE_e, 0x05, 0x40, 6 },
    { FF, READ_MODE_e, 0x05, 0x20, 5 },
    { PD, READ_MODE_e, 0x05, 0x10, 4 },
    { BZ_LSB, READ_MODE_e, 0x05, 0x0F, 0 },
    { TEMP_LSB, READ_MODE_e, 0x06, 0xFF, 0 },
    // Write Registers
    { P, WRITE_MODE_e, 0x01, 0x80, 7 },
    { IICaddr, WRITE_MODE_e, 0x01, 0x60, 5 },
    { INT, WRITE_MODE_e, 0x01, 0x04, 2 },
    { FAST, WRITE_MODE_e, 0x01, 0x02, 1 },
    { LOW, WRITE_MODE_e, 0x01, 0x01, 0 },
    { T_write, WRITE_MODE_e, 0x03, 0x80, 7 },
    { LP, WRITE_MODE_e, 0x03, 0x40, 6 },
    { PT, WRITE_MODE_e, 0x03, 0x20, 5 },
};

CommonFunctions_ts TLE493D_A1B6_commonFunctions = {
                                .init                  = TLE493D_A1B6_init,
                                .deinit                = TLE493D_A1B6_deinit,

                                .getTemperature        = TLE493D_A1B6_getTemperature,
                                .updateGetTemperature  = TLE493D_A1B6_updateGetTemperature,

                                .getFieldValues        = TLE493D_A1B6_getFieldValues,
                                .updateGetFieldValues  = TLE493D_A1B6_updateGetFieldValues,

                                .reset                 = TLE493D_A1B6_reset,
                                .getDiagnosis          = TLE493D_A1B6_getDiagnosis,
                                .calculateParity       = TLE493D_A1B6_calculateParity,

                                .setDefaultConfig      = TLE493D_A1B6_setDefaultConfig,
                                .updateRegisterMap     = TLE493D_A1B6_updateRegisterMap,
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
