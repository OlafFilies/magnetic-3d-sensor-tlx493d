// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_config_common.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_config_common.h"
#include "sensors_gen_2_common.h"

// sensor specicifc includes
#include "TLE493D_W2B6_config.h"
#include "TLE493D_W2B6.h"

extern struct ComLibraryFunctions_ts comLibIF_i2c;
extern void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr);


/*
  Listing of all register names for this sensor.
  Used to index "TLE493D_W2B6_regDef" defined below, so index values must match !
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
               TL_mag,
               CP,
               FP,
               IICaddr,
               PR,
               CA,
               INT,
               MODE,
               PRD,
               Type,
               HWV } TLE493D_W2B6_registerNames_te;


Register_ts TLE493D_W2B6_regDef[GEN_2_REG_MAP_SIZE] = {
    {BX_MSB,    READ_MODE_e,    0x00,   0xFF,   0},
    {BY_MSB,    READ_MODE_e,    0x01,   0xFF,   0},
    {BZ_MSB,    READ_MODE_e,    0x02,   0xFF,   0},
    {TEMP_MSB,  READ_MODE_e,    0x03,   0xFF,   0},
    {BX_LSB,    READ_MODE_e,    0x04,   0xF0,   4},
    {BY_LSB,    READ_MODE_e,    0x04,   0x0F,   0},
    {TEMP_LSB,  READ_MODE_e,    0x05,   0xC0,   6},
    {ID,        READ_MODE_e,    0x05,   0x30,   4},
    {BZ_LSB,    READ_MODE_e,    0x05,   0x0F,   0},
    {P,         READ_MODE_e,    0x06,   0x80,   7},
    {FF,        READ_MODE_e,    0x06,   0x40,   6},
    {CF,        READ_MODE_e,    0x06,   0x20,   5},
    {T,         READ_MODE_e,    0x06,   0x10,   4},
    {PD3,       READ_MODE_e,    0x06,   0x08,   3},
    {PD0,       READ_MODE_e,    0x06,   0x04,   2},
    {FRM,       READ_MODE_e,    0x06,   0x03,   0},
    {DT,        WRITE_MODE_e,   0x10,   0x80,   7},
    {AM,        WRITE_MODE_e,   0x10,   0x40,   6},
    {TRIG,      WRITE_MODE_e,   0x10,   0x30,   4},
    {X2,        WRITE_MODE_e,   0x10,   0x08,   3},
    {TL_mag,    WRITE_MODE_e,   0x10,   0x06,   1},
    {CP,        WRITE_MODE_e,   0x10,   0x01,   0},
    {FP,        WRITE_MODE_e,   0x11,   0x80,   7},
    {IICaddr,   WRITE_MODE_e,   0x11,   0x60,   5},
    {PR,        WRITE_MODE_e,   0x11,   0x10,   4},
    {CA,        WRITE_MODE_e,   0x11,   0x08,   3},
    {INT,       WRITE_MODE_e,   0x11,   0x04,   2},
    {MODE,      WRITE_MODE_e,   0x11,   0x03,   0},
    {PRD,       WRITE_MODE_e,   0x13,   0x80,   7},
    {Type,      WRITE_MODE_e,   0x16,   0x30,   4},
    {HWV,       WRITE_MODE_e,   0x16,   0x0F,   0}
};


CommonFunctions_ts TLE493D_W2B6_commonFunctions = {
                                .init                  = TLE493D_W2B6_init,
};

bool TLE493D_W2B6_init(Sensor_ts *sensor, SupportedComLibraryInterfaceTypes_te comLibIF) {
    // This sensor only supports I2C.
    if( comLibIF != I2C_e ) {
        assert(0);
        return false;
    }

    sensor->regMap            = (uint8_t*)malloc(sizeof(uint8_t) * GEN_2_REG_MAP_SIZE);
    sensor->regDef            = TLE493D_W2B6_regDef;
    sensor->functions         = &TLE493D_W2B6_commonFunctions;
    sensor->regMapSize        = GEN_2_REG_MAP_SIZE;
    sensor->sensorType        = TLE493D_W2B6_e;
    sensor->comIFType         = comLibIF;
    sensor->comLibIF          = &comLibIF_i2c;
    sensor->comLibObj.i2c_obj = NULL;

    setI2CParameters(&sensor->comLibIFParams, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return true;
}