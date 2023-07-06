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
#include "sensors_gen_2_config_common.h"
#include "sensors_gen_2_common.h"

// sensor specicifc includes
#include "TLE493D_A2B6_config.h"
#include "TLE493D_A2B6.h"


extern struct ComLibraryFunctions_ts comLibIF_i2c;


// framework functions
// TODO: replace by function pointers in comLibIF structure
extern void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr);
extern void frameworkReset(Sensor_ts *sensor);


/*
  Listing of all register names for this sensor.
  Used to index "TLE493D_A2B6_regDef" defined below, so index values must match !
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
               HWV } TLE493D_A2B6_registerNames_te;


static Register_ts TLE493D_A2B6_regDef[] = {
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
    { TL_mag,   READ_WRITE_MODE_e, 0x10, 0x06, 1, 2 },
    { CP,       READ_WRITE_MODE_e, 0x10, 0x01, 0, 1 },
    { FP,       READ_WRITE_MODE_e, 0x11, 0x80, 7, 1 },
    { IICaddr,  READ_WRITE_MODE_e, 0x11, 0x60, 5, 2 },
    { PR,       READ_WRITE_MODE_e, 0x11, 0x10, 4, 1 },
    { CA,       READ_WRITE_MODE_e, 0x11, 0x08, 3, 1 },
    { INT,      READ_WRITE_MODE_e, 0x11, 0x04, 2, 1 },
    { MODE,     READ_WRITE_MODE_e, 0x11, 0x03, 0, 2 },
    { PRD,      READ_WRITE_MODE_e, 0x13, 0x80, 7, 1 },
    { Type,     READ_MODE_e,       0x16, 0x30, 4, 2 },
    { HWV,      READ_MODE_e,       0x16, 0x0F, 0, 4 }
};


static CommonFunctions_ts TLE493D_A2B6_commonFunctions = {
                                .init                  = TLE493D_A2B6_init,
                                .deinit                = TLE493D_A2B6_deinit,

                                .getTemperature        = TLE493D_A2B6_getTemperature,
                                .updateGetTemperature  = TLE493D_A2B6_updateGetTemperature,

                                .getFieldValues        = TLE493D_A2B6_getFieldValues,
                                .updateGetFieldValues  = TLE493D_A2B6_updateGetFieldValues,

                                .reset                 = TLE493D_A2B6_reset,
                                .getDiagnosis          = TLE493D_A2B6_getDiagnosis,
                                .calculateParity       = TLE493D_A2B6_calculateParity,

                                .setDefaultConfig      = TLE493D_A2B6_setDefaultConfig,
                                .updateRegisterMap     = TLE493D_A2B6_updateRegisterMap,
                              };


// TODO: add parameter IICAddress or ad function to set address.
bool TLE493D_A2B6_init(Sensor_ts *sensor, SupportedComLibraryInterfaceTypes_te comLibIF) {
    // This sensor only supports I2C.
    if( comLibIF != I2C_e ) {
        assert(0);
        return false;
    }

    // regMap must be sensor specific, not sensor type specific, therefore malloc.
    sensor->regMap            = (uint8_t*) malloc(sizeof(uint8_t) * TLE493D_A2B6_REGISTER_MAP_SIZE);
    sensor->regDef            = TLE493D_A2B6_regDef;
    sensor->functions         = &TLE493D_A2B6_commonFunctions;
    sensor->regMapSize        = TLE493D_A2B6_REGISTER_MAP_SIZE;
    sensor->sensorType        = TLE493D_A2B6_e;
    sensor->comIFType         = comLibIF;
    sensor->comLibIF          = &comLibIF_i2c;
    sensor->comLibObj.i2c_obj = NULL;

    setI2CParameters(&sensor->comLibIFParams, GEN_2_STD_IIC_ADDR_WRITE_A0);

    return true;
}


bool TLE493D_A2B6_deinit(Sensor_ts *sensor) {
    free(sensor->regMap);
    free(sensor->comLibObj.i2c_obj);

    sensor->regMap            = NULL;
    sensor->comLibObj.i2c_obj = NULL;
    return true;
}


bool TLE493D_A2B6_updateGetTemperature(Sensor_ts *sensor, float *temp) {
    bool b = TLE493D_A2B6_updateRegisterMap(sensor);
    return b && TLE493D_A2B6_getTemperature(sensor, temp);
}


bool TLE493D_A2B6_getTemperature(Sensor_ts *sensor, float *temp) {
    int16_t value = 0;

    value   = ((uint16_t) sensor->regMap[sensor->regDef[TEMP_MSB].address]) << TLE493D_A2B6_REGISTER_SIZE_IN_BITS;
    value  |= (uint16_t) (sensor->regMap[sensor->regDef[TEMP_LSB].address] & sensor->regDef[TEMP_LSB].mask);
    value >>= (sensor->regDef[TEMP_LSB].offset - 2); // least significant 2 bits are implicit, therefore shift reduced by 2 !

    *temp = (float) ((((float) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF);

    return true;
}


bool TLE493D_A2B6_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    bool b = TLE493D_A2B6_updateRegisterMap(sensor);
    return b && TLE493D_A2B6_getFieldValues(sensor, x, y, z);
}


bool TLE493D_A2B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    valueX   = sensor->regMap[sensor->regDef[BX_MSB].address] << TLE493D_A2B6_REGISTER_SIZE_IN_BITS;
    valueX  |= ((sensor->regMap[sensor->regDef[BX_LSB].address] & sensor->regDef[BX_LSB].mask) << (TLE493D_A2B6_REGISTER_SIZE_IN_BITS - sensor->regDef[BX_LSB].numBits - sensor->regDef[BX_LSB].offset));
    valueX >>= (TLE493D_A2B6_REGISTER_SIZE_IN_BITS - sensor->regDef[BX_LSB].numBits);

    valueY   = sensor->regMap[sensor->regDef[BY_MSB].address] << TLE493D_A2B6_REGISTER_SIZE_IN_BITS;
    valueY  |= ((sensor->regMap[sensor->regDef[BY_LSB].address] & sensor->regDef[BY_LSB].mask) << (TLE493D_A2B6_REGISTER_SIZE_IN_BITS - sensor->regDef[BY_LSB].numBits - sensor->regDef[BY_LSB].offset));
    valueY >>= (TLE493D_A2B6_REGISTER_SIZE_IN_BITS - sensor->regDef[BY_LSB].numBits);

    valueZ   = sensor->regMap[sensor->regDef[BZ_MSB].address] << TLE493D_A2B6_REGISTER_SIZE_IN_BITS;
    valueZ  |= ((sensor->regMap[sensor->regDef[BZ_LSB].address] & sensor->regDef[BZ_LSB].mask) << (TLE493D_A2B6_REGISTER_SIZE_IN_BITS - sensor->regDef[BZ_LSB].numBits - sensor->regDef[BZ_LSB].offset));
    valueZ >>= (TLE493D_A2B6_REGISTER_SIZE_IN_BITS - sensor->regDef[BZ_LSB].numBits);

    *x = ((float) valueX) * GEN_2_MAG_FIELD_MULT;
    *y = ((float) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((float) valueZ) * GEN_2_MAG_FIELD_MULT;

    return true;
}


// TODO: not yet working !
bool TLE493D_A2B6_reset(Sensor_ts *sensor) {
//    assert(0);

    // sensor->i2c->wire->requestFrom(0xFF, 0);
    // sensor->i2c->wire->requestFrom(0xFF, 0);
    // sensor->i2c->wire->beginTransmission(0x00);
    // sensor->i2c->wire->endTransmission();
    // sensor->i2c->wire->beginTransmission(0x00);
    // sensor->i2c->wire->endTransmission();

    // // //If the uC has problems with this sequence: reset TwoWire-module.
    // sensor->i2c->wire->end();
    // sensor->i2c->wire->begin();

    // delayMicroseconds(30);


    // frameworkDelayMicroseconds(30);

    // uint8_t savedI2CAddress = sensor->comLibIFParams.i2c_params.address;
    // uint8_t resetValues[]   = { 0x00 };

    // // read 0 bytes from 0xFF
    // sensor->comLibIFParams.i2c_params.address = 0xFF;
    // sensor->comLibIF->init.i2c_init(sensor);
    // sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, resetValues, 0);
    // // sensor->comLibIF->init.i2c_init(sensor);
    // // sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, resetValues, 0);


    // // write 0 bytes to 0x00
    // // sensor->comLibIFParams.i2c_params.address = 0x00;
    // // sensor->comLibIF->init.i2c_init(sensor);
    // // sensor->comLibIF->transfer.i2c_transfer(sensor, resetValues, 0, NULL, 0);
    // // sensor->comLibIF->init.i2c_init(sensor);
    // // sensor->comLibIF->transfer.i2c_transfer(sensor, resetValues, 0, NULL, 0);
 
    // // stop  and restart to avoid hangups
    // sensor->comLibIF->deinit.i2c_deinit(sensor);
    // sensor->comLibIF->init.i2c_init(sensor);

    // frameworkDelayMicroseconds(30);

    // sensor->comLibIFParams.i2c_params.address = savedI2CAddress;



    frameworkReset(sensor);

    return true;
}


bool TLE493D_A2B6_getDiagnosis(Sensor_ts *sensor) {
    return true;
}


bool TLE493D_A2B6_calculateParity(Sensor_ts *sensor) {
    return true;
}


/**
 * - set 1-byte mode
 * - disable interrupts
 * - set parity flag
 * 
*/
void TLE493D_A2B6_get1ByteModeBuffer(uint8_t *buf, uint8_t *bufLen) {
    // old :
    // buf[0] = 0x11;
    // buf[1] = 0x94;

    buf[0]  = TLE493D_A2B6_regDef[PR].address;
    buf[1]  = TLE493D_A2B6_regDef[PR].mask | TLE493D_A2B6_regDef[INT].mask | TLE493D_A2B6_regDef[FP].mask;
    *bufLen = 2;
}


bool TLE493D_A2B6_enable1ByteMode(Sensor_ts *sensor) {
    uint8_t transBuffer[2];
    uint8_t bufLen = 0;

    TLE493D_A2B6_get1ByteModeBuffer(transBuffer, &bufLen);
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, sensor->regMapSize);
}


/**
 * - enable temeprature measurements
 * - hardcoded version also 
 *  - preserve all bits except parity and lower TL_mag bit
*/
void TLE493D_A2B6_getTemperatureMeasurementsBuffer(uint8_t *regMap, uint8_t *buf, uint8_t *bufLen) {
    // old :
    // buf[0] = 0x10;
    // buf[1] = regMap[0x10] & 0x7C;

    buf[0]  = TLE493D_A2B6_regDef[DT].address;
    buf[1]  = regMap[TLE493D_A2B6_regDef[DT].address] & ~(TLE493D_A2B6_regDef[DT].mask);
    *bufLen = 2;
}


bool TLE493D_A2B6_enableTemperatureMeasurements(Sensor_ts *sensor) {
    uint8_t transBuffer[2];
    uint8_t bufLen = 0;

    TLE493D_A2B6_getTemperatureMeasurementsBuffer(sensor->regMap, transBuffer, &bufLen);
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, sensor->regMapSize);
}


bool TLE493D_A2B6_setDefaultConfig(Sensor_ts *sensor) {
    bool b = TLE493D_A2B6_enable1ByteMode(sensor);
    b |= TLE493D_A2B6_enableTemperatureMeasurements(sensor);

    return b;
}


bool TLE493D_A2B6_updateRegisterMap(Sensor_ts *sensor) {
    // Currently only 1 interface is supported per sensor, either I2C or SPI for some 3rd generation sensors.
    // In case multiple interfaces are supported, switch according to IF type and call appropriate function.
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}
