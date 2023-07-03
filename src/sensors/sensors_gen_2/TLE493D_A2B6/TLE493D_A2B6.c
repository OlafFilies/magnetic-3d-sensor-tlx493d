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


extern void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr);


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


Register_ts TLE493D_A2B6_regDef[TLE493D_A2B6_REGISTER_MAP_SIZE] = {
    { BX_MSB,   READ_MODE_e, 0x00, 0xFF, 0 },
    { BY_MSB,   READ_MODE_e, 0x01, 0xFF, 0 },
    { BZ_MSB,   READ_MODE_e, 0x02, 0xFF, 0 }, 
    { TEMP_MSB, READ_MODE_e, 0x03, 0xFF, 0 },
    { BX_LSB,   READ_MODE_e, 0x04, 0xF0, 4 },
    { BY_LSB,   READ_MODE_e, 0x04, 0x0F, 0 },
    { TEMP_LSB, READ_MODE_e, 0x05, 0xC0, 6 },
    { ID,       READ_MODE_e, 0x05, 0x30, 4 },
    { BZ_LSB,   READ_MODE_e, 0x05, 0x0F, 0 },
};


CommonFunctions_ts TLE493D_A2B6_commonFunctions = {
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


bool TLE493D_A2B6_init(Sensor_ts *sensor, SupportedComLibraryInterfaceTypes_te comLibIF) {
    // This sensor only supports I2C.
    if( comLibIF != I2C_e ) {
        assert(0);
        return false;
    }

    sensor->regMap            = (uint8_t *) malloc(sizeof(uint8_t) * TLE493D_A2B6_REGISTER_MAP_SIZE);
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

    value = ((uint16_t) sensor->regMap[sensor->regDef[TEMP_MSB].address]) << 8;
    value |= (uint16_t) (sensor->regMap[sensor->regDef[TEMP_LSB].address] & sensor->regDef[TEMP_LSB].mask);
    value >>= 4;

    *temp = (float) ((((float) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF);

    return true;
}


bool TLE493D_A2B6_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    bool b = TLE493D_A2B6_updateRegisterMap(sensor);
    return b && TLE493D_A2B6_getFieldValues(sensor, x, y, z);
}


bool TLE493D_A2B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    valueX = sensor->regMap[sensor->regDef[BX_MSB].address] << 8;
    valueX |= (sensor->regMap[sensor->regDef[BX_LSB].address] & sensor->regDef[BX_LSB].mask);
    valueX >>= 4;

    valueY = sensor->regMap[sensor->regDef[BY_MSB].address] << 8;
    valueY |= (sensor->regMap[sensor->regDef[BY_LSB].address] & sensor->regDef[BY_LSB].mask) << sensor->regDef[BY_LSB].offset;
    valueY >>= 4;

    valueZ = sensor->regMap[sensor->regDef[BZ_MSB].address] << 8;
    valueZ |= (sensor->regMap[sensor->regDef[BZ_LSB].address] & sensor->regDef[BZ_LSB].mask) << sensor->regDef[BZ_LSB].offset;
    valueZ >>= 4;

    *x = ((float) valueX) * GEN_2_MAG_FIELD_MULT;
    *y = ((float) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((float) valueZ) * GEN_2_MAG_FIELD_MULT;

    return true;
}


// TODO: not yet working !
bool TLE493D_A2B6_reset(Sensor_ts *sensor) {
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


//     uint8_t resetValues[] = { 0x00 };

//     sensor->comLibIFParams.i2c_params.address = 0xFF;
//     sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, resetValues, 0);
//     sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, resetValues, 0);

//     sensor->comLibIFParams.i2c_params.address = 0x00;
//     sensor->comLibIF->transfer.i2c_transfer(sensor, resetValues, 0, NULL, 0);
//     sensor->comLibIF->transfer.i2c_transfer(sensor, resetValues, 0, NULL, 0);
 
//     sensor->comLibIF->deinit.i2c_deinit(sensor);
//    sensor->comLibIF->init.i2c_init(sensor);

//    frameworkDelayMicroseconds(30);

    return true;
}


bool TLE493D_A2B6_getDiagnosis(Sensor_ts *sensor) {
    return true;
}


bool TLE493D_A2B6_calculateParity(Sensor_ts *sensor) {
    return true;
}


void TLE493D_A2B6_get1ByteModeBuffer(uint8_t *buf, uint8_t *bufLen) {
    buf[0] = 0x11;
    buf[1] = 0x94;
    *bufLen = 2;
}


bool TLE493D_A2B6_enable1ByteMode(Sensor_ts *sensor) {
    uint8_t transBuffer[sensor->regMapSize];
    uint8_t bufLen = 0;

    TLE493D_A2B6_get1ByteModeBuffer(transBuffer, &bufLen);
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, sensor->regMapSize);
}


void TLE493D_A2B6_getTemperatureMeasurementsBuffer(uint8_t *regMap, uint8_t *buf, uint8_t *bufLen) {
    buf[0] = 0x10;
    buf[1] = regMap[16] & 0x7C;
    *bufLen = 2;
}


bool TLE493D_A2B6_enableTemperatureMeasurements(Sensor_ts *sensor) {
    uint8_t transBuffer[sensor->regMapSize];
    uint8_t bufLen = 0;

    TLE493D_A2B6_getTemperatureMeasurementsBuffer(sensor->regMap, transBuffer, &bufLen);
    return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, sensor->regMap, sensor->regMapSize);
}


bool TLE493D_A2B6_setDefaultConfig(Sensor_ts *sensor) {
    bool b = TLE493D_A2B6_enable1ByteMode(sensor);
    return b && TLE493D_A2B6_enableTemperatureMeasurements(sensor);
}


bool TLE493D_A2B6_updateRegisterMap(Sensor_ts *sensor) {
    // Currently only 1 interface is supported per sensor, either I2C or SPI for some 3rd generation sensors.
    // In case multiple interfaces are supported, switch according to IF type and call appropriate function.
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}
