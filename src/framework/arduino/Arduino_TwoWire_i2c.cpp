// std includes
#include <malloc.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
// #include "sensors_config_common.h"
// #include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_config_common.h"
// #include "sensors_gen_2_common.h"

// sensor specicifc includes
// #include "TLE493D_A2B6_config.h"
// #include "TLE493D_A2B6.h"

// project cpp includes
#include "arduino_defines.h"


extern "C" void frameworkDelayMicroseconds(uint32_t us);

      
extern "C" bool initIIC(Sensor_ts *sensor) {
    sensor->comLibObj.i2c_obj->wire->begin();
    frameworkDelayMicroseconds(30);
    return true;
}


extern "C" bool deinitIIC(Sensor_ts *sensor) {
    sensor->comLibObj.i2c_obj->wire->end();
    return true;
}


extern "C" bool transfer(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
     uint8_t   i2cAddress = sensor->comLibIFParams.i2c_params.address;
     TwoWire  *i2c        = sensor->comLibObj.i2c_obj->wire;

// log("addr :"); log(i2cAddress); log("\n");

    if( tx_len > 0 ) {
        i2c->beginTransmission(i2cAddress);

        uint8_t written = i2c->write(tx_buffer, tx_len);

// log("i2c written : "); log(written); log("\n");
// log("i2c tx_len : "); log(tx_len); log("\n");

        i2c->endTransmission(true);
        
        if( written != tx_len ) {
            return false;
        }
    }

    if( rx_len > 0 ) {
        uint8_t bytes_read = i2c->requestFrom(i2cAddress, rx_len);

// log("i2c bytes_read : "); log(bytes_read); log("\n");
// log("i2c rx_len : "); log(rx_len); log("\n");

        for(uint16_t i = 0; (i < rx_len) && (i2c->available() > 0); ++i) {
            rx_buffer[i] = i2c->read();
        }

        i2c->endTransmission(true);

        if( bytes_read != rx_len ) {
            return false;
        }
    }

    return true;
}


ComLibraryFunctions_ts  comLibIF_i2c = {
                                            .init     = { .i2c_init     = initIIC },
                                            .deinit   = { .i2c_deinit   = deinitIIC },
                                            .transfer = { .i2c_transfer = transfer },
                                       };


extern "C" void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr) {
    params->i2c_params.address = addr >> 1;
}


extern "C" void initI2CComLibIF(Sensor_ts *sensor, TwoWire &tw) {
    sensor->comLibObj.i2c_obj       = (I2CObject_ts *) malloc(sizeof(I2CObject_ts));
    sensor->comLibObj.i2c_obj->wire = &tw;

    sensor->comLibIF->init.i2c_init(sensor);
}


extern "C" void frameworkDelayMicroseconds(uint32_t us) {
    delayMicroseconds(us);
}
