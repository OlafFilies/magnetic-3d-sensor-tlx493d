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
#include "TwoWire_Lib.hpp"


extern "C" void frameworkDelayMicroseconds(uint32_t us);

      
extern "C" bool initIIC(Sensor_ts *sensor) {
    sensor->comLibObj.i2c_obj->wire->init();
    return true;
}


extern "C" bool deinitIIC(Sensor_ts *sensor) {
    sensor->comLibObj.i2c_obj->wire->deinit();
    return true;
}


extern "C" bool transferIIC(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
    return sensor->comLibObj.i2c_obj->wire->transfer(sensor->comLibIFParams.i2c_params.address, tx_buffer, tx_len, rx_buffer, rx_len);
}


ComLibraryFunctions_ts  comLibIF_i2c = {
                                            .init     = { .i2c_init     = initIIC },
                                            .deinit   = { .i2c_deinit   = deinitIIC },
                                            .transfer = { .i2c_transfer = transferIIC },
                                       };


extern "C" void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr) {
    params->i2c_params.address = addr >> 1;
}


bool initI2CComLibIF(Sensor_ts *sensor, TwoWire_Lib<TwoWire> &tw) {
    if( sensor->comIFType != I2C_e ) {
        return false;
    }

    // Need to dynamically allocate object, such that different sensor may use different TwoWire objects (Wire, Wire1, Wire2, ...)
    sensor->comLibObj.i2c_obj       = (I2CObject_ts *) malloc(sizeof(I2CObject_ts));
    sensor->comLibObj.i2c_obj->wire = &tw;
    sensor->comLibIF                = &comLibIF_i2c;

    sensor->comLibIF->init.i2c_init(sensor);

    return true;
}


// TODO: Provide function to delete TwoWire_Lib object from C in case it has been allocated explicitly by the following routine.
extern "C" bool initI2CComLibIF(Sensor_ts *sensor, TwoWire &tw) {
    if( sensor->comIFType != I2C_e ) {
        return false;
    }

    // Need to dynamically allocate object, such that different sensor may use different TwoWire objects (Wire, Wire1, Wire2, ...)
    sensor->comLibObj.i2c_obj       = (I2CObject_ts *) malloc(sizeof(I2CObject_ts));
    sensor->comLibObj.i2c_obj->wire = new TwoWire_Lib<TwoWire>(tw);
    sensor->comLibIF                = &comLibIF_i2c;

    sensor->comLibIF->init.i2c_init(sensor);
    return true;
}


// extern "C" void frameworkDelayMicroseconds(uint32_t us) {
//     delayMicroseconds(us);
// }


extern "C" void frameworkReset(Sensor_ts *sensor) {
    // // sensor->comLibObj.i2c_obj->wire->requestFrom(0xFF, 0);
    // // sensor->comLibObj.i2c_obj->wire->requestFrom(0xFF, 0);

    // sensor->comLibObj.i2c_obj->wire->beginTransmission(0x00);
    // sensor->comLibObj.i2c_obj->wire->endTransmission();
    // sensor->comLibObj.i2c_obj->wire->beginTransmission(0x00);
    // sensor->comLibObj.i2c_obj->wire->endTransmission();

    // // //If the uC has problems with this sequence: reset TwoWire-module.
    // sensor->comLibObj.i2c_obj->wire->end();
    // sensor->comLibObj.i2c_obj->wire->begin();

    // delayMicroseconds(30);
}
