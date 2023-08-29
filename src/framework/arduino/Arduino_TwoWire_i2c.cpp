// std includes
#include <malloc.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
// #include "sensors_config_common.h"
// #include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_common_defines.h"
// #include "sensors_gen_2_common.h"

// sensor specicifc includes
// #include "TLE493D_A2B6_config.h"
// #include "TLE493D_A2B6.h"

// project cpp includes
#include "arduino_defines.h"
#include "TwoWire_Lib.hpp"
#include "xmc_common.h"
#include "xmc_gpio.h"
#include "xmc_i2c.h"
#include <XMC1100.h>
#include "core_cm0.h"


#include "pal.h"


#ifndef USE_WIRE
extern "C" {
	#include "i2c.h"
}
#endif


extern "C" bool initIIC(Sensor_ts *sensor) {
    #ifdef USE_WIRE
        sensor->comLibObj.i2c_obj->wire->init();
    #else
	    I2C_init();
    #endif

    return true;

}


extern "C" bool deinitIIC(Sensor_ts *sensor) {
    #ifdef USE_WIRE
        sensor->comLibObj.i2c_obj->wire->deinit();
    #endif

    return true;
}


extern "C" bool transferIIC(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
    #ifdef USE_WIRE
        return sensor->comLibObj.i2c_obj->wire->transfer(sensor->comLibIFParams.i2c_params.address, tx_buffer, tx_len, rx_buffer, rx_len);
    #else

        if( tx_len > 0 ) {
    Serial.print("writing : "); Serial.print(tx_buffer[0]); Serial.print("   "); Serial.print(tx_buffer[1]); Serial.print("   "); Serial.flush();
        Serial.println(I2C_write(sensor->comLibIFParams.i2c_params.address, tx_buffer, tx_len)); Serial.flush();
        }

        if( rx_len > 0 ) {
    Serial.print("read : "); Serial.flush();
            Serial.println(I2C_read(sensor->comLibIFParams.i2c_params.address, rx_buffer, rx_len));
 
    Serial.print("rx : ");

            for(uint8_t i = 0; i < rx_len; ++i) {
                Serial.print(rx_buffer[i]);
                Serial.print("   ");
            }

            Serial.flush();
            Serial.println("");
        }

        return true;
    #endif
}


ComLibraryFunctions_ts  comLibIF_i2c = {
                                            .init     = { .i2c_init     = initIIC },
                                            .deinit   = { .i2c_deinit   = deinitIIC },
                                            .transfer = { .i2c_transfer = transferIIC },
                                       };


// TODO: change to use sensor as parameter to simplify the user interface across routines
extern "C" void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr) {
    #ifdef USE_WIRE
        params->i2c_params.address = addr >> 1;
    #else
        params->i2c_params.address = addr;
    #endif
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

    // Reset sequence
    // TLV, TLE, TLI
    Serial.println("frameworkReset ...");
    Serial.flush();
    // I2C_write_recover();
    // Serial.println("I2C_write_recover done.");
    // Serial.flush();
    // I2C_write_recover();
    // Serial.println("I2C_write_recover done.");
    // Serial.flush();

    I2C_write_reset();
    Serial.println("+I2C_write_reset done.");
    Serial.flush();

    I2C_write_reset();
    Serial.println("#I2C_write_reset done.");
    Serial.flush();

    wait(30);
    Serial.println("wait done.");
    Serial.flush();

//    NVIC_SystemReset();
// Serial.println("NVIC_SystemReset done.");

    // sensor->comLibIF->deinit.i2c_deinit(sensor);
    // sensor->comLibIF->deinit.i2c_deinit(sensor);
    // sensor->comLibIF->init.i2c_init(sensor);
    // sensor->comLibIF->init.i2c_init(sensor);
    // wait(30);

    Serial.println("frameworkReset done.");
}
