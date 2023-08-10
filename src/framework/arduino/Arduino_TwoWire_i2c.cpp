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
#include "xmc_common.h"
#include "xmc_gpio.h"
#include "xmc_i2c.h"
#include <XMC1100.h>
#include "core_cm0.h"

extern "C" {
	#include "i2c.h"
}


extern "C" bool initIIC(Sensor_ts *sensor) {
    // sensor->comLibObj.i2c_obj->wire->init();
	I2C_init();
    return true;
}


extern "C" bool deinitIIC(Sensor_ts *sensor) {
    // sensor->comLibObj.i2c_obj->wire->deinit();
    return true;
}


// extern "C" int32_t I2C_read(uint8_t addr, uint8_t *data, uint8_t count)
// {
// 	int32_t status = I2C_STATUS_OK;
// Serial.println("I2C_read"); Serial.flush();

// 	// make method atomic
// 	__disable_irq();

//   	XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION
// 						| XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION
// 						| XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED
// 						| XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED);

// 	// switch to I2C mode
// 	// (from GPIO edge Interrupt)
// 	I2C_mode_normal();

// 	// send addressing frame
// 	XMC_I2C_CH_MasterStart(XMC_I2C0_CH1, addr, XMC_I2C_CH_CMD_READ);
// // Serial.println("1"); Serial.flush();
                                                    
// 	while((XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
// 	{
// 		// Serial.println("waiting for ACK");
// 		delayMicroseconds(5);
// 	}
// // Serial.println("2"); Serial.flush();

// 	XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
//    	// XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFFU);

// 	for(uint8_t n = 0; n < (count - 1); ++n) {
// 		XMC_I2C_CH_MasterReceiveAck(XMC_I2C0_CH1);

// 		while((XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & (XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0U)
// 		{
// 		// Serial.println("waiting for REC");
// 		}

// 		XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);
// 		data[n] = XMC_I2C_CH_GetReceivedData(XMC_I2C0_CH1);
// 	}
// // Serial.println("3"); Serial.flush();

// 	XMC_I2C_CH_MasterReceiveNack(XMC_I2C0_CH1);

// 	// this loop solves the incorrect reads present in Arduino
// 	while((XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & (XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0U)
// 	{
// 		// Serial.println("waiting for REC 2");
// 	}
// // Serial.println("4"); Serial.flush();

// 	XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);

// 	if( count > 0 ) {
// 		data[count - 1] = XMC_I2C_CH_GetReceivedData(XMC_I2C0_CH1);
// 	}

// 	XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);
// // Serial.println("5"); Serial.flush();

// 	while ( XMC_USIC_CH_GetTransmitBufferStatus(XMC_I2C0_CH1) == XMC_USIC_CH_TBUF_STATUS_BUSY )
// 	{
// 		// Serial.println("waiting for BUSY");
// 	}
// // Serial.println("6"); Serial.flush();

// 	XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_USIC_CH_TBUF_STATUS_BUSY);
//     // XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFFU);

// 	I2C_mode_gpio_input();
//     __enable_irq();
// // Serial.println("7"); Serial.flush();
// 	return status;
// }


// extern "C" int32_t I2C_write(uint8_t addr, const uint8_t* data, uint8_t count)
// {
// 	int32_t status = I2C_STATUS_OK;
// Serial.println("I2C_write"); Serial.flush();

// 	// check peripheral not already used
// 	__disable_irq();

//     XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFF);
// 	I2C_mode_normal();

// 	// send addressing frame
// 	XMC_I2C_CH_MasterStart(XMC_I2C0_CH1, addr, XMC_I2C_CH_CMD_WRITE);

// Serial.println("1"); Serial.flush();
                                                    
// 	while((XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
// 	{
// 		Serial.println("waiting for ACK"); Serial.flush();
// 	}
// Serial.println("2"); Serial.flush();

// 	// send data
// 	XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED | XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED);

// 	for(uint8_t n = 0; n < count; ++n) {
// 		XMC_I2C_CH_MasterTransmit(XMC_I2C0_CH1, data[n]);

// 		while((XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
// 		{
// Serial.println("waiting for ACK 2"); Serial.flush();
// 		}

// 		XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
// 	}
// Serial.println("3"); Serial.flush();

// 	while (!XMC_USIC_CH_TXFIFO_IsEmpty(XMC_I2C0_CH1))
// 	{
// Serial.println("XMC_USIC_CH_TXFIFO_IsEmpty"); Serial.flush();
// 	}
// Serial.println("4"); Serial.flush();

// 	XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);

// 	while ( XMC_USIC_CH_GetTransmitBufferStatus(XMC_I2C0_CH1) == XMC_USIC_CH_TBUF_STATUS_BUSY )
// 	{
// Serial.println("waiting for XMC_USIC_CH_TBUF_STATUS_BUSY"); Serial.flush();
// 	}
// Serial.println("5"); Serial.flush();

// 	I2C_mode_gpio_input();
// 	XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED | XMC_I2C_CH_STATUS_FLAG_TRANSMIT_BUFFER_INDICATION);
//     XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFF);
// 	__enable_irq();
// Serial.println("6"); Serial.flush();
// 	return status;
// }


extern "C" bool transferIIC(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
	if( tx_len > 0 ) {
Serial.print("writing : "); Serial.print(tx_buffer[0]); Serial.print(tx_buffer[1]); Serial.flush();
    Serial.println(I2C_write(sensor->comLibIFParams.i2c_params.address, tx_buffer, tx_len)); Serial.flush();
	}

	if( rx_len > 0 ) {
Serial.print("read : "); Serial.flush();
		Serial.println(I2C_read(sensor->comLibIFParams.i2c_params.address, rx_buffer, rx_len)); Serial.flush();
	}

	return true;

    // return sensor->comLibObj.i2c_obj->wire->transfer(sensor->comLibIFParams.i2c_params.address, tx_buffer, tx_len, rx_buffer, rx_len);
}


ComLibraryFunctions_ts  comLibIF_i2c = {
                                            .init     = { .i2c_init     = initIIC },
                                            .deinit   = { .i2c_deinit   = deinitIIC },
                                            .transfer = { .i2c_transfer = transferIIC },
                                       };


// TODO: change to use sensor as parameter to simplify the user interface across routines
extern "C" void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr) {
    params->i2c_params.address = addr;
    // params->i2c_params.address = addr >> 1;
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
//     Serial.println("frameworkReset ...");
//     I2C_write_recover();
// Serial.println("I2C_write_recover done.");
//     I2C_write_recover();
// Serial.println("I2C_write_recover done.");

//     I2C_write_reset();
//  Serial.println("I2C_write_reset done.");
//     I2C_write_reset();
// Serial.println("I2C_write_reset done.");

//     wait(30);
// Serial.println("wait done.");

//    NVIC_SystemReset();
// Serial.println("NVIC_SystemReset done.");

    // sensor->comLibIF->deinit.i2c_deinit(sensor);
    // sensor->comLibIF->deinit.i2c_deinit(sensor);
    // sensor->comLibIF->init.i2c_init(sensor);
    // sensor->comLibIF->init.i2c_init(sensor);
    // wait(30);

    Serial.println("frameworkReset done.");
}
