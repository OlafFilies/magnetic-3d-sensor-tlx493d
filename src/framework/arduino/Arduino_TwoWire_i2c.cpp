// std includes
#include <malloc.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "sensor_types.h"

// common to same generation of sensors
#include "sensors_gen_2_common_defines.h"

// sensor specific includes

// project cpp includes
#include "arduino_defines.h"
#include "TwoWire_Lib.hpp"


extern "C" bool initIIC(Sensor_ts *sensor) {
    sensor->comLibObj.i2c_obj->wire->init();
    return true;

}


extern "C" bool deinitIIC(Sensor_ts *sensor) {
    sensor->comLibObj.i2c_obj->wire->deinit();
    return true;
}


extern "C" bool transferIIC(Sensor_ts *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
    return sensor->comLibObj.i2c_obj->wire->transfer(sensor->comLibIFParams.i2c_params.address, txBuffer, txLen, rxBuffer, rxLen);
}


ComLibraryFunctions_ts  comLibIF_i2c = {
                                            .init     = { .i2c_init     = initIIC },
                                            .deinit   = { .i2c_deinit   = deinitIIC },
                                            .transfer = { .i2c_transfer = transferIIC },
                                       };


// TODO: change to use sensor as parameter to simplify the user interface across routines
// extern "C" void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr) {
extern "C" void setI2CParameters(Sensor_ts *sensor, uint8_t addr) {
    sensor->comLibIFParams.i2c_params.address = addr >> 1;
}


bool initComLibIF(Sensor_ts *sensor, TwoWire_Lib<TwoWire> &tw) {
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
// extern "C" 
bool initComLibIF(Sensor_ts *sensor, TwoWire &tw) {
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


/* Defines for SysTick */
// multiplication factor for 1us resolution
#define _MULTIPLICATION_FACTOR			(32u)
//systick stop value
#define _SYSTICK_CTRL_DISABLE_Msk		(0UL << SysTick_CTRL_ENABLE_Pos)

/* Delay for 'time_us' us + t_call + t_cfg
 * t_call ~= 1.2us
 * t_cfg  ~= 1us
 *
 * As such, the actual delay time will be time_us + 2.2us
 *
 * Should support values up to 67s
 * */
void wait(uint32_t time_us)
{
	// sanity check
	if (!time_us) {
		return;
	}
	SysTick->LOAD  = (uint32_t)((time_us * _MULTIPLICATION_FACTOR) - 1UL);
	SysTick->VAL   = 0UL;
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	while((SysTick->CTRL & 0x10000) == 0UL) {
	}
	SysTick->CTRL=_SYSTICK_CTRL_DISABLE_Msk;
}


void wait_transmit(void)
{
	uint32_t count = 0;

	while (XMC_USIC_CH_GetTransmitBufferStatus(XMC_USIC0_CH1) == XMC_USIC_CH_TBUF_STATUS_BUSY)
	{
		wait(1);
		count++;

		if( count > 1000000 )
		{
//			NVIC_SystemReset();
		}
	}
}


// TOD: get channel from Wire object !
void i2cReadFrom0xFF(void)
{
	wait(10);
// TODO: get channel from used Wire object !
	XMC_I2C_CH_MasterStart(XMC_USIC0_CH1, 0xFFU, XMC_I2C_CH_CMD_READ);
	wait(10);
// TODO: get channel from used Wire object !
	XMC_I2C_CH_MasterStop(XMC_USIC0_CH1);
	wait_transmit();
	wait(10);
}

void i2cWriteTo0x00(void)
{
	wait(10);
// TODO: get channel from used Wire object !
	XMC_I2C_CH_MasterStart(XMC_USIC0_CH1, 0x00U, XMC_I2C_CH_CMD_WRITE);
	wait(10);
// TODO: get channel from used Wire object !
	XMC_I2C_CH_MasterStop(XMC_USIC0_CH1);
	wait_transmit();
	wait(10);
}


extern "C" void frameworkReset(Sensor_ts *sensor) {
    // Reset sequence
    // TLV, TLE, TLI
    Serial.println("frameworkReset ...");
    Serial.flush();

    // i2cReadFrom0xFF();
    // Serial.println("i2cReadFrom0xFF done.");
    // Serial.flush();
    // i2cReadFrom0xFF();
    // Serial.println("i2cReadFrom0xFF done.");
    // Serial.flush();

    i2cWriteTo0x00();
    Serial.println("i2cWriteTo0x00 done.");
    Serial.flush();

    // i2cWriteTo0x00();
    // Serial.println("i2cWriteTo0x00 done.");
    // Serial.flush();

    wait(30);
    // Serial.println("wait done.");
    // Serial.flush();

    Serial.println("frameworkReset done.");
}
