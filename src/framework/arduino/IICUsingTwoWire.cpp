// std includes
#include <malloc.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"

// common to same generation of sensors

// sensor specific includes

// project cpp includes
#include "types.hpp"
#include "IICUsingTwoWire.hpp"
#include "Logger.h"


extern "C" bool tlx493d_initIIC(TLx493D_t *sensor) {
    sensor->comLibObj.i2c_obj->wire->init();
    return true;

}


extern "C" bool tlx493d_deinitIIC(TLx493D_t *sensor) {
    sensor->comLibObj.i2c_obj->wire->deinit();
    return true;
}


extern "C" bool tlx493d_transferIIC(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
    return sensor->comLibObj.i2c_obj->wire->transfer(sensor->comLibIFParams.i2c_params.address >> 1, txBuffer, txLen, rxBuffer, rxLen);
}


TLx493D_ComLibraryFunctions_t  comLibIF_i2c = {
                                            .init     = { .i2c_init     = tlx493d_initIIC },
                                            .deinit   = { .i2c_deinit   = tlx493d_deinitIIC },
                                            .transfer = { .i2c_transfer = tlx493d_transferIIC },
                                       };


// // TODO: change to use sensor as parameter to simplify the user interface across routines
// extern "C" void tlx493d_setI2CParameters(TLx493D_t *sensor, uint8_t addr) {
//     sensor->comLibIFParams.i2c_params.address = addr >> 1;
// }


bool tlx493d_initCommunication(TLx493D_t *sensor, TwoWireWrapper<TwoWire> &tw, TLx493D_IICAddressType_t iicAdr) {
    sensor->comLibObj.i2c_obj                 = (TLx493D_I2CObject_t *) malloc(sizeof(TLx493D_I2CObject_t));
    sensor->comLibObj.i2c_obj->wire           = &tw;
    sensor->comLibIF                          = &comLibIF_i2c;
    sensor->comLibIFParams.i2c_params.address = sensor->functions->selectIICAddress(sensor, iicAdr);

    sensor->comLibIF->init.i2c_init(sensor);
    return true;
}


// TODO: Provide function to delete TwoWire_Lib object from C in case it has been allocated explicitly by the following routine.
bool tlx493d_initCommunication(TLx493D_t *sensor, TwoWire &tw, TLx493D_IICAddressType_t iicAdr) {
    sensor->comLibObj.i2c_obj                 = (TLx493D_I2CObject_t *) malloc(sizeof(TLx493D_I2CObject_t));
    sensor->comLibObj.i2c_obj->wire           = new TwoWireWrapper<TwoWire>(tw);
    sensor->comLibIF                          = &comLibIF_i2c;
    sensor->comLibIFParams.i2c_params.address = sensor->functions->selectIICAddress(sensor, iicAdr);

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


extern "C" void frameworkReset(TLx493D_t *sensor) {
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
