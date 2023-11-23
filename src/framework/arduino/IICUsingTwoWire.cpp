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
    sensor->comLibObj.iic_obj->wire->init();
    return true;

}


extern "C" bool tlx493d_deinitIIC(TLx493D_t *sensor) {
    sensor->comLibObj.iic_obj->wire->deinit();
    return true;
}


extern "C" bool tlx493d_transferIIC(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
    return sensor->comLibObj.iic_obj->wire->transfer(sensor->comLibIFParams.iic_params.address >> 1, txBuffer, txLen, rxBuffer, rxLen);
}


TLx493D_ComLibraryFunctions_t  comLibIF_iic = {
                                            .init     = { .iic_init     = tlx493d_initIIC },
                                            .deinit   = { .iic_deinit   = tlx493d_deinitIIC },
                                            .transfer = { .iic_transfer = tlx493d_transferIIC },
                                       };

 // TODO: provide central function to set to null SPI/IIC objects !
bool tlx493d_initCommunication(TLx493D_t *sensor, TwoWireWrapper &tw, TLx493D_IICAddressType_t iicAdr) {
    sensor->comLibObj.iic_obj                 = (TLx493D_I2CObject_t *) malloc(sizeof(TLx493D_I2CObject_t));
    sensor->comLibObj.iic_obj->wire           = &tw;
    sensor->comLibIF                          = &comLibIF_iic;
    sensor->comLibIFParams.iic_params.address = sensor->functions->selectIICAddress(sensor, iicAdr);

    sensor->comLibIF->init.iic_init(sensor);
    return true;
}


// TODO: Provide function to delete TwoWire_Lib object from C in case it has been allocated explicitly by the following routine.
bool tlx493d_initCommunication(TLx493D_t *sensor, TwoWire &tw, TLx493D_IICAddressType_t iicAdr) {
    sensor->comLibObj.iic_obj                 = (TLx493D_I2CObject_t *) malloc(sizeof(TLx493D_I2CObject_t));
    sensor->comLibObj.iic_obj->wire           = new TwoWireWrapper(tw);
    sensor->comLibIF                          = &comLibIF_iic;
    sensor->comLibIFParams.iic_params.address = sensor->functions->selectIICAddress(sensor, iicAdr);

    sensor->comLibIF->init.iic_init(sensor);
    return true;
}


// TODO: remove reset related stuff
// /* Defines for SysTick */
// // multiplication factor for 1us resolution
// #define _MULTIPLICATION_FACTOR			(32u)
// //systick stop value
// #define _SYSTICK_CTRL_DISABLE_Msk		(0UL << SysTick_CTRL_ENABLE_Pos)

// /* Delay for 'time_us' us + t_call + t_cfg
//  * t_call ~= 1.2us
//  * t_cfg  ~= 1us
//  *
//  * As such, the actual delay time will be time_us + 2.2us
//  *
//  * Should support values up to 67s
//  * */
// void wait(uint32_t time_us)
// {
// 	// sanity check
// 	if (!time_us) {
// 		return;
// 	}
// 	SysTick->LOAD  = (uint32_t)((time_us * _MULTIPLICATION_FACTOR) - 1UL);
// 	SysTick->VAL   = 0UL;
// 	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
// 	while((SysTick->CTRL & 0x10000) == 0UL) {
// 	}
// 	SysTick->CTRL=_SYSTICK_CTRL_DISABLE_Msk;
// }


// void wait_transmit(void)
// {
// 	uint32_t count = 0;

// 	while (XMC_USIC_CH_GetTransmitBufferStatus(XMC_USIC0_CH1) == XMC_USIC_CH_TBUF_STATUS_BUSY)
// 	{
// 		wait(1);
// 		count++;

// 		if( count > 1000000 )
// 		{
// //			NVIC_SystemReset();
// 		}
// 	}
// }


// // TOD: get channel from Wire object !
// void iicReadFrom0xFF(void)
// {
// 	wait(10);
// // TODO: get channel from used Wire object !
// 	XMC_I2C_CH_MasterStart(XMC_USIC0_CH1, 0xFFU, XMC_I2C_CH_CMD_READ);
// 	wait(10);
// // TODO: get channel from used Wire object !
// 	XMC_I2C_CH_MasterStop(XMC_USIC0_CH1);
// 	wait_transmit();
// 	wait(10);
// }

// void iicWriteTo0x00(void)
// {
// 	wait(10);
// // TODO: get channel from used Wire object !
// 	XMC_I2C_CH_MasterStart(XMC_USIC0_CH1, 0x00U, XMC_I2C_CH_CMD_WRITE);
// 	wait(10);
// // TODO: get channel from used Wire object !
// 	XMC_I2C_CH_MasterStop(XMC_USIC0_CH1);
// 	wait_transmit();
// 	wait(10);
// }


// extern "C" void frameworkReset(TLx493D_t *sensor) {
//     // Reset sequence
//     // TLV, TLE, TLI
//     Serial.println("frameworkReset ...");
//     Serial.flush();

//     // iicReadFrom0xFF();
//     // Serial.println("iicReadFrom0xFF done.");
//     // Serial.flush();
//     // iicReadFrom0xFF();
//     // Serial.println("iicReadFrom0xFF done.");
//     // Serial.flush();

//     iicWriteTo0x00();
//     Serial.println("iicWriteTo0x00 done.");
//     Serial.flush();

//     // iicWriteTo0x00();
//     // Serial.println("iicWriteTo0x00 done.");
//     // Serial.flush();

//     wait(30);
//     // Serial.println("wait done.");
//     // Serial.flush();

//     Serial.println("frameworkReset done.");
// }
