// std includes
#include <malloc.h>
#include <stddef.h>

// XMC related includes
#include "xmc_gpio.h"
#include "xmc_i2c.h"

// project c includes
// common to all sensors
#include "mtb_defines.h"
#include "sensor_types.h"
// #include "sensors_config_common.h"
// #include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_config_common.h"
// #include "sensors_gen_2_common.h"

// sensor specicifc includes
// #include "TLE493D_A2B6_config.h"
// #include "TLE493D_A2B6.h"


bool I2CInitFunc(Sensor_ts *sensor) {
    XMC_GPIO_CONFIG_t i2c_scl = {
        .mode = XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT2,
        .output_strength = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM
    };


    XMC_GPIO_CONFIG_t i2c_sda = {
        .mode = XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT2,
        .output_strength = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM
    };

    XMC_I2C_CH_CONFIG_t i2c_cfg = {
        .baudrate = 115200U,
        .address = 0U
    };

    I2CObject_ts  *i2c_obj = sensor->comLibObj.i2c_obj;
    XMC_USIC_CH_t *channel = i2c_obj->channel;

    XMC_I2C_CH_Init(channel, &i2c_cfg);

    XMC_I2C_CH_SetInputSource(channel, XMC_I2C_CH_INPUT_SCL, i2c_obj->sourceSCL);
    XMC_I2C_CH_SetInputSource(channel, XMC_I2C_CH_INPUT_SDA, i2c_obj->sourceSDA);
//    XMC_USIC_CH_SetInputSource(channel, XMC_USIC_CH_INPUT_DX0, XMC_I2C_config->input_source_dx0);
//    XMC_USIC_CH_SetInputSource(channel, XMC_USIC_CH_INPUT_DX1, XMC_I2C_config->input_source_dx1);



    /* configure i2c tx fifo */
    XMC_USIC_CH_TXFIFO_Configure(channel, 16, XMC_USIC_CH_FIFO_SIZE_16WORDS, (uint32_t) 15);

    /* configure i2c rx fifo */
    XMC_USIC_CH_RXFIFO_Configure(channel, 0, XMC_USIC_CH_FIFO_SIZE_16WORDS, (uint32_t) 15);
    
    // XMC_USIC_CH_SetInterruptNodePointer(channel, XMC_USIC_CH_INTERRUPT_NODE_POINTER_PROTOCOL, XMC_I2C_config->protocol_irq_service_request);
    // NVIC_SetPriority((IRQn_Type)XMC_I2C_config->protocol_irq_num, 3U);
    // NVIC_EnableIRQ((IRQn_Type)XMC_I2C_config->protocol_irq_num);

    // XMC_I2C_CH_EnableEvent(XMC_I2C_config->channel, (uint32_t) (XMC_I2C_CH_EVENT_NACK | XMC_I2C_CH_EVENT_DATA_LOST | XMC_I2C_CH_EVENT_ARBITRATION_LOST | XMC_I2C_CH_EVENT_ERROR));



    XMC_I2C_CH_Start(channel);

    XMC_GPIO_Init(i2c_obj->portSCL, i2c_obj->pinSCL, &i2c_scl);
    XMC_GPIO_Init(i2c_obj->portSDA, i2c_obj->pinSDA, &i2c_sda);

	XMC_I2C_CH_ClearStatusFlag(channel, 0xFFFFFFFF);

    return true;
}
 

bool I2CDeinitFunc(Sensor_ts *sensor) {
    I2CObject_ts  *i2c_obj = sensor->comLibObj.i2c_obj;
    XMC_USIC_CH_t *channel = i2c_obj->channel;

    // XMC_I2C_CH_Stop(channel);



	// // if((channel->CCR & USIC_CH_CCR_MODE_Msk) == XMC_USIC_CH_OPERATING_MODE_I2C)
	// // {
        XMC_I2C_CH_Stop(channel);

		// XMC_USIC_CH_SetInputSource(channel, XMC_USIC_CH_INPUT_DX0, XMC_INPUT_A);
		// XMC_USIC_CH_SetInputSource(channel, XMC_USIC_CH_INPUT_DX1, XMC_INPUT_A);

		// NVIC_DisableIRQ((IRQn_Type)XMC_I2C_config->protocol_irq_num);
		// XMC_I2C_CH_DisableEvent(channel, (uint32_t) (XMC_I2C_CH_EVENT_NACK | XMC_I2C_CH_EVENT_DATA_LOST | XMC_I2C_CH_EVENT_ARBITRATION_LOST | XMC_I2C_CH_EVENT_ERROR));
		
	    // Reset buffer and clear all Status Flags
		XMC_USIC_CH_TXFIFO_Flush(channel);
		XMC_USIC_CH_RXFIFO_Flush(channel);
		XMC_USIC_CH_SetTransmitBufferStatus(channel, XMC_USIC_CH_TBUF_STATUS_SET_IDLE);

    	XMC_I2C_CH_ClearStatusFlag(channel, 0xFFFFFFFF);


		/* Disable i2c tx fifo */
		XMC_USIC_CH_TXFIFO_Configure(channel, 16, XMC_USIC_CH_FIFO_DISABLED, (uint32_t) 15);

		/* Disable i2c rx fifo */
		XMC_USIC_CH_RXFIFO_Configure(channel, 0, XMC_USIC_CH_FIFO_DISABLED, (uint32_t) 15);
	// // }



    return true;
}


bool I2CTransferFunc(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
    XMC_USIC_CH_t *channel = sensor->comLibObj.i2c_obj->channel;


    if( tx_buffer != NULL ) {
    	XMC_I2C_CH_ClearStatusFlag(channel, 0xFFFFFFFF);
        XMC_I2C_CH_MasterStart(channel, sensor->comLibIFParams.i2c_params.address, XMC_I2C_CH_CMD_WRITE);

        while((XMC_I2C_CH_GetStatusFlag(channel) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
        {
        }

        XMC_I2C_CH_ClearStatusFlag(channel, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

        for(uint8_t n = 0; n < tx_len; ++n) {
            XMC_I2C_CH_MasterTransmit(channel, tx_buffer[n]);

            while((XMC_I2C_CH_GetStatusFlag(channel) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
            {
            }

            XMC_I2C_CH_ClearStatusFlag(channel, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
        }

        while (!XMC_USIC_CH_TXFIFO_IsEmpty(channel))
        {
        }

        XMC_I2C_CH_MasterStop(channel);

        while ( XMC_USIC_CH_GetTransmitBufferStatus(channel) == XMC_USIC_CH_TBUF_STATUS_BUSY )
		{
		}
    }


    if( rx_buffer != NULL ) {
      	XMC_I2C_CH_ClearStatusFlag(channel, 0xFFFFFFFF);
        XMC_I2C_CH_MasterStart(channel, sensor->comLibIFParams.i2c_params.address, XMC_I2C_CH_CMD_READ);

        // this loop solves the incorrect reads present in Arduino
        while((XMC_I2C_CH_GetStatusFlag(channel) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
        {
        }

        XMC_I2C_CH_ClearStatusFlag(channel, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

        for(uint8_t n = 0; n < (rx_len - 1); ++n) {
            XMC_I2C_CH_MasterReceiveAck(channel);

    		while((XMC_I2C_CH_GetStatusFlag(channel) & (XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0U)
    		{
    		}

    		XMC_I2C_CH_ClearStatusFlag(channel, XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);
            rx_buffer[n] = XMC_I2C_CH_GetReceivedData(channel);
        }

		XMC_I2C_CH_MasterReceiveNack(channel);

        // this loop solves the incorrect reads present in Arduino
		while((XMC_I2C_CH_GetStatusFlag(channel) & (XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0U)
		{
		}

		XMC_I2C_CH_ClearStatusFlag(channel, XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);

        if( rx_len > 0 ) {
		    rx_buffer[rx_len - 1] = XMC_I2C_CH_GetReceivedData(channel);
        }

        XMC_I2C_CH_MasterStop(channel);

        while ( XMC_USIC_CH_GetTransmitBufferStatus(channel) == XMC_USIC_CH_TBUF_STATUS_BUSY )
		{
		}

        printf("rx_buffer : "); 
        for(int i = 0; i < 23; ++i) {
            printf("%x   ", rx_buffer[i]);
        }
        printf("\n");
    }

    return true;
}


ComLibraryFunctions_ts  comLibIF_i2c = {
                                           .init.i2c_init         = I2CInitFunc,
                                           .deinit.i2c_deinit     = I2CDeinitFunc,
                                           .transfer.i2c_transfer = I2CTransferFunc,
                                       };


void setI2CParameters(ComLibraryParameters_ts *params, uint8_t addr) {
    params->i2c_params.address = addr;
}


void initI2CComLibIF(Sensor_ts *sensor, XMC_USIC_CH_t *const channel,
                     const uint8_t sourceSDA, const uint8_t sourceSCL,
                     XMC_GPIO_PORT_t *const portSDA, const uint8_t pinSDA,
                     XMC_GPIO_PORT_t *const portSCL, const uint8_t pinSCL) {

    // Need to dynamically allocate object, such that different sensor may use different channels and ports/pins
    sensor->comLibObj.i2c_obj = (I2CObject_ts *) malloc(sizeof(I2CObject_ts));

    sensor->comLibObj.i2c_obj->channel   = channel;
    sensor->comLibObj.i2c_obj->sourceSDA = sourceSDA;
    sensor->comLibObj.i2c_obj->sourceSCL = sourceSCL;
    sensor->comLibObj.i2c_obj->portSDA   = portSDA;
    sensor->comLibObj.i2c_obj->pinSDA    = pinSDA;
    sensor->comLibObj.i2c_obj->portSCL   = portSCL;
    sensor->comLibObj.i2c_obj->pinSCL    = pinSCL;

    sensor->comLibIF->init.i2c_init(sensor);
}


void frameworkDelayMicroseconds(uint32_t us) {
    XMC_DelayUs(us);
}


void frameworkReset(Sensor_ts *sensor) {
    XMC_USIC_CH_t *channel = sensor->comLibObj.i2c_obj->channel;

    XMC_I2C_CH_MasterStart(channel, 0xFF, XMC_I2C_CH_CMD_READ);
    // XMC_I2C_CH_MasterReceiveNack(channel);
    // XMC_I2C_CH_MasterStop(channel);


    I2CInitFunc(sensor);
    XMC_I2C_CH_MasterStart(channel, 0xFF, XMC_I2C_CH_CMD_READ);
	// XMC_I2C_CH_MasterReceiveNack(channel);
    // XMC_I2C_CH_MasterStop(channel);


    I2CInitFunc(sensor);
    XMC_I2C_CH_MasterStart(channel, 0x00, XMC_I2C_CH_CMD_WRITE);
    // XMC_I2C_CH_MasterStop(channel);


    I2CInitFunc(sensor);
    XMC_I2C_CH_MasterStart(channel, 0x00, XMC_I2C_CH_CMD_WRITE);
    // XMC_I2C_CH_MasterStop(channel);


    I2CDeinitFunc(sensor);
    I2CInitFunc(sensor);

    // sensor->comLibObj.i2c_obj->wire->requestFrom(0xFF, 0);
    // sensor->comLibObj.i2c_obj->wire->requestFrom(0xFF, 0);
    // sensor->comLibObj.i2c_obj->wire->beginTransmission(0x00);
    // sensor->comLibObj.i2c_obj->wire->endTransmission();
    // sensor->comLibObj.i2c_obj->wire->beginTransmission(0x00);
    // sensor->comLibObj.i2c_obj->wire->endTransmission();

    // // //If the uC has problems with this sequence: reset TwoWire-module.
    // sensor->comLibObj.i2c_obj->wire->end();
    // sensor->comLibObj.i2c_obj->wire->begin();

    XMC_DelayUs(30);
}
