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


    XMC_I2C_CH_Init(sensor->comLibObj->channel, &i2c_cfg);

    XMC_I2C_CH_SetInputSource(sensor->comLibObj->channel, XMC_I2C_CH_INPUT_SCL, sensor->comLibObj->sourceSCL);
    XMC_I2C_CH_SetInputSource(sensor->comLibObj->channel, XMC_I2C_CH_INPUT_SDA, sensor->comLibObj->sourceSDA);

    XMC_I2C_CH_Start(sensor->comLibObj->channel);

    XMC_GPIO_Init(sensor->comLibObj->portSCL, sensor->comLibObj->pinSCL, &i2c_scl);
    XMC_GPIO_Init(sensor->comLibObj->portSDA, sensor->comLibObj->pinSDA, &i2c_sda);

    return true;
}
 

bool I2CDeinitFunc(Sensor_ts *sensor) {
    return true;
}


bool I2CTransferFunc(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len) {
    if( tx_buffer != NULL ) {
        XMC_I2C_CH_MasterStart(XMC_I2C1_CH1, sensor->comLibIFParams.i2c_params.address, XMC_I2C_CH_CMD_WRITE);

        while((XMC_I2C_CH_GetStatusFlag(XMC_I2C1_CH1) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
        {
        }

        XMC_I2C_CH_ClearStatusFlag(XMC_I2C1_CH1,(uint32_t) XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

        for(size_t n = 0; n < tx_len; ++n) {
            XMC_I2C_CH_MasterTransmit(XMC_I2C1_CH1, tx_buffer[n]);

            while((XMC_I2C_CH_GetStatusFlag(XMC_I2C1_CH1) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
            {
            }

            XMC_I2C_CH_ClearStatusFlag(XMC_I2C1_CH1,(uint32_t)XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
        }

        while (!XMC_USIC_CH_TXFIFO_IsEmpty(XMC_I2C1_CH1))
        {
        }

        XMC_I2C_CH_MasterStop(XMC_I2C1_CH1);

        while ( XMC_USIC_CH_GetTransmitBufferStatus(XMC_I2C1_CH1) == XMC_USIC_CH_TBUF_STATUS_BUSY )
		{
		}

    }


    if( rx_buffer != NULL ) {
       XMC_I2C_CH_MasterStart(XMC_I2C1_CH1, sensor->comLibIFParams.i2c_params.address, XMC_I2C_CH_CMD_READ);

         while((XMC_I2C_CH_GetStatusFlag(XMC_I2C1_CH1) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
        {
        }

        XMC_I2C_CH_ClearStatusFlag(XMC_I2C1_CH1,(uint32_t) XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

        for(size_t n = 0; n < (rx_len - 1); ++n) {
            XMC_I2C_CH_MasterReceiveAck(XMC_I2C1_CH1);

    		while((XMC_I2C_CH_GetStatusFlag(XMC_I2C1_CH1) & (XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0U)
    		{
    		}

    		XMC_I2C_CH_ClearStatusFlag(XMC_I2C1_CH1, XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);
            rx_buffer[n] = XMC_I2C_CH_GetReceivedData(XMC_I2C1_CH1);
        }

		XMC_I2C_CH_MasterReceiveNack(XMC_I2C1_CH1);

		while((XMC_I2C_CH_GetStatusFlag(XMC_I2C1_CH1) & (XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION)) == 0U)
		{
		}

		XMC_I2C_CH_ClearStatusFlag(XMC_I2C1_CH1, XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION | XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION);
		rx_buffer[rx_len - 1] = XMC_I2C_CH_GetReceivedData(XMC_I2C1_CH1);
        XMC_I2C_CH_MasterStop(XMC_I2C1_CH1);

        while ( XMC_USIC_CH_GetTransmitBufferStatus(XMC_I2C1_CH1) == XMC_USIC_CH_TBUF_STATUS_BUSY )
		{
		}
    }

    return true;
}


ComLibraryFunctions_ts  comLibIF_i2c = {
                                           .init.i2c_init         = I2CInitFunc,
                                           .deinit.i2c_deinit     = I2CDeinitFunc,
                                           .transfer.i2c_transfer = I2CTransferFunc,
                                       };


void setI2CParameters(ComLibraryParameters_ts *params) {
    params->i2c_params.address = GEN_2_STD_IIC_ADDR_WRITE_A0;
}


void initComLibIF(Sensor_ts *sensor, XMC_USIC_CH_t *const channel,
                  const uint8_t sourceSDA, const uint8_t sourceSCL,
                  XMC_GPIO_PORT_t *const portSDA, const uint8_t pinSDA,
                  XMC_GPIO_PORT_t *const portSCL, const uint8_t pinSCL) {

    sensor->comLibObj = (ComLibraryObject_ts *) malloc(sizeof(ComLibraryObject_ts));
   
    sensor->comLibObj->channel   = channel;
    sensor->comLibObj->sourceSDA = sourceSDA;
    sensor->comLibObj->sourceSCL = sourceSCL;
    sensor->comLibObj->portSDA   = portSDA;
    sensor->comLibObj->pinSDA    = pinSDA;
    sensor->comLibObj->portSCL   = portSCL;
    sensor->comLibObj->pinSCL    = pinSCL;

    sensor->comLibIF->init.i2c_init(sensor);
}
