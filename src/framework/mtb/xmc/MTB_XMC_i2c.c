// std includes
#include <stddef.h>

// XMC related includes
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
