#ifndef TLX493D_IIC_USING_TWOWIRE_HPP
#define TLX493D_IIC_USING_TWOWIRE_HPP


/** std includes. */
#include <cstdbool>

/** Arduino includes. */
#include <Arduino.h>

/** project cpp includes. */
#include "TwoWireWrapper.hpp"

/** project c includes. */
#include "tlx493d_types.h"

/**
 * @brief The function `initCommunication` initializes the communication interface of the sensor.
 * In this case it initializes the I2C-interface. The initialized interfaces depends on the chosen sensor
 * and is determined automatically.
 * 
 * @param[in,out] sensor A pointer to a TLx493D_t struct, which represents the TLx493D sensor.
 * @param[in,out] tw A reference to an instance of the TwoWire object.
 * @param[in] iicAdr The I2C address of the sensor. See `TLx493D_IICAddressType_t` for the available ones.
 * @param[in] executeInit A boolean flag which determines if the initialization is executed or not.
 * 
 * @return Returns a boolean to indicate if the function execution was successful or not.
 * @retval 0 Error.
 * @retval 1 Success.
*/
namespace ifx {
    namespace tlx493d {
        bool initCommunication(TLx493D_t *sensor, TwoWire &tw, TLx493D_IICAddressType_t iicAdr, bool executeInit = false);
    }
}


#endif // TLX493D_IIC_USING_TWOWIRE_HPP
