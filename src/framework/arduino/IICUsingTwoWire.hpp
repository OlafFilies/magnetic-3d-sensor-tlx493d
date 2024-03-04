#ifndef TLX493D_IIC_USING_TWOWIRE_HPP
#define TLX493D_IIC_USING_TWOWIRE_HPP


// std includes
#include <cstdbool>

// Arduino includes
#include <Arduino.h>
// #include <Wire.h>

// project cpp includes
#include "TwoWireWrapper.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"


namespace ifx {
    namespace tlx493d {
        bool initCommunication(TLx493D_t *sensor, TwoWireWrapper &tw, TLx493D_IICAddressType_t iicAdr);
        bool initCommunication(TLx493D_t *sensor, TwoWire &tw, TLx493D_IICAddressType_t iicAdr);
    }
}


#endif // TLX493D_IIC_USING_TWOWIRE_HPP
