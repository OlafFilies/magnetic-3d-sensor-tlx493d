#ifndef TLX493D_IIC_USING_TWOWIRE_HPP
#define TLX493D_IIC_USING_TWOWIRE_HPP


// std includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Arduino includes
#include <Arduino.h>
// #include <Wire.h>

// project cpp includes
#include "TwoWireWrapper.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"


extern "C" bool tlx493d_transferIIC(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);
extern "C" void tlx493d_setI2CParameters(TLx493D_t *sensor, uint8_t addr);

bool tlx493d_initCommunication(TLx493D_t *sensor, TwoWireWrapper<TwoWire> &tw);
bool tlx493d_initCommunication(TLx493D_t *sensor, TwoWire &tw);


#endif // TLX493D_IIC_USING_TWOWIRE_HPP
