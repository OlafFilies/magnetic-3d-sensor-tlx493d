// std includes
#include <stdbool.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"

// common to same generation of sensors

// sensor specific includes

// project cpp includes
#include "IICUsingTwoWire.hpp"
#include "SPIUsingSPIClass.hpp"


// extern "C" bool tlx493d_transferSPI(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);
// extern "C" bool tlx493d_transferIIC(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen);


extern "C" bool transfer(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
    return sensor->comIFType == TLx493D_I2C_e ? tlx493d_transferIIC(sensor, txBuffer, txLen, rxBuffer, rxLen)
                                      : (sensor->comIFType == TLx493D_SPI_e ? tlx493d_transferSPI(sensor, txBuffer, txLen, rxBuffer, rxLen)
                                                                    : false);
}