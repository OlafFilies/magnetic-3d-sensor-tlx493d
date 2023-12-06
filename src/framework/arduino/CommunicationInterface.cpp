// std includes
#include <stdbool.h>

// Arduino includes
#include "types.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"

// common to same generation of sensors

// sensor specific includes

// project cpp includes
#include "IICUsingTwoWire.hpp"
#include "SPIUsingSPIClass.hpp"


extern "C" bool tlx493d_transfer(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
    return sensor->comIFType == TLx493D_I2C_e ? tlx493d_transferIIC(sensor, txBuffer, txLen, rxBuffer, rxLen)
                                              : (sensor->comIFType == TLx493D_SPI_e ? tlx493d_transferSPI(sensor, txBuffer, txLen, rxBuffer, rxLen)
                                                                                    : false);
}


void tlx493d_deinitCommunication(TLx493D_t *sensor) {
    if( sensor->comIFType == TLx493D_I2C_e ) {
        // sensor->comInterface.comLibObj.iic_obj->wire->deinit();
        sensor->comInterface.comLibFuncs->deinit.iic_deinit(sensor);

        if( sensor->comInterface.comLibObj.iic_obj->isToBeDeleted ) {
            delete sensor->comInterface.comLibObj.iic_obj->wire;
        }

        free(sensor->comInterface.comLibObj.iic_obj);
        sensor->comInterface.comLibObj.iic_obj = NULL;
    }
    else if( sensor->comIFType == TLx493D_SPI_e ) {
        // sensor->comInterface.comLibObj.spi_obj->spi->deinit();
        sensor->comInterface.comLibFuncs->deinit.spi_deinit(sensor);

        if( sensor->comInterface.comLibObj.iic_obj->isToBeDeleted ) {
            delete sensor->comInterface.comLibObj.spi_obj->spi;
        }

        free(sensor->comInterface.comLibObj.spi_obj);
        sensor->comInterface.comLibObj.spi_obj = NULL;
    }

    sensor->comInterface.comLibFuncs = NULL;
}
