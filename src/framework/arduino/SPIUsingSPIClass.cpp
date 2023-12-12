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
#include "SPIUsingSPIClass.hpp"


extern "C" bool tlx493d_initSPI(TLx493D_t *sensor) {
    sensor->comInterface.comLibObj.spi_obj->spi->init();

    sensor->comInterface.comLibObj.spi_obj->spi->getBus().setDataMode(SPI_MODE2);
    sensor->comInterface.comLibObj.spi_obj->spi->getBus().setClockDivider(SPI_CLOCK_DIV8);
    sensor->comInterface.comLibObj.spi_obj->spi->getBus().setBitOrder(MSBFIRST);

    return true;
}


extern "C" bool tlx493d_deinitSPI(TLx493D_t *sensor) {
    sensor->comInterface.comLibObj.spi_obj->spi->deinit();
    return true;
}


extern "C" bool tlx493d_transferSPI(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
    if( sensor->boardSupportInterface.boardSupportObj.k2go_obj != nullptr ) {
        sensor->boardSupportInterface.boardSupportObj.k2go_obj->k2go->controlSelect(true);
    }

    bool b = sensor->comInterface.comLibObj.spi_obj->spi->transfer(txBuffer, txLen, rxBuffer, rxLen);

    if( sensor->boardSupportInterface.boardSupportObj.k2go_obj != nullptr ) {
        sensor->boardSupportInterface.boardSupportObj.k2go_obj->k2go->controlSelect(false);
    }

    return b;

    // return sensor->comInterface.comLibObj.spi_obj->spi->transfer(txBuffer, txLen, rxBuffer, rxLen);
}


TLx493D_ComLibraryFunctions_t  comLibFuncs_spi = {
                                            .init     = { .spi_init     = tlx493d_initSPI },
                                            .deinit   = { .spi_deinit   = tlx493d_deinitSPI },
                                            .transfer = { .spi_transfer = tlx493d_transferSPI },
                                       };


bool tlx493d_initCommunication(TLx493D_t *sensor, SPIClassWrapper &spi) {
    sensor->comInterface.comLibObj.spi_obj      = (TLx493D_SPIObject_t *) malloc(sizeof(TLx493D_SPIObject_t));
    sensor->comInterface.comLibObj.spi_obj->spi = &spi;
    sensor->comInterface.comLibObj.spi_obj->isToBeDeleted          = false;
    // sensor->comInterface.isToBeDeleted          = false;
    sensor->comInterface.comLibFuncs            = &comLibFuncs_spi;

    sensor->comInterface.comLibFuncs->init.spi_init(sensor);
    return true;
}


bool tlx493d_initCommunication(TLx493D_t *sensor, SPIClass &spi) {
    sensor->comInterface.comLibObj.spi_obj      = (TLx493D_SPIObject_t *) malloc(sizeof(TLx493D_SPIObject_t));
    sensor->comInterface.comLibObj.spi_obj->spi = new SPIClassWrapper(spi);
    sensor->comInterface.comLibObj.spi_obj->isToBeDeleted          = true;
    // sensor->comInterface.isToBeDeleted          = true;

    sensor->comInterface.comLibFuncs            = &comLibFuncs_spi;

    sensor->comInterface.comLibFuncs->init.spi_init(sensor);  
    return true;
}
