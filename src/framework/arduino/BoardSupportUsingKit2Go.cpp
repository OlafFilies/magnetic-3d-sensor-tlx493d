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
#include "BoardSupportUsingKit2Go.hpp"


// extern "C" bool tlx493d_initSPI(TLx493D_t *sensor) {
//     sensor->comInterface.comLibObj.spi_obj->spi->init();

//     sensor->comInterface.comLibObj.spi_obj->spi->getBus().setDataMode(SPI_MODE2);
//     sensor->comInterface.comLibObj.spi_obj->spi->getBus().setClockDivider(SPI_CLOCK_DIV8);
//     sensor->comInterface.comLibObj.spi_obj->spi->getBus().setBitOrder(MSBFIRST);

//     return true;
// }


// extern "C" bool tlx493d_deinitSPI(TLx493D_t *sensor) {
//     sensor->comInterface.comLibObj.spi_obj->spi->deinit();
//     return true;
// }


// extern "C" bool tlx493d_transferSPI(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
//     return sensor->comInterface.comLibObj.spi_obj->spi->transfer(txBuffer, txLen, rxBuffer, rxLen);
// }


// TLx493D_ComLibraryFunctions_t  comLibFuncs_spi = {
//                                             .init     = { .spi_init     = tlx493d_initSPI },
//                                             .deinit   = { .spi_deinit   = tlx493d_deinitSPI },
//                                             .transfer = { .spi_transfer = tlx493d_transferSPI },
//                                        };


bool tlx493d_initBoardSupport(TLx493D_t *sensor, Kit2GoBoardSupport &bsc) {
    sensor->boardSupportInterface.boardSupportObj.k2go_obj                = (TLx493D_Kit2GoBoardSupportObject_t *) malloc(sizeof(TLx493D_Kit2GoBoardSupportObject_t));
    sensor->boardSupportInterface.boardSupportObj.k2go_obj->k2go          = &bsc;
    sensor->boardSupportInterface.boardSupportObj.k2go_obj->isToBeDeleted = false;
    return true;
}


// extern "C" bool tlx493d_initBoardSupport(TLx493D_t *sensor) {
//     sensor->boardSupportInterface.boardSupportObj.k2go_obj                = (TLx493D_Kit2GoBoardSupportObject_t *) malloc(sizeof(TLx493D_Kit2GoBoardSupportObject_t));
//     sensor->boardSupportInterface.boardSupportObj.k2go_obj->k2go          = new Kit2GoBoardSupport();
//     sensor->boardSupportInterface.boardSupportObj.k2go_obj->isToBeDeleted = true;

//     return true;
// }
