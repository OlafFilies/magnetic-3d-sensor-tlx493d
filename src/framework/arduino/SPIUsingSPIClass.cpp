/** std includes. */
#include <cstddef>
#include <cstdlib>
// #include <stddef.h>
// #include <stdlib.h>

/** project c includes. */
#include "tlx493d_types.h"

/** project cpp includes. */
#include "types.hpp"
#include "SPIUsingSPIClass.hpp"


namespace ifx {
    namespace tlx493d {
        static uint8_t     spiReadAddress = 0x00;
        static SPISettings settings;


        // static void initModeSPI(TLx493D_t *sensor) {
        //     sensor->comInterface.comLibObj.spi_obj->spi->getBus().setDataMode(SPI_MODE2);
        //     sensor->comInterface.comLibObj.spi_obj->spi->getBus().setClockDivider(SPI_CLOCK_DIV8);
        //     sensor->comInterface.comLibObj.spi_obj->spi->getBus().setBitOrder(MSBFIRST);
        // }


        static bool initSPI(TLx493D_t *sensor) {
            sensor->comInterface.comLibObj.spi_obj->spi->init(settings);
            // initModeSPI(sensor);

            // sensor->comInterface.comLibObj.spi_obj->spi.getBus().setDataMode(SPI_MODE2);
            // sensor->comInterface.comLibObj.spi_obj->spi.getBus().setClockDivider(SPI_CLOCK_DIV8);
            // sensor->comInterface.comLibObj.spi_obj->spi.getBus().setBitOrder(MSBFIRST);

            return true;
        }


        static bool deinitSPI(TLx493D_t *sensor) {
            sensor->comInterface.comLibObj.spi_obj->spi->deinit();
            return true;
        }


        static bool transferSPI(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
            if( sensor->boardSupportInterface.boardSupportObj.k2go_obj != NULL ) {
                sensor->boardSupportInterface.boardSupportObj.k2go_obj->k2go->enableSelect(true);
            }

            bool b = sensor->comInterface.comLibObj.spi_obj->spi->transfer(txBuffer, txLen, rxBuffer, rxLen, spiReadAddress);

            if( sensor->boardSupportInterface.boardSupportObj.k2go_obj != NULL ) {
                sensor->boardSupportInterface.boardSupportObj.k2go_obj->k2go->enableSelect(false);
            }

            return b;
        }


        static void setReadAddressSPI(TLx493D_t *sensor, uint8_t address) {
            (void) sensor;
            
            spiReadAddress = address;
        }


        static TLx493D_ComLibraryFunctions_t  comLibFuncs_spi = {
                                                    .init           = { .spi_init           = initSPI },
                                                    .deinit         = { .spi_deinit         = deinitSPI },
                                                    .transfer       = { .spi_transfer       = transferSPI },
                                                    .setReadAddress = { .spi_setReadAddress = setReadAddressSPI },
                                            };


        bool initCommunication(TLx493D_t *sensor, SPIClassWrapper &spi, bool executeInit,
                               uint32_t clockFreq, uint8_t bitOrder, uint8_t dataMode) {
            sensor->comInterface.comLibObj.spi_obj                = (TLx493D_SPIObject_t *) malloc(sizeof(TLx493D_SPIObject_t));
            sensor->comInterface.comLibObj.spi_obj->spi           = &spi;
            sensor->comInterface.comLibObj.spi_obj->isToBeDeleted = false;

            sensor->comInterface.comLibFuncs                      = &comLibFuncs_spi;

            if( executeInit ) {
                settings.clockFreq = clockFreq;
                settings.bitOrder  = bitOrder;
                settings.dataMode  = dataMode;
                sensor->comInterface.comLibFuncs->init.spi_init(sensor);
            }

            return true;
        }


        bool initCommunication(TLx493D_t *sensor, SPIClass &spi, bool executeInit, uint32_t clockFreq, uint8_t bitOrder, uint8_t dataMode) {
            sensor->comInterface.comLibObj.spi_obj                = (TLx493D_SPIObject_t *) malloc(sizeof(TLx493D_SPIObject_t));
            sensor->comInterface.comLibObj.spi_obj->spi           = new SPIClassWrapper(spi);
            sensor->comInterface.comLibObj.spi_obj->isToBeDeleted = true;

            sensor->comInterface.comLibFuncs                      = &comLibFuncs_spi;

            if( executeInit ) {
                settings.clockFreq = clockFreq;
                settings.bitOrder  = bitOrder;
                settings.dataMode  = dataMode;

                sensor->comInterface.comLibFuncs->init.spi_init(sensor);
                // initModeSPI(sensor);
            }

            return true;
        }
    }
}
