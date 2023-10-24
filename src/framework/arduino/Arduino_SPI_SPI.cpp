// std includes
#include <malloc.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "sensor_types.h"

// common to same generation of sensors
#include "sensors_gen_2_common_defines.h"

// sensor specific includes

// project cpp includes
#include "arduino_defines.h"
#include "SPI_Lib.hpp"


extern "C" bool initSPI(Sensor_ts *sensor) {
    sensor->comLibObj.spi_obj->spi->init();

    sensor->comLibObj.spi_obj->spi->getComIF().setDataMode(SPI_MODE2);
    sensor->comLibObj.spi_obj->spi->getComIF().setClockDivider(SPI_CLOCK_DIV2);
    sensor->comLibObj.spi_obj->spi->getComIF().setBitOrder(MSBFIRST);

    return true;
}


extern "C" bool deinitSPI(Sensor_ts *sensor) {
    sensor->comLibObj.spi_obj->spi->deinit();
    return true;
}


extern "C" bool transferSPI(Sensor_ts *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
    return sensor->comLibObj.spi_obj->spi->transfer(txBuffer, txLen, rxBuffer, rxLen);
}


ComLibraryFunctions_ts  comLibIF_spi = {
                                            .init     = { .spi_init     = initSPI },
                                            .deinit   = { .spi_deinit   = deinitSPI },
                                            .transfer = { .spi_transfer = transferSPI },
                                       };


// extern "C" void setSPIParameters(Sensor_ts *sensor, uint8_t addr) {
//     sensor->comLibIFParams.spi_params.address = addr >> 1;
// }


bool initComLibIF(Sensor_ts *sensor, SPI_Lib<SPIClass> &spi) {
    if( sensor->comIFType != SPI_e ) {
        return false;
    }

    // Need to dynamically allocate object, such that different sensor may use different TwoWire objects (Wire, Wire1, Wire2, ...)
    sensor->comLibObj.spi_obj      = (SPIObject_ts *) malloc(sizeof(SPIObject_ts));
    sensor->comLibObj.spi_obj->spi = &spi;
    sensor->comLibIF               = &comLibIF_spi;

    sensor->comLibIF->init.spi_init(sensor);

    return true;
}


// TODO: Provide function to delete TwoWire_Lib object from C in case it has been allocated explicitly by the following routine.
// extern "C" 
bool initComLibIF(Sensor_ts *sensor, SPIClass &spi) {
    if( sensor->comIFType != SPI_e ) {
        return false;
    }

    // Need to dynamically allocate object, such that different sensor may use different TwoWire objects (Wire, Wire1, Wire2, ...)
    sensor->comLibObj.spi_obj      = (SPIObject_ts *) malloc(sizeof(SPIObject_ts));
    sensor->comLibObj.spi_obj->spi = new SPI_Lib<SPIClass>(spi);
    sensor->comLibIF               = &comLibIF_spi;

    sensor->comLibIF->init.spi_init(sensor);
    
    return true;
}
