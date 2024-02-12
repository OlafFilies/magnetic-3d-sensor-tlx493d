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
#include "IICUsingTwoWire.hpp"
#include "Logger.h"


// extern "C"
static bool tlx493d_initIIC(TLx493D_t *sensor) {
    sensor->comInterface.comLibObj.iic_obj->wire->init();
    return true;

}


// extern "C"
static bool tlx493d_deinitIIC(TLx493D_t *sensor) {
    sensor->comInterface.comLibObj.iic_obj->wire->deinit();
    return true;
}


// extern "C"
static bool tlx493d_transferIIC(TLx493D_t *sensor, uint8_t *txBuffer, uint8_t txLen, uint8_t *rxBuffer, uint8_t rxLen) {
    return sensor->comInterface.comLibObj.iic_obj->wire->transfer(sensor->comInterface.comLibParams.iic_params.address >> 1, txBuffer, txLen, rxBuffer, rxLen);
}


// extern "C"
static void tlx493d_setReadAddressIIC(TLx493D_t *sensor, uint8_t address) {
    warn("Function 'tlx493d_setReadAddressIIC' not supported !");
}


TLx493D_ComLibraryFunctions_t  comLibFuncs_iic = {
                                            .init           = { .iic_init           = tlx493d_initIIC },
                                            .deinit         = { .iic_deinit         = tlx493d_deinitIIC },
                                            .transfer       = { .iic_transfer       = tlx493d_transferIIC },
                                            .setReadAddress = { .iic_setReadAddress = tlx493d_setReadAddressIIC },
                                       };


bool tlx493d_initCommunication(TLx493D_t *sensor, TwoWireWrapper &tw, TLx493D_IICAddressType_t iicAdr) {
    sensor->comInterface.comLibObj.iic_obj                = (TLx493D_I2CObject_t *) malloc(sizeof(TLx493D_I2CObject_t));
    sensor->comInterface.comLibObj.iic_obj->wire          = &tw;
    sensor->comInterface.comLibObj.iic_obj->isToBeDeleted = false;

    sensor->comInterface.comLibFuncs                      = &comLibFuncs_iic;

    sensor->comInterface.comLibParams.iic_params.address  = sensor->functions->selectIICAddress(sensor, iicAdr);

    sensor->comInterface.comLibFuncs->init.iic_init(sensor);
    return true;
}


bool tlx493d_initCommunication(TLx493D_t *sensor, TwoWire &tw, TLx493D_IICAddressType_t iicAdr) {
    sensor->comInterface.comLibObj.iic_obj                = (TLx493D_I2CObject_t *) malloc(sizeof(TLx493D_I2CObject_t));
    sensor->comInterface.comLibObj.iic_obj->wire          = new TwoWireWrapper(tw);
    sensor->comInterface.comLibObj.iic_obj->isToBeDeleted = true;

    sensor->comInterface.comLibFuncs                      = &comLibFuncs_iic;

    sensor->comInterface.comLibParams.iic_params.address  = sensor->functions->selectIICAddress(sensor, iicAdr);

    sensor->comInterface.comLibFuncs->init.iic_init(sensor);
    return true;
}
