// std includes
#include <stddef.h>

// XMC related includes
#include "xmc_i2c.h"

// project c includes
#include "mtb_defines.h"

// project c includes
// common to all sensors
#include "sensor_types.h"
// #include "sensors_config_common.h"
// #include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_config_common.h"
// #include "sensors_gen_2_common.h"

// sensor specicifc includes
// #include "TLE493D_A2B6_config.h"
// #include "TLE493D_A2B6.h"


/*
    Currently unused.
*/

// bool SPIInitFunc(TLx493D_t *sensor) {
//     return true;
// }


// bool SPIDeinitFunc(TLx493D_t *sensor) {
//     return true;
// }


// bool SPITransferFunc(TLx493D_t *sensor, uint8_t *tx_buffer, uint16_t tx_len, uint8_t *rx_buffer, uint16_t rx_len) {
//     return true;
// }


// TLx493D_ComLibraryFunctions_t  comLibIF_spi = {
//                                            .init.spi_init         = SPIInitFunc,
//                                            .deinit.spi_deinit     = SPIDeinitFunc,
//                                            .transfer.spi_transfer = SPITransferFunc,
//                                        };


// void setSPIParameters(TLx493D_ComLibraryParameters_t *params) {
//     params->spi_params.dummy = 0;
// }
