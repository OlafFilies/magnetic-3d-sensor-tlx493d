// std includes
#include <stdbool.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

// common to same generation of sensors
#include "tlx493d_gen_2_common_defines.h"
#include "tlx493d_gen_2_common.h"

#include "tlx493d_gen_3_common_defines.h"
#include "tlx493d_gen_3_common.h"


bool tlx493d_gen_3_readRegistersSPI(TLx493D_ts *sensor) {
    sensor->regMap[0] = GEN_3_SPI_READ_BIT_ON | GEN_3_SPI_AUTO_INC_BIT_OFF;
    return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}
