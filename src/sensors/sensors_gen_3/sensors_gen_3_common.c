// std includes
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_common_defines.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_common_defines.h"
#include "sensors_gen_2_common.h"

#include "sensors_gen_3_common_defines.h"
#include "sensors_gen_3_common.h"


#include "Logger.h"


void gen_3_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    // assert((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e));

logmsgui("bitfield : ", bitField);
    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
logmsgui("bitfield->address : ", bf->address);
logmsgui("bitfield->mask : ", bf->mask);
logmsgui("bitfield->offset : ", bf->offset);
logmsgui("newBitFieldValue : ", newBitFieldValue);

        sensor->regMap[bf->address] = (sensor->regMap[bf->address] & ~bf->mask) | ((newBitFieldValue << bf->offset) & bf->mask);
    }
}


bool gen_3_writeRegisterSPI(Sensor_ts* sensor, uint8_t bitField) {
    // assert((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e));

    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        uint8_t transBuffer[2] = { bf->address, sensor->regMap[bf->address] };

        return sensor->comLibIF->transfer.spi_transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
    }

    return false;
}


bool gen_3_readRegistersSPI(Sensor_ts *sensor) {
    // Currently only 1 interface is supported per sensor, either I2C or SPI for some 3rd generation sensors.
    // In case multiple interfaces are supported, switch according to IF type and call appropriate function.

    // TODO: add READ bit (mask 0x80 == high) + auto_inc (mask 0x40 == low) + register address
    // sensor->regMap[0] = 0;
    sensor->regMap[0] = GEN_3_SPI_READ_BIT_ON | GEN_3_SPI_AUTO_INC_BIT_OFF;
    return sensor->comLibIF->transfer.spi_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}
