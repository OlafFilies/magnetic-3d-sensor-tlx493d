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
#include "sensors_gen_1_common_defines.h"
#include "sensors_gen_1_common.h"


//todo: remove if not used.
void gen_1_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    if(sensor->regDef[bitField].accessMode == READ_MODE_e) {
        *bitFieldValue = (sensor->regMap[sensor->regDef[bitField].address] & sensor->regDef[bitField].mask) >> sensor->regDef[bitField].offset;
    }
}

uint8_t gen_1_returnBitfield(Sensor_ts *sensor, uint8_t bitField) {
    uint8_t bitFieldValue;
    if(sensor->regDef[bitField].accessMode == READ_MODE_e) {
        bitFieldValue = (sensor->regMap[sensor->regDef[bitField].address] & sensor->regDef[bitField].mask) >> sensor->regDef[bitField].offset;
    }
    return bitFieldValue;
}

void gen_1_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    if(sensor->regDef[bitField].accessMode == WRITE_MODE_e) {
        sensor->regMap[sensor->regDef[bitField].address + GEN_1_WRITE_REGISTERS_OFFSET] = (sensor->regMap[sensor->regDef[bitField].address + GEN_1_WRITE_REGISTERS_OFFSET] & ~sensor->regDef[bitField].mask) | (newBitFieldValue << sensor->regDef[bitField].offset);
    }
}

bool gen_1_writeRegister(Sensor_ts* sensor, uint8_t registerAddr) {
    bool err = false;

    if(sensor->regDef[registerAddr].accessMode == WRITE_MODE_e) {
        uint8_t transBuffer[2];
        uint8_t bufLen = 2;

        transBuffer[0] = sensor->regDef[registerAddr].address;
        transBuffer[1] = sensor->regMap[sensor->regDef[registerAddr].address];

        err = sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, NULL, 0);
    }

    return err;
}

//TODO: rename to transferreadregister to match common functions
bool gen_1_readRegisters(Sensor_ts *sensor) {
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, TLx493D_A1B6_READ_REGISTERS_MAX_COUNT);
}