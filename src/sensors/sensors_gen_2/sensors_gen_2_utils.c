#include "sensors_gen_2_utils.h"

void getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    if((sensor->regDef[bitField].accessMode == READ_MODE_e) || (sensor->regDef[bitField].accessMode == READ_WRITE_MODE_e)) {
        *bitFieldValue = (sensor->regMap[sensor->regDef[bitField].address] & sensor->regDef[bitField].mask) >> sensor->regDef[bitField].offset;
    }
}

void setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    if((sensor->regDef[bitField].accessMode == WRITE_MODE_e) || (sensor->regDef[bitField].accessMode == READ_WRITE_MODE_e)) {
        sensor->regMap[sensor->regDef[bitField].address] = (sensor->regMap[sensor->regDef[bitField].address] & ~sensor->regDef[bitField].mask) | ((newBitFieldValue << sensor->regDef[bitField].offset) & sensor->regDef[bitField].mask);
    }
}

bool writeRegister(Sensor_ts* sensor, uint8_t registerAddr) {
    bool err = false;

    if((sensor->regDef[registerAddr].accessMode == WRITE_MODE_e) || (sensor->regDef[registerAddr].accessMode == READ_WRITE_MODE_e)) {
        uint8_t transBuffer[2];
        uint8_t bufLen = 2;

        transBuffer[0] = sensor->regDef[registerAddr].address;
        transBuffer[1] = sensor->regMap[sensor->regDef[registerAddr].address];

        err = sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, bufLen, NULL, 0);
    }

    return err;
}