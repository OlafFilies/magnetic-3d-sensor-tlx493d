#include "sensors_gen_1_utils.h"
#include "sensors_gen_1_common_defines.h"

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

//todo: make this more intelligent to detect if MSB is a full register or not. also the shifting and masking can be handled inside this. 
int16_t gen_1_concat_values(uint8_t MSB, uint8_t LSB, bool IsMSBinOneRegsiter){
    int16_t value = 0x0000;

    if (IsMSBinOneRegsiter){
        value = MSB << 8;
        value |= (LSB & 0x0F) << 4;
    }
    else{
        value = (MSB & 0x0F) << 12;
        value |= LSB << 4;
    }

    value >>= 4;

    return value;

}