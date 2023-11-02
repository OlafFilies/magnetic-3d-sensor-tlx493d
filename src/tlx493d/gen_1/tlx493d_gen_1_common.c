// std includes
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

// common to same generation of sensors
#include "tlx493d_gen_1_common_defines.h"
#include "tlx493d_gen_1_common.h"


//todo: remove if not used. Note: So far not used 
void tlx493d_gen_1_getBitfield(TLx493D_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    TLx493D_Register_ts *bf = &sensor->regDef[bitField];

    if(bf->accessMode == TLx493D_READ_MODE_e) {
        *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }

    assert(bf->accessMode == TLx493D_READ_MODE_e);
}

uint8_t tlx493d_gen_1_returnBitfield(TLx493D_ts *sensor, uint8_t bitField) {
    TLx493D_Register_ts *bf = &sensor->regDef[bitField];
    uint8_t bitFieldValue;

    if(bf->accessMode == TLx493D_READ_MODE_e) {
        bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }

    assert(bf->accessMode == TLx493D_READ_MODE_e);

    return bitFieldValue;
}

void tlx493d_gen_1_setBitfield(TLx493D_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    TLx493D_Register_ts *bf = &sensor->regDef[bitField];
    
    if(bf->accessMode == TLx493D_WRITE_MODE_e) {
        sensor->regMap[bf->address + GEN_1_WRITE_REGISTERS_OFFSET] = (sensor->regMap[bf->address + GEN_1_WRITE_REGISTERS_OFFSET] & ~bf->mask) | (newBitFieldValue << bf->offset);
    }

    assert(bf->accessMode == TLx493D_WRITE_MODE_e);
}

bool tlx493d_gen_1_writeRegister(TLx493D_ts* sensor, uint8_t bitField) {
    bool ret = false;
    TLx493D_Register_ts *bf = &sensor->regDef[bitField];

    if(bf->accessMode == TLx493D_WRITE_MODE_e){
        uint8_t transBuffer[2] = { bf->address, sensor->regMap[bf->address] };

        ret = sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
    }

    assert(bf->accessMode == TLx493D_WRITE_MODE_e);

    return ret;
}

//TODO: rename to transferreadregister to match common functions
bool tlx493d_gen_1_readRegisters(TLx493D_ts *sensor) {
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, TLx493D_A1B6_READ_REGISTERS_MAX_COUNT);
}

bool tlx493d_gen_1_hasValidFuseParity(TLx493D_ts *sensor, uint8_t ffBF){
    TLx493D_Register_ts *bf = &sensor->regDef[ffBF];
    return ((sensor->regMap[bf->address] & bf->mask) != 0);
}

bool tlx493d_gen_1_isFunctional(TLx493D_ts *sensor, uint8_t ffBF){
    return tlx493d_gen_1_hasValidFuseParity(sensor, ffBF);
}

bool tlx493d_gen_1_hasValidTBit(TLx493D_ts *sensor, uint8_t tBF) {
    TLx493D_Register_ts *bf = &sensor->regDef[tBF];
    return ((sensor->regMap[bf->address] & bf->mask) == 0);
}

bool tlx493d_gen_1_hasValidPDBit(TLx493D_ts *sensor, uint8_t pdBF) {
    TLx493D_Register_ts *bf = &sensor->regDef[pdBF];
    return ((sensor->regMap[bf->address] & bf->mask) != 0);
}

bool tlx493d_gen_1_hasValidData(TLx493D_ts *sensor, uint8_t tBF, uint8_t pdBF){
    return tlx493d_gen_1_hasValidTBit(sensor, tBF) && tlx493d_gen_1_hasValidPDBit(sensor, pdBF);
}
