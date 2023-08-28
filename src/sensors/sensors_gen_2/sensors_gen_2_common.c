// std includes
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_config_common.h"
#include "sensors_gen_2_common.h"




/**
 * More generic version wrt size and offsets of MSB and LSB. Register values are in two's complement form.
 * Assumptions :
 *    - registers are 8 bits wide
*/
void concatBytes(Sensor_ts *sensor, Register_ts *msb, Register_ts *lsb, int16_t *result) {
    *result   = ((sensor->regMap[msb->address] & msb->mask) << 8U); // Set minus flag if highest bit is set
    *result >>= (8U - lsb->numBits); // shift back and make space for LSB
    *result  |= ((sensor->regMap[lsb->address] & lsb->mask) >> lsb->offset); // OR with LSB
}


void getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];

    if ((bf->accessMode == READ_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }

    assert((bf->accessMode == READ_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e));
}


void setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];

    if ((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        sensor->regMap[bf->address] = (sensor->regMap[bf->address] & ~bf->mask) | ((newBitFieldValue << bf->offset) & bf->mask);
    }

    assert((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e));
}


bool writeRegister(Sensor_ts* sensor, uint8_t bitField) {
    Register_ts *bf = &sensor->regDef[bitField];

    if ((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        uint8_t transBuffer[2] = {bf->address, sensor->regMap[bf->address]};

        return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
    }

    return false;
}