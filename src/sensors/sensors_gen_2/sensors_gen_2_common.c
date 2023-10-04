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


/**
 * More generic version wrt size and offsets of MSB and LSB. Register values are in two's complement form.
 * Assumptions :
 *    - registers are 8 bits wide
*/
void gen_2_concatBytes(Sensor_ts *sensor, Register_ts *msb, Register_ts *lsb, int16_t *result) {
    *result   = ((sensor->regMap[msb->address] & msb->mask) << 8U); // Set minus flag if highest bit is set
    *result >>= (8U - lsb->numBits); // shift back and make space for LSB
    *result  |= ((sensor->regMap[lsb->address] & lsb->mask) >> lsb->offset); // OR with LSB
}


void gen_2_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == READ_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }

    assert((bf->accessMode == READ_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e));
}


void gen_2_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        sensor->regMap[bf->address] = (sensor->regMap[bf->address] & ~bf->mask) | ((newBitFieldValue << bf->offset) & bf->mask);
    }

    assert((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e));
}


bool gen_2_writeRegister(Sensor_ts* sensor, uint8_t bitField) {
    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        uint8_t transBuffer[2] = {bf->address, sensor->regMap[bf->address]};

        return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
    }

    return false;
}


bool gen_2_readRegisters(Sensor_ts *sensor) {
    // Currently only 1 interface is supported per sensor, either I2C or SPI for some 3rd generation sensors.
    // In case multiple interfaces are supported, switch according to IF type and call appropriate function.
    return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}


// Fuse/mode parity bit FP
uint8_t gen_2_calculateFuseParityBit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.FP];

	// compute parity of MOD1 register
	uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.MOD1] & ~bf->mask);

	// add parity of MOD2:PRD register bits
	parity ^= calculateParity(sensor->regMap[sensor->commonRegisters.MOD2] & sensor->regDef[sensor->commonBitfields.PRD].mask);

	return getOddParity(parity) << bf->offset;
}


// Calculate bus (data) parity bit P
uint8_t gen_2_calculateBusParityBit(Sensor_ts *sensor) {
	// compute bus parity of data values in registers 0 to 5
	uint8_t parity = sensor->regMap[0];

	for (uint8_t i = 1; i < 6; ++i) {
		parity ^= sensor->regMap[i];
	}

	return getOddParity(calculateParity(parity)) << sensor->regDef[sensor->commonBitfields.P].offset;
}


// TODO: implement !
bool gen_2_hasValidData(Sensor_ts *sensor) {
    return true;
}


// TODO: implement !
bool gen_2_isFunctional(Sensor_ts *sensor) {
    return true;
}


bool gen_2_hasValidBusParity(Sensor_ts *sensor) {
    uint8_t p = sensor->commonBitfields.P;
    return gen_2_calculateBusParityBit(sensor) == (sensor->regMap[sensor->regDef[p].address] && sensor->regDef[p].mask);
}


bool gen_2_hasValidFuseParity(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.FF];
    return sensor->regMap[bf->address] && bf->mask != 0;
}


bool gen_2_hasValidConfigurationParity(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.CF];
    return sensor->regMap[bf->address] && bf->mask != 0;
}


bool gen_2_hasValidTBit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.T];
    return sensor->regMap[bf->address] && bf->mask == 0;
}


bool gen_2_hasValidPD3Bit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.PD3];
    return sensor->regMap[bf->address] && bf->mask != 0;
}


bool gen_2_hasValidPD0Bit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.PD0];
    return sensor->regMap[bf->address] && bf->mask != 0;
}

bool gen_2_setPowerMode(Sensor_ts *sensor, uint8_t mode) {
    bool b = gen_2_readRegisters(sensor);

    if (mode != 0b10 && mode <= 0b11) {
        gen_2_setBitfield(sensor, sensor->commonBitfields.MODE, mode);
        sensor->regMap[sensor->commonRegisters.MOD1] = (sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[sensor->commonBitfields.FP].mask) | gen_2_calculateFuseParityBit(sensor);
        return b && gen_2_writeRegister(sensor, sensor->commonBitfields.MODE);
    }
    else {
        return false;
    }
}
bool gen_2_setIICAddress(Sensor_ts *sensor, StandardIICAddresses_te addr) {
    bool b = gen_2_readRegisters(sensor);
    uint8_t bitfieldValue = 0;
    uint8_t regAddress = 0;

    switch (addr) {
        case 0:
            bitfieldValue = 0b00;
            regAddress = GEN_2_STD_IIC_ADDR_WRITE_A0;
            break;

        case 1:
            bitfieldValue = 0b01;
            regAddress = GEN_2_STD_IIC_ADDR_WRITE_A1;
            break;

        case 2:
            bitfieldValue = 0b10;
            regAddress = GEN_2_STD_IIC_ADDR_WRITE_A2;
            break;

        case 3:
            bitfieldValue = 0b11;
            regAddress = GEN_2_STD_IIC_ADDR_WRITE_A3;
            break;
        
        default:
            b = false;
            break;
    }

    gen_2_setBitfield(sensor, sensor->commonBitfields.IICADR, bitfieldValue);
    sensor->regMap[sensor->commonRegisters.MOD1] = (sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[sensor->commonBitfields.FP].mask) | gen_2_calculateFuseParityBit(sensor);
    b &= gen_2_writeRegister(sensor, sensor->commonBitfields.IICADR);
    setI2CParameters(&sensor->comLibIFParams, regAddress);
   
    return b;
}


uint8_t gen_2_getID(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.ID];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}


uint8_t gen_2_getFrameCounter(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.FRM];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}


uint8_t gen_2_getType(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.TYPE];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}


uint8_t gen_2_getHWV(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.HWV];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}
