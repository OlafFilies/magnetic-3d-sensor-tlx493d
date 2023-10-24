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


// framework functions
// TODO: replace by function pointers in comLibIF structure
// extern void setI2CParameters(Sensor_ts *sensor, uint8_t addr);


void gen_2_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
    // assert((bf->accessMode == READ_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e));

    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == READ_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
    }
}


void gen_2_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
    // assert((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e));

    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        sensor->regMap[bf->address] = (sensor->regMap[bf->address] & ~bf->mask) | ((newBitFieldValue << bf->offset) & bf->mask);
    }
}


bool gen_2_writeRegister(Sensor_ts* sensor, uint8_t bitField) {
    // assert((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e));

    Register_ts *bf = &sensor->regDef[bitField];

    if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
        uint8_t transBuffer[2] = { bf->address, sensor->regMap[bf->address] };

        return transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
        // return sensor->comLibIF->transfer.i2c_transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
    }

    return false;
}


// bool gen_2_readRegisters(Sensor_ts *sensor) {
//     // Currently only 1 interface is supported per sensor, either I2C or SPI for some 3rd generation sensors.
//     // In case multiple interfaces are supported, switch according to IF type and call appropriate function.
//     return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
//     // return sensor->comLibIF->transfer.i2c_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
// }


/***
 * 
*/
void gen_2_calculateTemperature(Sensor_ts *sensor, double *temp, uint8_t tempMSBBF, uint8_t tempLSBBF) {
   int16_t value = 0;

   concatBytes(sensor, tempMSBBF, tempLSBBF, &value);

   value <<= 2; // least significant 2 bits are implicit, therefore shift by 2 !
   *temp = (((double) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF;
}


/***
 * 
*/
void gen_2_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z,
                                  uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF, uint8_t bzMSBBF, uint8_t bzLSBBF) {
   int16_t valueX = 0, valueY = 0, valueZ = 0;

   concatBytes(sensor, bxMSBBF, bxLSBBF, &valueX);
   concatBytes(sensor, byMSBBF, byLSBBF, &valueY);
   concatBytes(sensor, bzMSBBF, bzLSBBF, &valueZ);

   *x = ((double) valueX) * GEN_2_MAG_FIELD_MULT;
   *y = ((double) valueY) * GEN_2_MAG_FIELD_MULT;
   *z = ((double) valueZ) * GEN_2_MAG_FIELD_MULT;
}


/***
 * 
*/
void gen_2_calculateMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp,
                                                uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF, uint8_t bzMSBBF, uint8_t bzLSBBF,
                                                uint8_t tempMSBBF, uint8_t tempLSBBF) {
   gen_2_calculateMagneticField(sensor, x, y, z, bxMSBBF, bxLSBBF, byMSBBF, byLSBBF, bzMSBBF, bzLSBBF);
   gen_2_calculateTemperature(sensor, temp, tempMSBBF, tempLSBBF);
}




// // Fuse/mode parity bit FP
// uint8_t gen_2_calculateFuseParityBit(Sensor_ts *sensor) {
//     Register_ts *bf = &sensor->regDef[sensor->commonBitfields.FP];

// 	// compute parity of MOD1 register
// 	uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.MOD1] & ~bf->mask);

// 	// add parity of MOD2:PRD register bits
// 	parity ^= calculateParity(sensor->regMap[sensor->commonRegisters.MOD2] & sensor->regDef[sensor->commonBitfields.PRD].mask);

// // TODO: remove shift left below and use setBitfield method instead of directly oring bit to byte !
// 	return getOddParity(parity) << bf->offset;
// }


// // Calculate bus (data) parity bit P
// uint8_t gen_2_calculateBusParityBit(Sensor_ts *sensor) {
// 	// compute bus parity of data values in registers 0 to 5
// 	uint8_t parity = sensor->regMap[0];

// 	for (uint8_t i = 1; i < 6; ++i) {
// 		parity ^= sensor->regMap[i];
// 	}

// // TODO: remove shift left below and use setBitfield method instead of directly oring bit to byte !
// 	return getOddParity(calculateParity(parity)) << sensor->regDef[sensor->commonBitfields.P].offset;
// }


// Fuse/mode parity bit FP
uint8_t gen_2_calculateFuseParity(Sensor_ts *sensor) {
	// compute parity of MOD1 register
	uint8_t parity = calculateParity(sensor->regMap[sensor->commonRegisters.MOD1] & ~sensor->regDef[sensor->commonBitfields.FP].mask);

	// add parity of MOD2:PRD register bits
	parity ^= calculateParity(sensor->regMap[sensor->commonRegisters.MOD2] & sensor->regDef[sensor->commonBitfields.PRD].mask);

// TODO: remove shift left below and use setBitfield method instead of directly oring bit to byte !
	return getOddParity(parity);
}


// Calculate bus (data) parity bit P
uint8_t gen_2_calculateBusParity(Sensor_ts *sensor) {
	// compute bus parity of data values in registers 0 to 5
	uint8_t parity = sensor->regMap[0];

	for (uint8_t i = 1; i < 6; ++i) {
		parity ^= sensor->regMap[i];
	}

// TODO: remove shift left below and use setBitfield method instead of directly oring bit to byte !
	return getOddParity(calculateParity(parity));
}


bool gen_2_hasValidData(Sensor_ts *sensor) {
    return gen_2_hasValidBusParity(sensor) && gen_2_hasValidTBit(sensor);
}


bool gen_2_hasValidTemperatureData(Sensor_ts *sensor) {
    return gen_2_hasValidData(sensor) && gen_2_hasValidPD3Bit(sensor);
}


bool gen_2_hasValidFieldData(Sensor_ts *sensor) {
    return gen_2_hasValidData(sensor) && gen_2_hasValidPD0Bit(sensor);
}


bool gen_2_isFunctional(Sensor_ts *sensor) {
    return gen_2_hasValidFuseParity(sensor) && gen_2_hasValidConfigurationParity(sensor);
}


bool gen_2_hasValidBusParity(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.P];
    return gen_2_calculateBusParity(sensor) == ((sensor->regMap[bf->address] & bf->mask) >> bf->offset);
}


bool gen_2_hasValidFuseParity(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.FF];
    return (sensor->regMap[bf->address] & bf->mask) != 0;
}


bool gen_2_hasValidConfigurationParity(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.CF];
    return (sensor->regMap[bf->address] & bf->mask) != 0;
}


bool gen_2_hasValidTBit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.T];
    return (sensor->regMap[bf->address] & bf->mask) == 0;
}


bool gen_2_hasValidPD3Bit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.PD3];
    return (sensor->regMap[bf->address] & bf->mask) != 0;
}


bool gen_2_hasValidPD0Bit(Sensor_ts *sensor) {
    Register_ts *bf = &sensor->regDef[sensor->commonBitfields.PD0];
    return (sensor->regMap[bf->address] & bf->mask) != 0;
}


bool gen_2_setPowerMode(Sensor_ts *sensor, uint8_t mode) {
    if( (mode != 0b10) && (mode <= 0b11) ){
        gen_2_setBitfield(sensor, sensor->commonBitfields.MODE, mode);
        gen_2_setBitfield(sensor, sensor->commonBitfields.FP, gen_2_calculateFuseParity(sensor));
        return gen_2_writeRegister(sensor, sensor->commonBitfields.MODE);
    }
    else {
        return false;
    }
}


bool gen_2_setIICAddress(Sensor_ts *sensor, StandardIICAddresses_te addr) {
    uint8_t bitfieldValue = 0;
    uint8_t deviceAddress = 0;

    switch (addr) {
        case GEN_2_STD_IIC_ADDR_A0:
            bitfieldValue = 0b00;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A0;
            break;

        case GEN_2_STD_IIC_ADDR_A1:
            bitfieldValue = 0b01;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A1;
            break;

        case GEN_2_STD_IIC_ADDR_A2:
            bitfieldValue = 0b10;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A2;
            break;

        case GEN_2_STD_IIC_ADDR_A3:
            bitfieldValue = 0b11;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A3;
            break;
        
        default:
            return false;
    }

    gen_2_setBitfield(sensor, sensor->commonBitfields.IICADR, bitfieldValue);
    gen_2_setBitfield(sensor, sensor->commonBitfields.FP, gen_2_calculateFuseParity(sensor));

    bool b = gen_2_writeRegister(sensor, sensor->commonBitfields.IICADR);
    setI2CParameters(sensor, deviceAddress);
    // setI2CParameters(&sensor->comLibIFParams, deviceAddress);

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




bool gen_2_hasValidIICadr(Sensor_ts *sensor, uint8_t id, uint8_t iicAdr) {
    Register_ts *idBf     = &sensor->regDef[id];
    Register_ts *iicAdrBf = &sensor->regDef[id];
    return ((sensor->regMap[idBf->address] & idBf->mask) >> idBf->offset) == ((sensor->regMap[iicAdrBf->address] & iicAdrBf->mask) >> iicAdrBf->offset);
}


bool gen_2_hasWakeup(Sensor_ts *sensor, uint8_t type) {
    Register_ts *typeBf = &sensor->regDef[type];
    return ((sensor->regMap[typeBf->address] & typeBf->mask) >> typeBf->offset) != 0b11;
}
