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

// #include "Logger.h"


// framework functions


// void tlx493d_gen_2_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue) {
//     Register_ts *bf = &sensor->regDef[bitField];

//     if((bf->accessMode == READ_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
//         *bitFieldValue = (sensor->regMap[bf->address] & bf->mask) >> bf->offset;
//     }
// }


// void tlx493d_gen_2_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue) {
//     Register_ts *bf = &sensor->regDef[bitField];

//     if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
//         sensor->regMap[bf->address] = (sensor->regMap[bf->address] & ~bf->mask) | ((newBitFieldValue << bf->offset) & bf->mask);
//     }
// }


// bool tlx493d_gen_2_writeRegister(Sensor_ts* sensor, uint8_t bitField) {
//     Register_ts *bf = &sensor->regDef[bitField];

//     if((bf->accessMode == WRITE_MODE_e) || (bf->accessMode == READ_WRITE_MODE_e)) {
//         uint8_t transBuffer[2] = { bf->address, sensor->regMap[bf->address] };

//         return transfer(sensor, transBuffer, sizeof(transBuffer), NULL, 0);
//     }

//     return false;
// }


// bool tlx493d_gen_2_readRegisters(Sensor_ts *sensor) {
//     return transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
// }


/***
 * TODO: set all options that must be set, eg MODE ?, reset all bits to defaults ?
*/
bool tlx493d_gen_2_setDefaultConfig(Sensor_ts *sensor, uint8_t configREG, uint8_t mod1REG, uint8_t mod2REG, uint8_t cpBF, uint8_t caBF, uint8_t intBF) {
// bool tlx493d_gen_2_setDefaultConfig(Sensor_ts *sensor, uint8_t configREG, uint8_t configResetValue, uint8_t mod1REG, uint8_t mod2REG, uint8_t caBF, uint8_t intBF) {
    sensor->regMap[configREG] = 0x00; // sensor->functions->getConfigResetValue();
    sensor->regMap[mod1REG]   = 0x00;
    sensor->regMap[mod2REG]   = 0x00;

    tlx493d_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));
    tlx493d_writeRegister(sensor, cpBF);

    tlx493d_setBitfield(sensor, caBF, 0);
    tlx493d_setBitfield(sensor, intBF, 1);

    if( sensor->functions->enable1ByteReadMode(sensor) ) {
        // if( sensor->functions->disableTemperatureMeasurement(sensor) ) {
            // Read registers in order to retrieve values in reserved register at 0x12 and in MOD2 in order to make sure we are not 
            // accidentally changing a preset values to 0.
            return sensor->functions->readRegisters(sensor);
        // }
    }

    return false;
}


bool tlx493d_gen_2_setPowerMode(Sensor_ts *sensor, uint8_t modeBF, uint8_t fpBF, uint8_t mode) {
    if( (mode != 0b10) && (mode <= 0b11) ){
        tlx493d_setBitfield(sensor, modeBF, mode);
        tlx493d_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));
        // tlx493d_setBitfield(sensor, sensor->commonBitfields.FP, tlx493d_gen_2_calculateFuseParity(sensor));
        return tlx493d_writeRegister(sensor, modeBF);
    }
    else {
        return false;
    }
}


bool tlx493d_gen_2_setIICAddress(Sensor_ts *sensor, uint8_t iicadrBF, uint8_t fpBF, StandardIICAddresses_te addr) {
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

    tlx493d_setBitfield(sensor, iicadrBF, bitfieldValue);
    tlx493d_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));
    // tlx493d_setBitfield(sensor, sensor->commonBitfields.FP, tlx493d_gen_2_calculateFuseParity(sensor));

    bool b = tlx493d_writeRegister(sensor, iicadrBF);
    setI2CParameters(sensor, deviceAddress);

    return b;
}


/***
 * 
*/
void tlx493d_gen_2_calculateTemperature(Sensor_ts *sensor, double *temp, uint8_t tempMSBBF, uint8_t tempLSBBF) {
    int16_t value = 0;

    tlx493d_concatBytes(sensor, tempMSBBF, tempLSBBF, &value);

    value <<= 2; // least significant 2 bits are implicit, therefore shift by 2 !
    *temp = (((double) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF;
}


void tlx493d_gen_2_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z,
                                  uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF, uint8_t bzMSBBF, uint8_t bzLSBBF) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    tlx493d_concatBytes(sensor, bxMSBBF, bxLSBBF, &valueX);
    tlx493d_concatBytes(sensor, byMSBBF, byLSBBF, &valueY);
    tlx493d_concatBytes(sensor, bzMSBBF, bzLSBBF, &valueZ);

    *x = ((double) valueX) * GEN_2_MAG_FIELD_MULT;  // TODO: get factor from registers : full, double, quadruple 
    *y = ((double) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((double) valueZ) * GEN_2_MAG_FIELD_MULT;                           
}


/***
 * 
*/
bool tlx493d_gen_2_setDisableTemperatureMeasurement(Sensor_ts *sensor, uint8_t dtBF, uint8_t cpBF, uint8_t dt) {
    tlx493d_setBitfield(sensor, dtBF, dt);
    tlx493d_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_writeRegister(sensor, dtBF);
}


/***
 * AM depends on value of DT !
*/
bool tlx493d_gen_2_setAngularMeasurement(Sensor_ts *sensor, uint8_t amBF, uint8_t dtBF, uint8_t cpBF, uint8_t am, uint8_t dt) {
    tlx493d_setBitfield(sensor, dtBF, dt);
    tlx493d_setBitfield(sensor, amBF, am);
    tlx493d_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_writeRegister(sensor, amBF);
}


/***
 * 
*/
bool tlx493d_gen_2_setUpdateRate(Sensor_ts *sensor, uint8_t fpBF, uint8_t prdBF, uint8_t ur) {
    uint8_t mod1 = sensor->regDef[fpBF].address;

    tlx493d_setBitfield(sensor, prdBF, ur);
    tlx493d_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

    uint8_t buf[4] = { mod1,
                       sensor->regMap[mod1],
                       sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
                       sensor->regMap[mod1 + 2]
                     };

    return transfer(sensor, buf, sizeof(buf), NULL, 0);
}


/***
 * 
*/
bool tlx493d_gen_2_setShortRangeSensitivity(Sensor_ts *sensor, uint8_t x2BF, uint8_t cpBF, uint8_t srs) {
    tlx493d_setBitfield(sensor, x2BF, srs);
    tlx493d_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_writeRegister(sensor, x2BF);
}


/***
 * TODO: implement checks ! CA only in Low Power and MCM mode, not in Fast Mode ! MODE depends on PR and TRIG !
*/
bool tlx493d_gen_2_setInterruptAndCollisionAvoidance(Sensor_ts *sensor, uint8_t intBF, uint8_t caBF, uint8_t fpBF, uint8_t prdBF, bool intIsOn, bool caIsOn) {
// tlx493d_setBitfield(sensor, sensor->commonBitfields.MODE, 0b11);
    tlx493d_setBitfield(sensor, intBF, intIsOn ? 0 : 1);
    tlx493d_setBitfield(sensor, caBF, caIsOn ? 1 : 0);
    tlx493d_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

    return tlx493d_writeRegister(sensor, intBF); 
}


// bool tlx493d_gen_2_setInterrupt(Sensor_ts *sensor, uint8_t intBF, uint8_t fpBF, uint8_t prdBF, uint8_t irq) {
//     tlx493d_setBitfield(sensor, intBF, irq);
//     tlx493d_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

//     return tlx493d_writeRegister(sensor, intBF);
// }


// /***
//  * CA only in Low Power and MCM mode, not in Fast Mode !
//  * MODE depends on PR and TRIG !
// */
// bool tlx493d_gen_2_setCollisionAvoidance(Sensor_ts *sensor, uint8_t ca) {
// // tlx493d_setBitfield(sensor, sensor->commonBitfields.MODE, 0b11);

//     tlx493d_setBitfield(sensor, CA, ca);
//     tlx493d_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     return tlx493d_writeRegister(sensor, CA);
// }


/***
 * This option depends on PR and MODE.
*/
bool tlx493d_gen_2_setTrigger(Sensor_ts *sensor, uint8_t trigBF, uint8_t cpBF, uint8_t trig) {
    tlx493d_setBitfield(sensor, trigBF, trig);
    tlx493d_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_writeRegister(sensor, trigBF);
}


bool tlx493d_gen_2_setTriggerBits(Sensor_ts *sensor, uint8_t trigBF, uint8_t cpBF, uint8_t bits) {
    bool b = tlx493d_readRegisters(sensor);

    if (bits >= 0 && bits <= 3) 
        tlx493d_setBitfield(sensor, trigBF, bits);
    else
        return false;
    
    tlx493d_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));
    // Register_ts *cp  = &sensor->regDef[cpBF];
    // sensor->regMap[cp->address] = (sensor->regMap[cp->address] & ~cp->mask) | sensor->functions->calculateConfigurationParity(sensor);
    
    b &= tlx493d_writeRegister(sensor, trigBF);
}


/***
 * 
*/
bool tlx493d_gen_2_setMagneticTemperatureCompensation(Sensor_ts *sensor, uint8_t tl_magBF, uint8_t cpBF, uint8_t mtc) {
    tlx493d_setBitfield(sensor, tl_magBF, mtc);
    tlx493d_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_writeRegister(sensor, tl_magBF);
}


/***
 * 
*/
bool tlx493d_gen_2_set1ByteReadMode(Sensor_ts *sensor, uint8_t prBF, uint8_t fpBF, uint8_t prdBF, uint8_t pr) {
    tlx493d_setBitfield(sensor, prBF, pr);
    tlx493d_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

    return tlx493d_writeRegister(sensor, prBF);
}


// Fuse/mode parity bit FP
uint8_t tlx493d_gen_2_calculateFuseParity(Sensor_ts *sensor, uint8_t fpBF, uint8_t prdBF) {
    Register_ts *fp  = &sensor->regDef[fpBF];
    Register_ts *prd = &sensor->regDef[prdBF];

	// uint8_t parity = calculateParity(sensor->regMap[fp->address] & ~fp->mask);
	// parity ^= calculateParity(sensor->regMap[prd->address] & prd->mask);

	uint8_t parity = sensor->regMap[fp->address] & ~fp->mask;
	parity ^= sensor->regMap[prd->address] & prd->mask;

	return tlx493d_getOddParity(tlx493d_calculateParity(parity));
}


// Calculate bus (data) parity bit P
uint8_t tlx493d_gen_2_calculateBusParity(Sensor_ts *sensor, uint8_t to) {
	uint8_t parity = sensor->regMap[0];

	for (uint8_t i = 1; i < to; ++i) {
		parity ^= sensor->regMap[i];
	}

	return tlx493d_getOddParity(tlx493d_calculateParity(parity));
}


/***
 * 
*/
uint8_t tlx493d_gen_2_calculateConfigurationParity(Sensor_ts *sensor, uint8_t cpBF) {
    Register_ts *cp     = &sensor->regDef[cpBF];
	uint8_t      parity = tlx493d_calculateParity(sensor->regMap[cp->address] & ~cp->mask);
	return tlx493d_getEvenParity(parity);
}


/***
 * Always regs 0x07 - 0x10 ?
*/
uint8_t tlx493d_gen_2_calculateConfigurationParityWakeup(Sensor_ts *sensor, uint8_t cpBF, uint8_t from, uint8_t to) {
    Register_ts *cp     = &sensor->regDef[cpBF];
	uint8_t      parity = sensor->regMap[from];

    for(uint8_t i = from + 1; i <= to; ++i) {
        parity ^= i != cp->address ? sensor->regMap[i]
                                   : sensor->regMap[i] & ~cp->mask;
    }

	return tlx493d_getEvenParity(tlx493d_calculateParity(parity));
}


bool tlx493d_gen_2_hasValidFuseParity(Sensor_ts *sensor, uint8_t ffBF) {
    Register_ts *ff = &sensor->regDef[ffBF];
    return (sensor->regMap[ff->address] & ff->mask) != 0;
}


bool tlx493d_gen_2_hasValidBusParity(Sensor_ts *sensor, uint8_t pBF) {
    Register_ts *p = &sensor->regDef[pBF];
    return sensor->functions->calculateBusParity(sensor) == ((sensor->regMap[p->address] & p->mask) >> p->offset);
    // return tlx493d_gen_2_calculateBusParity(sensor) == ((sensor->regMap[bf->address] & bf->mask) >> bf->offset);
}


bool tlx493d_gen_2_hasValidConfigurationParity(Sensor_ts *sensor, uint8_t cfBF) {
    Register_ts *cf = &sensor->regDef[cfBF];
    return (sensor->regMap[cf->address] & cf->mask) != 0;
}


bool tlx493d_gen_2_hasValidData(Sensor_ts *sensor) {
    return sensor->functions->hasValidBusParity(sensor) && sensor->functions->hasValidTBit(sensor);
    // return tlx493d_gen_2_hasValidBusParity(sensor) && tlx493d_gen_2_hasValidTBit(sensor);
}


bool tlx493d_gen_2_hasValidTemperatureData(Sensor_ts *sensor) {
    return sensor->functions->hasValidData(sensor) && sensor->functions->hasValidPD3Bit(sensor);
    // return tlx493d_gen_2_hasValidData(sensor) && tlx493d_gen_2_hasValidPD3Bit(sensor);
}


bool tlx493d_gen_2_hasValidMagneticFieldData(Sensor_ts *sensor) {
    return sensor->functions->hasValidData(sensor) && sensor->functions->hasValidPD0Bit(sensor);
    // return tlx493d_gen_2_hasValidData(sensor) && tlx493d_gen_2_hasValidPD0Bit(sensor);
}


bool tlx493d_gen_2_isFunctional(Sensor_ts *sensor) {
    return sensor->functions->hasValidFuseParity(sensor) && sensor->functions->hasValidConfigurationParity(sensor);
    // return tlx493d_gen_2_hasValidFuseParity(sensor) && tlx493d_gen_2_hasValidConfigurationParity(sensor);
}


bool tlx493d_gen_2_hasValidTBit(Sensor_ts *sensor, uint8_t tBF) {
    Register_ts *t = &sensor->regDef[tBF];
    return (sensor->regMap[t->address] & t->mask) == 0;
}


bool tlx493d_gen_2_hasValidPD3Bit(Sensor_ts *sensor, uint8_t pd3BF) {
    Register_ts *pd3 = &sensor->regDef[pd3BF];
    return (sensor->regMap[pd3->address] & pd3->mask) != 0;
}


bool tlx493d_gen_2_hasValidPD0Bit(Sensor_ts *sensor, uint8_t pd0BF) {
    Register_ts *pd0 = &sensor->regDef[pd0BF];
    return (sensor->regMap[pd0->address] & pd0->mask) != 0;
}


bool tlx493d_gen_2_hasValidIICadr(Sensor_ts *sensor, uint8_t idBF, uint8_t iicAdrBF) {
    Register_ts *id     = &sensor->regDef[idBF];
    Register_ts *iicAdr = &sensor->regDef[iicAdrBF];
    return ((sensor->regMap[id->address] & id->mask) >> id->offset) == ((sensor->regMap[iicAdr->address] & iicAdr->mask) >> iicAdr->offset);
}


bool tlx493d_gen_2_hasWakeup(Sensor_ts *sensor, uint8_t typeBF) {
    Register_ts *type = &sensor->regDef[typeBF];
    return ((sensor->regMap[type->address] & type->mask) >> type->offset) != 0b11;
}


uint8_t tlx493d_gen_2_getID(Sensor_ts *sensor, uint8_t idBF) {
    Register_ts *bf = &sensor->regDef[idBF];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}


uint8_t tlx493d_gen_2_getFrameCounter(Sensor_ts *sensor, uint8_t frmBF) {
    Register_ts *frm = &sensor->regDef[frmBF];
    return (sensor->regMap[frm->address] && frm->mask) >> frm->offset;
}


uint8_t tlx493d_gen_2_getType(Sensor_ts *sensor, uint8_t typeBF) {
    Register_ts *type = &sensor->regDef[typeBF];
    return (sensor->regMap[type->address] && type->mask) >> type->offset;
}


uint8_t tlx493d_gen_2_getHWV(Sensor_ts *sensor, uint8_t hwvBF) {
    Register_ts *hwv = &sensor->regDef[hwvBF];
    return (sensor->regMap[hwv->address] && hwv->mask) >> hwv->offset;
}
