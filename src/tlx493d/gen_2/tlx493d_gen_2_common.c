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


/***
 * TODO: set all options that must be set, eg MODE ?, reset all bits to defaults ?
*/
bool tlx493d_gen_2_setDefaultConfig(TLx493D_ts *sensor, uint8_t configREG, uint8_t mod1REG, uint8_t mod2REG, uint8_t cpBF, uint8_t caBF, uint8_t intBF) {
// bool tlx493d_gen_2_setDefaultConfig(TLx493D_ts *sensor, uint8_t configREG, uint8_t configResetValue, uint8_t mod1REG, uint8_t mod2REG, uint8_t caBF, uint8_t intBF) {
    sensor->regMap[configREG] = 0x00; // sensor->functions->getConfigResetValue();
    sensor->regMap[mod1REG]   = 0x00;
    sensor->regMap[mod2REG]   = 0x00;

    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));
    tlx493d_common_writeRegister(sensor, cpBF);

    tlx493d_common_setBitfield(sensor, caBF, 0);
    tlx493d_common_setBitfield(sensor, intBF, 1);

    if( sensor->functions->enable1ByteReadMode(sensor) ) {
        // if( sensor->functions->disableTemperatureMeasurement(sensor) ) {
            // Read registers in order to retrieve values in reserved register at 0x12 and in MOD2 in order to make sure we are not 
            // accidentally changing a preset values to 0.
            return sensor->functions->readRegisters(sensor);
        // }
    }

    return false;
}


bool tlx493d_gen_2_setPowerMode(TLx493D_ts *sensor, uint8_t modeBF, uint8_t fpBF, TLx493D_PowerModeType_te mode) {
    if( (mode != 0b10) && (mode <= 0b11) ){
        tlx493d_common_setBitfield(sensor, modeBF, mode);
        tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));
        // tlx493d_common_setBitfield(sensor, sensor->commonBitfields.FP, tlx493d_gen_2_calculateFuseParity(sensor));
        return tlx493d_common_writeRegister(sensor, modeBF);
    }
    else {
        return false;
    }
}


bool tlx493d_gen_2_setIICAddress(TLx493D_ts *sensor, uint8_t iicadrBF, uint8_t fpBF, TLx493D_IICAddressType_te addr) {
// bool tlx493d_gen_2_setIICAddress(TLx493D_ts *sensor, uint8_t iicadrBF, uint8_t fpBF, StandardIICAddresses_te addr) {
    uint8_t bitfieldValue = 0;
    uint8_t deviceAddress = 0;

    switch (addr) {
        case TLx493D_IIC_ADDR_A0_e:
            bitfieldValue = 0b00;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A0;
            break;

        case TLx493D_IIC_ADDR_A1_e:
            bitfieldValue = 0b01;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A1;
            break;

        case TLx493D_IIC_ADDR_A2_e:
            bitfieldValue = 0b10;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A2;
            break;

        case TLx493D_IIC_ADDR_A3_e:
            bitfieldValue = 0b11;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A3;
            break;
        
        default:
            return false;
    }

    tlx493d_common_setBitfield(sensor, iicadrBF, bitfieldValue);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    bool b = tlx493d_common_writeRegister(sensor, iicadrBF);
    TLx493D_setI2CParameters(sensor, deviceAddress);

    return b;
}


/***
 * 
*/
void tlx493d_gen_2_calculateTemperature(TLx493D_ts *sensor, double *temp, uint8_t tempMSBBF, uint8_t tempLSBBF) {
    int16_t value = 0;

    tlx493d_common_concatBytes(sensor, tempMSBBF, tempLSBBF, &value);

    value <<= 2; // least significant 2 bits are implicit, therefore shift by 2 !
    *temp = (((double) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF;
}


void tlx493d_gen_2_calculateMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z,
                                  uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF, uint8_t bzMSBBF, uint8_t bzLSBBF) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    tlx493d_common_concatBytes(sensor, bxMSBBF, bxLSBBF, &valueX);
    tlx493d_common_concatBytes(sensor, byMSBBF, byLSBBF, &valueY);
    tlx493d_common_concatBytes(sensor, bzMSBBF, bzLSBBF, &valueZ);

    *x = ((double) valueX) * GEN_2_MAG_FIELD_MULT;  // TODO: get factor from registers : full, double, quadruple 
    *y = ((double) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((double) valueZ) * GEN_2_MAG_FIELD_MULT;                           
}


/***
 * 
*/
bool tlx493d_gen_2_setDisableTemperatureMeasurement(TLx493D_ts *sensor, uint8_t dtBF, uint8_t cpBF, uint8_t dt) {
    tlx493d_common_setBitfield(sensor, dtBF, dt);
    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_common_writeRegister(sensor, dtBF);
}


/***
 * AM depends on value of DT !
*/
bool tlx493d_gen_2_setAngularMeasurement(TLx493D_ts *sensor, uint8_t amBF, uint8_t dtBF, uint8_t cpBF, uint8_t am, uint8_t dt) {
    tlx493d_common_setBitfield(sensor, dtBF, dt);
    tlx493d_common_setBitfield(sensor, amBF, am);
    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_common_writeRegister(sensor, amBF);
}


/***
 * 
*/
bool tlx493d_gen_2_setUpdateRate(TLx493D_ts *sensor, uint8_t fpBF, uint8_t prdBF, TLx493D_UpdateRateType_te ur) {
    uint8_t mod1 = sensor->regDef[fpBF].address;

    tlx493d_common_setBitfield(sensor, prdBF, ur);
    tlx493d_common_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

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
bool tlx493d_gen_2_setShortRangeSensitivity(TLx493D_ts *sensor, uint8_t x2BF, uint8_t cpBF, uint8_t srs) {
    tlx493d_common_setBitfield(sensor, x2BF, srs);
    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_common_writeRegister(sensor, x2BF);
}


/***
 * TODO: implement checks ! CA only in Low Power and MCM mode, not in Fast Mode ! MODE depends on PR and TRIG !
*/
bool tlx493d_gen_2_setInterruptAndCollisionAvoidance(TLx493D_ts *sensor, uint8_t intBF, uint8_t caBF, uint8_t fpBF, uint8_t prdBF, bool intIsOn, bool caIsOn) {
// tlx493d_common_setBitfield(sensor, sensor->commonBitfields.MODE, 0b11);
    tlx493d_common_setBitfield(sensor, intBF, intIsOn ? 0 : 1);
    tlx493d_common_setBitfield(sensor, caBF, caIsOn ? 1 : 0);
    tlx493d_common_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

    return tlx493d_common_writeRegister(sensor, intBF); 
}


// bool tlx493d_gen_2_setInterrupt(TLx493D_ts *sensor, uint8_t intBF, uint8_t fpBF, uint8_t prdBF, uint8_t irq) {
//     tlx493d_common_setBitfield(sensor, intBF, irq);
//     tlx493d_common_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

//     return tlx493d_common_writeRegister(sensor, intBF);
// }


// /***
//  * CA only in Low Power and MCM mode, not in Fast Mode !
//  * MODE depends on PR and TRIG !
// */
// bool tlx493d_gen_2_setCollisionAvoidance(TLx493D_ts *sensor, uint8_t ca) {
// // tlx493d_common_setBitfield(sensor, sensor->commonBitfields.MODE, 0b11);

//     tlx493d_common_setBitfield(sensor, CA, ca);
//     tlx493d_common_setBitfield(sensor, FP, tlx493d_gen_2_calculateFuseParity(sensor, FP, PRD));

//     return tlx493d_common_writeRegister(sensor, CA);
// }


/***
 * This option depends on PR and MODE.
*/
bool tlx493d_gen_2_setTrigger(TLx493D_ts *sensor, uint8_t trigBF, uint8_t cpBF, uint8_t trig) {
    // bool b = tlx493d_common_readRegisters(sensor);

    if( trig >= 0 && trig <= 3 ) 
        tlx493d_common_setBitfield(sensor, trigBF, trig);
    else
        return false;

    // tlx493d_common_setBitfield(sensor, trigBF, trig);
    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_common_writeRegister(sensor, trigBF);
}


/***
 * 
*/
bool tlx493d_gen_2_setMagneticTemperatureCompensation(TLx493D_ts *sensor, uint8_t tl_magBF, uint8_t cpBF, uint8_t mtc) {
    tlx493d_common_setBitfield(sensor, tl_magBF, mtc);
    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_common_writeRegister(sensor, tl_magBF);
}


/***
 * 
*/
bool tlx493d_gen_2_set1ByteReadMode(TLx493D_ts *sensor, uint8_t prBF, uint8_t fpBF, uint8_t prdBF, uint8_t pr) {
    tlx493d_common_setBitfield(sensor, prBF, pr);
    tlx493d_common_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

    return tlx493d_common_writeRegister(sensor, prBF);
}


// Fuse/mode parity bit FP
uint8_t tlx493d_gen_2_calculateFuseParity(TLx493D_ts *sensor, uint8_t fpBF, uint8_t prdBF) {
    TLx493D_Register_ts *fp  = &sensor->regDef[fpBF];
    TLx493D_Register_ts *prd = &sensor->regDef[prdBF];

	// uint8_t parity = calculateParity(sensor->regMap[fp->address] & ~fp->mask);
	// parity ^= calculateParity(sensor->regMap[prd->address] & prd->mask);

	uint8_t parity = sensor->regMap[fp->address] & ~fp->mask;
	parity ^= sensor->regMap[prd->address] & prd->mask;

	return tlx493d_common_getOddParity(tlx493d_common_calculateParity(parity));
}


// Calculate bus (data) parity bit P
uint8_t tlx493d_gen_2_calculateBusParity(TLx493D_ts *sensor, uint8_t to) {
	uint8_t parity = sensor->regMap[0];

	for (uint8_t i = 1; i < to; ++i) {
		parity ^= sensor->regMap[i];
	}

	return tlx493d_common_getOddParity(tlx493d_common_calculateParity(parity));
}


/***
 * 
*/
uint8_t tlx493d_gen_2_calculateConfigurationParity(TLx493D_ts *sensor, uint8_t cpBF) {
    TLx493D_Register_ts *cp     = &sensor->regDef[cpBF];
	uint8_t      parity = tlx493d_common_calculateParity(sensor->regMap[cp->address] & ~cp->mask);
	return tlx493d_common_getEvenParity(parity);
}


/***
 * Always regs 0x07 - 0x10 ?
*/
uint8_t tlx493d_gen_2_calculateConfigurationParityWakeup(TLx493D_ts *sensor, uint8_t cpBF, uint8_t from, uint8_t to) {
    TLx493D_Register_ts *cp     = &sensor->regDef[cpBF];
	uint8_t      parity = sensor->regMap[from];

    for(uint8_t i = from + 1; i <= to; ++i) {
        parity ^= i != cp->address ? sensor->regMap[i]
                                   : sensor->regMap[i] & ~cp->mask;
    }

	return tlx493d_common_getEvenParity(tlx493d_common_calculateParity(parity));
}


bool tlx493d_gen_2_hasValidFuseParity(TLx493D_ts *sensor, uint8_t ffBF) {
    TLx493D_Register_ts *ff = &sensor->regDef[ffBF];
    return (sensor->regMap[ff->address] & ff->mask) != 0;
}


bool tlx493d_gen_2_hasValidBusParity(TLx493D_ts *sensor, uint8_t pBF) {
    TLx493D_Register_ts *p = &sensor->regDef[pBF];
    return sensor->functions->calculateBusParity(sensor) == ((sensor->regMap[p->address] & p->mask) >> p->offset);
    // return tlx493d_gen_2_calculateBusParity(sensor) == ((sensor->regMap[bf->address] & bf->mask) >> bf->offset);
}


bool tlx493d_gen_2_hasValidConfigurationParity(TLx493D_ts *sensor, uint8_t cfBF) {
    TLx493D_Register_ts *cf = &sensor->regDef[cfBF];
    return (sensor->regMap[cf->address] & cf->mask) != 0;
}


bool tlx493d_gen_2_hasValidData(TLx493D_ts *sensor) {
    return sensor->functions->hasValidBusParity(sensor) && sensor->functions->hasValidTBit(sensor);
    // return tlx493d_gen_2_hasValidBusParity(sensor) && tlx493d_gen_2_hasValidTBit(sensor);
}


bool tlx493d_gen_2_hasValidTemperatureData(TLx493D_ts *sensor) {
    return sensor->functions->hasValidData(sensor) && sensor->functions->hasValidPD3Bit(sensor);
    // return tlx493d_gen_2_hasValidData(sensor) && tlx493d_gen_2_hasValidPD3Bit(sensor);
}


bool tlx493d_gen_2_hasValidMagneticFieldData(TLx493D_ts *sensor) {
    return sensor->functions->hasValidData(sensor) && sensor->functions->hasValidPD0Bit(sensor);
    // return tlx493d_gen_2_hasValidData(sensor) && tlx493d_gen_2_hasValidPD0Bit(sensor);
}


bool tlx493d_gen_2_isFunctional(TLx493D_ts *sensor) {
    return sensor->functions->hasValidFuseParity(sensor) && sensor->functions->hasValidConfigurationParity(sensor);
    // return tlx493d_gen_2_hasValidFuseParity(sensor) && tlx493d_gen_2_hasValidConfigurationParity(sensor);
}


bool tlx493d_gen_2_hasValidTBit(TLx493D_ts *sensor, uint8_t tBF) {
    TLx493D_Register_ts *t = &sensor->regDef[tBF];
    return (sensor->regMap[t->address] & t->mask) == 0;
}


bool tlx493d_gen_2_hasValidPD3Bit(TLx493D_ts *sensor, uint8_t pd3BF) {
    TLx493D_Register_ts *pd3 = &sensor->regDef[pd3BF];
    return (sensor->regMap[pd3->address] & pd3->mask) != 0;
}


bool tlx493d_gen_2_hasValidPD0Bit(TLx493D_ts *sensor, uint8_t pd0BF) {
    TLx493D_Register_ts *pd0 = &sensor->regDef[pd0BF];
    return (sensor->regMap[pd0->address] & pd0->mask) != 0;
}


bool tlx493d_gen_2_hasValidIICadr(TLx493D_ts *sensor, uint8_t idBF, uint8_t iicAdrBF) {
    TLx493D_Register_ts *id     = &sensor->regDef[idBF];
    TLx493D_Register_ts *iicAdr = &sensor->regDef[iicAdrBF];
    return ((sensor->regMap[id->address] & id->mask) >> id->offset) == ((sensor->regMap[iicAdr->address] & iicAdr->mask) >> iicAdr->offset);
}


bool tlx493d_gen_2_hasWakeup(TLx493D_ts *sensor, uint8_t typeBF) {
    TLx493D_Register_ts *type = &sensor->regDef[typeBF];
    return ((sensor->regMap[type->address] & type->mask) >> type->offset) != 0b11;
}


uint8_t tlx493d_gen_2_getID(TLx493D_ts *sensor, uint8_t idBF) {
    TLx493D_Register_ts *bf = &sensor->regDef[idBF];
    return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
}


uint8_t tlx493d_gen_2_getFrameCounter(TLx493D_ts *sensor, uint8_t frmBF) {
    TLx493D_Register_ts *frm = &sensor->regDef[frmBF];
    return (sensor->regMap[frm->address] && frm->mask) >> frm->offset;
}


uint8_t tlx493d_gen_2_getType(TLx493D_ts *sensor, uint8_t typeBF) {
    TLx493D_Register_ts *type = &sensor->regDef[typeBF];
    return (sensor->regMap[type->address] && type->mask) >> type->offset;
}


uint8_t tlx493d_gen_2_getHWV(TLx493D_ts *sensor, uint8_t hwvBF) {
    TLx493D_Register_ts *hwv = &sensor->regDef[hwvBF];
    return (sensor->regMap[hwv->address] && hwv->mask) >> hwv->offset;
}
