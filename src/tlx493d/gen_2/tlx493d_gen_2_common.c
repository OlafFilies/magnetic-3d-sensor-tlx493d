// std includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// project c includes
#include "Logger.h"

// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

// common to same generation of sensors
#include "tlx493d_gen_2_common_defines.h"
#include "tlx493d_gen_2_common.h"


void tlx493d_gen_2_calculateTemperature(TLx493D_t *sensor, double *temp, uint8_t tempMSBBF, uint8_t tempLSBBF) {
    int16_t value = 0;

    tlx493d_common_concatBytes(sensor, tempMSBBF, tempLSBBF, &value);

    value <<= 2; // least significant 2 bits are implicit, therefore shift by 2 !
    *temp = (((double) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_MULT) + GEN_2_TEMP_REF;
}


void tlx493d_gen_2_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z,
                                          uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                          uint8_t bzMSBBF, uint8_t bzLSBBF) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    tlx493d_common_concatBytes(sensor, bxMSBBF, bxLSBBF, &valueX);
    tlx493d_common_concatBytes(sensor, byMSBBF, byLSBBF, &valueY);
    tlx493d_common_concatBytes(sensor, bzMSBBF, bzLSBBF, &valueZ);

    *x = ((double) valueX) * GEN_2_MAG_FIELD_MULT;  // TODO: get factor from registers : full, double, quadruple 
    *y = ((double) valueY) * GEN_2_MAG_FIELD_MULT;
    *z = ((double) valueZ) * GEN_2_MAG_FIELD_MULT;                           
}


bool tlx493d_gen_2_setOneConfigBitfield(TLx493D_t *sensor, uint8_t bfBF, uint8_t cpBF, uint8_t val) {
    tlx493d_common_setBitfield(sensor, bfBF, val);
    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_common_writeRegister(sensor, cpBF);
}


bool tlx493d_gen_2_setTwoConfigBitfields(TLx493D_t *sensor, uint8_t firstBF, uint8_t secondBF, uint8_t cpBF, uint8_t first, uint8_t second) {
    tlx493d_common_setBitfield(sensor, firstBF, first);
    tlx493d_common_setBitfield(sensor, secondBF, second);
    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_common_writeRegister(sensor, cpBF);
}


bool tlx493d_gen_2_setMeasurement(TLx493D_t *sensor, uint8_t dtBF, uint8_t amBF, uint8_t cpBF, TLx493D_MeasurementType_t val) {
    uint8_t dt = 0;
    uint8_t am = 0;

    switch(val) {
        case TLx493D_BxByBzTemp_e : dt = 0;
                                    am = 0;
                                    break;

        case TLx493D_BxByBz_e : dt = 1;
                                am = 0;
                                break;
        
        case TLx493D_BxBy_e : dt = 1;
                              am = 1;
                              break;
        
        default : errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_MeasurementType_t");
                  return false;
    }

    return tlx493d_gen_2_setTwoConfigBitfields(sensor, dtBF, amBF, cpBF, dt, am);
}


//  // This option depends on PR and MODE.
bool tlx493d_gen_2_setTrigger(TLx493D_t *sensor, uint8_t trigBF, uint8_t cpBF, TLx493D_TriggerType_t val) {
    uint8_t trig = 0;

    switch(val) {
        case TLx493D_NO_ADC_ON_READ_e : trig = 0;
                                        break;

        case TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e : trig = 1;
                                                      break;
        
        case TLx493D_ADC_ON_READ_AFTER_REG_05_e : trig = 2;
                                                  break;
        
        default : errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_TriggerType_t");
                  return false;
    }

    return tlx493d_gen_2_setOneConfigBitfield(sensor, trigBF, cpBF, trig);
}


bool tlx493d_gen_2_setSensitivity(TLx493D_t *sensor, uint8_t x2BF, uint8_t cpBF, TLx493D_SensitivityType_t val) {
    uint8_t sens = 0;

    switch(val) {
        case TLx493D_FULL_RANGE_e : sens = 0;
                                    break;

        case TLx493D_SHORT_RANGE_e : sens = 1;
                                     break;
        
        default : errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_SensitivityType_t");
                  return false;
    }

    return tlx493d_gen_2_setOneConfigBitfield(sensor, x2BF, cpBF, sens);
}


// bool tlx493d_gen_2_setMagneticTemperatureCompensation(TLx493D_t *sensor, uint8_t tl_magBF, uint8_t cpBF, uint8_t mtc) {
//     tlx493d_common_setBitfield(sensor, tl_magBF, mtc);
//     tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

//     return tlx493d_common_writeRegister(sensor, tl_magBF);
// }


// TODO: set all options that must be set, eg MODE ?, reset all bits to defaults ?
// TODO: Differentiate first call after power-up/reset and subsequent calls ! For subsequent calls the regMap values must be considered as they may be different from reset values !
bool tlx493d_gen_2_setDefaultConfig(TLx493D_t *sensor, uint8_t configREG, uint8_t mod1REG, uint8_t mod2REG, uint8_t cpBF, uint8_t caBF, uint8_t intBF) {
// TODO: cleanup : set reset values and really set CP bit ? Here or in init ?
    // sensor->functions->setResetValues(sensor);

    tlx493d_common_setBitfield(sensor, caBF, 0);
    tlx493d_common_setBitfield(sensor, intBF, 1);

    if( sensor->functions->enable1ByteReadMode(sensor) ) {
        // return sensor->functions->readRegisters(sensor);

        sensor->functions->readRegisters(sensor);

        if( tlx493d_common_returnBitfield(sensor, cpBF) == 0x01 ) {
            tlx493d_common_setBitfield(sensor, cpBF, 0x00);
        }
        else {
            tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));
        }

        // printRegisters(sensor);

        tlx493d_common_writeRegister(sensor, cpBF);
    }

    return false;
}


bool tlx493d_gen_2_setIICAddress(TLx493D_t *sensor, uint8_t iicadrBF, uint8_t fpBF, TLx493D_IICAddressType_t addr) {
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

    bool b = tlx493d_common_writeRegister(sensor, fpBF);
    tlx493d_setI2CParameters(sensor, deviceAddress);

    return b;
}


bool tlx493d_gen_2_set1ByteReadMode(TLx493D_t *sensor, uint8_t prBF, uint8_t fpBF, uint8_t prdBF, uint8_t pr) {
    tlx493d_common_setBitfield(sensor, prBF, pr);
    tlx493d_common_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

    return tlx493d_common_writeRegister(sensor, prBF);
}


// CA only in Low Power and MCM mode, not in Fast Mode !
// MODE depends on PR and TRIG !
bool tlx493d_gen_2_setCollisionAvoidance(TLx493D_t *sensor, uint8_t caBF, uint8_t fpBF, uint8_t prdBF, uint8_t ca) {
    tlx493d_common_setBitfield(sensor, caBF, ca);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    return tlx493d_common_writeRegister(sensor, caBF);
}


bool tlx493d_gen_2_setInterrupt(TLx493D_t *sensor, uint8_t intBF, uint8_t fpBF, uint8_t prdBF, uint8_t irq) {
    tlx493d_common_setBitfield(sensor, intBF, irq);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    return tlx493d_common_writeRegister(sensor, intBF);
}


bool tlx493d_gen_2_setPowerMode(TLx493D_t *sensor, uint8_t modeBF, uint8_t fpBF, TLx493D_PowerModeType_t val) {
    uint8_t mod1 = sensor->regDef[fpBF].address;
    uint8_t mode = 0;

    switch(val) {
        case LOW_POWER_MODE_e : mode = 0b00;
                                break;

        case MASTER_CONTROLLED_MODE_e : mode = 0b01;
                                        break;

        case FAST_MODE_e : mode = 0b11;
                                  break;

        default : errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_PowerModeType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, modeBF, mode);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    uint8_t buf[4] = { mod1,
                       sensor->regMap[mod1],
                       sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
                       sensor->regMap[mod1 + 2]
                     };

    return transfer(sensor, buf, sizeof(buf), NULL, 0);
}


bool tlx493d_gen_2_setUpdateRate(TLx493D_t *sensor, uint8_t fpBF, uint8_t prdBF, TLx493D_UpdateRateType_t val) {
    uint8_t mod1 = sensor->regDef[fpBF].address;
    uint8_t rate = 0;

    switch(val) {
        case UPDATE_RATE_770_HZ_e : rate = 0b000;
                                    break;

        case UPDATE_RATE_97_HZ_e : rate = 0b001;
                                   break;

        case UPDATE_RATE_24_HZ_e : rate = 0b010;
                                   break;

        case UPDATE_RATE_12_HZ_e : rate = 0b011;
                                   break;

        case UPDATE_RATE_6_HZ_e : rate = 0b100;
                                  break;

        case UPDATE_RATE_3_HZ_e : rate = 0b101;
                                  break;

        case UPDATE_RATE_0_4_HZ_e : rate = 0b110;
                                    break;

        case UPDATE_RATE_0_05_HZ_e : rate = 0b111;
                                     break;

        default : return false;
    }

    tlx493d_common_setBitfield(sensor, prdBF, rate);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    uint8_t buf[4] = { mod1,
                       sensor->regMap[mod1],
                       sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
                       sensor->regMap[mod1 + 2]
                     };

    return transfer(sensor, buf, sizeof(buf), NULL, 0);
}


bool tlx493d_gen_2_hasValidData(TLx493D_t *sensor) {
    return sensor->functions->hasValidBusParity(sensor) && sensor->functions->hasValidTBit(sensor);
}


bool tlx493d_gen_2_isFunctional(TLx493D_t *sensor) {
    return sensor->functions->hasValidFuseParity(sensor) && sensor->functions->hasValidConfigurationParity(sensor);
}


// bool tlx493d_gen_2_hasWakeUp(TLx493D_t *sensor, uint8_t typeBF) {
//     TLx493D_Register_t *type = &sensor->regDef[typeBF];
//     return ((sensor->regMap[type->address] & type->mask) >> type->offset) != 0b11;
// }


bool tlx493d_gen_2_isWakeUpEnabled(TLx493D_t *sensor, uint8_t waBF) {
    return tlx493d_common_returnBitfield(sensor, waBF) != 0;

    // TLx493D_Register_t *wa = &sensor->regDef[waBF];
    // return (sensor->regMap[wa->address] & wa->mask) != 0;
}


bool tlx493d_gen_2_enableWakeUpMode(TLx493D_t *sensor, uint8_t tstBF, uint8_t wuBF, uint8_t cpbBF) {
    tlx493d_common_setBitfield(sensor, wuBF, 1);
    tlx493d_common_setBitfield(sensor, tstBF, 0);
    tlx493d_common_setBitfield(sensor, cpbBF, 1); // sensor->functions->calculateConfigurationParity(sensor));

    uint8_t txBuffer[5] = {
                              sensor->regDef[wuBF].address,
                              sensor->regMap[sensor->regDef[wuBF].address],
                              sensor->regMap[sensor->regDef[wuBF].address + 1],
                              sensor->regMap[sensor->regDef[wuBF].address + 2],
                              sensor->regMap[sensor->regDef[wuBF].address + 3]
                          };
    
    return transfer(sensor, txBuffer, sizeof(txBuffer), NULL, 0);
}


bool tlx493d_gen_2_disableWakeUpMode(TLx493D_t *sensor, uint8_t wuBF, uint8_t cpbBF) {
    tlx493d_common_setBitfield(sensor, wuBF, 0);
    tlx493d_common_setBitfield(sensor, cpbBF, sensor->functions->calculateConfigurationParity(sensor));

    uint8_t txBuffer[5] = {
                              sensor->regDef[wuBF].address,
                              sensor->regMap[sensor->regDef[wuBF].address],
                              sensor->regMap[sensor->regDef[wuBF].address + 1],
                              sensor->regMap[sensor->regDef[wuBF].address + 2],
                              sensor->regMap[sensor->regDef[wuBF].address + 3]
                          };
    
    return transfer(sensor, txBuffer, sizeof(txBuffer), NULL, 0);
}


bool tlx493d_gen_2_setThreshold(TLx493D_t *sensor, uint8_t msbsBF, uint8_t lsbsBF, uint8_t cpbBF, int16_t threshold12Bits) {
    TLx493D_Register_t *msbs = &sensor->regDef[msbsBF];
    TLx493D_Register_t *lsbs = &sensor->regDef[lsbsBF];

    int16_t thresh11Bits = threshold12Bits >> 1;

    uint8_t lower = thresh11Bits & (lsbs->mask >> lsbs->offset);
    uint8_t upper = (thresh11Bits >> lsbs->numBits) & (msbs->mask >> msbs->offset);

    print("threshold12Bits = %#X / %d   thresh11Bits = %#X / %d  upper = %#X / %d  lower = %#X / %d\n",
          threshold12Bits, threshold12Bits, thresh11Bits, thresh11Bits, upper, upper, lower, lower);

    tlx493d_common_setBitfield(sensor, msbsBF, upper);
    tlx493d_common_setBitfield(sensor, lsbsBF, lower);

    printRegisters(sensor);
    return true;

    // tlx493d_common_setBitfield(sensor, cpbBF, sensor->functions->calculateConfigurationParity(sensor));

    // uint8_t txBuffer[11] = {
    //                            0x07,
    //                            sensor->regMap[0x07],
    //                            sensor->regMap[0x08],
    //                            sensor->regMap[0x09],
    //                            sensor->regMap[0x0A],
    //                            sensor->regMap[0x0B],
    //                            sensor->regMap[0x0C],
    //                            sensor->regMap[0x0D],
    //                            sensor->regMap[0x0E],
    //                            sensor->regMap[0x0F],
    //                            sensor->regMap[0x10]
    //                        };

    // return transfer(sensor, txBuffer, sizeof(txBuffer), NULL, 0);
}

// bool tlx493d_gen_2_setLowerWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold);
// bool tlx493d_gen_2_setLowerWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold);
// bool tlx493d_gen_2_setLowerWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold);

// bool tlx493d_gen_2_setUpperWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold);
// bool tlx493d_gen_2_setUpperWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold);
// bool tlx493d_gen_2_setUpperWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold);

// bool tlx493d_gen_2_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);
// bool tlx493d_gen_2_setWakeUpThresholds(TLx493D_t *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);

// bool tlx493d_gen_2_softReset(TLx493D_t *sensor);


// Fuse/mode parity bit FP
uint8_t tlx493d_gen_2_calculateFuseParity(TLx493D_t *sensor, uint8_t fpBF, uint8_t prdBF) {
    TLx493D_Register_t *fp  = &sensor->regDef[fpBF];
    TLx493D_Register_t *prd = &sensor->regDef[prdBF];

	uint8_t parity = sensor->regMap[fp->address] & ~fp->mask;
	parity ^= sensor->regMap[prd->address] & prd->mask;

	return tlx493d_common_getOddParity(tlx493d_common_calculateParity(parity));
}


// Calculate bus (data) parity bit P
uint8_t tlx493d_gen_2_calculateBusParity(TLx493D_t *sensor, uint8_t to) {
	uint8_t parity = sensor->regMap[0];

	for (uint8_t i = 1; i < to; ++i) {
		parity ^= sensor->regMap[i];
	}

	return tlx493d_common_getOddParity(tlx493d_common_calculateParity(parity));
}


uint8_t tlx493d_gen_2_calculateConfigurationParity(TLx493D_t *sensor, uint8_t cpBF) {
    TLx493D_Register_t *cp     = &sensor->regDef[cpBF];
	uint8_t              parity = tlx493d_common_calculateParity(sensor->regMap[cp->address] & ~cp->mask);
	return tlx493d_common_getEvenParity(parity);
}


// TODO: Always regs 0x07 - 0x10 ?
uint8_t tlx493d_gen_2_calculateConfigurationParityWakeUp(TLx493D_t *sensor, uint8_t cpBF) {
// uint8_t tlx493d_gen_2_calculateConfigurationParityWakeUp(TLx493D_t *sensor, uint8_t waBF, uint8_t tstBF, uint8_t phBF, uint8_t cpBF) {
// uint8_t tlx493d_gen_2_calculateConfigurationParityWakeup(TLx493D_t *sensor, uint8_t cpBF, uint8_t from, uint8_t to) {
    TLx493D_Register_t *cp     = &sensor->regDef[cpBF];
	uint8_t              parity = sensor->regMap[0x07]; //from];

    for(uint8_t i = 0x08; i <= 0x0C; ++i) {
    // for(uint8_t i = from + 1; i <= 0x0C; ++i) {
        parity ^= sensor->regMap[i];
    }

    parity ^= sensor->regMap[0x0D] & ~0x7F; // WA
    parity ^= sensor->regMap[0x0E] & ~0x3F; // TST
    parity ^= sensor->regMap[0x0F] & ~0x3F; // PH
//     parity ^= (sensor->regMap[0x0D] & ~sensor->regDef[waBF].mask);
//     parity ^= (sensor->regMap[0x0E] & ~sensor->regDef[tstBF].mask);
//     parity ^= (sensor->regMap[0x0F] & ~sensor->regDef[phBF].mask);

    parity ^= sensor->regMap[0x10] & ~cp->mask;

	return tlx493d_common_getOddParity(tlx493d_common_calculateParity(parity));
}


bool tlx493d_gen_2_hasValidFuseParity(TLx493D_t *sensor, uint8_t ffBF) {
    TLx493D_Register_t *ff = &sensor->regDef[ffBF];
    return (sensor->regMap[ff->address] & ff->mask) != 0;
}


bool tlx493d_gen_2_hasValidBusParity(TLx493D_t *sensor, uint8_t pBF) {
    TLx493D_Register_t *p = &sensor->regDef[pBF];
    return sensor->functions->calculateBusParity(sensor) == ((sensor->regMap[p->address] & p->mask) >> p->offset);
}


bool tlx493d_gen_2_hasValidConfigurationParity(TLx493D_t *sensor, uint8_t cfBF) {
    TLx493D_Register_t *cf = &sensor->regDef[cfBF];
    return (sensor->regMap[cf->address] & cf->mask) != 0;
}


bool tlx493d_gen_2_hasValidIICadr(TLx493D_t *sensor, uint8_t idBF, uint8_t iicAdrBF) {
    TLx493D_Register_t *id     = &sensor->regDef[idBF];
    TLx493D_Register_t *iicAdr = &sensor->regDef[iicAdrBF];
    return ((sensor->regMap[id->address] & id->mask) >> id->offset) == ((sensor->regMap[iicAdr->address] & iicAdr->mask) >> iicAdr->offset);
}


bool tlx493d_gen_2_hasValidTBit(TLx493D_t *sensor, uint8_t tBF) {
    TLx493D_Register_t *t = &sensor->regDef[tBF];
    return (sensor->regMap[t->address] & t->mask) == 0;
}


// bool tlx493d_gen_2_hasValidTemperatureData(TLx493D_t *sensor) {
//     return sensor->functions->hasValidData(sensor) && sensor->functions->hasValidPD3Bit(sensor);
// }


// bool tlx493d_gen_2_hasValidMagneticFieldData(TLx493D_t *sensor) {
//     return sensor->functions->hasValidData(sensor) && sensor->functions->hasValidPD0Bit(sensor);
// }


// bool tlx493d_gen_2_hasValidPD3Bit(TLx493D_t *sensor, uint8_t pd3BF) {
//     TLx493D_Register_t *pd3 = &sensor->regDef[pd3BF];
//     return (sensor->regMap[pd3->address] & pd3->mask) != 0;
// }


// bool tlx493d_gen_2_hasValidPD0Bit(TLx493D_t *sensor, uint8_t pd0BF) {
//     TLx493D_Register_t *pd0 = &sensor->regDef[pd0BF];
//     return (sensor->regMap[pd0->address] & pd0->mask) != 0;
// }


// uint8_t tlx493d_gen_2_getID(TLx493D_t *sensor, uint8_t idBF) {
//     TLx493D_Register_t *bf = &sensor->regDef[idBF];
//     return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
// }


// uint8_t tlx493d_gen_2_getFrameCounter(TLx493D_t *sensor, uint8_t frmBF) {
//     TLx493D_Register_t *frm = &sensor->regDef[frmBF];
//     return (sensor->regMap[frm->address] && frm->mask) >> frm->offset;
// }


// uint8_t tlx493d_gen_2_getType(TLx493D_t *sensor, uint8_t typeBF) {
//     TLx493D_Register_t *type = &sensor->regDef[typeBF];
//     return (sensor->regMap[type->address] && type->mask) >> type->offset;
// }


// uint8_t tlx493d_gen_2_getHWV(TLx493D_t *sensor, uint8_t hwvBF) {
//     TLx493D_Register_t *hwv = &sensor->regDef[hwvBF];
//     return (sensor->regMap[hwv->address] && hwv->mask) >> hwv->offset;
// }
