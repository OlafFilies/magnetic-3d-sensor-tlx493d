// std includes
#include <stdbool.h>
#include <stddef.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"
#include "Logger.h"

// common to same generation of sensors
#include "tlx493d_gen_3_common_defines.h"
#include "tlx493d_gen_3_common.h"


bool tlx493d_gen_3_readRegistersSPI(TLx493D_t *sensor) {
    return tlx493d_transfer(sensor, NULL, 0, sensor->regMap, sensor->regMapSize);
}


void tlx493d_gen_3_calculateRawTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, int16_t *temperature) {
    tlx493d_common_calculateRawTemperature(sensor, tempMSBBF, tempLSBBF, temperature);
}


void tlx493d_gen_3_calculateRawMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                             uint8_t bzMSBBF, uint8_t bzLSBBF, int16_t *x, int16_t *y, int16_t *z) {
    tlx493d_common_calculateRawMagneticField(sensor, bxMSBBF, bxLSBBF, byMSBBF, byLSBBF, bzMSBBF, bzLSBBF, x, y, z);
}


void tlx493d_gen_3_calculateTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, double *temperature) {
    int16_t value = 0;
// printRegisters(sensor);
    tlx493d_common_concatBytes(sensor, tempMSBBF, tempLSBBF, &value);
    *temperature = (((double) value - GEN_3_TEMP_OFFSET) / GEN_3_TEMP_SENSITIVITY) + GEN_3_TEMP_REF;
    // *temperature = (((double) value - GEN_3_TEMP_OFFSET) / GEN_3_TEMP_MULT) + GEN_3_TEMP_REF;
}


// TODO: cleanup and check code for short and xtra short ranges 1
void tlx493d_gen_3_calculateMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                          uint8_t bzMSBBF, uint8_t bzLSBBF, uint8_t tempMSBBF, uint8_t tempLSBBF,
                                          double *x, double *y, double *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    tlx493d_common_concatBytes(sensor, bxMSBBF, bxLSBBF, &valueX);
    tlx493d_common_concatBytes(sensor, byMSBBF, byLSBBF, &valueY);
    tlx493d_common_concatBytes(sensor, bzMSBBF, bzLSBBF, &valueZ);

    double sf = sensor->functions->getSensitivityScaleFactor(sensor);

    int16_t rawTemp;
    tlx493d_gen_3_calculateRawTemperature(sensor, tempMSBBF, tempLSBBF, &rawTemp);
    double temp = (double) rawTemp;

    *x = ((GEN_3_O0x + temp * (GEN_3_O1x + temp * (GEN_3_O2x + temp * GEN_3_O3x)))
          + ((double) valueX) * (GEN_3_L0x + temp * (GEN_3_L1x + temp * (GEN_3_L2x + temp * GEN_3_L3x))) / sf)
       / GEN_3_FULL_RANGE_FIELD_SENSITIVITY;

    *y = ((GEN_3_O0y + temp * (GEN_3_O1y + temp * (GEN_3_O2y + temp * GEN_3_O3y)))
          + ((double) valueY) * (GEN_3_L0y + temp * (GEN_3_L1y + temp * (GEN_3_L2y + temp * GEN_3_L3y))) / sf)
       / GEN_3_FULL_RANGE_FIELD_SENSITIVITY;

    *z = ((GEN_3_O0z + temp * (GEN_3_O1z + temp * (GEN_3_O2z + temp * GEN_3_O3z)))
          + ((double) valueZ) * (GEN_3_L0z + temp * (GEN_3_L1z + temp * (GEN_3_L2z + temp * GEN_3_L3z))) / sf)
       / GEN_3_FULL_RANGE_FIELD_SENSITIVITY;


// double sensitivity = sf * GEN_3_FULL_RANGE_FIELD_SENSITIVITY;
// double r    = sf;
    // *x = (r * (GEN_3_O0x + temp * (GEN_3_O1x + temp * (GEN_3_O2x + temp * GEN_3_O3x)))
    //       + ((double) valueX) * (GEN_3_L0x + temp * (GEN_3_L1x + temp * (GEN_3_L2x + temp * GEN_3_L3x))))
    //    / sensitivity;

    // *y = (r * (GEN_3_O0y + temp * (GEN_3_O1y + temp * (GEN_3_O2y + temp * GEN_3_O3y)))
    //       + ((double) valueY) * (GEN_3_L0y + temp * (GEN_3_L1y + temp * (GEN_3_L2y + temp * GEN_3_L3y))))
    //    / sensitivity;

    // *z = (r * (GEN_3_O0z + temp * (GEN_3_O1z + temp * (GEN_3_O2z + temp * GEN_3_O3z)))
    //       + ((double) valueZ) * (GEN_3_L0z + temp * (GEN_3_L1z + temp * (GEN_3_L2z + temp * GEN_3_L3z))))
    //    / sensitivity;
}


bool tlx493d_gen_3_setMeasurement(TLx493D_t *sensor, uint8_t channelBF, TLx493D_MeasurementType_t val) {
    uint8_t channel = 0;

    switch(val) {
        case TLx493D_BxByBzTemp_e : channel = 0b0000;
                                    break;

        // case TLx493D_VHall_Bias_e : channel = 0b0001;
        //                             break;
        
        // case TLx493D_Spintest_e : channel = 0b0010;
        //                           break;

        // case TLx493D_SAT_test_e : channel = 0b1000;
        //                           break;
        
        // case TLx493D_BxTemp_e : channel = 0b1100;
        //                         break;

        // case TLx493D_BxBy_e : channel = 0b1101;
        //                       break;
        
        // case TLx493D_BzTemp_e : channel = 0b1110;
        //                         break;
        
        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_MeasurementType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, channelBF, channel);
// tlx493d_common_writeRegister(sensor, channelBF);
// sensor->functions->readRegisters(sensor);
// printRegisters(sensor);
// return true;

    return tlx493d_common_writeRegister(sensor, channelBF);
}


//  // This option depends on PR and MODE.
bool tlx493d_gen_3_setTrigger(TLx493D_t *sensor, uint8_t trigBF, TLx493D_TriggerType_t val) {
    uint8_t trig = 0;

    switch(val) {
        case TLx493D_NO_ADC_ON_READ_e : trig = 0b00;
                                        break;

        case TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e : trig = 0b01;
                                                      break;
        
        case TLx493D_ADC_ON_READ_AFTER_REG_05_e : trig = 0b10;
                                                  break;
        
        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_TriggerType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, trigBF, trig);
    return tlx493d_common_writeRegister(sensor, trigBF);
}


bool tlx493d_gen_3_setSensitivity(TLx493D_t *sensor, uint8_t shortBF, uint8_t xtrShortBF, TLx493D_SensitivityType_t val) {
    uint8_t shortVal      = 0;
    uint8_t extraShortVal = 0;

    switch(val) {
        case TLx493D_FULL_RANGE_e : shortVal      = 0;
                                    extraShortVal = 0;
                                    break;

        case TLx493D_SHORT_RANGE_e : shortVal      = 1;
                                     extraShortVal = 0;
                                     break;

        case TLx493D_EXTRA_SHORT_RANGE_e : shortVal      = 0;
                                           extraShortVal = 1;
                                           break;
        
        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_SensitivityType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, shortBF, shortVal);
    tlx493d_common_setBitfield(sensor, xtrShortBF, extraShortVal);
    return tlx493d_common_writeRegister(sensor, shortBF);
}


bool tlx493d_gen_3_setDefaultConfig(TLx493D_t *sensor, uint8_t configREG, uint8_t mod1REG, uint8_t mod2REG, uint8_t cpBF, uint8_t caBF, uint8_t intBF) {
return false;
}


bool tlx493d_gen_3_setIICAddress(TLx493D_t *sensor, uint8_t iicadrBF, uint8_t fpBF, TLx493D_IICAddressType_t addr) {
    // uint8_t bitfieldValue = 0;
    // uint8_t deviceAddress = 0;

    // switch (addr) {
    //     case TLx493D_IIC_ADDR_A0_e:
    //         bitfieldValue = 0b00;
    //         deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A0;
    //         break;

    //     case TLx493D_IIC_ADDR_A1_e:
    //         bitfieldValue = 0b01;
    //         deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A1;
    //         break;

    //     case TLx493D_IIC_ADDR_A2_e:
    //         bitfieldValue = 0b10;
    //         deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A2;
    //         break;

    //     case TLx493D_IIC_ADDR_A3_e:
    //         bitfieldValue = 0b11;
    //         deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A3;
    //         break;
        
    //     default:
    //         return false;
    // }

    // tlx493d_common_setBitfield(sensor, iicadrBF, bitfieldValue);
    // tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    // bool b = tlx493d_common_writeRegister(sensor, fpBF);
    // tlx493d_common_setIICAddress(sensor, deviceAddress);

    // return b;
    return false;
}


bool tlx493d_gen_3_set1ByteReadMode(TLx493D_t *sensor, uint8_t prBF, uint8_t fpBF, uint8_t prdBF, uint8_t pr) {
    // tlx493d_common_setBitfield(sensor, prBF, pr);
    // tlx493d_common_setBitfield(sensor, fpBF, tlx493d_gen_3_calculateFuseParity(sensor, fpBF, prdBF));

    // return tlx493d_common_writeRegister(sensor, prBF);

    return false;
}


bool tlx493d_gen_3_setCollisionAvoidance(TLx493D_t *sensor, uint8_t caBF, uint8_t fpBF, uint8_t prdBF, uint8_t ca) {
    // tlx493d_common_setBitfield(sensor, caBF, ca);
    // tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    // return tlx493d_common_writeRegister(sensor, caBF);

return false;
}


bool tlx493d_gen_3_setInterrupt(TLx493D_t *sensor, uint8_t intBF, uint8_t irq) {
    tlx493d_common_setBitfield(sensor, intBF, irq);
    return tlx493d_common_writeRegister(sensor, intBF);
}


bool tlx493d_gen_3_setPowerMode(TLx493D_t *sensor, uint8_t modeBF, TLx493D_PowerModeType_t val) {
    uint8_t mode = 0;

    switch(val) {
        case TLx493D_LOW_POWER_MODE_e : mode = 0b0;
                                        break;

        case TLx493D_MASTER_CONTROLLED_MODE_e : mode = 0b1;
                                                break;

        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_PowerModeType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, modeBF, mode);
    return tlx493d_common_writeRegister(sensor, modeBF);
}


bool tlx493d_gen_3_setUpdateRate(TLx493D_t *sensor, uint8_t fUpdateBF, TLx493D_UpdateRateType_t val) {
    uint8_t rate = 0;

    switch(val) {
        case TLx493D_UPDATE_RATE_1000_HZ_e : rate = 0b00;
                                             break;

        case TLx493D_UPDATE_RATE_125_HZ_e : rate = 0b01;
                                            break;

        case TLx493D_UPDATE_RATE_31_HZ_e : rate = 0b10;
                                           break;

        case TLx493D_UPDATE_RATE_16_HZ_e : rate = 0b11;
                                           break;

        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_UpdateRateType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, fUpdateBF, rate);
    return tlx493d_common_writeRegister(sensor, fUpdateBF);
}


bool tlx493d_gen_3_hasValidData(TLx493D_t *sensor, uint8_t measBF) {
    return !sensor->functions->isInTestMode(sensor) && (tlx493d_common_returnBitfield(sensor, measBF) != 0);

    // TLx493D_Register_t *meas = &sensor->regDef[measBF];
    // return !sensor->functions->isInTestMode(sensor) && ((sensor->regMap[meas->address] & meas->mask) != 0);
}


bool tlx493d_gen_3_isFunctional(TLx493D_t *sensor) {
    return sensor->functions->hasValidFuseParity(sensor);
}


bool tlx493d_gen_3_isWakeUpEnabled(TLx493D_t *sensor, uint8_t wuBF) {
    return tlx493d_common_returnBitfield(sensor, wuBF) != 0;
}


bool tlx493d_gen_3_enableWakeUpMode(TLx493D_t *sensor, uint8_t wuBF) {
    bool b = true;

    if( ! sensor->functions->hasValidWakeUpParity(sensor) ) {
        error("Wake up parity is not correct, cannot enter wake up mode for sensor %s !\n", tlx493d_common_getTypeAsString(sensor));
        return false;
    }

    if( sensor->functions->isInTestMode(sensor) ) {
        info("isInTestMode %s !\n", tlx493d_common_getTypeAsString(sensor));
        b = sensor->functions->setMeasurement(sensor, 0);
    }


    tlx493d_common_setBitfield(sensor, wuBF, 1);
    return b && tlx493d_common_writeRegister(sensor, wuBF);
}


bool tlx493d_gen_3_disableWakeUpMode(TLx493D_t *sensor, uint8_t wuBF) {
    tlx493d_common_setBitfield(sensor, wuBF, 0);
    return tlx493d_common_writeRegister(sensor, wuBF);
}


bool tlx493d_gen_3_setThreshold(TLx493D_t *sensor, uint8_t msbsBF, uint8_t lsbsBF, int16_t threshold10Bits) {
    TLx493D_Register_t *msbs = &sensor->regDef[msbsBF];
    TLx493D_Register_t *lsbs = &sensor->regDef[lsbsBF];

    uint8_t lower = threshold10Bits & (lsbs->mask >> lsbs->offset);
    uint8_t upper = (threshold10Bits >> lsbs->numBits) & (msbs->mask >> msbs->offset);

    tlx493d_common_setBitfield(sensor, msbsBF, upper);
    bool b = tlx493d_common_writeRegister(sensor, msbsBF);

    tlx493d_common_setBitfield(sensor, lsbsBF, lower);
    return b && tlx493d_common_writeRegister(sensor, lsbsBF);
}


bool tlx493d_gen_3_setWakeUpThresholdsAsInteger(TLx493D_t *sensor,
                                                uint8_t xlMSBBF, uint8_t xlLSBBF, uint8_t xhMSBBF, uint8_t xhLSBBF,
                                                uint8_t ylMSBBF, uint8_t ylLSBBF, uint8_t yhMSBBF, uint8_t yhLSBBF,
                                                uint8_t zlMSBBF, uint8_t zlLSBBF, uint8_t zhMSBBF, uint8_t zhLSBBF,
                                                int16_t xlTh, int16_t xhTh, int16_t ylTh, int16_t yhTh, int16_t zlTh, int16_t zhTh) {
    bool retVal = tlx493d_gen_3_setThreshold(sensor, xlMSBBF, xlLSBBF, xlTh);
    retVal &= tlx493d_gen_3_setThreshold(sensor, xhMSBBF, xhLSBBF, xhTh);

    retVal &= tlx493d_gen_3_setThreshold(sensor, ylMSBBF, ylLSBBF, ylTh);
    retVal &= tlx493d_gen_3_setThreshold(sensor, yhMSBBF, yhLSBBF, yhTh);

    retVal &= tlx493d_gen_3_setThreshold(sensor, zlMSBBF, zlLSBBF, zlTh);
    retVal &= tlx493d_gen_3_setThreshold(sensor, zhMSBBF, zhLSBBF, zhTh);

    sensor->functions->readRegisters(sensor);
    return retVal;
}


bool tlx493d_gen_3_setWakeUpThresholds(TLx493D_t *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh) {
    return false;
}


bool tlx493d_gen_3_softwareReset(TLx493D_t *sensor, uint8_t resetBF) {
   tlx493d_common_setBitfield(sensor, resetBF, 1);
   return tlx493d_common_writeRegister(sensor, resetBF);
}


bool tlx493d_gen_3_hasValidFuseParity(TLx493D_t *sensor, uint8_t fpfBF) {
    return tlx493d_common_returnBitfield(sensor, fpfBF) != 0;

    // TLx493D_Register_t *fpf = &sensor->regDef[fpfBF];
    // return (sensor->regMap[fpf->address] & fpf->mask) != 0;
}


bool tlx493d_gen_3_hasValidWakeUpParity(TLx493D_t *sensor, uint8_t wupBF) {
    return tlx493d_common_returnBitfield(sensor, wupBF) != 0;

    // TLx493D_Register_t *wup = &sensor->regDef[wupBF];
    // return (sensor->regMap[wup->address] & wup->mask) != 0;
}


bool tlx493d_gen_3_isInTestMode(TLx493D_t *sensor, uint8_t testBF) {
    return tlx493d_common_returnBitfield(sensor, testBF) != 0;

    // TLx493D_Register_t *test = &sensor->regDef[testBF];
    // return (sensor->regMap[test->address] & test->mask) != 0;
}



// bool tlx493d_gen_3_hasValidIICadr(TLx493D_t *sensor, uint8_t idBF, uint8_t iicAdrBF) {
//     return tlx493d_common_returnBitfield(idBF) == tlx493d_common_returnBitfield(iicAdrBF);
// }


double tlx493d_gen_3_getSensitivityScaleFactor(TLx493D_t *sensor, TLx493D_AvailableSensitivityType_t sens, uint8_t x2BF, uint8_t x4BF) {
    switch(sens) {
        case TLx493D_HAS_X4_e : {
                                    TLx493D_Register_t *x2 = &sensor->regDef[x2BF];
                                    TLx493D_Register_t *x4 = &sensor->regDef[x4BF];
                                    
                                    bool x2IsSet = (sensor->regMap[x2->address] & x2->mask) != 0;
                                    bool x4IsSet = (sensor->regMap[x4->address] & x4->mask) != 0;

                                    return x2IsSet ? 2.0 : (x4IsSet ? 4.0 : 1.0);
        }
    
        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, sens, "TLx493D_AvailableSensitivityType_t");
                  return 0.0;
    }
}


uint8_t tlx493d_gen_3_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr) {
    switch(addr) {
        case TLx493D_IIC_ADDR_A0_e : return GEN_3_STD_IIC_ADDR_WRITE_A0;

        case TLx493D_IIC_ADDR_A1_e : return GEN_3_STD_IIC_ADDR_WRITE_A1;

        case TLx493D_IIC_ADDR_A2_e : return GEN_3_STD_IIC_ADDR_WRITE_A2;

        case TLx493D_IIC_ADDR_A3_e : return GEN_3_STD_IIC_ADDR_WRITE_A3;

        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, addr, "TLx493D_IICAddressType_t");
                  return 0;
    }
}


void tlx493d_gen_3_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens,
                                                          double xInmT, double yInmT, double zInmT,
                                                          int16_t *x, int16_t *y, int16_t *z) {
    double sf   = sensor->functions->getSensitivityScaleFactor(sensor);
    double temp = (double) rawTemp;

    *x = (xInmT * GEN_3_FULL_RANGE_FIELD_SENSITIVITY - (GEN_3_O0x + temp * (GEN_3_O1x + temp * (GEN_3_O2x + temp * GEN_3_O3x))))
       / ((GEN_3_L0x + temp * (GEN_3_L1x + temp * (GEN_3_L2x + temp * GEN_3_L3x))) / sf);

    *y = (yInmT * GEN_3_FULL_RANGE_FIELD_SENSITIVITY - (GEN_3_O0y + temp * (GEN_3_O1y + temp * (GEN_3_O2y + temp * GEN_3_O3y))))
       / ((GEN_3_L0y + temp * (GEN_3_L1y + temp * (GEN_3_L2y + temp * GEN_3_L3y))) / sf);

    *z = (zInmT * GEN_3_FULL_RANGE_FIELD_SENSITIVITY - (GEN_3_O0z + temp * (GEN_3_O1z + temp * (GEN_3_O2z + temp * GEN_3_O3z))))
       / ((GEN_3_L0z + temp * (GEN_3_L1z + temp * (GEN_3_L2z + temp * GEN_3_L3z))) / sf);
}
