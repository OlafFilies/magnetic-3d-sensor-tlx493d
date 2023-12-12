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


bool tlx493d_gen_3_setMeasurement(TLx493D_t *sensor, uint8_t channelBF, TLx493D_MeasurementType_t val) {
    uint8_t channel = 0;

    switch(val) {
        case TLx493D_BxByBzTemp_e : channel = 0b0000;
                                    break;

        case TLx493D_VHall_Bias_e : channel = 0b0001;
                                    break;
        
        case TLx493D_Spintest_e : channel = 0b0010;
                                  break;

        case TLx493D_SAT_test_e : channel = 0b1000;
                                  break;
        
        case TLx493D_BxTemp_e : channel = 0b1100;
                                break;

        case TLx493D_BxBy_e : channel = 0b1101;
                              break;
        
        case TLx493D_BzTemp_e : channel = 0b1110;
                              break;
        
        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_MeasurementType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, channelBF, channel);
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


// TODO: same as for gen_2 => make common ?
bool tlx493d_gen_3_setWakeUpThresholdsAsInteger(TLx493D_t *sensor,
                                                uint8_t xlMSBBF, uint8_t xlLSBBF, uint8_t xhMSBBF, uint8_t xhLSBBF,
                                                uint8_t ylMSBBF, uint8_t ylLSBBF, uint8_t yhMSBBF, uint8_t yhLSBBF,
                                                uint8_t zlMSBBF, uint8_t zlLSBBF, uint8_t zhMSBBF, uint8_t zhLSBBF,
                                                int16_t xlTh, int16_t xhTh, int16_t ylTh, int16_t yhTh, int16_t zlTh, int16_t zhTh) {
    bool retVal = tlx493d_gen_2_setThreshold(sensor, xlMSBBF, xlLSBBF, xlTh);
    retVal &= tlx493d_gen_2_setThreshold(sensor, xhMSBBF, xhLSBBF, xhTh);

    retVal &= tlx493d_gen_2_setThreshold(sensor, ylMSBBF, ylLSBBF, ylTh);
    retVal &= tlx493d_gen_2_setThreshold(sensor, yhMSBBF, yhLSBBF, yhTh);

    retVal &= tlx493d_gen_2_setThreshold(sensor, zlMSBBF, zlLSBBF, zlTh);
    retVal &= tlx493d_gen_2_setThreshold(sensor, zhMSBBF, zhLSBBF, zhTh);

    sensor->functions->readRegisters(sensor);
printRegisters(sensor);
    return retVal;
}


bool tlx493d_gen_3_hasValidFuseParity(TLx493D_t *sensor, uint8_t fpfBF) {
    TLx493D_Register_t *fpf = &sensor->regDef[fpfBF];
    return (sensor->regMap[fpf->address] & fpf->mask) != 0;
}


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
