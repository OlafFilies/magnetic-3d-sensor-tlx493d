#ifndef TLX493D_GEN_2_COMMON_H
#define TLX493D_GEN_2_COMMON_H

// std includes
#include <stdbool.h>


// project c includes
// common to all sensors
#include "tlx493d_types.h"


void tlx493d_gen_2_calculateTemperature(TLx493D_t *sensor, double *temp, uint8_t tempMSBBF, uint8_t tempLSBBF);
void tlx493d_gen_2_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z,
                                  uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF, uint8_t bzMSBBF, uint8_t bzLSBBF);

bool tlx493d_gen_2_setOneConfigBitfield(TLx493D_t *sensor, uint8_t firstBF, uint8_t cpBF, uint8_t first);
bool tlx493d_gen_2_setTwoConfigBitfields(TLx493D_t *sensor, uint8_t firstBF, uint8_t secondBF, uint8_t cpBF, uint8_t first, uint8_t second);

bool tlx493d_gen_2_setMeasurement(TLx493D_t *sensor, uint8_t dtBF, uint8_t amBF, uint8_t cpBF, TLx493D_MeasurementType_t val);
bool tlx493d_gen_2_setTrigger(TLx493D_t *sensor, uint8_t trigBF, uint8_t cpBF, TLx493D_TriggerType_t val);
bool tlx493d_gen_2_setSensitivity(TLx493D_t *sensor, uint8_t x2BF, uint8_t cpBF, TLx493D_SensitivityType_t val);
// bool tlx493d_gen_2_setMagneticTemperatureCompensation(TLx493D_t *sensor, uint8_t tl_magBF, uint8_t cpBF, uint8_t mtc);


bool tlx493d_gen_2_setDefaultConfig(TLx493D_t *sensor, uint8_t configREG, uint8_t mod1REG, uint8_t mod2REG, uint8_t cpBF, uint8_t caBF, uint8_t intBF);
bool tlx493d_gen_2_setIICAddress(TLx493D_t *sensor, uint8_t iicadrBF, uint8_t fpBF, TLx493D_IICAddressType_t addr); // Gen. 1 and 2
bool tlx493d_gen_2_set1ByteReadMode(TLx493D_t *sensor, uint8_t prBF, uint8_t fpBF, uint8_t prdBF, uint8_t pr);

bool tlx493d_gen_2_setCollisionAvoidance(TLx493D_t *sensor, uint8_t caBF, uint8_t fpBF, uint8_t prdBF, uint8_t ca);
bool tlx493d_gen_2_setInterrupt(TLx493D_t *sensor, uint8_t intBF, uint8_t fpBF, uint8_t prdBF, uint8_t irq);

bool tlx493d_gen_2_setPowerMode(TLx493D_t *sensor, uint8_t modeBF, uint8_t fpBF, TLx493D_PowerModeType_t mode);
bool tlx493d_gen_2_setUpdateRate(TLx493D_t *sensor, uint8_t fpBF, uint8_t prdBF, TLx493D_UpdateRateType_t ur);

bool tlx493d_gen_2_hasValidData(TLx493D_t *sensor);
bool tlx493d_gen_2_isFunctional(TLx493D_t *sensor);


// bool tlx493d_gen_2_hasWakeUp(TLx493D_t *sensor, uint8_t typeBF);
bool tlx493d_gen_2_isWakeUpEnabled(TLx493D_t *sensor, uint8_t waBF);
bool tlx493d_gen_2_enableWakeUpMode(TLx493D_t *sensor, uint8_t tBF, uint8_t wuBF, uint8_t cpbBF);
bool tlx493d_gen_2_disableWakeUpMode(TLx493D_t *sensor, uint8_t wuBF, uint8_t cpbBF);

bool tlx493d_gen_2_setThreshold(TLx493D_t *sensor, uint8_t msbsBF, uint8_t lsbsBF, uint8_t cpbBF, int16_t threshold12Bits);

// bool tlx493d_gen_2_setLowerWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold);
// bool tlx493d_gen_2_setLowerWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold);
// bool tlx493d_gen_2_setLowerWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold);

// bool tlx493d_gen_2_setUpperWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold);
// bool tlx493d_gen_2_setUpperWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold);
// bool tlx493d_gen_2_setUpperWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold);

// bool tlx493d_gen_2_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);
// bool tlx493d_gen_2_setWakeUpThresholds(TLx493D_t *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);


// // utilities
// bool tlx493d_gen_2_softReset(TLx493D_t *sensor);


uint8_t tlx493d_gen_2_calculateFuseParity(TLx493D_t *sensor, uint8_t fpBF, uint8_t prdBF);
uint8_t tlx493d_gen_2_calculateBusParity(TLx493D_t *sensor, uint8_t to);
uint8_t tlx493d_gen_2_calculateConfigurationParity(TLx493D_t *sensor, uint8_t cpBF);
uint8_t tlx493d_gen_2_calculateConfigurationParityWakeUp(TLx493D_t *sensor, uint8_t cpBF); //, uint8_t from, uint8_t to);

bool tlx493d_gen_2_hasValidFuseParity(TLx493D_t *sensor, uint8_t ffBF);
bool tlx493d_gen_2_hasValidBusParity(TLx493D_t *sensor, uint8_t pBF);
bool tlx493d_gen_2_hasValidConfigurationParity(TLx493D_t *sensor, uint8_t cpBF);

bool tlx493d_gen_2_hasValidIICadr(TLx493D_t *sensor, uint8_t idBF, uint8_t iicAdrBF);
bool tlx493d_gen_2_hasValidTBit(TLx493D_t *sensor, uint8_t tBF) ;


// bool tlx493d_gen_2_hasValidTemperatureData(TLx493D_t *sensor);
// bool tlx493d_gen_2_hasValidMagneticFieldData(TLx493D_t *sensor);
// bool tlx493d_gen_2_hasValidPD3Bit(TLx493D_t *sensor, uint8_t pd3BF);
// bool tlx493d_gen_2_hasValidPD0Bit(TLx493D_t *sensor, uint8_t pd0BF);


// uint8_t tlx493d_gen_2_getID(TLx493D_t *sensor, uint8_t idBF);
// uint8_t tlx493d_gen_2_getFrameCounter(TLx493D_t *sensor, uint8_t frmBF) ;
// uint8_t tlx493d_gen_2_getType(TLx493D_t *sensor, uint8_t typeBF);
// uint8_t tlx493d_gen_2_getHWV(TLx493D_t *sensor, uint8_t hwvBF);


#endif // TLX493D_GEN_2_COMMON_H