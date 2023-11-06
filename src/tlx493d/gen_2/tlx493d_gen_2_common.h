#ifndef TLX493D_GEN_2_COMMON_H
#define TLX493D_GEN_2_COMMON_H

// std includes
#include <stdbool.h>


// project c includes
// common to all sensors
#include "tlx493d_types.h"


void tlx493d_gen_2_calculateTemperature(TLx493D_ts *sensor, double *temp, uint8_t tempMSBBF, uint8_t tempLSBBF);
void tlx493d_gen_2_calculateMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z,
                                  uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF, uint8_t bzMSBBF, uint8_t bzLSBBF);
// void tlx493d_gen_2_calculateMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp,
//                                                 uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF, uint8_t bzMSBBF, uint8_t bzLSBBF,
//                                                 uint8_t tempMSBBF, uint8_t tempLSBBF);                


// bool selectMeasuredValues(TLx493D_ts *sensor, enum <possible combinations> mVals); // Bx/By/Bz, Bx/By, Bx/By/Temp, ...
// or do it separately ? Like :
bool tlx493d_gen_2_setDisableTemperatureMeasurement(TLx493D_ts *sensor, uint8_t dtBF, uint8_t cpBF, uint8_t dt);
bool tlx493d_gen_2_setAngularMeasurement(TLx493D_ts *sensor, uint8_t amBF, uint8_t dtBF, uint8_t cpBF, uint8_t am, uint8_t dt);

bool tlx493d_gen_2_setTrigger(TLx493D_ts *sensor, uint8_t trigBF, uint8_t cpBF, uint8_t trig);
bool tlx493d_gen_2_setShortRangeSensitivity(TLx493D_ts *sensor, uint8_t x2BF, uint8_t cpBF, uint8_t srs);


bool tlx493d_gen_2_setDefaultConfig(TLx493D_ts *sensor, uint8_t configREG, uint8_t mod1REG, uint8_t mod2REG, uint8_t cpBF, uint8_t caBF, uint8_t intBF);

bool tlx493d_gen_2_setIICAddress(TLx493D_ts *sensor, uint8_t iicadrBF, uint8_t fpBF, TLx493D_IICAddressType_te addr); // Gen. 1 and 2
// bool tlx493d_gen_2_setIICAddress(TLx493D_ts *sensor, uint8_t iicadrBF, uint8_t fpBF, StandardIICAddresses_te addr);

// bool setInterruptAndCollisionAvoidance(TLx493D_ts *sensor, enum <possible combinations> eVal);
bool tlx493d_gen_2_setInterruptAndCollisionAvoidance(TLx493D_ts *sensor, uint8_t intBF, uint8_t caBF, uint8_t fpBF, uint8_t prdBF, bool intIsOn, bool caIsOn);

// bool setPowerMode(TLx493D_ts *sensor, enum <possible combinations> mode);  // value of mode is sensor / generation specific !
bool tlx493d_gen_2_setPowerMode(TLx493D_ts *sensor, uint8_t modeBF, uint8_t fpBF, TLx493D_PowerModeType_te mode);

// value of update rate is sensor / generation specific !
// bool setUpdateRate(TLx493D_ts *sensor, enum <possible combinations> rate);
bool tlx493d_gen_2_setUpdateRate(TLx493D_ts *sensor, uint8_t fpBF, uint8_t prdBF, TLx493D_UpdateRateType_te ur);

bool tlx493d_gen_2_hasValidData(TLx493D_ts *sensor);
bool tlx493d_gen_2_isFunctional(TLx493D_ts *sensor);


// bool tlx493d_isWakeUpActive(TLx493D_ts *sensor);
// bool tlx493d_enableWakeUpMode(TLx493D_ts *sensor);
// bool tlx493d_disableWakeUpMode(TLx493D_ts *sensor);

// bool tlx493d_setLowerWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold);
// bool tlx493d_setLowerWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold);
// bool tlx493d_setLowerWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold);

// bool tlx493d_setUpperWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold);
// bool tlx493d_setUpperWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold);
// bool tlx493d_setUpperWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold);

// bool tlx493d_setWakeUpThresholdsAsInteger(TLx493D_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);
// // thesholds im mT, to be converted to proper format
// bool tlx493d_setWakeUpThresholds(TLx493D_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);


// // utilities
// bool tlx493d_softReset(TLx493D_ts *sensor);


bool tlx493d_gen_2_setMagneticTemperatureCompensation(TLx493D_ts *sensor, uint8_t tl_magBF, uint8_t cpBF, uint8_t mtc);
bool tlx493d_gen_2_set1ByteReadMode(TLx493D_ts *sensor, uint8_t prBF, uint8_t fpBF, uint8_t prdBF, uint8_t pr);

// TODO: cleanup
uint8_t tlx493d_gen_2_calculateFuseParity(TLx493D_ts *sensor, uint8_t fpBF, uint8_t prdBF);
uint8_t tlx493d_gen_2_calculateBusParity(TLx493D_ts *sensor, uint8_t to);
uint8_t tlx493d_gen_2_calculateConfigurationParity(TLx493D_ts *sensor, uint8_t cpBF);
uint8_t tlx493d_gen_2_calculateConfigurationParityWakeup(TLx493D_ts *sensor, uint8_t cpBF, uint8_t from, uint8_t to);

bool tlx493d_gen_2_hasValidFuseParity(TLx493D_ts *sensor, uint8_t ffBF);
bool tlx493d_gen_2_hasValidBusParity(TLx493D_ts *sensor, uint8_t pBF);
bool tlx493d_gen_2_hasValidConfigurationParity(TLx493D_ts *sensor, uint8_t cpBF);

bool tlx493d_gen_2_hasValidTemperatureData(TLx493D_ts *sensor);
bool tlx493d_gen_2_hasValidMagneticFieldData(TLx493D_ts *sensor);
bool tlx493d_gen_2_hasValidTBit(TLx493D_ts *sensor, uint8_t tBF) ;
bool tlx493d_gen_2_hasValidPD3Bit(TLx493D_ts *sensor, uint8_t pd3BF);
bool tlx493d_gen_2_hasValidPD0Bit(TLx493D_ts *sensor, uint8_t pd0BF);

bool tlx493d_gen_2_hasValidIICadr(TLx493D_ts *sensor, uint8_t idBF, uint8_t iicAdrBF);
bool tlx493d_gen_2_hasWakeup(TLx493D_ts *sensor, uint8_t typeBF);

uint8_t tlx493d_gen_2_getID(TLx493D_ts *sensor, uint8_t idBF);
uint8_t tlx493d_gen_2_getFrameCounter(TLx493D_ts *sensor, uint8_t frmBF) ;
uint8_t tlx493d_gen_2_getType(TLx493D_ts *sensor, uint8_t typeBF);
uint8_t tlx493d_gen_2_getHWV(TLx493D_ts *sensor, uint8_t hwvBF);


#endif // TLX493D_GEN_2_COMMON_H