#ifndef TLX493D_A2B6_H
#define TLX493D_A2B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common.h"

// common to same generation of sensors
#include "tlx493d_gen_2_common.h"

// sensor specific includes


// common functions
bool TLx493D_A2B6_init(TLx493D_ts *sensor);
bool TLx493D_A2B6_deinit(TLx493D_ts *sensor);

bool TLx493D_A2B6_readRegisters(TLx493D_ts *sensor);

void TLx493D_A2B6_calculateTemperature(TLx493D_ts *sensor, double *temp);
bool TLx493D_A2B6_getTemperature(TLx493D_ts *sensor, double *temp);

void TLx493D_A2B6_calculateMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z);
bool TLx493D_A2B6_getMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z);

void TLx493D_A2B6_calculateMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_A2B6_getMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp);

bool TLx493D_A2B6_selectMeasureValues(TLx493D_ts *sensor, TLx493D_MeasureType_te mVals);

bool TLx493D_A2B6_setTrigger(TLx493D_ts *sensor, uint8_t trig);
bool TLx493D_A2B6_setSensitivity(TLx493D_ts *sensor, TLx493D_SensitivityType_te sens);


bool TLx493D_A2B6_setDefaultConfig(TLx493D_ts *sensor);
bool TLx493D_A2B6_setIICAddress(TLx493D_ts *sensor, TLx493D_IICAddressType_te address);

bool TLx493D_A2B6_enableInterrupt(TLx493D_ts *sensor);
bool TLx493D_A2B6_disableInterrupt(TLx493D_ts *sensor);

bool TLx493D_A2B6_enableCollisionAvoidance(TLx493D_ts *sensor);
bool TLx493D_A2B6_disableCollisionAvoidance(TLx493D_ts *sensor);

bool TLx493D_A2B6_setPowerMode(TLx493D_ts *sensor, TLx493D_PowerModeType_te mode);
bool TLx493D_A2B6_setUpdateRate(TLx493D_ts *sensor, TLx493D_UpdateRateType_te rate);


bool TLx493D_A2B6_hasValidData(TLx493D_ts *sensor);
bool TLx493D_A2B6_isFunctional(TLx493D_ts *sensor);

bool TLx493D_A2B6_isWakeUpActive(TLx493D_ts *sensor);
bool TLx493D_A2B6_enableWakeUpMode(TLx493D_ts *sensor);
bool TLx493D_A2B6_disableWakeUpMode(TLx493D_ts *sensor);

bool TLx493D_A2B6_setLowerWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold);
bool TLx493D_A2B6_setLowerWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold);
bool TLx493D_A2B6_setLowerWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold);

bool TLx493D_A2B6_setUpperWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold);
bool TLx493D_A2B6_setUpperWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold);
bool TLx493D_A2B6_setUpperWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold);

bool TLx493D_A2B6_setWakeUpThresholdsAsInteger(TLx493D_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);
// thesholds im mT, to be converted to proper format
bool TLx493D_A2B6_setWakeUpThresholds(TLx493D_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);

bool TLx493D_A2B6_softReset(TLx493D_ts *sensor);


// utilities
bool TLx493D_A2B6_enableTemperatureMeasurement(TLx493D_ts *sensor);
bool TLx493D_A2B6_disableTemperatureMeasurement(TLx493D_ts *sensor);
bool TLx493D_A2B6_enableAngularMeasurement(TLx493D_ts *sensor);
bool TLx493D_A2B6_disableAngularMeasurement(TLx493D_ts *sensor);

bool TLx493D_A2B6_setNoTriggerOnReadTriggerOption(TLx493D_ts *sensor);
bool TLx493D_A2B6_setTriggerOnReadBeforeFirstMSBTriggerOption(TLx493D_ts *sensor);
bool TLx493D_A2B6_setTriggerOnReadAfterRegister05TriggerOption(TLx493D_ts *sensor);

bool TLx493D_A2B6_enableShortRangeSensitivity(TLx493D_ts *sensor);
bool TLx493D_A2B6_disableShortRangeSensitivity(TLx493D_ts *sensor);

// bool TLx493D_A2B6_setTC0MagneticTemperatureCompensation(TLx493D_ts *sensor);
// bool TLx493D_A2B6_setTC1MagneticTemperatureCompensation(TLx493D_ts *sensor);
// bool TLx493D_A2B6_setTC2MagneticTemperatureCompensation(TLx493D_ts *sensor);
// bool TLx493D_A2B6_setTC3MagneticTemperatureCompensation(TLx493D_ts *sensor);


bool TLx493D_A2B6_enable1ByteReadMode(TLx493D_ts *sensor);
bool TLx493D_A2B6_disable1ByteReadMode(TLx493D_ts *sensor);

uint8_t TLx493D_A2B6_calculateFuseParity(TLx493D_ts *sensor);
uint8_t TLx493D_A2B6_calculateBusParity(TLx493D_ts *sensor);
uint8_t TLx493D_A2B6_calculateConfigurationParity(TLx493D_ts *sensor);

// bool TLx493D_A2B6_hasValidTemperatureData(TLx493D_ts *sensor);
// bool TLx493D_A2B6_hasValidMagneticFieldData(TLx493D_ts *sensor);
bool TLx493D_A2B6_hasValidTBit(TLx493D_ts *sensor);
// bool TLx493D_A2B6_hasValidPD0Bit(TLx493D_ts *sensor);
// bool TLx493D_A2B6_hasValidPD3Bit(TLx493D_ts *sensor);

// bool TLx493D_A2B6_hasValidIICadr(TLx493D_ts *sensor);
// bool TLx493D_A2B6_hasWakeup(TLx493D_ts *sensor);

bool TLx493D_A2B6_hasValidFuseParity(TLx493D_ts *sensor);
bool TLx493D_A2B6_hasValidBusParity(TLx493D_ts *sensor);
bool TLx493D_A2B6_hasValidConfigurationParity(TLx493D_ts *sensor);

void TLx493D_A2B6_setResetValues(TLx493D_ts *sensor);


#endif /** TLX493D_A2B6_H */