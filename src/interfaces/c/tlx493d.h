#ifndef TLX493D_H
#define TLX493D_H

// project c includes
// common to all sensors
#include "tlx493d_types.h"


#ifdef __cplusplus

extern "C" {

#endif


// functions related to the initialization of the sensor data structure
bool tlx493d_init(TLx493D_t *sensor, TLx493D_SupportedSensorType_t sensorType);
bool tlx493d_deinit(TLx493D_t *sensor);


// functions related to the retrieval of data from the sensor
bool tlx493d_readRegisters(TLx493D_t *sensor);

bool tlx493d_getTemperature(TLx493D_t *sensor, double *temp);
bool tlx493d_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);
bool tlx493d_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);

bool tlx493d_getRawTemperature(TLx493D_t *sensor, uint16_t *temp);
bool tlx493d_getRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z);
bool tlx493d_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temp);


// functions related to the "Config" register
bool tlx493d_setMeasurement(TLx493D_t *sensor, TLx493D_MeasurementType_t meas);
bool tlx493d_setTrigger(TLx493D_t *sensor, TLx493D_TriggerType_t trigger);
bool tlx493d_setSensitivity(TLx493D_t *sensor, TLx493D_SensitivityType_t range);


// functions related to the "Mod1" and "Mod2" registers
bool tlx493d_setDefaultConfig(TLx493D_t *sensor);
bool tlx493d_setIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr); // Gen. 1 and 2

bool tlx493d_enableCollisionAvoidance(TLx493D_t *sensor);
bool tlx493d_disableCollisionAvoidance(TLx493D_t *sensor);

bool tlx493d_enableInterrupt(TLx493D_t *sensor);
bool tlx493d_disableInterrupt(TLx493D_t *sensor);

bool tlx493d_setPowerMode(TLx493D_t *sensor, TLx493D_PowerModeType_t mode);  // value of mode is sensor / generation specific !

// value of update rate is sensor / generation specific !
bool tlx493d_setUpdateRate(TLx493D_t *sensor, TLx493D_UpdateRateType_t rate);


// functions related to the "Diag" register
bool tlx493d_hasValidData(TLx493D_t *sensor);
// bool tlx493d_hasValidTemperatureData(TLx493D_t *sensor);
// bool tlx493d_hasValidMagneticFieldData(TLx493D_t *sensor);

bool tlx493d_isFunctional(TLx493D_t *sensor);


// functions available only to a subset of sensors with wake-up functionality
// functions related to the "WU" register
bool tlx493d_hasWakeUp(TLx493D_t *sensor);
bool tlx493d_isWakeUpEnabled(TLx493D_t *sensor);
bool tlx493d_enableWakeUpMode(TLx493D_t *sensor);
bool tlx493d_disableWakeUpMode(TLx493D_t *sensor);

bool tlx493d_setLowerWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold);
bool tlx493d_setLowerWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold);
bool tlx493d_setLowerWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold);

bool tlx493d_setUpperWakeUpThresholdX(TLx493D_t *sensor, int16_t threshold);
bool tlx493d_setUpperWakeUpThresholdY(TLx493D_t *sensor, int16_t threshold);
bool tlx493d_setUpperWakeUpThresholdZ(TLx493D_t *sensor, int16_t threshold);

bool tlx493d_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);
// thesholds im mT, to be converted to proper format
bool tlx493d_setWakeUpThresholds(TLx493D_t *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);


// utilities
bool tlx493d_softReset(TLx493D_t *sensor);
const char *tlx493d_getTypeAsString(TLx493D_t *sensor);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_H
