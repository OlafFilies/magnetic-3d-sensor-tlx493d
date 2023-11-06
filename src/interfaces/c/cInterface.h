#ifndef TLx493D_C_INTERFACE_H
#define TLx493D_C_INTERFACE_H

// project c includes
// common to all sensors
#include "tlx493d_types.h"


#ifdef __cplusplus

extern "C" {

#endif


// functions related to the initialization of the sensor data structure
bool tlx493d_init(TLx493D_ts *sensor, TLx493D_SupportedSensorType_te sensorType);
bool tlx493d_deinit(TLx493D_ts *sensor);


// functions related to the retrieval of data from the sensor
bool tlx493d_readRegisters(TLx493D_ts *sensor);

bool tlx493d_getTemperature(TLx493D_ts *sensor, double *temp);
bool tlx493d_getMagneticField(TLx493D_ts *sensor, double *x, double *y, double *z);
bool tlx493d_getMagneticFieldAndTemperature(TLx493D_ts *sensor, double *x, double *y, double *z, double *temp);


// functions related to the "Config" register
bool tlx493d_selectMeasureValues(TLx493D_ts *sensor, TLx493D_MeasureType_te meas);
// or do it separately ? Like :
// bool tlx493d_enableTemperatureMeasurement(TLx493D_ts *sensor);
// bool tlx493d_disableTemperatureMeasurement(TLx493D_ts *sensor);
// bool tlx493d_enableAngularMeasurement(TLx493D_ts *sensor);
// bool tlx493d_disableAngularMeasurement(TLx493D_ts *sensor);

bool tlx493d_setTrigger(TLx493D_ts *sensor, uint8_t trigger);
bool tlx493d_setSensitivity(TLx493D_ts *sensor, TLx493D_SensitivityType_te range);


// functions related to the "Mod1" and "Mod2" registers
bool tlx493d_setDefaultConfig(TLx493D_ts *sensor);
bool tlx493d_setIICAddress(TLx493D_ts *sensor, TLx493D_IICAddressType_te addr); // Gen. 1 and 2

bool tlx493d_enableInterrupt(TLx493D_ts *sensor);
bool tlx493d_disableInterrupt(TLx493D_ts *sensor);

bool tlx493d_enableCollisionAvoidance(TLx493D_ts *sensor);
bool tlx493d_disableCollisionAvoidance(TLx493D_ts *sensor);

bool tlx493d_setPowerMode(TLx493D_ts *sensor, TLx493D_PowerModeType_te mode);  // value of mode is sensor / generation specific !

// value of update rate is sensor / generation specific !
bool tlx493d_setUpdateRate(TLx493D_ts *sensor, TLx493D_UpdateRateType_te rate);


// functions related to the "Diag" register
bool tlx493d_hasValidData(TLx493D_ts *sensor);
// bool tlx493d_hasValidTemperatureData(TLx493D_ts *sensor);
// bool tlx493d_hasValidMagneticFieldData(TLx493D_ts *sensor);

bool tlx493d_isFunctional(TLx493D_ts *sensor);


// functions available only to a subset of sensors with wake-up functionality
// functions related to the "WU" register
bool tlx493d_isWakeUpActive(TLx493D_ts *sensor);
bool tlx493d_enableWakeUpMode(TLx493D_ts *sensor);
bool tlx493d_disableWakeUpMode(TLx493D_ts *sensor);

bool tlx493d_setLowerWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold);
bool tlx493d_setLowerWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold);
bool tlx493d_setLowerWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold);

bool tlx493d_setUpperWakeUpThresholdX(TLx493D_ts *sensor, int16_t threshold);
bool tlx493d_setUpperWakeUpThresholdY(TLx493D_ts *sensor, int16_t threshold);
bool tlx493d_setUpperWakeUpThresholdZ(TLx493D_ts *sensor, int16_t threshold);

bool tlx493d_setWakeUpThresholdsAsInteger(TLx493D_ts *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);
// thesholds im mT, to be converted to proper format
bool tlx493d_setWakeUpThresholds(TLx493D_ts *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);


// utilities
bool tlx493d_softReset(TLx493D_ts *sensor);
const char *tlx493d_getTypeAsString(TLx493D_ts *sensor);


#ifdef __cplusplus

}

#endif


#endif // TLx493D_C_INTERFACE_H
