#ifndef TLx493D_A1B6_H
#define TLx493D_A1B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"

// common to same generation of sensors

// sensor specicifc includes


#ifdef __cplusplus

extern "C" {

#endif


typedef enum {
    TLx493D_A1B6_LOW_POWER_PERIOD_100MS_default,
    TLx493D_A1B6_LOW_POWER_PERIOD_12MS
} TLx493D_A1B6_Reg_LOW_POWER_PERIOD_t;


typedef enum {
	POWERDOWNMODE = 0,
	FASTMODE,
	LOWPOWERMODE,
	ULTRALOWPOWERMODE,
	MASTERCONTROLLEDMODE
} TLx493D_A1B6_PowerMode_t;	

typedef struct{
	uint8_t FAST;
	uint8_t LOW_POWER;
	uint8_t LP;
	uint16_t MEAS_TIME;
} TLx493D_A1B6_PowerModeCombinations_t;


// common functions
bool TLx493D_A1B6_init(TLx493D_t *sensor);
bool TLx493D_A1B6_deinit(TLx493D_t *sensor);

bool TLx493D_A1B6_readRegisters(TLx493D_t *sensor);

void TLx493D_A1B6_calculateRawTemperature(TLx493D_t *sensor, uint16_t *temperature);
bool TLx493D_A1B6_getRawTemperature(TLx493D_t *sensor, uint16_t *temperature);

void TLx493D_A1B6_calculateRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z);
bool TLx493D_A1B6_getRawMagneticField(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z);

void TLx493D_A1B6_calculateRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature);
bool TLx493D_A1B6_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temperature);


void TLx493D_A1B6_calculateTemperature(TLx493D_t *sensor, double *temp);
bool TLx493D_A1B6_getTemperature(TLx493D_t *sensor, double *temp);

void TLx493D_A1B6_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);
bool TLx493D_A1B6_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);

void TLx493D_A1B6_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_A1B6_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);


bool TLx493D_A1B6_setMeasurement(TLx493D_t *sensor, TLx493D_MeasurementType_t mVals);
// TODO: replace next 2 functions withe the one above !
bool TLx493D_A1B6_enableTemperatureMeasurement(TLx493D_t *sensor);
bool TLx493D_A1B6_disableTemperatureMeasurement(TLx493D_t *sensor);

bool TLx493D_A1B6_setTrigger(TLx493D_t *sensor, TLx493D_TriggerType_t trig);
bool TLx493D_A1B6_setSensitivity(TLx493D_t *sensor, TLx493D_SensitivityType_t sens);


bool TLx493D_A1B6_setDefaultConfig(TLx493D_t *sensor);
bool TLx493D_A1B6_setIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr);

bool TLx493D_A1B6_enable1ByteReadMode(TLx493D_t *sensor);

bool TLx493D_A1B6_enableCollisionAvoidance(TLx493D_t *sensor);
bool TLx493D_A1B6_disableCollisionAvoidance(TLx493D_t *sensor);

bool TLx493D_A1B6_enableInterrupt(TLx493D_t *sensor);
bool TLx493D_A1B6_disableInterrupt(TLx493D_t *sensor);

bool TLx493D_A1B6_setPowerMode(TLx493D_t *sensor, TLx493D_PowerModeType_t mode);
bool TLx493D_A1B6_setPowerMode_int(TLx493D_t *sensor, TLx493D_A1B6_PowerMode_t mode);
// bool TLx493D_A1B6_setLowPowerPeriod(TLx493D_t *sensor, TLx493D_A1B6_Reg_LOW_POWER_PERIOD_t lp_period);
bool TLx493D_A1B6_setUpdateRate(TLx493D_t *sensor, TLx493D_UpdateRateType_t rate);


bool TLx493D_A1B6_hasValidData(TLx493D_t *sensor);
bool TLx493D_A1B6_isFunctional(TLx493D_t *sensor);

bool TLx493D_A1B6_hasWakeUp(TLx493D_t *sensor);
bool TLx493D_A1B6_isWakeUpEnabled(TLx493D_t *sensor);
bool TLx493D_A1B6_enableWakeUpMode(TLx493D_t *sensor);
bool TLx493D_A1B6_disableWakeUpMode(TLx493D_t *sensor);

bool TLx493D_A1B6_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th);
bool TLx493D_A1B6_setWakeUpThresholds(TLx493D_t *sensor, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);

bool TLx493D_A1B6_softwareReset(TLx493D_t *sensor);

// utilities
void TLx493D_A1B6_calculateParity(TLx493D_t *sensor);
// TODO: TLx493D_A1B6_calculateParity to be replaced by appropriate one of the next 3 !
uint8_t TLx493D_A1B6_calculateFuseParity(TLx493D_t *sensor);
uint8_t TLx493D_A1B6_calculateBusParity(TLx493D_t *sensor);
uint8_t TLx493D_A1B6_calculateConfigurationParity(TLx493D_t *sensor);

bool TLx493D_A1B6_hasValidTBit(TLx493D_t *sensor);
bool TLx493D_A1B6_hasValidIICadr(TLx493D_t *sensor);

bool TLx493D_A1B6_hasValidFuseParity(TLx493D_t *sensor);
bool TLx493D_A1B6_hasValidBusParity(TLx493D_t *sensor);
bool TLx493D_A1B6_hasValidConfigurationParity(TLx493D_t *sensor);

void TLx493D_A1B6_setResetValues(TLx493D_t *sensor);
// TODO: maybe replace the following function by the one above if appropriate
void TLx493D_A1B6_setReservedRegisterValues(TLx493D_t *senor);

uint8_t TLx493D_A1B6_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr);

void TLx493D_A1B6_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens, double mT, int16_t *rawMF);

void TLx493D_A1B6_getSensitivityScaleFactor(TLx493D_t *sensor, double *sf);


// TODO : the following function may be replaced by one of the above common functions if appropriate
bool TLx493D_A1B6_writeRegister(TLx493D_t* sensor, uint8_t bitField);
bool TLx493D_A1B6_transferWriteRegisters(TLx493D_t *sensor);

bool TLx493D_A1B6_enableParityTest(TLx493D_t *sensor);
bool TLx493D_A1B6_disableParityTest(TLx493D_t *sensor);


void TLx493D_A1B6_getBitfield(TLx493D_t *sensor, uint8_t bitField, uint8_t *bitFieldValue);
uint8_t TLx493D_A1B6_returnBitfield(TLx493D_t *sensor, uint8_t bitField);
void TLx493D_A1B6_setBitfield(TLx493D_t *sensor, uint8_t bitField, uint8_t newBitFieldValue);


bool TLx493D_A1B6_hasValidPDBit(TLx493D_t *sensor);


#ifdef __cplusplus

}


#endif


#endif // TLx493D_A1B6_H
