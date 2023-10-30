#ifndef TLx493D_A1B6_H
#define TLx493D_A1B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common.h"

// common to same generation of sensors
// #include "tlx493d_gen_1_common.h"

// sensor specicifc includes

//enums
typedef enum {
    GEN_1_STD_IIC_ADDR_00 = 0,
    GEN_1_STD_IIC_ADDR_01,
    GEN_1_STD_IIC_ADDR_10,
    GEN_1_STD_IIC_ADDR_11
} TLx493D_StandardIICAddresses_te;

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
bool TLx493D_A1B6_init(Sensor_ts *sensor);
bool TLx493D_A1B6_deinit(Sensor_ts *sensor);

void TLx493D_A1B6_calculateTemperature(Sensor_ts *sensor, double *temp);
bool TLx493D_A1B6_getTemperature(Sensor_ts *sensor, double *temp);

void TLx493D_A1B6_calculateMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);
bool TLx493D_A1B6_getMagneticField(Sensor_ts *sensor, double *x, double *y, double *z);

// bool TLx493D_A1B6_reset(Sensor_ts *sensor);
// bool TLx493D_A1B6_getDiagnosis(Sensor_ts *sensor);
void TLx493D_A1B6_calculateParity(Sensor_ts *sensor);

bool TLx493D_A1B6_setDefaultConfig(Sensor_ts *sensor);

void TLx493D_A1B6_setReservedRegisterValues(Sensor_ts *senor);

bool TLx493D_A1B6_enableTemperatureMeasurement(Sensor_ts *sensor);
bool TLx493D_A1B6_disableTemperatureMeasurement(Sensor_ts *sensor);

bool TLx493D_A1B6_transferWriteRegisters(Sensor_ts *sensor);

bool TLx493D_A1B6_enableParityTest(Sensor_ts *sensor);
bool TLx493D_A1B6_disableParityTest(Sensor_ts *sensor);

void TLE493D_A2B6_calculateMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);
bool TLE493D_A2B6_getMagneticFieldAndTemperature(Sensor_ts *sensor, double *x, double *y, double *z, double *temp);

bool TLx493D_A1B6_setIICAddress(Sensor_ts *sensor, TLx493D_StandardIICAddresses_te addr);

bool TLx493D_A1B6_setLowPowerPeriod(Sensor_ts *sensor, TLx493D_A1B6_Reg_LOW_POWER_PERIOD_t lp_period);

bool TLx493D_A1B6_setPowerMode(Sensor_ts *sensor, TLx493D_A1B6_PowerMode_t mode);

bool TLx493D_A1B6_enableInterrupt(Sensor_ts *sensor);
bool TLx493D_A1B6_disableInterrupt(Sensor_ts *sensor);

// bool TLx493D_A1B6_transferRegisterMap(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len);

void TLx493D_A1B6_getBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t *bitFieldValue);
uint8_t TLx493D_A1B6_returnBitfield(Sensor_ts *sensor, uint8_t bitField);
void TLx493D_A1B6_setBitfield(Sensor_ts *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool TLx493D_A1B6_writeRegister(Sensor_ts* sensor, uint8_t bitField);

bool TLx493D_A1B6_readRegisters(Sensor_ts *sensor);

bool TLx493D_A1B6_hasValidFuseParity(Sensor_ts *sensor);
bool TLx493D_A1B6_isFunctional(Sensor_ts *sensor);
bool TLx493D_A1B6_hasValidTBit(Sensor_ts *sensor);
bool TLx493D_A1B6_hasValidPDBit(Sensor_ts *sensor);
bool TLx493D_A1B6_hasValidData(Sensor_ts *sensor);


#endif // TLx493D_A1B6_H
