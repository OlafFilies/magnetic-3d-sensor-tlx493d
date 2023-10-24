#ifndef TLx493D_A1B6_H
#define TLx493D_A1B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_1_common.h"

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

void TLx493D_A1B6_calculateTemperature(Sensor_ts *sensor, float *temp);
bool TLx493D_A1B6_getTemperature(Sensor_ts *sensor, float *temp);

void TLx493D_A1B6_calculateFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
bool TLx493D_A1B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

// bool TLx493D_A1B6_reset(Sensor_ts *sensor);
// bool TLx493D_A1B6_getDiagnosis(Sensor_ts *sensor);
void TLx493D_A1B6_calculateParity(Sensor_ts *sensor);

bool TLx493D_A1B6_setDefaultConfig(Sensor_ts *sensor);

void TLx493D_A1B6_setReservedRegisterValues(Sensor_ts *senor);

bool TLx493D_A1B6_enableTemperatureMeasurements(Sensor_ts *sensor);
bool TLx493D_A1B6_disableTemperatureMeasurements(Sensor_ts *sensor);

bool TLx493D_A1B6_transferWriteRegisters(Sensor_ts *sensor);

bool TLx493D_A1B6_enableParityTest(Sensor_ts *sensor);
bool TLx493D_A1B6_disableParityTest(Sensor_ts *sensor);

void TLE493D_A2B6_calculateSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);
bool TLE493D_A2B6_getSensorValues(Sensor_ts *sensor, float *x, float *y, float *z, float *temp);

bool TLx493D_A1B6_setIICAddress(Sensor_ts *sensor, TLx493D_StandardIICAddresses_te addr);

bool TLx493D_A1B6_setLowPowerPeriod(Sensor_ts *sensor, TLx493D_A1B6_Reg_LOW_POWER_PERIOD_t lp_period);

bool TLx493D_A1B6_setPowerMode(Sensor_ts *sensor, TLx493D_A1B6_PowerMode_t mode);

bool TLx493D_A1B6_enableInterrupt(Sensor_ts *sensor);
bool TLx493D_A1B6_disableInterrupt(Sensor_ts *sensor);

bool TLx493D_A1B6_transferRegisterMap(Sensor_ts *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len);


#endif // TLx493D_A1B6_H
