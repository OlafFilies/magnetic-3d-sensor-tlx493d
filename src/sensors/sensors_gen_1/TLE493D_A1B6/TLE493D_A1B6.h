#ifndef TLE493D_A1B6_H
#define TLE493D_A1B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_config_common.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_1_config_common.h"
#include "sensors_gen_1_common.h"

// sensor specicifc includes
#include "TLE493D_A1B6_config.h"

#define TLE493D_A1B6_WRITE_REGISTER_MAP_SIZE            13          //remove: fix at the end of development
#define TLE493D_A1B6_READ_REGISTER_MAP_SIZE             8          //remove: fix at the end of development
#define TLE493D_A1B6_REGISTER_MAP_SIZE                  (TLE493D_A1B6_WRITE_REGISTER_MAP_SIZE + TLE493D_A1B6_READ_REGISTER_MAP_SIZE)
#define TLE493D_A1B6_WRITE_REGISTERS_MAX_COUNT          4

//register enums
typedef enum {
    TLE493D_A1B6_Temp_ENABLE_default,
    TLE493D_A1B6_Temp_DISABLE
} TLE493D_A1B6_Reg_Temp_NEN;

typedef enum {
    TLE493D_A1B6_ODD_PARITY,
    TLE493D_A1B6_EVEN_PARITY
} TLE493D_A1B6_Reg_PARITY;

typedef enum {
    TLE493D_A1B6_CONFIG_00_default,
    TLE493D_A1B6_CONFIG_01,
    TLE493D_A1B6_CONFIG_10,
    TLE493D_A1B6_CONFIG_11
} TLE493D_A1B6_Reg_IICADDR;

typedef enum {
    TLE493D_A1B6_INT_ENABLE_default,
    TLE493D_A1B6_INT_DISABLE
} TLE493D_A1B6_Reg_INT;

typedef enum {
    TLE493D_A1B6_FAST_MODE_DISABLE_default,
    TLE493D_A1B6_FAST_MODE_ENABLE
} TLE493D_A1B6_Reg_FAST_MODE_NEN;

typedef enum {
    TLE493D_A1B6_LOW_POWER_MODE_DISABLE_default,
    TLE493D_A1B6_LOW_POWER_MODE_ENABLE
} TLE493D_A1B6_Reg_LOW_POWER_MODE_NEN;

typedef enum {
    TLE493D_A1B6_LOW_POWER_PERIOD_100MS_default,
    TLE493D_A1B6_LOW_POWER_PERIOD_12MS
} TLE493D_A1B6_Reg_LOW_POWER_PERIOD;

typedef enum {
    TLE493D_A1B6_PARITY_TEST_ENABLE_default,
    TLE493D_A1B6_PARITY_TEST_DISABLE
} TLE493D_A1B6_Reg_PARITY_TEST_NEN;

// common functions
bool TLE493D_A1B6_init(Sensor_ts *sensor);
bool TLE493D_A1B6_deinit(Sensor_ts *sensor);

// bool TLE493D_A1B6_getTemperature(Sensor_ts *sensor, float *temp);
// bool TLE493D_A1B6_updateGetTemperature(Sensor_ts *sensor, float *temp);

// bool TLE493D_A1B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
// bool TLE493D_A1B6_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

// bool TLE493D_A1B6_reset(Sensor_ts *sensor);
// bool TLE493D_A1B6_getDiagnosis(Sensor_ts *sensor);
// bool TLE493D_A1B6_calculateParity(Sensor_ts *sensor);

bool TLE493D_A1B6_setDefaultConfig(Sensor_ts *sensor);
// bool TLE493D_A1B6_updateRegisterMap(Sensor_ts *sensor);
bool TLE493D_A1B6_setWriteRegisterDefaultValues(Sensor_ts *sensor);

bool TLE493D_A1B6_enableTemperatureMeasurements(Sensor_ts *sensor);
bool TLE493D_A1B6_disableTemperatureMeasurements(Sensor_ts *sensor);

bool TLE493D_A1B6_loadWriteRegisters(Sensor_ts *sensor);


#endif // TLE493D_A1B6_H
