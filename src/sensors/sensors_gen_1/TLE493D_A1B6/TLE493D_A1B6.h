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


#define TLE493D_A1B6_REGISTER_MAP_SIZE          22    //remove: fix at the end of development

// common functions
bool TLE493D_A1B6_init(Sensor_ts *sensor, SupportedComLibraryInterfaceTypes_te comLibIF);
bool TLE493D_A1B6_deinit(Sensor_ts *sensor);

bool TLE493D_A1B6_getTemperature(Sensor_ts *sensor, float *temp);
bool TLE493D_A1B6_updateGetTemperature(Sensor_ts *sensor, float *temp);

bool TLE493D_A1B6_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
bool TLE493D_A1B6_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);

bool TLE493D_A1B6_reset(Sensor_ts *sensor);
bool TLE493D_A1B6_getDiagnosis(Sensor_ts *sensor);
bool TLE493D_A1B6_calculateParity(Sensor_ts *sensor);

bool TLE493D_A1B6_setDefaultConfig(Sensor_ts *sensor);
bool TLE493D_A1B6_updateRegisterMap(Sensor_ts *sensor);



#endif // TLE493D_A1B6_H
