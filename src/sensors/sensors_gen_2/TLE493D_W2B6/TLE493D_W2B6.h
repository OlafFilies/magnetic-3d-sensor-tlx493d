#ifndef TLE493D_W2B6_H
#define TLE493D_W2B6_H


#include <stdbool.h>
#include <stdint.h>


// common to all sensors
#include "sensors_config_common.h"
#include "sensors_common.h"

// common to generation 1 sensors
#include "sensors_gen_2_config_common.h"
#include "sensors_gen_2_common.h"

#include "TLE493D_W2B6_config.h"

#include "sensors.h"


typedef struct TLE493D_W2B6_t TLE493D_W2B6_t;


struct TLE493D_W2B6_t {
    sensorTypes_t  sensorType;
    uint8_t       *registerMap;
    I2C_t         *I2C;
};


void TLE493D_W2B6_initObject(TLE493D_W2B6_t *sensor);


// All interface functions exposed in user API
bool TLE493D_W2B6_init(TLE493D_W2B6_t *sensor);
bool TLE493D_W2B6_deinit(TLE493D_W2B6_t *sensor);
uint32_t TLE493D_W2B6_getTemperature(TLE493D_W2B6_t *sensor);
bool TLE493D_W2B6_getFieldValues(TLE493D_W2B6_t *sensor, uint32_t *x, uint32_t *y, uint32_t *z);


#endif // TLE493D_W2B6_H
