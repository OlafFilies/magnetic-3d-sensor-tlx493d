#ifndef TLV493D_A1B6_H
#define TLV493D_A1B6_H


#include <stdbool.h>
#include <stdint.h>


// common to all sensors
#include "sensors_config_common.h"
#include "sensors_common.h"

// common to generation 1 sensors
#include "sensors_gen_1_config_common.h"
#include "sensors_gen_1_common.h"

#include "TLV493D_A1B6_config.h"

#include "sensors.h"


#define TLV493D_A1B6_getTemperature  (gen_1_getTemperature)


typedef struct TLV493D_A1B6_t TLV493D_A1B6_t;


struct TLV493D_A1B6_t {
    sensorTypes_t  sensorType;
    uint8_t       *registerMap;
    I2C_t         *I2C;
};


void TLV493D_A1B6_initObject(TLV493D_A1B6_t *sensor);


// All interface functions exposed in user API
bool TLV493D_A1B6_init(TLV493D_A1B6_t *sensor);
bool TLV493D_A1B6_deinit(TLV493D_A1B6_t *sensor);
bool TLV493D_A1B6_getFieldValues(TLV493D_A1B6_t *sensor, uint32_t *x, uint32_t *y, uint32_t *z);


#endif // TLV493D_A1B6_H
