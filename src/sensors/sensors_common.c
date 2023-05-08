#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


#include "sensor_types.h"
#include "sensors_common.h"
#include "sensors.h"

#include "TLE493D_A1B6/TLE493D_A1B6.h"
#include "TLV493D_A1B6/TLV493D_A1B6.h"
#include "TLE493D_P2B6/TLE493D_P2B6.h"
#include "TLE493D_W2B6/TLE493D_W2B6.h"


bool init(void *sensor) {
    sensorTypes_t sensorType = getSensorType(sensor);
    
    switch(sensorType) {
        case TLE493D_A1B6 : return TLE493D_A1B6_init(sensor);
                            break;

        case TLV493D_A1B6 : return TLV493D_A1B6_init(sensor);
                            break;

        case TLE493D_P2B6 : return TLE493D_P2B6_init(sensor);
                            break;

        case TLE493D_W2B6 : return TLE493D_W2B6_init(sensor);
                            break;

        default:            printf("ERROR : unknown sensor type %d !\n", sensorType);
    }

    printf("init done.\n");
    return true;
}


bool deinit(void *sensor) {
    sensorTypes_t sensorType = getSensorType(sensor);
    
    switch(sensorType) {
         case TLE493D_A1B6 : return TLE493D_A1B6_deinit(sensor);
                            break;

        case TLV493D_A1B6 : return TLV493D_A1B6_deinit(sensor);
                            break;

        case TLE493D_P2B6 : return TLE493D_P2B6_deinit(sensor);
                            break;

        case TLE493D_W2B6 : return TLE493D_W2B6_deinit(sensor);
                            break;

        default:            printf("ERROR : unknown sensor type %d !\n", sensorType);
    }

    printf("deinit\n");
    return true;
}


uint32_t getTemperature(void *sensor) {
    sensorTypes_t sensorType = getSensorType(sensor);
    
    switch(sensorType) {
         case TLE493D_A1B6 : return TLE493D_A1B6_getTemperature(sensor);
                            break;

        case TLV493D_A1B6 : return TLV493D_A1B6_getTemperature(sensor);
                            break;

        case TLE493D_P2B6 : return TLE493D_P2B6_getTemperature(sensor);
                            break;

        case TLE493D_W2B6 : return TLE493D_W2B6_getTemperature(sensor);
                            break;

        default:            printf("ERROR : unknown sensor type %d !\n", sensorType);
    }

    printf("getTemperature\n");
    return true;
}


bool getFieldValues(void *sensor, uint32_t *x, uint32_t *y, uint32_t *z) {
    sensorTypes_t sensorType = getSensorType(sensor);
    
    switch(sensorType) {
         case TLE493D_A1B6 : return TLE493D_A1B6_getFieldValues(sensor, x, y, z);
                            break;

        case TLV493D_A1B6 : return TLV493D_A1B6_getFieldValues(sensor, x, y, z);
                            break;

        case TLE493D_P2B6 : return TLE493D_P2B6_getFieldValues(sensor, x, y, z);
                            break;

        case TLE493D_W2B6 : return TLE493D_W2B6_getFieldValues(sensor, x, y, z);
                            break;

        default:            printf("ERROR : unknown sensor type %d !\n", sensorType);
    }

    printf("getFieldValues\n");
    return true;
}
