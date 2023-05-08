#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


#include "TLE493D_W2B6.h"

#include "i2c.h"


void TLE493D_W2B6_initObject(TLE493D_W2B6_t *sensor) {
    sensor->sensorType = TLE493D_W2B6;

    printf("TLE493D_W2B6_initObject\n");
}


bool TLE493D_W2B6_init(TLE493D_W2B6_t *sensor) {
    printf("TLE493D_W2B6_init\n");
    return true;
}


bool TLE493D_W2B6_deinit(TLE493D_W2B6_t *sensor) {
    printf("TLE493D_W2B6_deinit\n");
    return true;
}


uint32_t TLE493D_W2B6_getTemperature(TLE493D_W2B6_t *sensor) {
    printf("TLE493D_W2B6_getTemperature\n");
    return true;
}


bool TLE493D_W2B6_getFieldValues(TLE493D_W2B6_t *sensor, uint32_t *x, uint32_t *y, uint32_t *z) {
    printf("TLE493D_W2B6_getFieldValues\n");
    return true;
}
