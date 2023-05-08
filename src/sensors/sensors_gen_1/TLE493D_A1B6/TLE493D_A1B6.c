#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


#include "TLE493D_A1B6.h"

#include "i2c.h"


void TLE493D_A1B6_initObject(TLE493D_A1B6_t *sensor) {
    sensor->sensorType = TLE493D_A1B6;
    sensor->I2C        = malloc(sizeof(I2C_t));

    initI2C(sensor->I2C);
    printf("TLE493D_A1B6_initObject done.\n");
}


bool TLE493D_A1B6_init(TLE493D_A1B6_t *sensor) {
    printf("TLE493D_A1B6_init\n");

    return true;
}


bool TLE493D_A1B6_deinit(TLE493D_A1B6_t *sensor) {
    printf("TLE493D_A1B6_deinit\n");

    return true;
}


bool TLE493D_A1B6_getFieldValues(TLE493D_A1B6_t *sensor, uint32_t *x, uint32_t *y, uint32_t *z) {
    printf("TLE493D_A1B6_getFieldValues\n");
    return true;
}
