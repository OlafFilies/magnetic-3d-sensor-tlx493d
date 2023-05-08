#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


#include "TLV493D_A1B6.h"

#include "i2c.h"


void TLV493D_A1B6_initObject(TLV493D_A1B6_t *sensor) {
    sensor->sensorType = TLV493D_A1B6;

    printf("TLV493D_A1B6_initObject\n");
}


bool TLV493D_A1B6_init(TLV493D_A1B6_t *sensor) {
    printf("TLV493D_A1B6_init\n");
    return true;
}


bool TLV493D_A1B6_deinit(TLV493D_A1B6_t *sensor) {
    printf("TLV493D_A1B6_deinit\n");
    return true;
}


bool TLV493D_A1B6_getFieldValues(TLV493D_A1B6_t *sensor, uint32_t *x, uint32_t *y, uint32_t *z) {
    printf("TLE493D_A1B6_getFieldValues\n");
    return true;
}
