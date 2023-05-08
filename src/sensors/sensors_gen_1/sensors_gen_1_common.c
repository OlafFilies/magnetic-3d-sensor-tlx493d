#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


bool gen_1_init(void *sensor) {
    printf("gen_1_init\n");
    return true;
}


bool gen_1_deinit(void *sensor) {
    printf("gen_1_deinit\n");
    return true;
}


uint32_t gen_1_getTemperature(void *sensor) {
    printf("gen_1_getTemperature\n");
    // readBytes(...)
    // readRegisterMap(..);
    return 0;
}
