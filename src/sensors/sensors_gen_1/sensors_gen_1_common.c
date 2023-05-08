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

    
    // uint8_t regAddress = 0;
    // uint8_t *bytes     = malloc(numBytes);

    // bool b = readBytes(p2b6->I2C, regAddress, TLE493D_P2B6_REGISTER_MAP_SIZE, bytes);


    // uint16_t temperature = 0;

    // bool b = readBytes(p2b6->I2C, 4, 2, &temperature);
    // double temp = ... * temperature;


    return 0;
}
