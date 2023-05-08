#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


#include "sensors.h"

// Only for instantiations of specific sensors
#include "TLE493D_A1B6/TLE493D_A1B6.h"
#include "TLV493D_A1B6/TLV493D_A1B6.h"
#include "TLE493D_P2B6/TLE493D_P2B6.h"
#include "TLE493D_W2B6/TLE493D_W2B6.h"

#include "i2c.h"


int main(int argc, char *argv[]) {
    TLE493D_A1B6_t *a1b6 = createSensor(TLE493D_A1B6);
    bool            b    = init(a1b6);
    // initCommInterface(a1b6);
    // setAddress maybe better included in init(...) with address as additional parameter
    setI2CAddress(a1b6->I2C, 55);

    uint32_t x, y, z;
    b = getFieldValues(a1b6,  &x,  &y, &z);
    uint32_t temperature = getTemperature(a1b6);
    printf("temperature = %d\n", temperature);
    printType(a1b6->sensorType);
    // readBytes normally not called at this level, but inside getter functions
    readBytes(a1b6->I2C, 0, 0, NULL);
 

    printf("\n\n");

    // Second sensor
    TLE493D_P2B6_t *p2b6 = createSensor(TLE493D_P2B6);
    b = init(p2b6);
    // initCommInterface(p2b6);
    setI2CAddress(p2b6->I2C, 55);

    b = getFieldValues(p2b6,  &x,  &y, &z);
    temperature = getTemperature(p2b6);
    printf("temperature = %d\n", temperature);
    printType(p2b6->sensorType);

    readBytes(p2b6->I2C, 0, 0, NULL);
}










