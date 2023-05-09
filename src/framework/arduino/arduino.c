#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "arduino.h"


bool initI2C(I2C_t *i2c) {
    printf("I2C->initI2C\n");
}


void setI2CAddress(I2C_t *i2c, uint8_t addr) {
    printf("I2C->setI2CAddress\n");
}


bool readBytes(I2C_t *i2c, uint8_t regAddress, uint8_t numBytes, uint8_t *bytes) {
    // ...
    printf("I2C->readBytes\n");
    return true;
}


bool readWriteBytes(I2C_t *i2c, uint8_t regAddress, uint8_t numBytes, uint8_t *bytes) {
    printf("I2C->readWriteBytes\n");
    return true;
}
