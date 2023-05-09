#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "pal.h"


void initI2C(I2C_t *i2c);

void setI2CAddress(I2C_t *i2c, uint8_t addr);

bool readBytes(I2C_t *i2c, uint8_t regAddress, uint8_t numBytes, uint8_t *bytes);

bool readWriteBytes(I2C_t *i2c, uint8_t regAddress, uint8_t numBytes, uint8_t *bytes);
