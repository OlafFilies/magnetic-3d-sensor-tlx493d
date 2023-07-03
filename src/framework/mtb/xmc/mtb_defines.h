#ifndef MTB_DEFINES_H
#define MTB_DEFINES_H


// std includes
#include <stdio.h>

// MTB includes

// XMC includes
#include "xmc_gpio.h"
#include "xmc_i2c.h"


#define log  printf


typedef struct I2CObject_ts {
    XMC_USIC_CH_t    *channel;
    uint8_t           sourceSDA;
    uint8_t           sourceSCL;
    XMC_GPIO_PORT_t  *portSDA;
    uint8_t           pinSDA;
    XMC_GPIO_PORT_t  *portSCL;
    uint8_t           pinSCL;
} I2CObject_ts;


typedef struct SPIObject_ts SPIObject_ts;
typedef struct Sensor_ts    Sensor_ts;


extern void initI2CComLibIF(Sensor_ts *sensor, XMC_USIC_CH_t *const channel,
                            const uint8_t sourceSDA, const uint8_t sourceSCL,
                            XMC_GPIO_PORT_t *const portSDA, const uint8_t pinSDA,
                            XMC_GPIO_PORT_t *const portSCL, const uint8_t pinSCL);


#endif // MTB_DEFINES_H
